#!/usr/bin/env python3
"""
app.py - CANSentinel local web UI.

A browser front-end for the whole offline toolkit, so you can work the pipeline
without the command line:

    Log explorer -> Find control -> Counter & checksum -> Profile -> Replay

It runs entirely on your machine (binds 127.0.0.1) and is strictly READ-ONLY:
it never opens a CAN interface and never transmits. It only reads logs you load,
and the Replay/immobilizer tab is a pure simulation over a recorded log. Nothing
leaves your PC.

Run:
    pip install -r requirements.txt
    python app.py                 # http://127.0.0.1:5000
    PORT=5050 python app.py       # use another port (e.g. if 5000 is busy)

Every endpoint is a thin wrapper over the same functions the CLI tools use
(find_events, checksum_counter, vehicle_profile, live_decode, immobilizer), so
results match the command line exactly.
"""
from __future__ import annotations

import json
import os
import tempfile
import uuid

import numpy as np
from flask import Flask, request, jsonify, render_template

import find_events as fe
import checksum_counter as cc
import vehicle_profile as vp
import live_decode as ld
import immobilizer as imo
import find_continuous as fc
import find_gears as fg

app = Flask(__name__)

UPLOAD_DIR = os.path.join(tempfile.gettempdir(), "cansentinel_uploads")
os.makedirs(UPLOAD_DIR, exist_ok=True)
LOGS: dict[str, str] = {}   # token -> saved path


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _save_upload(file_storage) -> str:
    token = uuid.uuid4().hex[:12]
    safe = os.path.basename(file_storage.filename or "log.csv")
    path = os.path.join(UPLOAD_DIR, f"{token}_{safe}")
    file_storage.save(path)
    LOGS[token] = path
    return token


def _id_table(path: str):
    frames = fe.load_log(path)
    series = fe.build_series(frames)
    ids = []
    for cid in sorted(series):
        s = series[cid]
        cyc = float(np.median(np.diff(s.t)) * 1000) if s.t.size > 1 else 0.0
        ids.append({"id": f"0x{cid:X}", "frames": s.n, "dlc": s.length,
                    "cycle_ms": round(cyc, 1)})
    return len(frames), len(series), ids


def _log_meta(path: str) -> dict:
    """Best-effort capture header for JSON logs: make/model/year/action and the
    recorded press labels, so you can see what the log is and what you did.

    Only dict-shaped JSON captures carry a header; bare-list JSON, JSON-lines
    and CSV/candump logs have none, so this returns {} for them.
    """
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as fh:
            raw = fh.read()
    except OSError:
        return {}
    if not raw.lstrip().startswith("{"):
        return {}
    try:
        doc = json.loads(raw)
    except ValueError:
        return {}
    if not isinstance(doc, dict):
        return {}
    meta = doc.get("meta") if isinstance(doc.get("meta"), dict) else {}
    out: dict = {}
    for k in ("make", "model", "year", "action"):
        v = (meta or {}).get(k)
        if v in (None, ""):
            v = doc.get(k)
        if v not in (None, ""):
            out[k] = v
    presses = doc.get("presses")
    if isinstance(presses, list) and presses:
        out["presses"] = len(presses)
        labels = [p.get("label") for p in presses
                  if isinstance(p, dict) and p.get("label")]
        if labels:
            out["press_labels"] = labels[:8]
    if doc.get("saved"):
        out["saved"] = doc.get("saved")
    return out


def _as_float(d: dict, key: str, default: float) -> float:
    try:
        return float(d.get(key))
    except (TypeError, ValueError):
        return default


# ---------------------------------------------------------------------------
# upload / log explorer
# ---------------------------------------------------------------------------

@app.post("/api/upload")
def upload():
    if "file" not in request.files:
        return jsonify(error="no file"), 400
    f = request.files["file"]
    if not f.filename:
        return jsonify(error="empty filename"), 400
    try:
        token = _save_upload(f)
        nframes, nids, ids = _id_table(LOGS[token])
        return jsonify(token=token, filename=f.filename, frames=nframes,
                       n_ids=nids, ids=ids, meta=_log_meta(LOGS[token]))
    except Exception as e:
        return jsonify(error=f"could not read log: {e}"), 400


# ---------------------------------------------------------------------------
# find a control  (find_events, incl. the human-cadence filter)
# ---------------------------------------------------------------------------

@app.post("/api/find")
def find():
    d = request.get_json(force=True)
    token = d.get("token")
    if token not in LOGS:
        return jsonify(error="load a log first"), 400
    baseline = LOGS.get(d.get("baseline_token")) if d.get("baseline_token") else None
    try:
        events = int(d["events"]) if d.get("events") not in (None, "", "auto") else None
    except (ValueError, TypeError):
        events = None
    mode = d.get("mode", "auto")
    only = fe._parse_id_list(d.get("only_ids") or None)
    min_gap = _as_float(d, "min_gap", 0.30)
    settle_gap = _as_float(d, "settle_gap", 1.0)
    drop = bool(d.get("drop_implausible"))
    try:
        cands = fe.analyze(trace=LOGS[token], n_events=events, event_times_path=None,
                           baseline=baseline, tol=0.5, mode=mode, idle_frac_max=0.2,
                           only_ids=only, exclude_ids=None, top=12,
                           min_gap=min_gap, settle_gap=settle_gap,
                           drop_implausible=drop)
    except Exception as e:
        return jsonify(error=str(e)), 400
    out = []
    for c in cands:
        if c.bit is not None:
            loc = f"byte {c.byte}, bit {c.bit}"
            start = c.byte * 8 + c.bit
            codes = ""
        else:
            loc = f"byte {c.byte}"
            start = c.byte * 8
            codes = ", ".join(f"0x{v:02X}" for v in c.active_values[:8])
        out.append({
            "id": f"0x{c.can_id:X}", "kind": c.kind, "loc": loc,
            "start_bit": start, "idle": c.idle, "codes": codes,
            "count": c.count, "score": round(c.score, 3), "detail": c.detail,
        })
    return jsonify(candidates=out)


# ---------------------------------------------------------------------------
# rolling counter + checksum  (checksum_counter)
# ---------------------------------------------------------------------------

@app.post("/api/checksum")
def checksum():
    d = request.get_json(force=True)
    token = d.get("token")
    if token not in LOGS:
        return jsonify(error="load a log first"), 400
    try:
        frames = fe.load_log(LOGS[token])
        series = fe.build_series(frames)
    except Exception as e:
        return jsonify(error=str(e)), 400
    want = d.get("id")
    if want:
        try:
            cid = int(want, 16)
        except ValueError:
            return jsonify(error=f"bad id {want!r}"), 400
        if cid not in series:
            return jsonify(error=f"ID {want} not in log"), 400
        targets = [cid]
    else:
        targets = sorted(series)
    results = []
    for cid in targets:
        s = series[cid]
        counters = cc.detect_counters(s.payload)
        cset = {h.byte for h in counters}
        checks = cc.detect_checksum(s.payload, cset)
        best = checks[0] if checks else None
        results.append({
            "id": f"0x{cid:X}", "frames": s.n, "dlc": s.length,
            "counters": [f"byte {h.byte} {h.part} (0..{h.modulus-1}, {h.match*100:.0f}%)"
                         for h in counters],
            "checksum": (best.formula if best else None),
            "checksum_algo": (best.algo if best else None),
            "checksum_match": (round(best.match * 100, 1) if best else None),
            "confirmed": bool(best and best.match >= 0.999),
        })
    return jsonify(results=results)


# ---------------------------------------------------------------------------
# vehicle profile  (vehicle_profile)
# ---------------------------------------------------------------------------

@app.get("/api/profile/example")
def profile_example():
    return jsonify(profile=vp.EXAMPLE_PROFILE)


@app.post("/api/profile/validate")
def profile_validate():
    d = request.get_json(force=True)
    token = d.get("token")
    if token not in LOGS:
        return jsonify(error="load a log first"), 400
    profile = d.get("profile")
    if not isinstance(profile, dict):
        return jsonify(error="profile must be a JSON object"), 400
    try:
        results, score, verdict = vp.validate(LOGS[token], profile)
    except Exception as e:
        return jsonify(error=str(e)), 400
    return jsonify(
        score=round(score, 3), verdict=verdict,
        signals=[{"name": r.name, "role": r.role, "ok": r.ok, "detail": r.detail}
                 for r in results],
    )


# ---------------------------------------------------------------------------
# replay / immobilizer simulation  (live_decode + immobilizer)
# ---------------------------------------------------------------------------

@app.get("/api/replay/example")
def replay_example():
    # live_decode uses a decode profile (signals carry "maps_to"); this is the
    # shape replay / the immobilizer sim expect, distinct from the validate
    # profile above.
    return jsonify(profile=ld._TEST_PROFILE)


@app.post("/api/replay")
def replay():
    d = request.get_json(force=True)
    token = d.get("token")
    if token not in LOGS:
        return jsonify(error="load a log first"), 400
    profile = d.get("profile")
    if not isinstance(profile, dict):
        return jsonify(error="profile must be a JSON object"), 400
    try:
        frames = list(ld.frames_from_log(LOGS[token]))
    except Exception as e:
        return jsonify(error=str(e)), 400

    speeds: list[float] = []
    gears: set[str] = set()
    ign = 0
    nt = 0
    buttons: list[dict] = []
    t0 = t1 = None
    try:
        for st in ld.replay_states(frames, profile):
            nt += 1
            if t0 is None:
                t0 = st.t
            t1 = st.t
            if st.speed_kmh is not None:
                speeds.append(st.speed_kmh)
            if st.gear:
                gears.add(st.gear)
            if st.ignition_on:
                ign += 1
            if st.button and len(buttons) < 60:
                buttons.append({"t": round(st.t - t0, 3), "button": st.button})
    except Exception as e:
        return jsonify(error=f"decode failed: {e}"), 400

    base = t0 or 0.0
    summary = {
        "ticks": nt,
        "span_s": round((t1 - t0), 3) if (t0 is not None and t1 is not None) else 0,
        "speed_min": (round(min(speeds), 2) if speeds else None),
        "speed_max": (round(max(speeds), 2) if speeds else None),
        "gears": sorted(gears),
        "ignition_on_ticks": ign,
        "buttons": buttons,
    }
    out = {"summary": summary}

    if d.get("immobilizer"):
        code = d.get("code")
        if isinstance(code, str):
            code = [c.strip() for c in code.split(",") if c.strip()]
        code = code or []
        inh = profile.get("inhibit", {}) or {}
        method = {"type": inh.get("type") or inh.get("method") or "physical", **inh}
        cfg = imo.ImmobilizerConfig(code=code, inhibit_method=method)
        try:
            _im, act, transitions, btns = ld.run_immobilizer(frames, profile, cfg)
        except Exception as e:
            return jsonify(error=f"immobilizer sim failed: {e}"), 400
        out["immobilizer"] = {
            "code": code,
            "final_state": transitions[-1][1] if transitions else None,
            "inhibit_asserted": act.asserted,
            "transitions": [{"t": round(t - base, 3), "state": s, "info": info}
                            for (t, s, info) in transitions][:100],
            "actions": [{"t": round(t - base, 3), "action": a, "method": m}
                        for (t, a, m) in act.log][:100],
            "buttons": [{"t": round(t - base, 3), "button": b} for (t, b) in btns][:100],
        }
    return jsonify(**out)


# ---------------------------------------------------------------------------
# continuous signals  (find_continuous)  - the standstill-gate speed finder
# ---------------------------------------------------------------------------

@app.post("/api/continuous")
def continuous():
    d = request.get_json(force=True)
    token = d.get("token")
    if token not in LOGS:
        return jsonify(error="load a log first"), 400
    only = fe._parse_id_list(d.get("only_ids") or None)
    try:
        cands = fc.analyze(LOGS[token], reference_path=None, top=12, only_ids=only)
    except Exception as e:
        return jsonify(error=str(e)), 400
    out = [{
        "id": f"0x{c.can_id:X}", "byte": c.start_bit // 8, "width": c.width,
        "endian": c.endian, "signed": c.signed,
        "vmin": round(c.vmin, 1), "vmax": round(c.vmax, 1),
        "score": round(c.score, 3), "detail": c.detail,
    } for c in cands]
    return jsonify(candidates=out)


# ---------------------------------------------------------------------------
# gear selector  (find_gears)  - the park-gate finder
# ---------------------------------------------------------------------------

@app.post("/api/gear")
def gear():
    d = request.get_json(force=True)
    token = d.get("token")
    if token not in LOGS:
        return jsonify(error="load a log first"), 400
    only = fe._parse_id_list(d.get("only_ids") or None)
    try:
        positions = int(d["positions"]) if d.get("positions") not in (None, "", "auto") else None
    except (ValueError, TypeError):
        positions = None
    try:
        cands = fg.analyze(LOGS[token], positions=positions, top=12, only_ids=only)
    except Exception as e:
        return jsonify(error=str(e)), 400
    out = [{
        "id": f"0x{c.can_id:X}", "byte": c.byte, "bit": c.bit, "width": c.width,
        "start_bit": c.byte * 8 + c.bit,
        "values": [f"0x{v:X}" for v in c.values],
        "dwells": c.n_dwells, "score": round(c.score, 3), "detail": c.detail,
    } for c in cands]
    return jsonify(candidates=out)


# ---------------------------------------------------------------------------
# page
# ---------------------------------------------------------------------------

@app.get("/")
def index():
    return render_template("index.html")


if __name__ == "__main__":
    import sys
    port = int(os.environ.get("PORT", "5000"))
    for i, a in enumerate(sys.argv[1:], 1):
        if a == "--port" and i + 1 < len(sys.argv):
            port = int(sys.argv[i + 1])
        elif a.isdigit():
            port = int(a)
    print(f"CANSentinel UI -> http://127.0.0.1:{port}  (read-only; Ctrl+C to stop)")
    app.run(host="127.0.0.1", port=port, debug=False)
