#!/usr/bin/env python3
"""
vehicle_profile.py - define a per-PLATFORM CAN profile once, then auto-certify
new models against it from a short log.

This is the coverage engine. Full reverse-engineering of a car is expensive;
most cars share a CAN architecture with many siblings (a "platform"). You RE one
car on a platform with find_events.py + the CSS continuous tooling, write the
result as a profile here, and then *validate* every sibling model from a short
recording in seconds instead of re-doing the whole RE.

It is READ-ONLY. It never opens a bus and never transmits. It loads a recorded
log, checks that the signals a profile promises are actually present and behave
plausibly, and prints a per-signal PASS/FAIL plus an overall match verdict.

A profile captures, per platform, the three things the immobilizer needs:

  * GATE signals (safety-critical): the signals that prove the car is stopped and
    off, so the device may only ever act at standstill - e.g. vehicle speed,
    gear = Park/Neutral, engine state. These are weighted most heavily; if they
    don't validate, the car must NOT be trusted to a data-based inhibit.
  * INHIBIT-TARGET signals: the start/crank-enable field the data profile acts on
    (if a safe data inhibit exists for this platform; else method = physical).
  * FEATURE signals: lock/unlock, buttons - for alarm / remote features.

Usage:
  python vehicle_profile.py --validate <log> --profile <profile.json>
  python vehicle_profile.py --example > my-platform.json   # scaffold to edit
  python vehicle_profile.py --selftest                     # no hardware needed
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass

import numpy as np

# reuse the format-agnostic loader + per-ID matrices from the finder
from find_events import load_log, build_series, IdSeries


# ---------------------------------------------------------------------------
# Field extraction (matches cantools / CSS conventions)
# ---------------------------------------------------------------------------

def _le_field(s: IdSeries, start_bit: int, length: int) -> np.ndarray:
    """Little-endian (Intel) field: LSB-first start bit across the payload."""
    mask = (1 << length) - 1
    out = np.empty(s.payload.shape[0], dtype=np.float64)
    for i, row in enumerate(s.payload):
        v = int.from_bytes(bytes(row.tolist()), "little")
        out[i] = (v >> start_bit) & mask
    return out


def _be_field(s: IdSeries, byte: int, width: int) -> np.ndarray:
    """Big-endian (Motorola) byte-aligned field."""
    L = s.payload.shape[1]
    shift = 8 * (L - byte - width)
    mask = (1 << (8 * width)) - 1
    out = np.empty(s.payload.shape[0], dtype=np.float64)
    for i, row in enumerate(s.payload):
        v = int.from_bytes(bytes(row.tolist()), "big")
        out[i] = (v >> shift) & mask
    return out


def _median_cycle_ms(s: IdSeries) -> float:
    if s.t.size < 2:
        return float("inf")
    return float(np.median(np.diff(s.t)) * 1000.0)


# ---------------------------------------------------------------------------
# Per-signal validation
# ---------------------------------------------------------------------------

@dataclass
class SigResult:
    name: str
    role: str
    ok: bool
    detail: str


_ROLE_WEIGHT = {"gate": 3.0, "inhibit-target": 2.0, "feature": 1.0}


def _check_cycle(s: IdSeries, spec: dict, notes: list[str]) -> None:
    want = spec.get("cycle_ms")
    if want is None:
        return
    got = _median_cycle_ms(s)
    # generous 2.5x tolerance - cycle times vary with bus load / sampling
    if got == float("inf") or not (want / 2.5 <= got <= want * 2.5):
        notes.append(f"cycle {got:.0f}ms (expected ~{want}ms)")


def validate_signal(name: str, spec: dict, series: dict[int, IdSeries]) -> SigResult:
    role = spec.get("role", "feature")
    try:
        cid = int(str(spec["id"]), 16) if isinstance(spec["id"], str) else int(spec["id"])
    except (KeyError, ValueError):
        return SigResult(name, role, False, "profile error: bad/missing id")

    s = series.get(cid)
    if s is None:
        return SigResult(name, role, False, f"ID 0x{cid:X} ABSENT from log")

    notes: list[str] = []
    _check_cycle(s, spec, notes)
    stype = spec.get("type", "discrete")
    ok = True

    if stype == "continuous":
        start = spec.get("start_bit")
        length = spec.get("length")
        endian = spec.get("endian", "little")
        if start is None or length is None:
            if "byte" in spec and "width" in spec:
                raw = _be_field(s, spec["byte"], spec["width"]) if endian == "big" \
                    else _le_field(s, spec["byte"] * 8, spec["width"] * 8)
            else:
                return SigResult(name, role, False, "profile error: no field geometry")
        else:
            raw = _be_field(s, start // 8, length // 8) if endian == "big" \
                else _le_field(s, start, length)
        scale = float(spec.get("scale", 1.0))
        offset = float(spec.get("offset", 0.0))
        val = raw * scale + offset
        exp = spec.get("expect", {})
        lo, hi = exp.get("min", -1e9), exp.get("max", 1e9)
        vmin, vmax = float(np.min(val)), float(np.max(val))
        if not (lo - abs(lo) * 0.1 - 1 <= vmin and vmax <= hi + abs(hi) * 0.1 + 1):
            ok = False
            notes.append(f"range [{vmin:.1f},{vmax:.1f}] outside expected [{lo},{hi}]")
        if exp.get("must_reach_rest") and vmin > (abs(hi) * 0.05 + 1):
            ok = False
            notes.append(f"never reaches rest (min {vmin:.1f})")
        if ok and not notes:
            notes.append(f"range [{vmin:.1f},{vmax:.1f}] {spec.get('unit','')}".strip())

    elif stype in ("discrete", "value-byte"):
        byte = spec.get("byte")
        if byte is None or byte >= s.payload.shape[1]:
            return SigResult(name, role, False, f"byte {byte} out of range (DLC {s.payload.shape[1]})")
        col = s.payload[:, byte]
        if stype == "discrete":
            bit = spec.get("bit", 0)
            bits = (col >> bit) & 1
            distinct = len(np.unique(bits))
            if distinct < 2:
                # a gate bit that never changes in a short idle log can be fine
                # (e.g. Park bit held) - only fail if the profile expects motion
                if spec.get("expect_change", False):
                    ok = False
                    notes.append(f"bit {bit} never changes")
                else:
                    notes.append(f"bit {bit} present, constant={int(bits[0])}")
            else:
                notes.append(f"bit {bit} toggles ({distinct} states)")
        else:  # value-byte
            vals, counts = np.unique(col, return_counts=True)
            idle = spec.get("idle", 0)
            present = int(idle) in vals.tolist()
            if not present:
                ok = False
                notes.append(f"idle 0x{int(idle):02X} not seen on byte {byte}")
            else:
                notes.append(f"byte {byte} enum, {len(vals)} distinct values")
    else:
        return SigResult(name, role, False, f"unknown signal type {stype!r}")

    return SigResult(name, role, ok, "; ".join(notes) if notes else "ok")


def validate(log_path: str, profile: dict) -> tuple[list[SigResult], float, str]:
    frames = load_log(log_path)
    series = build_series(frames)
    results = [validate_signal(n, spec, series) for n, spec in profile.get("signals", {}).items()]

    # weighted match score; gate failures are disqualifying for a data inhibit
    tot_w = sum(_ROLE_WEIGHT.get(r.role, 1.0) for r in results) or 1.0
    got_w = sum(_ROLE_WEIGHT.get(r.role, 1.0) for r in results if r.ok)
    score = got_w / tot_w
    gate_ok = all(r.ok for r in results if r.role == "gate")

    if not gate_ok:
        verdict = "FAIL - safety gate signals did not validate; do NOT use a data inhibit here"
    elif score >= 0.99:
        verdict = "PASS - full match; this model fits the platform profile"
    elif score >= 0.75:
        verdict = "PARTIAL - gates OK; some feature/target signals differ (likely a trim/year variant)"
    else:
        verdict = "FAIL - too many signals differ; this is probably a different platform"
    return results, score, verdict


# ---------------------------------------------------------------------------
# Example scaffold
# ---------------------------------------------------------------------------

EXAMPLE_PROFILE = {
    "platform": "example-platform",
    "models": ["Make Model 2015-2019", "Make Sibling 2016-2020"],
    "bitrate": 500000,
    "notes": "Geometry below is a TEMPLATE - fill it from find_events.py + the "
             "CSS continuous tooling on one real car of this platform.",
    "inhibit": {"method": "physical",
                "detail": "Set to 'data' with an inhibit-target signal once a "
                          "stationary-gated data inhibit is validated for this platform."},
    "signals": {
        "vehicle_speed": {
            "role": "gate", "type": "continuous",
            "id": "0x1A0", "start_bit": 0, "length": 16, "endian": "little",
            "scale": 0.01, "offset": 0, "unit": "km/h",
            "expect": {"min": 0, "max": 260, "must_reach_rest": True},
            "cycle_ms": 20,
        },
        "gear_park": {
            "role": "gate", "type": "discrete",
            "id": "0x3C0", "byte": 2, "bit": 0, "idle": 1, "cycle_ms": 50,
        },
        "lock_state": {
            "role": "feature", "type": "discrete",
            "id": "0x470", "byte": 0, "bit": 0, "idle": 0, "cycle_ms": 100,
        },
        "wheel_buttons": {
            "role": "feature", "type": "value-byte",
            "id": "0x5C1", "byte": 4, "idle": 0, "codes": ["0x01", "0x02", "0x04"],
            "cycle_ms": 50,
        },
    },
}


# ---------------------------------------------------------------------------
# Self-test: synthesize a log that matches the example, and one that doesn't
# ---------------------------------------------------------------------------

def _synth_log(path: str, match: bool) -> None:
    import os
    t0 = 1_700_000_000.0
    rows: list[tuple[str, str, str]] = []

    def emit(t, cid, d):
        rows.append((f"{t:.6f}", f"{cid:03X}", bytes(d).hex()))

    dur = 12.0
    # vehicle_speed 0x1A0 @20ms: ramp 0..60 km/h then back to 0 (reaches rest)
    t = t0
    while t < t0 + dur:
        frac = (t - t0) / dur
        kmh = 60 * (1 - abs(2 * frac - 1))  # triangle 0->60->0
        raw = int(round(kmh / 0.01)) & 0xFFFF
        d = bytearray(8)
        d[0] = raw & 0xFF
        d[1] = (raw >> 8) & 0xFF
        emit(t, 0x1A0 if match else 0x1A0, d)
        t += 0.02
    # gear_park 0x3C0 @50ms: byte2 bit0 = 1 (in Park) the whole idle log
    t = t0
    while t < t0 + dur:
        d = bytearray(8)
        d[2] = 0x01
        emit(t, 0x3C0, d)
        t += 0.05
    # lock_state 0x470 @100ms: byte0 bit0 toggles a couple of times
    t = t0
    st = 0
    flips = [t0 + 3, t0 + 7]
    fi = 0
    while t < t0 + dur:
        while fi < len(flips) and t >= flips[fi]:
            st ^= 1
            fi += 1
        d = bytearray(8)
        d[0] = st
        emit(t, 0x470, d)
        t += 0.1
    # wheel_buttons 0x5C1 @50ms: byte4 enum idle 0, codes appear
    t = t0
    hits = {t0 + 2: 1, t0 + 5: 2, t0 + 9: 4}
    while t < t0 + dur:
        d = bytearray(8)
        for ht, code in hits.items():
            if abs(t - ht) < 0.15:
                d[4] = code
        emit(t, 0x5C1, d)
        t += 0.05

    if not match:
        # break the gate: make speed never reach rest (stuck high) -> must FAIL
        rows = [r for r in rows if r[1] != "1A0"]
        t = t0
        while t < t0 + dur:
            raw = int(round(90 / 0.01)) & 0xFFFF  # constant 90 km/h, never rests
            d = bytearray(8)
            d[0] = raw & 0xFF
            d[1] = (raw >> 8) & 0xFF
            emit(t, 0x1A0, d)
            t += 0.02

    rows.sort(key=lambda x: float(x[0]))
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("epoch;id;data\n")
        for r in rows:
            fh.write(";".join(r) + "\n")


def _selftest() -> int:
    import os
    import tempfile
    ok = True
    good = os.path.join(tempfile.gettempdir(), "profile_match.csv")
    bad = os.path.join(tempfile.gettempdir(), "profile_mismatch.csv")
    _synth_log(good, match=True)
    _synth_log(bad, match=False)

    print("[selftest] MATCHING log vs example profile:")
    res, score, verdict = validate(good, EXAMPLE_PROFILE)
    for r in res:
        print(f"  {'PASS' if r.ok else 'FAIL'} [{r.role:14}] {r.name}: {r.detail}")
    print(f"  score {score:.2f} -> {verdict}")
    ok &= verdict.startswith("PASS")

    print("\n[selftest] MISMATCH log (speed never rests -> gate fails):")
    res, score, verdict = validate(bad, EXAMPLE_PROFILE)
    for r in res:
        print(f"  {'PASS' if r.ok else 'FAIL'} [{r.role:14}] {r.name}: {r.detail}")
    print(f"  score {score:.2f} -> {verdict}")
    ok &= verdict.startswith("FAIL")

    print("\n[selftest]", "ALL PASSED" if ok else "SOME FAILED")
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Validate a recorded log against a platform CAN profile (read-only).",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__,
    )
    ap.add_argument("--validate", metavar="LOG", help="a recorded CAN log to certify")
    ap.add_argument("--profile", help="profile JSON to validate against")
    ap.add_argument("--example", action="store_true", help="print an example profile and exit")
    ap.add_argument("--selftest", action="store_true", help="synthesize logs and verify")
    args = ap.parse_args(argv)

    if args.example:
        print(json.dumps(EXAMPLE_PROFILE, indent=2))
        return 0
    if args.selftest:
        return _selftest()
    if not args.validate or not args.profile:
        ap.error("need --validate LOG --profile JSON (or --example / --selftest)")

    with open(args.profile, "r", encoding="utf-8") as fh:
        profile = json.load(fh)
    res, score, verdict = validate(args.validate, profile)
    print(f"\nPlatform: {profile.get('platform','?')}   "
          f"log: {args.validate}\n")
    for r in res:
        print(f"  {'PASS' if r.ok else 'FAIL'} [{r.role:14}] {r.name}: {r.detail}")
    print(f"\n  match score {score:.2f}\n  {verdict}")
    return 0 if not verdict.startswith("FAIL") else 2


if __name__ == "__main__":
    sys.exit(main())
