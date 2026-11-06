#!/usr/bin/env python3
"""
app.py - CANSentinel local web UI.

A browser front-end for the toolkit so you can work without the command line:
load a CAN log, say what you did, and it finds the button/lock ID or cracks the
counter/checksum. Runs entirely on your machine (binds to 127.0.0.1); it only
reads logs you load - nothing is transmitted and nothing leaves your PC.

Run:
    pip install -r requirements.txt
    python app.py
    # then open http://127.0.0.1:5000

Everything it does is a thin wrapper over the same functions the CLI tools use
(find_events.py, checksum_counter.py), so results match the command line exactly.
"""
from __future__ import annotations

import os
import tempfile
import uuid

from flask import Flask, request, jsonify, render_template_string

import find_events as fe
import checksum_counter as cc

app = Flask(__name__)
UPLOAD_DIR = os.path.join(tempfile.gettempdir(), "cansentinel_uploads")
os.makedirs(UPLOAD_DIR, exist_ok=True)
LOGS: dict[str, str] = {}   # token -> saved path


# ---------------------------------------------------------------------------
# API
# ---------------------------------------------------------------------------

def _save_upload(file_storage) -> str:
    token = uuid.uuid4().hex[:12]
    safe = os.path.basename(file_storage.filename or "log.csv")
    path = os.path.join(UPLOAD_DIR, f"{token}_{safe}")
    file_storage.save(path)
    LOGS[token] = path
    return token


@app.post("/api/upload")
def upload():
    if "file" not in request.files:
        return jsonify(error="no file"), 400
    f = request.files["file"]
    if not f.filename:
        return jsonify(error="empty filename"), 400
    try:
        token = _save_upload(f)
        frames = fe.load_log(LOGS[token])
        series = fe.build_series(frames)
        ids = []
        for cid in sorted(series):
            s = series[cid]
            import numpy as np
            cyc = float(np.median(np.diff(s.t)) * 1000) if s.t.size > 1 else 0.0
            ids.append({"id": f"0x{cid:X}", "frames": s.n, "dlc": s.length,
                        "cycle_ms": round(cyc, 1)})
        return jsonify(token=token, filename=f.filename, frames=len(frames),
                       n_ids=len(series), ids=ids)
    except Exception as e:
        return jsonify(error=f"could not read log: {e}"), 400


@app.post("/api/find")
def find():
    d = request.get_json(force=True)
    token = d.get("token")
    if token not in LOGS:
        return jsonify(error="load a log first"), 400
    baseline = LOGS.get(d.get("baseline_token")) if d.get("baseline_token") else None
    try:
        events = int(d["events"]) if d.get("events") not in (None, "", "auto") else None
    except ValueError:
        events = None
    mode = d.get("mode", "auto")
    only = fe._parse_id_list(d.get("only_ids") or None)
    try:
        cands = fe.analyze(trace=LOGS[token], n_events=events, event_times_path=None,
                           baseline=baseline, tol=0.5, mode=mode, idle_frac_max=0.2,
                           only_ids=only, exclude_ids=None, top=12)
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
    targets = []
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
# UI
# ---------------------------------------------------------------------------

PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>CANSentinel</title>
<style>
  :root{--bg:#0f1115;--panel:#171a21;--line:#262b36;--fg:#e6e9ef;--mut:#8b93a7;
        --accent:#4ea1ff;--good:#37d19a;--warn:#f0b24a;--chip:#1f2530;}
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--fg);
       font:15px/1.5 ui-sans-serif,system-ui,Segoe UI,Roboto,Arial}
  header{padding:18px 24px;border-bottom:1px solid var(--line);display:flex;
         align-items:baseline;gap:12px}
  header h1{font-size:19px;margin:0;letter-spacing:.3px}
  header .sub{color:var(--mut);font-size:13px}
  main{max-width:1000px;margin:0 auto;padding:24px 16px}
  .panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;
         padding:18px;margin-bottom:18px}
  .panel h2{font-size:14px;margin:0 0 12px;color:var(--mut);text-transform:uppercase;
            letter-spacing:.6px}
  label{display:block;font-size:13px;color:var(--mut);margin:10px 0 4px}
  input,select{background:#0c0e13;border:1px solid var(--line);color:var(--fg);
       border-radius:7px;padding:9px 10px;font-size:14px;width:100%}
  .row{display:flex;gap:14px;flex-wrap:wrap}
  .row>div{flex:1;min-width:150px}
  button{background:var(--accent);color:#061018;border:0;border-radius:7px;
       padding:10px 16px;font-weight:600;font-size:14px;cursor:pointer;margin-top:12px}
  button.ghost{background:var(--chip);color:var(--fg);border:1px solid var(--line)}
  button:disabled{opacity:.5;cursor:not-allowed}
  .chips{display:flex;gap:8px;flex-wrap:wrap;margin-top:10px}
  .chip{background:var(--chip);border:1px solid var(--line);border-radius:999px;
        padding:4px 11px;font-size:12px;color:var(--mut)}
  table{width:100%;border-collapse:collapse;margin-top:12px;font-size:13px}
  th,td{text-align:left;padding:8px 10px;border-bottom:1px solid var(--line)}
  th{color:var(--mut);font-weight:600;font-size:12px;text-transform:uppercase}
  tr.top td{background:rgba(78,161,255,.08)}
  .mono{font-family:ui-monospace,SFMono-Regular,Menlo,monospace}
  .tag{font-size:11px;padding:2px 7px;border-radius:5px;background:var(--chip)}
  .tag.pulse{color:#8fd4ff}.tag.toggle{color:#c6a6ff}.tag.value{color:#ffd28f}
  .ok{color:var(--good)}.bad{color:var(--warn)}
  .tabs{display:flex;gap:6px;margin-bottom:16px}
  .tabs button{margin:0;background:var(--chip);color:var(--fg);
       border:1px solid var(--line);font-weight:500}
  .tabs button.active{background:var(--accent);color:#061018;border-color:var(--accent)}
  .hide{display:none}.hint{color:var(--mut);font-size:12px;margin-top:6px}
  .err{color:var(--warn);margin-top:10px;font-size:13px}
  .score-bar{height:6px;background:var(--chip);border-radius:3px;overflow:hidden;width:70px;display:inline-block;vertical-align:middle}
  .score-bar>i{display:block;height:100%;background:var(--good)}
</style></head><body>
<header><h1>CANSentinel</h1><span class="sub">CAN anti-theft toolkit &middot; local, read-only</span></header>
<main>
  <div class="panel">
    <h2>1 &middot; Load a CAN log</h2>
    <div class="row"><div>
      <input type="file" id="logfile" accept=".csv,.log,.txt,.candump,.dump,.asc">
      <div class="hint">webCAN / CANsub CSV, candump, or generic epoch;id;data. Read-only.</div>
    </div><div style="flex:0">
      <button id="loadBtn">Load</button>
    </div></div>
    <div id="summary" class="chips hide"></div>
    <div id="loadErr" class="err"></div>
  </div>

  <div class="panel" id="work" style="opacity:.5;pointer-events:none">
    <div class="tabs">
      <button class="active" data-tab="find">Find a control</button>
      <button data-tab="chk">Counter &amp; checksum</button>
    </div>

    <div id="tab-find">
      <h2>Find a button / lock / switch</h2>
      <div class="row">
        <div><label>How many times you did it</label>
          <input id="events" type="number" min="1" placeholder="e.g. 4" value="4"></div>
        <div><label>Encoding</label>
          <select id="mode">
            <option value="auto">auto (try all)</option>
            <option value="pulse">pulse bit (momentary button)</option>
            <option value="toggle">toggle bit (lock / switch / gear)</option>
            <option value="value">value byte (wheel / console cluster)</option>
          </select></div>
        <div><label>Only these IDs (optional)</label>
          <input id="onlyids" placeholder="0x200,0x5C1"></div>
      </div>
      <div class="hint">Count the number of times you physically did the action and enter it.
        On/off doesn't matter &mdash; the tool tolerates it.</div>
      <button id="findBtn">Find it</button>
      <div id="findErr" class="err"></div>
      <div id="findOut"></div>
    </div>

    <div id="tab-chk" class="hide">
      <h2>Rolling counter &amp; checksum (for transmitting)</h2>
      <div class="row"><div>
        <label>CAN ID (blank = scan every ID)</label>
        <input id="chkid" placeholder="0x1A0"></div></div>
      <div class="hint">Finds the counter and cracks the checksum (sum8 / xor8 / CRC-8)
        so a frame you transmit is accepted by the ECU.</div>
      <button id="chkBtn">Analyze</button>
      <div id="chkErr" class="err"></div>
      <div id="chkOut"></div>
    </div>
  </div>
</main>
<script>
let TOKEN=null;
const $=s=>document.querySelector(s);

$("#loadBtn").onclick=async()=>{
  const f=$("#logfile").files[0]; const e=$("#loadErr"); e.textContent="";
  if(!f){e.textContent="Choose a log file first.";return;}
  const fd=new FormData(); fd.append("file",f);
  $("#loadBtn").disabled=true; $("#loadBtn").textContent="Loading…";
  try{
    const r=await fetch("/api/upload",{method:"POST",body:fd}); const j=await r.json();
    if(j.error){e.textContent=j.error;}
    else{
      TOKEN=j.token;
      const idlist=j.ids.slice(0,40).map(x=>`<span class="chip mono">${x.id} · ${x.frames}f · ${x.cycle_ms}ms</span>`).join("");
      $("#summary").classList.remove("hide");
      $("#summary").innerHTML=`<span class="chip">${j.frames} frames</span>`+
        `<span class="chip">${j.n_ids} IDs</span>`+idlist;
      $("#work").style.opacity=1; $("#work").style.pointerEvents="auto";
    }
  }catch(ex){e.textContent=ex;}
  $("#loadBtn").disabled=false; $("#loadBtn").textContent="Load";
};

document.querySelectorAll(".tabs button").forEach(b=>b.onclick=()=>{
  document.querySelectorAll(".tabs button").forEach(x=>x.classList.remove("active"));
  b.classList.add("active");
  $("#tab-find").classList.toggle("hide",b.dataset.tab!=="find");
  $("#tab-chk").classList.toggle("hide",b.dataset.tab!=="chk");
});

$("#findBtn").onclick=async()=>{
  const e=$("#findErr"); e.textContent=""; $("#findOut").innerHTML="";
  const body={token:TOKEN,events:$("#events").value,mode:$("#mode").value,only_ids:$("#onlyids").value};
  $("#findBtn").disabled=true; $("#findBtn").textContent="Searching…";
  try{
    const r=await fetch("/api/find",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(body)});
    const j=await r.json();
    if(j.error){e.textContent=j.error;}
    else if(!j.candidates.length){e.textContent="No candidates. Try mode=value, or check you triggered the action during the log.";}
    else{
      let h=`<table><tr><th>#</th><th>CAN ID</th><th>Where</th><th>Type</th><th>start-bit</th><th>Codes</th><th>Match</th><th>Score</th></tr>`;
      j.candidates.forEach((c,i)=>{
        const cls=i===0?' class="top"':'';
        h+=`<tr${cls}><td>${i+1}</td><td class="mono">${c.id}</td><td>${c.loc}</td>`+
           `<td><span class="tag ${c.kind.split('-')[0]}">${c.kind}</span></td>`+
           `<td class="mono">${c.start_bit}</td><td class="mono">${c.codes||'—'}</td>`+
           `<td>${c.detail}</td>`+
           `<td><span class="score-bar"><i style="width:${Math.max(0,Math.min(100,c.score*100))}%"></i></span> ${c.score}</td></tr>`;
      });
      h+="</table><div class='hint'>Top row is the best match. Confirm by re-recording with a different count.</div>";
      $("#findOut").innerHTML=h;
    }
  }catch(ex){e.textContent=ex;}
  $("#findBtn").disabled=false; $("#findBtn").textContent="Find it";
};

$("#chkBtn").onclick=async()=>{
  const e=$("#chkErr"); e.textContent=""; $("#chkOut").innerHTML="";
  $("#chkBtn").disabled=true; $("#chkBtn").textContent="Analyzing…";
  try{
    const r=await fetch("/api/checksum",{method:"POST",headers:{"Content-Type":"application/json"},
      body:JSON.stringify({token:TOKEN,id:$("#chkid").value.trim()})});
    const j=await r.json();
    if(j.error){e.textContent=j.error;}
    else{
      let h=`<table><tr><th>CAN ID</th><th>Counter</th><th>Checksum</th><th>Match</th></tr>`;
      j.results.forEach(x=>{
        const cs=x.checksum?`<span class="mono">${x.checksum}</span>`:'<span class="hint">none found</span>';
        const m=x.checksum_match!=null?(x.confirmed?`<span class="ok">${x.checksum_match}% ✓</span>`:`<span class="bad">${x.checksum_match}%</span>`):'—';
        h+=`<tr><td class="mono">${x.id}</td><td>${x.counters.join('<br>')||'—'}</td><td>${cs}</td><td>${m}</td></tr>`;
      });
      h+="</table><div class='hint'>✓ = formula reproduces the real checksum on 100% of frames.</div>";
      $("#chkOut").innerHTML=h;
    }
  }catch(ex){e.textContent=ex;}
  $("#chkBtn").disabled=false; $("#chkBtn").textContent="Analyze";
};
</script></body></html>"""


@app.get("/")
def index():
    return render_template_string(PAGE)


if __name__ == "__main__":
    print("CANSentinel UI -> http://127.0.0.1:5000  (Ctrl+C to stop)")
    app.run(host="127.0.0.1", port=5000, debug=False)
