"use strict";
const $ = (s) => document.querySelector(s);
const $$ = (s) => document.querySelectorAll(s);
let TOKEN = null;
let META = null;

/* ---- tabs ---- */
$$(".tab").forEach((t) => t.addEventListener("click", () => {
  $$(".tab").forEach((x) => x.classList.remove("active"));
  $$(".panel").forEach((x) => x.classList.remove("active"));
  t.classList.add("active");
  $("#tab-" + t.dataset.tab).classList.add("active");
}));

/* ---- helpers ---- */
const esc = (s) => String(s).replace(/[&<>"]/g, (c) =>
  ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));

async function postJSON(url, body) {
  const r = await fetch(url, {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  return r.json();
}

async function uploadFile(file) {
  const fd = new FormData();
  fd.append("file", file);
  const r = await fetch("/api/upload", { method: "POST", body: fd });
  return r.json();
}

function needLog(errEl) {
  if (!TOKEN) { errEl.textContent = "Load a log first (Log tab)."; return true; }
  return false;
}

function busy(btn, on, label) {
  btn.disabled = on;
  if (on) { btn.dataset.lbl = btn.textContent; btn.textContent = "Working…"; }
  else { btn.textContent = label || btn.dataset.lbl || btn.textContent; }
}

/* ---- Log ---- */
$("#logBtn").onclick = async () => {
  const err = $("#logErr"); err.textContent = "";
  const f = $("#logFile").files[0];
  if (!f) { err.textContent = "Choose a file."; return; }
  busy($("#logBtn"), true);
  try {
    const j = await uploadFile(f);
    if (j.error) { err.textContent = j.error; }
    else {
      TOKEN = j.token;
      META = j.meta || {};
      renderMeta();
      const st = $("#logStatus");
      st.textContent = j.filename + " · " + j.frames.toLocaleString() + " frames · " + j.n_ids + " IDs";
      st.className = "badge loaded";
      const info = $("#logInfo");
      info.hidden = false;
      info.innerHTML = "Loaded <b>" + esc(j.filename) + "</b> — " +
        j.frames.toLocaleString() + " frames across " + j.n_ids + " CAN IDs. Now use the other tabs.";
      let h = "<table><tr><th>CAN ID</th><th>Frames</th><th>DLC</th><th>Cycle (ms)</th></tr>";
      j.ids.forEach((x) => {
        h += "<tr><td class='mono'>" + x.id + "</td><td>" + x.frames +
          "</td><td>" + x.dlc + "</td><td>" + x.cycle_ms + "</td></tr>";
      });
      $("#idTableWrap").innerHTML = h + "</table>";
    }
  } catch (ex) { err.textContent = ex; }
  busy($("#logBtn"), false, "Load log");
};

/* ---- Live capture ---- */
let LIVE = { connected: false, timer: null, seq: 0, ids: {} };

function lvRender() {
  const keys = Object.keys(LIVE.ids).sort();
  if (!keys.length) { $("#lvMon").innerHTML = "<p class='hint'>Waiting for frames…</p>"; return; }
  let h = "<table><tr><th>ID</th><th>last data</th><th>count</th><th>dir</th></tr>";
  keys.forEach((k) => { const r = LIVE.ids[k]; h += "<tr><td class='mono'>" + k + "</td><td class='mono'>" + esc(r.data) + "</td><td>" + r.count + "</td><td>" + r.dir + "</td></tr>"; });
  $("#lvMon").innerHTML = h + "</table>";
}
async function lvPoll() {
  if (!LIVE.connected) return;
  try {
    const r = await fetch("/api/live/frames?since=" + LIVE.seq);
    const j = await r.json();
    if (j.seq) LIVE.seq = j.seq;
    (j.frames || []).forEach((f) => {
      const e = LIVE.ids[f.id_hex] || (LIVE.ids[f.id_hex] = { count: 0, data: "", dir: f.dir });
      e.count++; e.data = f.data_hex; e.dir = f.dir;
    });
    lvRender();
    $("#lvStatus").textContent = "connected · " + Object.keys(LIVE.ids).length + " IDs";
  } catch (ex) { /* transient */ }
}
$("#lvConnect").onclick = async () => {
  const err = $("#lvErr"); err.textContent = "";
  busy($("#lvConnect"), true);
  try {
    const j = await postJSON("/api/live/connect", {
      driver: $("#lvDriver").value, bitrate: $("#lvBitrate").value,
      channel: $("#lvChannel").value, listen_only: $("#lvListen").checked,
      serial_port: $("#lvPort").value,
    });
    if (j.error) { err.textContent = j.error; }
    else {
      LIVE.connected = true; LIVE.seq = 0; LIVE.ids = {};
      $("#lvStatus").textContent = "connected · " + ((j.driver && j.driver.name) || "");
      $("#lvStatus").className = "badge loaded";
      $("#lvSim").hidden = ($("#lvDriver").value !== "simulator");
      if (LIVE.timer) clearInterval(LIVE.timer);
      LIVE.timer = setInterval(lvPoll, 600);
    }
  } catch (ex) { err.textContent = ex; }
  busy($("#lvConnect"), false, "Connect");
};
$("#lvDisconnect").onclick = async () => {
  try { await postJSON("/api/live/disconnect", {}); } catch (ex) { /* ignore */ }
  LIVE.connected = false; if (LIVE.timer) clearInterval(LIVE.timer);
  $("#lvStatus").textContent = "not connected"; $("#lvStatus").className = "badge muted";
  $("#lvSim").hidden = true;
};
async function lvLoadPorts() {
  try {
    const r = await fetch("/api/live/ports"); const j = await r.json();
    const sel = $("#lvPort"); const cur = sel.value;
    sel.innerHTML = "<option value=''>(none)</option>" +
      (j.ports || []).map((p) => "<option value='" + esc(p.device) + "'>" + esc(p.device) +
        (p.description ? " — " + esc(p.description) : "") + "</option>").join("");
    sel.value = cur;
    if (!(j.ports || []).length) {
      sel.innerHTML = "<option value=''>(no serial ports — install pyserial / check the reader)</option>";
    }
  } catch (ex) { /* ignore */ }
}
$("#lvPorts").onclick = lvLoadPorts;
lvLoadPorts();
$$(".lvsim").forEach((b) => b.onclick = () => postJSON("/api/live/sim/trigger", { event: b.dataset.ev }));
$("#rvStart").onclick = async () => {
  const err = $("#lvErr"); err.textContent = "";
  if (!LIVE.connected) { err.textContent = "Connect first."; return; }
  const j = await postJSON("/api/live/record/start", {
    make: $("#rvMake").value, model: $("#rvModel").value, year: $("#rvYear").value,
    action: $("#rvAction").value, count: $("#rvCount").value,
  });
  if (j.error) { err.textContent = j.error; return; }
  $("#rvStart").hidden = true; $("#rvStop").hidden = false; $("#rvState").textContent = "recording…";
};
$("#rvStop").onclick = async () => {
  const j = await postJSON("/api/live/record/stop", {});
  $("#rvStop").hidden = true; $("#rvStart").hidden = false; $("#rvState").textContent = "";
  if (j.error) { $("#lvErr").textContent = j.error; return; }
  TOKEN = j.token; META = j.meta || {}; renderMeta();
  const st = $("#logStatus");
  st.textContent = j.filename + " · " + j.frames.toLocaleString() + " frames · " + j.n_ids + " IDs";
  st.className = "badge loaded";
  const done = $("#rvDone"); done.hidden = false;
  done.innerHTML = "Recorded <b>" + esc(j.filename) + "</b> — " + j.frames.toLocaleString() +
    " frames, " + j.n_ids + " IDs. <b>Loaded into the analysis</b> — go to Find / Speed / Gear. " +
    "<a href='/api/live/download?token=" + encodeURIComponent(j.token) + "'>Download file</a>";
  let h = "<table><tr><th>CAN ID</th><th>Frames</th><th>DLC</th><th>Cycle (ms)</th></tr>";
  j.ids.forEach((x) => { h += "<tr><td class='mono'>" + x.id + "</td><td>" + x.frames + "</td><td>" + x.dlc + "</td><td>" + x.cycle_ms + "</td></tr>"; });
  $("#idTableWrap").innerHTML = h + "</table>";
};

/* ---- Find ---- */
$("#findBtn").onclick = async () => {
  const err = $("#findErr"); err.textContent = ""; $("#findOut").innerHTML = "";
  if (needLog(err)) return;
  busy($("#findBtn"), true);
  try {
    let baseTok = null;
    const bf = $("#fBaseline").files[0];
    if (bf) {
      const bj = await uploadFile(bf);
      if (bj.error) { err.textContent = "baseline: " + bj.error; busy($("#findBtn"), false, "Find"); return; }
      baseTok = bj.token;
    }
    const j = await postJSON("/api/find", {
      token: TOKEN, events: $("#fEvents").value.trim(), mode: $("#fMode").value,
      only_ids: $("#fOnly").value.trim(), baseline_token: baseTok,
      min_gap: $("#fMinGap").value, settle_gap: $("#fSettle").value,
      drop_implausible: $("#fDrop").checked,
    });
    if (j.error) { err.textContent = j.error; }
    else if (!j.candidates.length) { $("#findOut").innerHTML = "<p class='lede'>No candidates. Try mode=value, add a baseline, or check the event count.</p>"; }
    else {
      let h = "<table><tr><th class='rank'>#</th><th>ID</th><th>field</th><th>kind</th><th>codes</th><th>count</th><th>score</th><th>detail</th></tr>";
      j.candidates.forEach((c, i) => {
        const w = Math.max(6, Math.min(120, c.score * 110));
        h += "<tr class='" + (i === 0 ? "top1" : "") + "'><td class='rank'>" + (i + 1) +
          "</td><td class='mono'>" + c.id + "</td><td>" + esc(c.loc) +
          "</td><td><span class='pill'>" + c.kind + "</span></td><td class='mono'>" + esc(c.codes || "—") +
          "</td><td>" + c.count + "</td><td><span class='scorebar' style='width:" + w + "px'></span> " +
          c.score.toFixed(3) + "</td><td>" + esc(c.detail) + "</td></tr>";
      });
      $("#findOut").innerHTML = h + "</table>";
    }
  } catch (ex) { err.textContent = ex; }
  busy($("#findBtn"), false, "Find");
};

/* ---- Speed / continuous ---- */
$("#contBtn").onclick = async () => {
  const err = $("#contErr"); err.textContent = ""; $("#contOut").innerHTML = "";
  if (needLog(err)) return;
  busy($("#contBtn"), true);
  try {
    const j = await postJSON("/api/continuous", { token: TOKEN, only_ids: $("#cOnly").value.trim() });
    if (j.error) { err.textContent = j.error; }
    else if (!j.candidates.length) {
      $("#contOut").innerHTML = "<p class='lede'>No continuous candidates. Make sure you drove a clear 0→speed→0 maneuver during the capture.</p>";
    } else {
      let h = "<table><tr><th class='rank'>#</th><th>ID</th><th>field</th><th>type</th><th>raw range</th><th>score</th><th>detail</th></tr>";
      j.candidates.forEach((c, i) => {
        const w = Math.max(6, Math.min(120, c.score * 110));
        const typ = c.width + "-bit" + (c.width === 16 ? " " + c.endian : "") + (c.signed ? " signed" : "");
        h += "<tr class='" + (i === 0 ? "top1" : "") + "'><td class='rank'>" + (i + 1) +
          "</td><td class='mono'>" + c.id + "</td><td>byte " + c.byte +
          "</td><td><span class='pill'>" + typ + "</span></td><td class='mono'>" + c.vmin + "–" + c.vmax +
          "</td><td><span class='scorebar' style='width:" + w + "px'></span> " + c.score.toFixed(3) +
          "</td><td>" + esc(c.detail) + "</td></tr>";
      });
      $("#contOut").innerHTML = h + "</table>";
    }
  } catch (ex) { err.textContent = ex; }
  busy($("#contBtn"), false, "Find continuous");
};

/* ---- Gear ---- */
$("#gearBtn").onclick = async () => {
  const err = $("#gearErr"); err.textContent = ""; $("#gearOut").innerHTML = "";
  if (needLog(err)) return;
  busy($("#gearBtn"), true);
  try {
    const j = await postJSON("/api/gear", { token: TOKEN, positions: $("#gPos").value.trim(), only_ids: $("#gOnly").value.trim() });
    if (j.error) { err.textContent = j.error; }
    else if (!j.candidates.length) {
      $("#gearOut").innerHTML = "<p class='lede'>No gear-like fields. Shift through the gears during the capture and set how many positions you used.</p>";
    } else {
      let h = "<table><tr><th class='rank'>#</th><th>ID</th><th>field</th><th>held codes</th><th>dwells</th><th>score</th></tr>";
      j.candidates.forEach((c, i) => {
        const w = Math.max(6, Math.min(120, c.score * 110));
        h += "<tr class='" + (i === 0 ? "top1" : "") + "'><td class='rank'>" + (i + 1) +
          "</td><td class='mono'>" + c.id + "</td><td>byte " + c.byte + ", bit " + c.bit + ", w" + c.width +
          "</td><td class='mono'>" + c.values.join(", ") + "</td><td>" + c.dwells +
          "</td><td><span class='scorebar' style='width:" + w + "px'></span> " + c.score.toFixed(3) + "</td></tr>";
      });
      $("#gearOut").innerHTML = h + "</table>";
    }
  } catch (ex) { err.textContent = ex; }
  busy($("#gearBtn"), false, "Find gear");
};

/* ---- Checksum ---- */
$("#chkBtn").onclick = async () => {
  const err = $("#chkErr"); err.textContent = ""; $("#chkOut").innerHTML = "";
  if (needLog(err)) return;
  busy($("#chkBtn"), true);
  try {
    const j = await postJSON("/api/checksum", { token: TOKEN, id: $("#chkId").value.trim() });
    if (j.error) { err.textContent = j.error; }
    else {
      let h = "<table><tr><th>CAN ID</th><th>Counter</th><th>Checksum</th><th>Match</th></tr>";
      j.results.forEach((x) => {
        const cs = x.checksum ? "<span class='mono'>" + esc(x.checksum) + "</span>" : "<span class='hint'>none</span>";
        const m = x.checksum_match != null
          ? (x.confirmed ? "<span class='ok'>" + x.checksum_match + "% ✓</span>"
                         : "<span class='bad'>" + x.checksum_match + "%</span>") : "—";
        h += "<tr><td class='mono'>" + x.id + "</td><td>" + (x.counters.join("<br>") || "—") +
          "</td><td>" + cs + "</td><td>" + m + "</td></tr>";
      });
      h += "</table><p class='hint'>✓ = formula reproduces the real checksum on 100% of frames.</p>";
      $("#chkOut").innerHTML = h;
    }
  } catch (ex) { err.textContent = ex; }
  busy($("#chkBtn"), false, "Analyze");
};

/* ---- Profile ---- */
$("#profExBtn").onclick = async () => {
  const r = await fetch("/api/profile/example"); const j = await r.json();
  $("#profJson").value = JSON.stringify(j.profile, null, 2);
};
$("#profValBtn").onclick = async () => {
  const err = $("#profErr"); err.textContent = "";
  $("#profOut").innerHTML = ""; $("#profVerdict").hidden = true;
  if (needLog(err)) return;
  let profile;
  try { profile = JSON.parse($("#profJson").value); }
  catch (e) { err.textContent = "Profile JSON is invalid: " + e.message; return; }
  busy($("#profValBtn"), true);
  try {
    const j = await postJSON("/api/profile/validate", { token: TOKEN, profile });
    if (j.error) { err.textContent = j.error; }
    else {
      const v = $("#profVerdict"); v.hidden = false;
      const kind = j.verdict.startsWith("PASS") ? "pass" : j.verdict.startsWith("PARTIAL") ? "partial" : "fail";
      v.className = "verdict " + kind;
      v.textContent = j.verdict + "   ·   score " + (j.score * 100).toFixed(0) + "%";
      let h = "<table><tr><th>signal</th><th>role</th><th>status</th><th>detail</th></tr>";
      j.signals.forEach((s) => {
        h += "<tr><td>" + esc(s.name) + "</td><td><span class='pill'>" + esc(s.role) +
          "</span></td><td>" + (s.ok ? "<span class='ok'>OK</span>" : "<span class='bad'>FAIL</span>") +
          "</td><td>" + esc(s.detail) + "</td></tr>";
      });
      $("#profOut").innerHTML = h + "</table>";
    }
  } catch (ex) { err.textContent = ex; }
  busy($("#profValBtn"), false, "Validate against log");
};

/* ---- Replay ---- */
$("#repExBtn").onclick = async () => {
  const r = await fetch("/api/replay/example"); const j = await r.json();
  $("#repJson").value = JSON.stringify(j.profile, null, 2);
};
$("#repBtn").onclick = async () => {
  const err = $("#repErr"); err.textContent = ""; $("#repOut").innerHTML = "";
  if (needLog(err)) return;
  let profile;
  try { profile = JSON.parse($("#repJson").value); }
  catch (e) { err.textContent = "Decode profile JSON is invalid: " + e.message; return; }
  busy($("#repBtn"), true);
  try {
    const j = await postJSON("/api/replay", {
      token: TOKEN, profile, code: $("#repCode").value.trim(), immobilizer: $("#repImmo").checked,
    });
    if (j.error) { err.textContent = j.error; busy($("#repBtn"), false, "Run replay"); return; }
    const s = j.summary;
    let h = "<h2 class='sub'>Decoded state</h2><div class='cards'>" +
      card("ticks", s.ticks) + card("span", s.span_s + " s") +
      card("speed", (s.speed_min == null ? "—" : s.speed_min + "–" + s.speed_max + " km/h")) +
      card("gears", (s.gears.length ? s.gears.join(" ") : "—")) +
      card("ignition-on ticks", s.ignition_on_ticks) + "</div>";
    if (s.buttons.length) {
      h += "<table><tr><th>t (s)</th><th>button decoded</th></tr>";
      s.buttons.forEach((b) => { h += "<tr><td class='mono'>" + b.t + "</td><td>" + esc(b.button) + "</td></tr>"; });
      h += "</table>";
    } else {
      h += "<p class='hint'>No button symbols decoded (profile signals may be absent from this log).</p>";
    }
    if (j.immobilizer) {
      const im = j.immobilizer;
      h += "<h2 class='sub'>Immobilizer simulation <span class='sim'>READ-ONLY</span></h2><div class='cards'>" +
        card("final state", im.final_state || "—") +
        card("inhibit", im.inhibit_asserted ? "<span class='warn'>ASSERTED</span>" : "<span class='ok'>released</span>") +
        card("code", (im.code.length ? im.code.join(",") : "—")) + "</div>";
      h += stateTable("State transitions", ["t", "state", "info"], im.transitions, (x) =>
        ["<span class='mono'>" + x.t + "</span>", "<b>" + esc(x.state) + "</b>", esc(x.info)]);
      h += stateTable("Actuator actions (simulated)", ["t", "action", "method"], im.actions, (x) =>
        ["<span class='mono'>" + x.t + "</span>", (x.action === "ASSERT" ? "<span class='warn'>ASSERT</span>" : "<span class='ok'>RELEASE</span>"), esc(x.method)]);
    }
    $("#repOut").innerHTML = h;
  } catch (ex) { err.textContent = ex; }
  busy($("#repBtn"), false, "Run replay");
};

function renderMeta() {
  const m = META || {};
  const title = [m.year, m.make, m.model].filter(Boolean).join(" ");
  const action = m.action ? esc(m.action) : "";
  const labels = (m.press_labels && m.press_labels.length)
    ? m.press_labels.map(esc).join(", ") : "";
  const lm = $("#logMeta"), fm = $("#findMeta");
  if (!(title || action || labels || m.presses)) { lm.hidden = true; fm.hidden = true; return; }
  let chips = "";
  if (title) chips += "<span class='mchip'>" + esc(title) + "</span>";
  if (action) chips += "<span class='mchip hot'>" + action + "</span>";
  if (m.presses) chips += "<span class='mchip'>" + m.presses + " press entr" + (m.presses === 1 ? "y" : "ies") + "</span>";
  if (labels) chips += "<span class='mchip'>“" + labels + "”</span>";
  lm.hidden = false;
  lm.innerHTML = "<div class='mk'>Capture</div><div class='mchips'>" + chips + "</div>";
  fm.hidden = false;
  fm.innerHTML = "Hunting: <b>" + (action || "(unlabelled)") + "</b>" +
    (title ? " on " + esc(title) : "") +
    (labels ? " · you did “" + labels + "”" : "");
}

function card(k, v) { return "<div class='card'><div class='k'>" + esc(k) + "</div><div class='v'>" + v + "</div></div>"; }
function stateTable(title, cols, rows, fmt) {
  if (!rows || !rows.length) return "<h2 class='sub'>" + esc(title) + "</h2><p class='hint'>none</p>";
  let h = "<h2 class='sub'>" + esc(title) + "</h2><table><tr>" + cols.map((c) => "<th>" + c + "</th>").join("") + "</tr>";
  rows.forEach((r) => { h += "<tr>" + fmt(r).map((c) => "<td>" + c + "</td>").join("") + "</tr>"; });
  return h + "</table>";
}
