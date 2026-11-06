# Agent handoff — CANSentinel

You are picking up the **CANSentinel** project on the user's PC. This is the
brief; read it fully before acting. The repo is self-contained — read the code
and `HANDOFF.md` for anything here that needs more depth.

## What the project is

A CAN-bus **anti-theft immobilizer** toolkit for the user's own vehicles /
product. Two sides:

1. **Analysis (read-only):** reverse-engineer a car's CAN bus from recorded logs
   — find the ID/byte/bit for buttons and locks, and crack rolling
   counters/checksums so transmitted frames are accepted.
2. **Control (on-device reference):** a state machine that immobilizes the car,
   gated so it is safe.

Pure Python 3 + numpy (+ Flask for the UI). Every tool runs offline with
`--selftest` — no hardware, no live bus.

## Get it running (on the user's PC)

```bash
git clone https://github.com/natiHayon/CANSentinel.git
cd CANSentinel
python3 -m venv .venv && . .venv/bin/activate     # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python app.py                                     # web UI at http://127.0.0.1:5000
# sanity-check the whole toolkit:
for f in find_events checksum_counter vehicle_profile immobilizer live_decode; do python $f.py --selftest; done
```
All five self-tests should print `ALL PASSED`.

## The files (the pipeline)

```
find_events.py  →  checksum_counter.py  →  vehicle_profile.py  →  live_decode.py  →  immobilizer.py
 find the IDs       crack counter/CRC        certify a platform     raw frames →         run it safely
 (button/lock/       so transmitted           from one car's RE      decoded state        on-device
  switch/gear)       frames are accepted                             (replay or live)
app.py = Flask web UI wrapping find_events + checksum_counter.
```

| File | What it does | Entry point |
|---|---|---|
| `find_events.py` | finds CAN ID/byte/bit for a discrete action via repetition (`--events N`); handles pulse bits, toggle bits, enum bytes | `analyze(...)` / CLI `--trace --events` |
| `checksum_counter.py` | finds rolling counter; cracks checksum (sum8/xor8/CRC-8 incl. AUTOSAR, J1850); verifies on 100% of frames | `detect_counters`, `detect_checksum` |
| `vehicle_profile.py` | defines a platform profile (JSON); validates/certifies a model from a short log | `validate(...)` / CLI `--validate --profile` |
| `live_decode.py` | decodes profile signals from frames into `VehicleState`; replay a log or wire a live reader; can run the immobilizer | `replay_states`, `run_immobilizer` |
| `immobilizer.py` | control state machine: code via car buttons, standstill-gated inhibit, fail-open | `Immobilizer`, `--selftest` |

Log formats auto-detected: webCAN/CANsub CSV, candump, generic `epoch;id;data`.

## SAFETY INVARIANTS — never remove these from the control logic

These live in `immobilizer.py` and are what make the product viable/insurable:

1. **Standstill-only.** The inhibit asserts ONLY at a positively-confirmed,
   sustained standstill (speed ≈ 0, gear P/N when known).
2. **Fail-open.** Stale/unhealthy bus, watchdog timeout, or any inability to
   confirm standstill → release the inhibit. When in doubt, the car drives.
3. **No kill-while-moving mode exists.** Block-start and block-drive-away are
   both standstill actions.
4. The per-vehicle disable payload is **profile data**, never hardcoded in the
   tools. Keep the analysis tools read-only.

## Current state (verified)

- All five CLI tools pass `--selftest`.
- The Flask UI was smoke-tested: load a log → "Find a control" returns the right
  ID; "Counter & checksum" cracks counter + CRC.
- No real vehicle logs analyzed yet — the user has recordings to feed in.

## Next tasks (in rough priority)

1. **Run the user's real logs** through the UI / `find_events.py`. For each log,
   get from the user: what action they did and how many times. Find the button/
   lock IDs; save confirmed findings.
2. **Confirm the user's reader/log format** plugs into `find_events.load_log`
   cleanly; extend the loader if their format differs.
3. **Build real vehicle profiles.** `python vehicle_profile.py --example`
   scaffolds one; fill geometry from `find_events.py` (discrete) + a
   continuous-signal tool (speed/gear gates). One profile per platform.
4. **Checksum edge cases:** if a car's checksum needs an AUTOSAR E2E data-id
   constant, a byte subset, or the CAN ID folded in, extend the search in
   `checksum_counter.py` (it already flags when nothing hits 100%).
5. **Wire the live reader:** `live_decode.frames_from_python_can()` is a skeleton
   — connect it to the user's interface (listen-only on a vehicle).
6. **UI:** optionally add profile-validation and immobilizer-replay tabs.
7. **On-device port:** `immobilizer.py` is a reference; port to the target MCU,
   preserving the safety invariants above.

## Constraints / scope for you (the agent)

- Analysis tools stay **read-only** (no CAN TX).
- Do **not** build a generic "disable any car" injector. Any inhibit is
  per-validated-profile, standstill-gated, fail-open; disable payloads are
  profile data from the user's own RE on their own vehicles.
- Keep dependencies minimal (numpy, flask). Everything should keep running via
  `--selftest` with no hardware.

## Good first move

Ask the user for one recorded log and what they did during it, run it through
`find_events.py` (or the UI), and confirm you recover a plausible button/lock ID
before moving on to profiles.
