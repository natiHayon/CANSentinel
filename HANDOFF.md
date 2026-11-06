# HANDOFF — CAN anti-theft immobilizer toolkit

Status as of this handoff. Read this first if you're picking the project up (you,
a developer, or a future session).

## What this is

A read-and-analyze toolkit plus an on-device control state machine for a CAN-bus
anti-theft immobilizer. It reverse-engineers the signals an installed device
needs (buttons, locks, standstill/speed/gear, counters/checksums) and runs a
safety-gated immobilizer on top of a per-vehicle profile.

All code is in ``. Pure Python 3 + numpy. Every tool runs offline with
`--selftest` (no hardware, no bus access).

## The pipeline (5 tools)

```
find_events.py  →  checksum_counter.py  →  vehicle_profile.py  →  live_decode.py  →  immobilizer.py
 find the IDs       crack counter/CRC so     certify a whole        raw frames →        run it safely
 (buttons, locks,   transmitted frames       platform from one      decoded state       on-device
  switches, gear)   are ECU-accepted         car's RE               (replay or live)    (standstill, fail-open)
```

| File | Does | Key command |
|---|---|---|
| `find_events.py` | finds CAN ID/byte/bit for a discrete action (button pulse, lock toggle, enum byte) via repetition | `--trace log.csv --events N` |
| `checksum_counter.py` | finds rolling counter + cracks checksum (sum8/xor8/CRC-8 incl. AUTOSAR, J1850) | `--trace log.csv --id 0x1A0` |
| `vehicle_profile.py` | defines a platform profile; auto-certifies sibling models from a short log | `--validate log.csv --profile p.json` |
| `live_decode.py` | raw frames → `VehicleState`; replay a log or wire a live reader; can run the full immobilizer | `--replay log.csv --profile p.json --immobilizer --code ...` |
| `immobilizer.py` | the control state machine: code-entry via the car's buttons, standstill-gated inhibit, fail-open | `--selftest` |

Verify everything at once:
```bash
for f in find_events checksum_counter vehicle_profile immobilizer live_decode; do
  echo "== $f =="; python3 $f.py --selftest; done
```

## SAFETY INVARIANTS — do not remove these

These are structural in `immobilizer.py` and are what make the product insurable.
Anyone modifying the control logic must preserve them:

1. **Standstill-only.** The inhibit is asserted ONLY at a positively-confirmed,
   sustained standstill (speed ≈ 0, gear P/N when known). No code path disables a
   moving vehicle.
2. **Fail-open.** Stale/unhealthy bus, watchdog timeout, or any inability to
   confirm standstill → release the inhibit. When in doubt, the car drives.
3. **No kill-while-moving mode exists.** "Block start" and "block drive-away" are
   both standstill actions caught at the moment of an unauthorized start.
4. The per-vehicle disable payload is **profile data**, never hardcoded in the
   tools.

## Design decisions (the strategy)

- **Coverage is won by platform reuse,** not per-car grinding: RE one car on a
  platform (VW MQB, Toyota TNGA, GM Global, …), profile it, auto-certify the
  siblings with `vehicle_profile.py`.
- **Ship with a physical start-circuit interrupt as the floor (100% coverage),
  convert each platform to the data inhibit as its profile validates.** The
  "~80% over data" is a metric that climbs with the profile library, not a launch
  gate. The ~20% hard cases (gateway/SecOC) stay on the physical interrupt.

## Open items / next builds

1. **Reader/log format.** Tools auto-detect webCAN, candump, and generic
   `epoch;id;data`. Confirm the project's actual reader output plugs in cleanly;
   if not, extend `find_events.load_log`.
2. **Fill real profiles.** `vehicle_profile.py --example` scaffolds one; fill the
   geometry from `find_events.py` + the CSS continuous tooling on a real car per
   target platform.
3. **Checksum edge cases.** `checksum_counter.py` covers sum8/xor8/common CRC-8.
   If a car's checksum needs an AUTOSAR E2E data-id constant, a byte subset, or
   the CAN ID folded in, extend the search there (it already flags when nothing
   hits 100%).
4. **Wire the live reader.** `live_decode.frames_from_python_can()` is a skeleton;
   connect it to the real interface (listen-only on a vehicle).
5. **On-device port.** `immobilizer.py` is a reference implementation; port the
   state machine to the target MCU, keeping the safety invariants above.

## Repo / branch / how to get the code

- Branch: `claude/can-bus-id-detection-gh1m9r` (off `main`).
- Commits add only ``; nothing else in the repo is touched.

**If pushing from a cloud session was blocked** (no GitHub write access for the
automation), the commits exist only in that session until either (a) GitHub
access is granted and they're pushed, or (b) they're transferred via a git
bundle:
```bash
# on the machine that has the bundle + a clone of this repo:
git fetch /path/to/hrm-can-tools.bundle \
    claude/can-bus-id-detection-gh1m9r:claude/can-bus-id-detection-gh1m9r
git checkout claude/can-bus-id-detection-gh1m9r
```
To grant push access instead: connect GitHub at https://claude.ai/connect-github
and install the Claude GitHub App on the repo
(https://github.com/apps/claude/installations/select_target).
