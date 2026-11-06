# CANSentinel

> CAN-bus anti-theft immobilizer toolkit: find button/lock signals, crack rolling counters and checksums, certify a whole vehicle platform from one car, and run a standstill-gated, fail-open immobilizer. Read-only analysis + an on-device reference state machine. Every tool runs offline with `--selftest`.

## Quick start — web UI (no command line)

```bash
git clone https://github.com/natiHayon/CANSentinel.git
cd CANSentinel
python3 -m venv .venv && . .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python app.py                                      # open http://127.0.0.1:5000
```

The UI (`app.py`, Flask, runs only on your machine) lets you **load a CAN log,
say how many times you did an action, and get the button/lock ID** — and a second
tab cracks the **rolling counter + checksum** of any message. Thin wrapper over
the same functions the CLI tools use, so results match exactly. Read-only.

## Command line

This folder adds `find_events.py`: a read-only tool that finds the CAN
**ID / byte / bit** behind a **discrete action** — a button press, a lock/unlock,
a stalk or switch, a gear position. It is the thing you said is painful ("very
hard to find the IDs for the buttons and for lock/unlock"). It never opens a CAN
interface and never transmits — it only analyzes a log you already recorded.

Read the strategy section at the bottom before you pour more months into the
hardware. There's a wall on the control side that better reverse-engineering does
not get you past, and a safety risk that is the real threat to the business.

---

## 1. Why this is a separate tool from the CSS Electronics skills

The CSS skills (the repo you shared) are excellent, but they're tuned for
**continuous** signals — speed, RPM — where you *sweep* a value and correlate a
smooth ramp. Buttons and locks are the opposite:

| Control | How it's encoded on CAN | Right way to find it |
|---|---|---|
| Momentary button (window, horn, start) | a **bit that pulses** 0→1→0 while held | count rising edges |
| Lock / gear / switch | a **bit that toggles** between stable states | count state changes |
| Steering-wheel / console cluster | a **"which button" enum byte** (idle value + one code per button) | find the byte that leaves idle in bursts |

That enum-byte case is the one people miss and then burn days on: a whole bank of
steering-wheel buttons is usually **one byte on one message ID**, not twelve
separate bits. `find_events.py` checks all three encodings at once.

The trick that makes it robust: **repetition instead of timing.** Trigger the
action a known number of times; the tool finds the field that changed *exactly
that many times* while everything else on the bus was quiet. No precise
timestamps, no fighting your own reaction lag.

## 2. Install

Nothing to install beyond Python 3 + numpy (already present here). It reads logs
in three formats, auto-detected:

- **webCAN / CANsub CSV** (what the CSS tools and your CANsub produce)
- **candump** (Linux SocketCAN: `(1700000000.123) can0 123#DEADBEEF`)
- **generic** `epoch<sep>id_hex<sep>data_hex` (`;`, `,` or whitespace)

So it works with your current reader regardless of what it is — if it can dump
timestamp + ID + data bytes, this reads it.

## 3. Prove it works with zero hardware

```bash
python3 find_events.py --selftest
```

This synthesizes a bus with background counters + noise and three planted
controls (a pulse button, a lock toggle, a steering-wheel enum byte) and confirms
the finder recovers all three. Run it first so you trust the output.

## 4. Use it on a real car

**The workflow (per control):**

1. Start your reader logging.
2. Trigger ONE action a known number of times, roughly evenly — e.g. press
   volume-up **8 times**, or lock/unlock **10 times**. Leave a second or two
   between each. Do nothing else.
3. Stop logging. Run:

```bash
# a button you pressed 8 times:
python3 find_events.py --trace volup.csv --events 8

# lock/unlock toggled 10 times:
python3 find_events.py --trace lock.csv --events 10 --mode toggle

# optional: a quiet "did nothing" log to gate out unrelated traffic:
python3 find_events.py --trace lock.csv --baseline quiet.csv --events 10
```

The output ranks candidates with the exact ID, byte, bit (and the **cantools
start-bit** so you can drop it straight into a DBC), plus the decoded button
codes for an enum byte.

**If you logged press timestamps** (some rigs can), use timing confirmation with
an automatic ±2 s lag search:

```bash
python3 find_events.py --trace run.csv --event-times presses.txt --tol 0.4
```

`presses.txt` is one epoch timestamp per line, or a CSS-Electronics sidecar CSV.

**Options that matter:**
- `--mode auto|pulse|toggle|value` — restrict the hypothesis if you know it.
- `--baseline FILE` — a quiet recording; fields active here but idle in baseline
  are the only candidates. This is your strongest noise filter.
- `--idle-frac 0.2` — raise it if the control stays active a long time.
- `--only-ids 0x200,0x5C1` / `--exclude-ids ...` — narrow the search.
- `--help` for everything.

**Confirming a hit (don't skip this):** re-record while triggering the action and
watch that one byte/bit change. A candidate that only looks right in one log is a
coincidence until it repeats.

## 5a. The coverage engine: `vehicle_profile.py`

Finding IDs on one car is step one. The thing that actually **maximizes coverage
per unit of reverse-engineering effort** is not re-RE'ing every car — it's
exploiting that carmakers reuse one CAN architecture across many models/years (a
*platform*). RE one car on a platform, write it as a profile, then **auto-certify
every sibling model from a short log** instead of redoing the work.

```bash
python3 vehicle_profile.py --example > my-platform.json   # scaffold
# ...fill in the real geometry from find_events.py + the CSS continuous tools...
python3 vehicle_profile.py --validate newcar.csv --profile my-platform.json
python3 vehicle_profile.py --selftest                     # no hardware
```

A profile captures the three things the immobilizer needs, by role:

- **gate** (safety-critical): speed, gear = Park/Neutral, engine state — the
  signals that prove the car is **stopped and off** so the device may only act at
  standstill. Weighted heaviest. **If the gates don't validate, the verdict is
  FAIL and the car must not be trusted to a data inhibit** — it falls back to the
  physical interrupt. That fail-safe default is deliberate.
- **inhibit-target**: the start/crank-enable field the data profile acts on (only
  if a safe data inhibit is validated for the platform; else `method: physical`).
- **feature**: lock/unlock, buttons — for alarm / remote features.

The validator checks each signal is present at the right cycle time and decodes
to a physically plausible value, then prints PASS / PARTIAL / FAIL + a match
score. PARTIAL usually means a trim/year variant (gates fine, a feature differs);
a low score means it's a different platform. This is what turns "RE every car"
into "RE one platform, validate the rest in minutes."

## 5b. The on-device brain: `immobilizer.py`

The control state machine that sits on top of a validated profile. Reference
implementation + simulator (pure stdlib) — port the logic to your MCU, keep the
actuator wiring.

```bash
python3 immobilizer.py --selftest   # 6 scenarios, no hardware
```

- **Boots ARMED** (car blocked). Owner clears it by entering a **code on the
  car's own buttons** — the button events come straight from `find_events.py`
  detections, so there's no extra keypad to hide.
- **Asserts the inhibit only at a confirmed, sustained standstill** (speed ≈ 0,
  gear P/N when known); **releases on any motion, doubt, stale bus, or fault.**
  There is no code path that disables a moving car — that's structural.
- **Actuator is pluggable:** the profile supplies the inhibit *method* — a data
  descriptor (your ~80%) or a physical output (the ~20% analog). No car-specific
  disable payload lives in this file; it's profile data from your own RE.

Self-test scenarios (all pass): thief held at standstill; owner's button code
disarms + releases; armed-while-moving never asserts; moving-then-stopped asserts
only after the stop is confirmed; stale bus → fail-open + FAULT; wrong codes stay
locked.

The files form the pipeline: **`find_events.py`** (find the IDs) →
**`checksum_counter.py`** (crack the counter/CRC so transmitted frames are
accepted) → **`vehicle_profile.py`** (certify a platform) → **`live_decode.py`**
(raw frames → decoded state, replay logs or a live reader) → **`immobilizer.py`**
(run it safely on-device).

## 5c. The transmit-side keystone: `checksum_counter.py`

When the immobilizer *sends* a data-inhibit frame, many protected ECUs reject it
unless the **rolling counter** holds the next expected value and the **checksum /
CRC** is computed by that ECU's exact algorithm. This tool reverse-engineers both
from a recorded log so your transmitted frames get accepted.

```bash
python3 checksum_counter.py --trace log.csv --id 0x1A0
python3 checksum_counter.py --trace log.csv          # scan every ID
python3 checksum_counter.py --selftest               # no hardware
```

It finds:
- **Counter:** a byte or 4-bit nibble that increments +1 per frame (reported with
  location and modulus).
- **Checksum:** tried in order — `sum8` (`(sum + k) & 0xFF`), `xor8`, then a
  curated set of automotive **CRC-8** variants (AUTOSAR 0x2F, SAE-J1850 0x1D,
  CRC-8 0x07, Maxim 0x31, CDMA2000 0x9B, DVB-S2 0xD5, …), each over the other
  bytes.

It prints the **exact formula** and, crucially, **verifies it reproduces the real
checksum on 100% of frames** before claiming CONFIRMED. A confirmed recipe goes
into the profile's inhibit method so the device computes a valid counter + CRC for
every frame it sends. If nothing hits 100%, it says so and points at the usual
culprits (an AUTOSAR E2E "data id" constant, a byte subset, or the CAN ID folded
in) rather than pretending.

Note: this is the opposite stage from `find_events.py`. There, counters/checksums
are filtered out as noise so they don't drown a button. Here they ARE the target,
because transmitting needs them right. Both stages read the same untouched log.

## 5d. The connector: `live_decode.py`

Turns raw CAN frames into decoded `VehicleState` ticks and runs them through the
immobilizer. One decoder, two front-ends:

- **Replay a recorded log** (now, and to validate on logs you already have) —
  read-only, nothing transmitted.
- **A live reader** (your hardware) — implement one generator that yields
  `(t, can_id, data)` and feed the same decoder. A python-can skeleton is in
  `frames_from_python_can()` (commented; listen-only on a vehicle).

```bash
# decode a log into a readable timeline (speed/gear/engine/ignition + button edges):
python3 live_decode.py --replay mylog.csv --profile p.json

# replay the WHOLE stack and watch what the immobilizer would have done:
python3 live_decode.py --replay mylog.csv --profile p.json \
    --immobilizer --code LOCK,LOCK,VOLUP,UNLOCK

python3 live_decode.py --selftest    # no hardware
```

The profile's signals gain a `maps_to` hint (`speed` / `gear` / `engine` /
`ignition` / `button`) so the decoder knows which VehicleState field each drives;
button signals carry a code→symbol map (value byte) or a symbol (pulse bit) so the
code-entry sequence is driven by the car's own buttons. If `maps_to` is omitted
it's inferred from the signal name.

This is how you validate the entire system against a log you already recorded —
tick by tick — before the device ever touches a bus.

## 5. How it avoids the classic traps

- **Rolling counters / checksums are rejected wholesale** (monotonic increment,
  or high-entropy bytes), so a counter's slow high bit can't masquerade as a
  "toggle" — that was the first bug the self-test caught.
- **Pulse vs toggle are counted differently** (rising edges vs state changes), so
  a lock toggled N times scores on N, not on half of N.
- **Enum bytes are rewarded for showing structured codes** — a byte that returns
  a clean set of button codes is ranked as the control cluster it is.

---

# The straight read on the immobilizer project

You asked me to be a partner, not a yes-man. Here it is.

## The uncomfortable truth, first

**[Likely] You have been treating two very different problems as one.**

1. **Reading the bus — buttons, locks, state.** Easy, safe, and now largely
   solved by the tool above. Passive, read-only, no risk. Keep going here; it also
   gives you proximity/remote/alarm features for free later (a lock/unlock frame
   is just another discrete signal to detect, then act on).

2. **Blocking the car over the data bus — your "non-negotiable 80%."** This is a
   different engineering discipline (you must *transmit*, with safety guarantees),
   and on newer vehicles it hits a wall that no amount of better ID-finding gets
   you past.

The thing you're frustrated by (finding IDs) is the *easy* half. Solving it
better does not move the hard half.

## The wall: message authentication and the central gateway

**[Certain] Reading any frame does not mean you can forge a useful one.** Finding
the "engine start enable" or "no-start" message tells you what it looks like — it
does not let you inject it on cars that authenticate messages.

- **[Certain] Many 2018+ vehicles (and most 2020+ premium / EV platforms) run a
  central gateway that segments the CAN buses.** The OBD2 port you tap often sees
  only a filtered subset; the powertrain/immobilizer traffic lives on an internal
  segment you can't reach from the port, and the gateway won't forward your
  injected frames onto it.
- **[Certain] A growing share implement SecOC (AUTOSAR Secure Onboard
  Communication): safety/control frames carry a cryptographic MAC + freshness
  counter.** A replayed or spoofed control frame is rejected. You cannot forge
  these without the per-vehicle key, which you don't have and can't extract at
  scale.

**[Likely] Net effect: a pure-CAN "disable the car" that is reliable AND safe
across 80% of a *modern* fleet is not achievable.** It's very achievable on older
/ simpler vehicles (roughly pre-2018, and plenty of economy cars after). So the
honest version of your target is **"80% of the cars I actually sell into,"** and
that depends entirely on which model years and segments you're targeting. That's
the single most important thing to pin down — it decides whether the premise
holds.

## The real risk is not a bricked account — it's a crash

**[Certain] RED risk. A device that disables a car over the data bus can, if it
misfires, disable a car that is moving.** False trigger, bus error storm, firmware
hang, a spoofed frame the ECU interprets unexpectedly — any of these while the
car is in motion is a potential injury/death and a product-liability event that
ends the company. This is categorically different from the account/platform risks
in your playbook. It cannot be "tested small and accepted."

This is non-negotiable from the engineering side, and it happens to also be the
right design constraint:

- **The immobilizer must engage ONLY when the vehicle is confirmed stationary and
  off** (you already read the bus — gate on speed = 0, engine off, park/neutral,
  for a sustained window).
- **It must FAIL OPEN.** Any fault, watchdog timeout, or loss of confidence → the
  device stops acting and the car is drivable. Never the reverse.
- **It must never act on a bus it hasn't positively identified** as a vehicle it
  has a validated profile for.

A crude "flood the bus so nothing starts" (bus-off / DoS) approach violates all of
this — it's indiscriminate, it can trip while driving, and it sets fault codes.
Don't build the product on it.

## What I'd do instead (protecting your goal, changing the method)

Your goal is **reliable, hidden, mostly-digital immobilization + an upsell path to
alarm/remote.** Here's the path that actually gets there:

1. **[recommend] Keep immobilization logic as a safe, stationary-only "start
   inhibit," not a running-car kill.** Prevent *starting* (or prevent *drive
   engagement* at standstill). This removes the moving-car catastrophe by design
   and is also what insurers and regulators expect from an aftermarket
   immobilizer. It still meets your product promise: car won't start without the
   code.

2. **Build a per-vehicle profile library, powered by exactly the read tooling
   above.** For each make/model/year you support: the start-enable / crank / drive
   signals, the standstill confirmation signals, and the lock/unlock frames. Ship
   profiles; the device picks the right one on install. This is your moat and it
   compounds — every car you reverse-engineer makes the product cover more fleet.
   The button/lock finder here is the front end of that pipeline.

3. **Accept a hybrid where the data bus can't safely win.** You already said 20%
   relay is fine. The honest split isn't 80/20 by *principle* — it's "data where
   the vehicle profile makes a data-based start-inhibit safe and reliable; a
   minimal physical interrupt where it doesn't (gateway/SecOC cars)." Keep the
   device finger-sized; the physical option is a single low-side start-circuit
   interrupt, not a bank of relays.

4. **Sequence the market: older/simpler vehicles first.** That's where pure-CAN
   works today, where the theft rate is highest, and where you get revenue and a
   profile library before you fight the hard platforms. Use that cash and data to
   decide whether the newer-fleet fight is worth it.

## What this session delivered vs. what's next

- **Delivered:** `find_events.py` — the read-side that unblocks "finding IDs for
  buttons and lock/unlock," format-agnostic to your existing reader, with a
  self-test.
- **Not delivered, deliberately:** a turnkey "disable any car" injector. A generic
  vehicle-disabling transmitter is unsafe without the per-vehicle safety
  interlocks and validated profiles above, and I'm not going to hand you something
  that can brick a moving car. The *control* side should be built per-validated-
  profile, stationary-gated, fail-open — and I'll help you build that once we fix
  the two decisions it depends on (see below).

## Coverage-maximizing sequencing (the plan you picked)

You chose "whatever maximizes coverage." Here's the sequence that covers the most
real-world cars per unit of RE effort:

1. **Rank platforms by (installed base in your market × theft rate), RE the top
   ones first.** One VW MQB profile ≈ dozens of VAG models; one Toyota TNGA
   profile ≈ most of the modern Toyota line. Spend RE effort where it buys the
   most cars.
2. **Profile per platform** with `find_events.py` (discrete: locks/buttons/gear)
   + the CSS continuous tooling (speed/RPM gates). Write each as a
   `vehicle_profile.py` profile.
3. **Auto-certify siblings** from short logs with the validator — don't re-RE
   them. Your coverage grows far faster than your RE hours.
4. **Ship from day one with the physical interrupt as the floor** (100% coverage,
   revenue now), and **convert each certified platform to the data inhibit** as
   its profile validates. The "80% over data" is then a metric that climbs with
   your library, not a launch gate.
5. **Sequence older/simpler platforms first** (open CAN, no gateway/SecOC, highest
   theft) to build cash + a profile library before fighting newer platforms.

## One thing still open — tell me this

**Your current reader / log format:** what hardware are you capturing with, and
what does one row of its log look like? `find_events.py` and `vehicle_profile.py`
already auto-detect webCAN/candump/generic, but knowing your exact format lets me
confirm the plug-in is clean and tailor the capture workflow.

Next build (ready when you are): a **platform-ranking worksheet** for your target
market (where to spend RE effort first), and a thin **live-decode layer** that
turns raw CAN frames into the `VehicleState` ticks `immobilizer.py` consumes,
wired to your specific reader once I know its log format.
