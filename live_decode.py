#!/usr/bin/env python3
"""
live_decode.py - the connector: turn raw CAN frames into decoded VehicleState
ticks and (optionally) run them through the immobilizer state machine.

It decodes the signals a vehicle profile declares (speed, gear, engine,
ignition, buttons) from each frame, carries the latest value forward, and emits
a VehicleState stream. Two front-ends share one interface:

  * REPLAY a recorded log  (what you use now, and to validate on logs you already
    have) - read-only, nothing is transmitted.
  * a LIVE reader          (your hardware) - implement one generator that yields
    (t, can_id, data) and feed it to the same decoder. A python-can skeleton is
    in frames_from_python_can() below (commented; enable when wiring hardware).

So you can replay a log you already recorded straight through find_events ->
profile -> THIS -> immobilizer and SEE, tick by tick, what the device would have
done - before you ever let it touch a bus.

Profile signal conventions (added on top of vehicle_profile.py)
---------------------------------------------------------------
Each signal may carry `maps_to` to say what VehicleState field it drives:
  maps_to: "speed"     continuous, -> speed_kmh (uses scale/offset)
  maps_to: "engine"    discrete bit (on_value, default 1) OR continuous rpm with
                       {"rpm_running_above": N} -> engine_running
  maps_to: "ignition"  discrete bit (on_value, default 1) -> ignition_on
  maps_to: "gear"      value byte + {"gear_map": {"0":"P",...}}  OR a park bit
                       {"bit": n, "park_value": 1} -> gear ('P'/'N'/... or 'D')
  maps_to: "button"    value byte + {"codes": {"1":"VOLUP",...}} OR a pulse bit
                       {"bit": n, "symbol": "LOCK"} -> emits a button symbol on
                       each idle->active edge (fed to the immobilizer's code entry)
If `maps_to` is absent it is inferred from the signal name (vehicle_speed->speed,
gear*->gear, ignition*->ignition, engine*/rpm*->engine, lock*/button*->button).

Usage
-----
  python live_decode.py --replay log.csv --profile p.json            # decode timeline
  python live_decode.py --replay log.csv --profile p.json --immobilizer --code LOCK,LOCK,VOLUP,UNLOCK
  python live_decode.py --selftest                                   # no hardware
"""
from __future__ import annotations

import argparse
import json
import sys

from find_events import load_log, Frame
from immobilizer import (
    Immobilizer, ImmobilizerConfig, VehicleState, LoggingActuator, State,
)


# ---------------------------------------------------------------------------
# Per-frame field decoding
# ---------------------------------------------------------------------------

def _le(data: bytes, start_bit: int, length: int) -> int:
    v = int.from_bytes(data, "little")
    return (v >> start_bit) & ((1 << length) - 1)


def _be(data: bytes, byte: int, width: int) -> int:
    v = int.from_bytes(data, "big")
    shift = 8 * (len(data) - byte - width)
    return (v >> shift) & ((1 << (8 * width)) - 1)


def _raw_continuous(data: bytes, spec: dict) -> float | None:
    try:
        if spec.get("endian", "little") == "big":
            if "byte" in spec and "width" in spec:
                raw = _be(data, spec["byte"], spec["width"])
            else:
                raw = _be(data, spec["start_bit"] // 8, spec["length"] // 8)
        else:
            if "start_bit" in spec and "length" in spec:
                raw = _le(data, spec["start_bit"], spec["length"])
            else:
                raw = _le(data, spec["byte"] * 8, spec["width"] * 8)
    except (KeyError, IndexError):
        return None
    return raw * float(spec.get("scale", 1.0)) + float(spec.get("offset", 0.0))


def _bit(data: bytes, byte: int, bit: int) -> int | None:
    if byte >= len(data):
        return None
    return (data[byte] >> bit) & 1


def _infer_role(name: str, spec: dict) -> str | None:
    if "maps_to" in spec:
        return spec["maps_to"]
    n = name.lower()
    if "speed" in n:
        return "speed"
    if n.startswith("gear") or "gear" in n:
        return "gear"
    if "ignition" in n or n.startswith("ign"):
        return "ignition"
    if "engine" in n or "rpm" in n:
        return "engine"
    if "lock" in n or "button" in n or "wheel" in n:
        return "button"
    return None


# ---------------------------------------------------------------------------
# Profile-driven decoder: maintains latest state, detects button edges
# ---------------------------------------------------------------------------

class ProfileDecoder:
    def __init__(self, profile: dict):
        self.sig = profile.get("signals", {})
        # map CAN id -> list of (name, spec, role)
        self.by_id: dict[int, list[tuple[str, dict, str]]] = {}
        for name, spec in self.sig.items():
            role = _infer_role(name, spec)
            if role is None:
                continue
            cid = int(str(spec["id"]), 16)
            self.by_id.setdefault(cid, []).append((name, spec, role))
        # latest decoded fields
        self.speed = None
        self.gear = None
        self.engine = None
        self.ignition = False
        # per-button last-value for edge detection
        self._btn_prev: dict[str, int] = {}

    def feed(self, fr: Frame) -> str | None:
        """Update state from one frame; return a button symbol if an edge fired."""
        emitted = None
        for name, spec, role in self.by_id.get(fr.can_id, []):
            if role == "speed":
                v = _raw_continuous(fr.data, spec)
                if v is not None:
                    self.speed = v
            elif role == "ignition":
                b = _bit(fr.data, spec.get("byte", 0), spec.get("bit", 0))
                if b is not None:
                    self.ignition = (b == spec.get("on_value", 1))
            elif role == "engine":
                if "rpm_running_above" in spec:
                    v = _raw_continuous(fr.data, spec)
                    if v is not None:
                        self.engine = v > spec["rpm_running_above"]
                else:
                    b = _bit(fr.data, spec.get("byte", 0), spec.get("bit", 0))
                    if b is not None:
                        self.engine = (b == spec.get("on_value", 1))
            elif role == "gear":
                if "gear_map" in spec:
                    byte = spec.get("byte", 0)
                    if byte < len(fr.data):
                        self.gear = spec["gear_map"].get(str(fr.data[byte]), self.gear)
                elif "bit" in spec:
                    b = _bit(fr.data, spec.get("byte", 0), spec["bit"])
                    if b is not None:
                        self.gear = "P" if b == spec.get("park_value", 1) else "D"
            elif role == "button":
                emitted = self._button_edge(name, spec, fr.data) or emitted
        return emitted

    def _button_edge(self, name: str, spec: dict, data: bytes) -> str | None:
        if "codes" in spec:  # value-byte cluster
            byte = spec.get("byte", 0)
            if byte >= len(data):
                return None
            val = data[byte]
            prev = self._btn_prev.get(name, spec.get("idle", 0))
            self._btn_prev[name] = val
            idle = spec.get("idle", 0)
            if val != idle and prev == idle:        # idle -> code edge
                return spec["codes"].get(str(val))
            return None
        if "bit" in spec:  # pulse bit
            b = _bit(data, spec.get("byte", 0), spec["bit"])
            if b is None:
                return None
            prev = self._btn_prev.get(name, spec.get("idle", 0))
            self._btn_prev[name] = b
            active = 1 - spec.get("idle", 0)
            if b == active and prev != active:       # idle -> active edge
                return spec.get("symbol", name)
            return None
        return None

    def state(self, t: float, button: str | None = None) -> VehicleState:
        return VehicleState(
            t=t, speed_kmh=self.speed, gear=self.gear,
            engine_running=self.engine, ignition_on=self.ignition,
            button=button, bus_healthy=True,
        )


# ---------------------------------------------------------------------------
# Front-ends: replay a log  /  (skeleton) live python-can source
# ---------------------------------------------------------------------------

def frames_from_log(path: str):
    for fr in load_log(path):
        yield fr


def frames_from_python_can(channel: str, bitrate: int):  # pragma: no cover
    """LIVE source skeleton - enable when wiring real hardware.

    import can, time
    bus = can.Bus(channel=channel, interface='...', bitrate=bitrate,
                  receive_own_messages=False)   # listen-only on a vehicle!
    for msg in bus:
        yield Frame(t=time.time(), can_id=msg.arbitration_id, data=bytes(msg.data))
    """
    raise NotImplementedError("wire this to your reader; yields Frame(t, can_id, data)")


# ---------------------------------------------------------------------------
# Decode-only timeline  /  full immobilizer replay
# ---------------------------------------------------------------------------

def replay_states(frames, profile: dict):
    dec = ProfileDecoder(profile)
    for fr in frames:
        btn = dec.feed(fr)
        yield dec.state(fr.t, btn)


def run_immobilizer(frames, profile: dict, cfg: ImmobilizerConfig):
    act = LoggingActuator()
    im = Immobilizer(cfg, act)
    last_state = None
    transitions = []
    buttons = []
    for st in replay_states(frames, profile):
        if st.button:
            buttons.append((st.t, st.button))
        new = im.update(st)
        if new is not last_state:
            transitions.append((st.t, new.value,
                                f"speed={st.speed_kmh} gear={st.gear} eng={st.engine_running}"))
            last_state = new
    return im, act, transitions, buttons


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------

_TEST_PROFILE = {
    "platform": "test",
    "signals": {
        "vehicle_speed": {"maps_to": "speed", "id": "0x100", "start_bit": 0,
                          "length": 16, "endian": "little", "scale": 0.01},
        "ignition": {"maps_to": "ignition", "id": "0x101", "byte": 0, "bit": 0},
        "gear_park": {"maps_to": "gear", "id": "0x102", "byte": 0, "bit": 0, "park_value": 1},
        "wheel_buttons": {"maps_to": "button", "id": "0x5C1", "byte": 4, "idle": 0,
                          "codes": {"1": "VOLUP", "2": "VOLDN", "4": "MODE"}},
        "lock_btn": {"maps_to": "button", "id": "0x470", "byte": 0, "bit": 0,
                     "idle": 0, "symbol": "LOCK"},
    },
}


def _selftest() -> int:
    import numpy as np
    t0 = 0.0
    frames: list[Frame] = []

    def push(t, cid, d):
        frames.append(Frame(t, cid, bytes(d)))

    # Scenario: ignition ON, parked, speed 0. Owner enters code LOCK,VOLUP,MODE
    # then drives away. Expect: inhibit held at standstill until code -> DISARMED,
    # then free while driving.
    code = ["LOCK", "VOLUP", "MODE"]
    cfg = ImmobilizerConfig(code=code,
                            inhibit_method={"type": "data", "desc": "x"},
                            standstill_confirm_s=0.5, rearm_after_ign_off_s=5)

    t = t0
    # ignition on + parked + stopped for 3s (inhibit should assert)
    while t < 3.0:
        push(t, 0x101, [1])              # ignition on
        push(t + 0.001, 0x102, [1])      # gear park bit = 1
        push(t + 0.002, 0x100, [0, 0])   # speed 0
        t += 0.1
    # enter the button code
    bt = 3.2
    push(bt, 0x470, [1]); push(bt + 0.05, 0x470, [0])          # LOCK pulse
    push(bt + 0.3, 0x5C1, [0, 0, 0, 0, 1]); push(bt + 0.35, 0x5C1, [0, 0, 0, 0, 0])  # VOLUP
    push(bt + 0.6, 0x5C1, [0, 0, 0, 0, 4]); push(bt + 0.65, 0x5C1, [0, 0, 0, 0, 0])  # MODE
    # keep parked a moment, then drive away (speed rises, gear D)
    t = 4.2
    while t < 5.0:
        push(t, 0x101, [1]); push(t + 0.001, 0x102, [0]); push(t + 0.002, 0x100, [0, 0])
        t += 0.1
    t = 5.0
    while t < 8.0:
        spd = int((t - 5.0) * 20 / 0.01) & 0xFFFF   # accelerate
        push(t, 0x101, [1]); push(t + 0.001, 0x102, [0])
        push(t + 0.002, 0x100, [spd & 0xFF, (spd >> 8) & 0xFF])
        t += 0.1

    frames.sort(key=lambda f: f.t)
    im, act, trans, buttons = run_immobilizer(iter(frames), _TEST_PROFILE, cfg)

    print("[selftest] decoded button edges:", [b[1] for b in buttons])
    print("[selftest] state transitions:")
    for tt, s, ctx in trans:
        print(f"    t={tt:5.2f}  -> {s:10}  ({ctx})")
    print(f"[selftest] inhibit actions: {[(round(t,2),a) for t,a,_ in act.log]}")

    ok = True
    # must have detected the three code buttons in order
    ok &= [b[1] for b in buttons][:3] == code
    # must have asserted the inhibit while parked, before the code
    first_assert = next((t for t, a, _ in act.log if a == "ASSERT"), None)
    ok &= (first_assert is not None and first_assert < 3.2)
    # must end DISARMED (owner authenticated) and NOT asserting while driving
    ok &= (im.state is State.DISARMED and not act.asserted)
    print("\n[selftest]", "ALL PASSED" if ok else "SOME FAILED")
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Decode CAN frames into VehicleState; optionally run the immobilizer.",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--replay", metavar="LOG", help="a recorded CAN log to replay")
    ap.add_argument("--profile", help="profile JSON")
    ap.add_argument("--immobilizer", action="store_true", help="run the state machine over the log")
    ap.add_argument("--code", help="unlock button sequence, comma-separated, e.g. LOCK,LOCK,VOLUP")
    ap.add_argument("--every", type=float, default=1.0,
                    help="print a decoded sample every N seconds (decode-only mode)")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args(argv)

    if args.selftest:
        return _selftest()
    if not args.replay or not args.profile:
        ap.error("need --replay LOG --profile JSON (or --selftest)")

    with open(args.profile, "r", encoding="utf-8") as fh:
        profile = json.load(fh)
    frames = list(frames_from_log(args.replay))

    if args.immobilizer:
        if not args.code:
            ap.error("--immobilizer needs --code (the unlock button sequence)")
        inhibit = profile.get("inhibit", {"type": "physical"})
        cfg = ImmobilizerConfig(code=[c.strip() for c in args.code.split(",")],
                                inhibit_method=inhibit)
        im, act, trans, buttons = run_immobilizer(iter(frames), profile, cfg)
        print(f"\nReplayed {len(frames)} frames through the immobilizer.\n")
        print("Button edges detected:", [b[1] for b in buttons] or "(none)")
        print("\nState transitions:")
        for tt, s, ctx in trans:
            print(f"  t={tt:8.2f}  -> {s:10}  ({ctx})")
        print("\nInhibit actions (type from profile):")
        for tt, a, m in act.log:
            print(f"  t={tt:8.2f}  {a:8} [{m}]")
        print(f"\nFinal: {im.state.value}, inhibit {'ASSERTED' if act.asserted else 'released'}")
        return 0

    # decode-only timeline
    print(f"Decoding {len(frames)} frames with profile '{profile.get('platform','?')}':\n")
    next_print = None
    for st in replay_states(iter(frames), profile):
        if next_print is None:
            next_print = st.t
        if st.button:
            print(f"  t={st.t:8.2f}  BUTTON {st.button}")
        if st.t >= next_print:
            print(f"  t={st.t:8.2f}  speed={st.speed_kmh} gear={st.gear} "
                  f"engine={st.engine_running} ign={st.ignition_on}")
            next_print = st.t + args.every
    return 0


if __name__ == "__main__":
    sys.exit(main())
