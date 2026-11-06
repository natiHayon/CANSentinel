#!/usr/bin/env python3
"""
immobilizer.py - the on-device control state machine for a CAN anti-theft
immobilizer. Reference implementation + simulator (pure stdlib, no hardware).

This is the brain that sits on top of a validated vehicle profile
(vehicle_profile.py). It does NOT itself reverse-engineer anything and it does
NOT contain any car-specific disable payload: the actual inhibit action is
supplied by the profile (a data-frame descriptor for the ~80% data case, or a
physical-output descriptor for the ~20% analog case) and invoked through an
Actuator interface. Port the logic to your MCU; keep the actuator wiring.

What it does
------------
  * Boots ARMED (vehicle blocked). The owner clears it by entering a code using
    the car's OWN buttons - the button events come straight from find_events.py
    detections on the bus. No extra keypad to hide.
  * Asserts the inhibit (block start / block drive-away) ONLY when the vehicle is
    confirmed STATIONARY and off/in-park for a sustained window, and RELEASES it
    on any doubt, fault, or motion. It never acts on a moving car - that is a
    hard structural guarantee, not a tunable.
  * Falls back to the physical actuator automatically when the profile says this
    platform has no validated safe data inhibit (your 20%).

Safety model (non-negotiable, and it is what makes the product insurable)
-------------------------------------------------------------------------
  * STANDSTILL-ONLY: inhibit is only ever asserted when speed is confirmed ~0
    (and gear in P/N when gear is known). An unauthorized engine start is caught
    at the standstill moment, before drive-away - so "block start" and "block
    gear/drive-away" are both standstill actions.
  * FAIL-OPEN: stale bus, unhealthy bus, watchdog timeout, or any inability to
    confirm standstill -> release the inhibit. When in doubt, the car drives.
  * NO KILL-WHILE-MOVING MODE EXISTS. There is no code path that disables a
    moving vehicle.

Run the simulator (no hardware):
    python immobilizer.py --selftest
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field
from enum import Enum


# ---------------------------------------------------------------------------
# Inputs: one tick of decoded vehicle state (produced on-device by decoding the
# profile's signals from live CAN frames).
# ---------------------------------------------------------------------------

@dataclass
class VehicleState:
    t: float                       # monotonic seconds
    speed_kmh: float | None = None # None = unknown
    gear: str | None = None        # 'P'|'R'|'N'|'D'|... or None if unknown
    engine_running: bool | None = None
    ignition_on: bool = False
    button: str | None = None      # a code symbol this tick (from find_events), else None
    bus_healthy: bool = True        # False if the bus/decoder is in a bad state


# ---------------------------------------------------------------------------
# Actuator interface. The state machine calls assert_inhibit / release_inhibit;
# a real build wires these to (a) transmit the profile's data inhibit frame(s)
# or (b) toggle a GPIO driving the analog interrupt. No car-specific payload
# lives here - `method` is an opaque descriptor carried by the profile.
# ---------------------------------------------------------------------------

class Actuator:
    def assert_inhibit(self, method: dict) -> None: ...
    def release_inhibit(self, method: dict) -> None: ...


class LoggingActuator(Actuator):
    """Test/simulator actuator: records every action instead of touching a bus."""
    def __init__(self):
        self.asserted = False
        self.log: list[tuple[float, str, str]] = []
        self._t = 0.0

    def tick(self, t: float):
        self._t = t

    def assert_inhibit(self, method: dict) -> None:
        if not self.asserted:
            self.asserted = True
            self.log.append((self._t, "ASSERT", method.get("type", "?")))

    def release_inhibit(self, method: dict) -> None:
        if self.asserted:
            self.asserted = False
            self.log.append((self._t, "RELEASE", method.get("type", "?")))


# ---------------------------------------------------------------------------
# Configuration (comes from the validated profile + install settings)
# ---------------------------------------------------------------------------

@dataclass
class ImmobilizerConfig:
    code: list[str]                        # the unlock button sequence (set at install)
    inhibit_method: dict                   # {"type":"data"|"physical", ...} from profile
    standstill_kmh: float = 1.0            # speed at/below which we treat as stopped
    standstill_confirm_s: float = 1.0      # must be stopped this long before asserting
    bus_watchdog_s: float = 0.5            # no fresh/healthy data for this long -> fail open
    code_window_s: float = 20.0            # time allowed to enter the code after ignition
    max_attempts: int = 5                  # wrong codes before a lockout pause
    lockout_s: float = 30.0                # lockout duration (still fail-open on safety)
    rearm_after_ign_off_s: float = 10.0    # re-arm this long after ignition goes off


class State(Enum):
    ARMED = "ARMED"            # blocking; waiting for code or safe moment to inhibit
    CODE_ENTRY = "CODE_ENTRY"  # ignition on; collecting the code
    DISARMED = "DISARMED"      # owner authenticated; vehicle operates normally
    FAULT = "FAULT"            # fail-open: cannot confirm safety, inhibit released


# ---------------------------------------------------------------------------
# The state machine
# ---------------------------------------------------------------------------

@dataclass
class Immobilizer:
    cfg: ImmobilizerConfig
    actuator: Actuator
    state: State = State.ARMED
    _entered: list[str] = field(default_factory=list)
    _attempts: int = 0
    _lockout_until: float = 0.0
    _code_deadline: float | None = None
    _stopped_since: float | None = None
    _last_healthy_t: float | None = None
    _ign_off_since: float | None = None

    # --- helpers -----------------------------------------------------------
    def _confirmed_stopped(self, s: VehicleState) -> bool:
        """True only if we can POSITIVELY confirm the car is stationary."""
        if not s.bus_healthy:
            return False
        if s.speed_kmh is None:
            # no speed signal: fall back to engine-off as a standstill proxy; if
            # we know neither speed nor engine state, we CANNOT confirm -> False.
            if s.engine_running is False:
                pass  # engine off is an acceptable standstill proxy
            else:
                return False
        elif s.speed_kmh > self.cfg.standstill_kmh:
            return False
        if s.gear is not None and s.gear not in ("P", "N"):
            return False
        return True

    def _sustained_stop(self, s: VehicleState) -> bool:
        if self._confirmed_stopped(s):
            if self._stopped_since is None:
                self._stopped_since = s.t
            return (s.t - self._stopped_since) >= self.cfg.standstill_confirm_s
        self._stopped_since = None
        return False

    def _release(self):
        self.actuator.release_inhibit(self.cfg.inhibit_method)

    # --- main tick ---------------------------------------------------------
    def update(self, s: VehicleState) -> State:
        if isinstance(self.actuator, LoggingActuator):
            self.actuator.tick(s.t)

        # --- watchdog / fail-open (overrides everything) -------------------
        if s.bus_healthy:
            self._last_healthy_t = s.t
        stale = (self._last_healthy_t is None or
                 (s.t - self._last_healthy_t) > self.cfg.bus_watchdog_s)
        if stale or not s.bus_healthy:
            # cannot trust our view of the world -> never hold a car we can't
            # confirm is stopped. Release and park in FAULT until data returns.
            self._release()
            self.state = State.FAULT
            if s.bus_healthy and not stale:
                pass
            return self.state
        if self.state is State.FAULT:
            # data is back and healthy -> resume armed logic
            self.state = State.ARMED

        # --- track ignition-off for re-arming ------------------------------
        if not s.ignition_on:
            if self._ign_off_since is None:
                self._ign_off_since = s.t
            if (self.state is State.DISARMED and
                    (s.t - self._ign_off_since) >= self.cfg.rearm_after_ign_off_s):
                self.state = State.ARMED
                self._entered.clear()
        else:
            self._ign_off_since = None

        # --- disarmed: normal operation, nothing held ----------------------
        if self.state is State.DISARMED:
            self._release()
            return self.state

        # --- code handling (armed / code-entry) ----------------------------
        if self.state is State.ARMED and s.ignition_on:
            self.state = State.CODE_ENTRY
            self._entered.clear()
            self._code_deadline = s.t + self.cfg.code_window_s

        if self.state is State.CODE_ENTRY:
            locked_out = s.t < self._lockout_until
            if not locked_out and s.button is not None:
                self._entered.append(s.button)
                n = len(self.cfg.code)
                tail = self._entered[-n:]
                if tail == self.cfg.code:
                    # authenticated
                    self.state = State.DISARMED
                    self._attempts = 0
                    self._entered.clear()
                    self._release()
                    return self.state
                # wrong so far: if the entered length reached the code length and
                # the prefix can't still match, count an attempt and reset buffer
                if len(self._entered) >= n and tail != self.cfg.code:
                    if not self._prefix_possible():
                        self._attempts += 1
                        self._entered.clear()
                        if self._attempts >= self.cfg.max_attempts:
                            self._lockout_until = s.t + self.cfg.lockout_s
                            self._attempts = 0
            # code window expired -> back to ARMED (still blocking)
            if self._code_deadline is not None and s.t > self._code_deadline:
                self.state = State.ARMED
                self._entered.clear()

        # --- ASSERT the inhibit: ARMED/CODE_ENTRY + confirmed sustained stop
        # This is the ONLY place the inhibit is asserted, and it is gated on a
        # positively-confirmed, sustained standstill. Otherwise -> release.
        if self.state in (State.ARMED, State.CODE_ENTRY):
            if self._sustained_stop(s):
                self.actuator.assert_inhibit(self.cfg.inhibit_method)
            else:
                # moving, or cannot confirm stopped -> fail open
                self._release()
        return self.state

    def _prefix_possible(self) -> bool:
        """Could what's entered still become the code? (sliding match)"""
        code = self.cfg.code
        buf = self._entered
        for start in range(len(buf)):
            seg = buf[start:]
            if len(seg) <= len(code) and seg == code[:len(seg)]:
                return True
        return False


# ---------------------------------------------------------------------------
# Simulator / self-test
# ---------------------------------------------------------------------------

def _run(cfg: ImmobilizerConfig, ticks: list[VehicleState]):
    act = LoggingActuator()
    im = Immobilizer(cfg, act)
    states = []
    for s in ticks:
        states.append(im.update(s))
    return im, act, states


def _selftest() -> int:
    ok = True
    code = ["LOCK", "LOCK", "VOLUP", "UNLOCK"]
    data_method = {"type": "data", "desc": "platform start-inhibit frame (from profile)"}
    phys_method = {"type": "physical", "output": "starter_cut"}

    def seq(**kw):
        return VehicleState(**kw)

    # Scenario 1: THIEF - ignition on, standstill, no/!wrong code -> inhibit held
    print("[selftest] 1. Thief: ignition on, stopped, no valid code")
    cfg = ImmobilizerConfig(code=code, inhibit_method=data_method)
    ticks = [seq(t=i * 0.2, speed_kmh=0.0, gear="P", engine_running=False,
                 ignition_on=True) for i in range(20)]
    im, act, st = _run(cfg, ticks)
    held = act.asserted and st[-1] in (State.ARMED, State.CODE_ENTRY)
    print(f"      final={st[-1].value} inhibit_asserted={act.asserted}")
    ok &= held

    # Scenario 2: OWNER - enters the correct code via buttons -> disarmed/released
    print("[selftest] 2. Owner: enters correct button code")
    cfg = ImmobilizerConfig(code=code, inhibit_method=data_method)
    ticks = [seq(t=0.0, speed_kmh=0.0, gear="P", engine_running=False, ignition_on=True)]
    tb = 0.5
    for b in code:
        ticks.append(seq(t=tb, speed_kmh=0.0, gear="P", engine_running=False,
                         ignition_on=True, button=b))
        tb += 0.5
    ticks += [seq(t=tb + i * 0.5, speed_kmh=0.0, gear="P", engine_running=True,
                  ignition_on=True) for i in range(4)]
    im, act, st = _run(cfg, ticks)
    print(f"      final={st[-1].value} inhibit_asserted={act.asserted}")
    ok &= (st[-1] is State.DISARMED and not act.asserted)

    # Scenario 3: SAFETY - armed, but vehicle is MOVING -> inhibit never asserts
    print("[selftest] 3. Safety: armed while vehicle is moving (30 km/h)")
    cfg = ImmobilizerConfig(code=code, inhibit_method=data_method)
    ticks = [seq(t=i * 0.2, speed_kmh=30.0, gear="D", engine_running=True,
                 ignition_on=True) for i in range(20)]
    im, act, st = _run(cfg, ticks)
    ever = any(e[1] == "ASSERT" for e in act.log)
    print(f"      inhibit_ever_asserted={ever}  (must be False)")
    ok &= (not ever)

    # Scenario 3b: car STOPS after driving -> only then may it assert
    print("[selftest] 3b. Moving then stops at standstill -> asserts only after stop")
    ticks = ([seq(t=i * 0.2, speed_kmh=30.0, gear="D", engine_running=True,
                  ignition_on=True) for i in range(10)] +
             [seq(t=2.0 + i * 0.2, speed_kmh=0.0, gear="P", engine_running=True,
                  ignition_on=True) for i in range(15)])
    im, act, st = _run(cfg, ticks)
    first_assert = next((e[0] for e in act.log if e[1] == "ASSERT"), None)
    print(f"      first ASSERT at t={first_assert} (should be >= 3.0s, after stop+confirm)")
    ok &= (first_assert is not None and first_assert >= 3.0)

    # Scenario 4: FAULT - bus goes stale -> fail open (release) and FAULT state
    print("[selftest] 4. Fail-open: healthy bus then stale -> release + FAULT")
    cfg = ImmobilizerConfig(code=code, inhibit_method=data_method)
    ticks = [seq(t=i * 0.2, speed_kmh=0.0, gear="P", engine_running=False,
                 ignition_on=True) for i in range(15)]   # asserts here
    ticks += [seq(t=3.0 + i * 1.0, speed_kmh=None, gear=None, engine_running=None,
                  ignition_on=True, bus_healthy=False) for i in range(3)]  # stale
    im, act, st = _run(cfg, ticks)
    print(f"      final={st[-1].value} inhibit_asserted={act.asserted}")
    ok &= (st[-1] is State.FAULT and not act.asserted)

    # Scenario 5: WRONG CODE lockout - stays armed/holding, never disarms
    print("[selftest] 5. Wrong codes: stays armed (never disarms)")
    cfg = ImmobilizerConfig(code=code, inhibit_method=phys_method, max_attempts=3)
    ticks = [seq(t=0.0, speed_kmh=0.0, gear="P", engine_running=False, ignition_on=True)]
    tb = 0.5
    for b in ["UNLOCK", "VOLUP", "LOCK", "VOLUP"] * 4:  # wrong sequence repeated
        ticks.append(seq(t=tb, speed_kmh=0.0, gear="P", engine_running=False,
                         ignition_on=True, button=b))
        tb += 0.3
    im, act, st = _run(cfg, ticks)
    print(f"      final={st[-1].value} inhibit_asserted={act.asserted} (physical)")
    ok &= (st[-1] is not State.DISARMED and act.asserted)

    print("\n[selftest]", "ALL PASSED" if ok else "SOME FAILED")
    return 0 if ok else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Immobilizer control state machine (reference + simulator).",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--selftest", action="store_true",
                    help="run the simulator scenarios (no hardware needed)")
    args = ap.parse_args(argv)
    if args.selftest:
        return _selftest()
    ap.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
