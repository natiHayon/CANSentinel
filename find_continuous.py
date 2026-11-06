#!/usr/bin/env python3
"""
find_continuous.py - locate the CAN field carrying a CONTINUOUS signal
(vehicle speed, RPM, coolant temp, throttle position, ...).

This is a READ-ONLY analysis tool. It reads a recorded log; it never opens a CAN
interface and never transmits.

Why a separate finder
---------------------
find_events.py recovers DISCRETE controls (buttons, locks) by counting events.
Continuous signals are the opposite: a multi-byte number that moves *smoothly*.
Event-counting throws them away (they look like they change every frame). You
find a continuous signal by driving a KNOWN maneuver - roll 0 -> X -> 0, or rev
the engine up and back down - and looking for the field whose decoded value is
smooth, spans a real range, and traces that maneuver.

For CANSentinel this is how you recover the immobilizer's STANDSTILL gate
(vehicle speed) on a new car: the geometry it finds (ID / start-bit / width /
endianness / scale) is exactly what vehicle_profile.py needs for a `continuous`
gate signal.

How it ranks (two ways)
-----------------------
  (A) no reference     - physical plausibility: the value changes smoothly
                         (small steps relative to its range), sweeps a wide
                         range, and traces a clean up-then-down path. Rolling
                         counters and checksums are rejected outright.
  (B) --reference F    - a file of "<epoch> <value>" lines (e.g. a GPS speed
                         trace); candidates are scored by lag-tolerant
                         correlation to it, and a scale/offset is fitted.

Examples
--------
  python find_continuous.py --trace drive_0_30_0.json --top 10
  python find_continuous.py --trace drive.json --reference gps_speed.txt
  python find_continuous.py --selftest
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass

import numpy as np

from find_events import load_log, build_series, _parse_id_list


# ---------------------------------------------------------------------------
# Candidate model
# ---------------------------------------------------------------------------

@dataclass
class ContCandidate:
    can_id: int
    start_bit: int        # byte * 8 (fields are byte-aligned here)
    width: int            # 8 or 16
    endian: str           # 'little' | 'big'  ('-' for 8-bit)
    signed: bool
    vmin: float
    vmax: float
    score: float = 0.0
    scale: float = 1.0
    offset: float = 0.0
    detail: str = ""


def _fields(payload: np.ndarray):
    """Yield (start_byte, width, endian, signed, raw_series) byte-aligned fields."""
    F, L = payload.shape
    p = payload.astype(np.int64)
    for b in range(L):
        yield b, 8, "-", False, p[:, b]
        col = p[:, b]
        if int(col.max()) >= 128:                      # signed only if it differs
            yield b, 8, "-", True, np.where(col >= 128, col - 256, col)
    for b in range(L - 1):
        le = p[:, b] + 256 * p[:, b + 1]
        be = 256 * p[:, b] + p[:, b + 1]
        yield b, 16, "little", False, le
        yield b, 16, "big", False, be
        if int(le.max()) >= 32768:
            yield b, 16, "little", True, np.where(le >= 32768, le - 65536, le)
        if int(be.max()) >= 32768:
            yield b, 16, "big", True, np.where(be >= 32768, be - 65536, be)


def _reject(v: np.ndarray, has_ref: bool = False) -> bool:
    """True if the series is a rolling counter, a checksum/noise byte, or flat."""
    n = v.size
    if n < 8:
        return True
    rng = float(v.max() - v.min())
    if rng <= 0:
        return True                                    # constant -> not a signal
    d = np.diff(v)
    change_frac = float(np.count_nonzero(d)) / (n - 1)
    neg_frac = float(np.mean(d < 0))
    pos_frac = float(np.mean(d > 0))
    norm_step = float(np.median(np.abs(d))) / rng
    # checksum / high-entropy noise: changes nearly every frame in big jumps
    if change_frac > 0.8 and norm_step > 0.15:
        return True
    # near-monotonic accumulator that changes almost every frame: a rolling
    # counter or odometer (including a wide field whose high byte is a counter) -
    # it only ever moves one way, unlike a physical signal driven up AND down.
    # Skip this when a reference trace is given, so a one-way capture still
    # survives to be correlated.
    if not has_ref and change_frac > 0.8 and (neg_frac < 0.05 or pos_frac < 0.05):
        return True
    return False


def _plausibility(v: np.ndarray) -> float:
    """Score a continuous-signal candidate with no reference trace."""
    n = v.size
    rng = float(v.max() - v.min())
    d = np.diff(v)
    norm_step = float(np.median(np.abs(d))) / rng
    smooth = 1.0 / (1.0 + 30.0 * norm_step)            # small steps -> ~1
    distinct = int(np.unique(v).size)
    rangeterm = min(1.0, np.log1p(distinct) / np.log1p(64))
    sd = np.sign(d)
    sd = sd[sd != 0]
    turns = int(np.count_nonzero(np.diff(sd) != 0)) if sd.size > 1 else 0
    turnrate = turns / max(1, sd.size)
    traj = 1.0 / (1.0 + 8.0 * turnrate)                # few direction changes -> ~1
    return smooth * rangeterm * traj


def _load_reference(path: str) -> np.ndarray:
    """Load '<epoch> <value>' rows (space/comma/semicolon separated)."""
    rows = []
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for ln in fh:
            ln = ln.strip()
            if not ln or ln[0] in "#;":
                continue
            parts = ln.replace(",", " ").replace(";", " ").split()
            if len(parts) < 2:
                continue
            try:
                rows.append((float(parts[0]), float(parts[1])))
            except ValueError:
                continue
    return np.array(sorted(rows), dtype=np.float64) if rows else np.empty((0, 2))


def _corr_score(t: np.ndarray, v: np.ndarray, ref: np.ndarray) -> tuple[float, float, float]:
    """Lag-tolerant correlation of a candidate to a reference; fit scale/offset.

    Returns (best_|r|, scale, offset). ref is (K,2) of (epoch, value).
    """
    rt, rv = ref[:, 0], ref[:, 1]
    best = (0.0, 1.0, 0.0)
    for lag in np.linspace(-2.0, 2.0, 21):
        rs = np.interp(t, rt + lag, rv, left=rv[0], right=rv[-1])
        if np.std(rs) < 1e-9 or np.std(v) < 1e-9:
            continue
        r = float(np.corrcoef(v, rs)[0, 1])
        if abs(r) > abs(best[0]):
            a, b = np.polyfit(v, rs, 1)            # rs ~= a*v + b  -> scale,offset
            best = (r, float(a), float(b))
    return abs(best[0]), best[1], best[2]


def analyze(trace: str, reference_path: str | None = None, top: int = 10,
            only_ids: set[int] | None = None,
            exclude_ids: set[int] | None = None) -> list[ContCandidate]:
    frames = load_log(trace)
    series = build_series(frames)
    ref = _load_reference(reference_path) if reference_path else None

    cands: list[ContCandidate] = []
    for cid, s in series.items():
        if only_ids and cid not in only_ids:
            continue
        if exclude_ids and cid in exclude_ids:
            continue
        has_ref = ref is not None and ref.size > 0
        for b, width, endian, signed, raw in _fields(s.payload):
            v = raw.astype(np.float64)
            if _reject(v, has_ref=has_ref):
                continue
            c = ContCandidate(can_id=cid, start_bit=b * 8, width=width,
                              endian=endian, signed=signed,
                              vmin=float(v.min()), vmax=float(v.max()))
            if ref is not None and ref.size:
                r, scale, offset = _corr_score(s.t, v, ref)
                c.score = r
                c.scale, c.offset = scale, offset
                c.detail = (f"r={r:.2f}, fit y={scale:.4g}*raw+{offset:.4g} "
                            f"[raw {c.vmin:.0f}..{c.vmax:.0f}]")
            else:
                c.score = _plausibility(v)
                c.detail = (f"raw {c.vmin:.0f}..{c.vmax:.0f}, "
                            f"{int(np.unique(v).size)} levels")
            cands.append(c)

    # keep the best-scoring field per CAN ID so one signal does not fill the list
    best_by_id: dict[int, ContCandidate] = {}
    for c in cands:
        cur = best_by_id.get(c.can_id)
        if cur is None or c.score > cur.score:
            best_by_id[c.can_id] = c
    out = sorted(best_by_id.values(), key=lambda c: c.score, reverse=True)
    return out[:top]


def _fmt(rank: int, c: ContCandidate) -> str:
    end = "" if c.width == 8 else f" {c.endian}-endian"
    sgn = " signed" if c.signed else ""
    cant = f"cantools start-bit {c.start_bit}"
    return (f"  #{rank}  ID 0x{c.can_id:X}  byte {c.start_bit // 8}  "
            f"{c.width}-bit{end}{sgn}  ({cant})\n"
            f"        score {c.score:.3f}   {c.detail}")


# ---------------------------------------------------------------------------
# Self-test: synthesize a bus with a planted speed ramp, confirm recovery
# ---------------------------------------------------------------------------

def _selftest() -> int:
    import os
    import tempfile

    rng = np.random.default_rng(0)
    t0 = 1_700_000_000.0
    dur, hz = 20.0, 50.0
    n = int(dur * hz)
    # a known maneuver: 0 -> 100 km/h -> 0. raw = km/h / 0.01 (0..10000), 16-bit LE.
    kmh = np.concatenate([np.linspace(0, 100, n // 2),
                          np.linspace(100, 0, n - n // 2)])
    raw = np.round(kmh / 0.01).astype(int)

    rows = []

    def emit(t, cid, d):
        rows.append((f"{t:.6f}", f"{cid:03X}", bytes(d).hex()))

    counter = 0
    for i in range(n):
        t = t0 + i / hz
        # 0x100 = noise ID: byte0 rolling counter, byte1 checksum-ish, byte2 static
        d = bytearray(8)
        d[0] = counter & 0xFF
        counter += 1
        d[1] = int(rng.integers(0, 256))
        d[2] = 0xA5
        emit(t, 0x100, d)
        # 0x200 = the speed signal: 16-bit little-endian on bytes 0-1
        r = int(raw[i]) & 0xFFFF
        d2 = bytearray(8)
        d2[0] = r & 0xFF
        d2[1] = (r >> 8) & 0xFF
        emit(t, 0x200, d2)

    path = os.path.join(tempfile.gettempdir(), "can_cont_selftest.csv")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("epoch;id;data\n")
        for rrow in sorted(rows, key=lambda x: float(x[0])):
            fh.write(";".join(rrow) + "\n")
    print(f"[selftest] synthesized bus -> {path}")

    cands = analyze(path, top=8)
    ok = True
    if not cands:
        print("  FAIL: no candidates"); return 1
    top = cands[0]
    print(f"  top = ID 0x{top.can_id:X} byte {top.start_bit // 8} "
          f"{top.width}-bit {top.endian} score {top.score:.3f}")
    hit = (top.can_id == 0x200 and top.start_bit == 0
           and top.width == 16 and top.endian == "little")
    print(f"  {'PASS' if hit else 'FAIL'} recover 16-bit LE speed on 0x200 byte 0")
    ok &= hit
    # the rolling counter / checksum on 0x100 must not surface as a top signal
    no_noise = all(c.can_id != 0x100 for c in cands[:3])
    print(f"  {'PASS' if no_noise else 'FAIL'} counter/checksum 0x100 kept out of top 3")
    ok &= no_noise

    # reference-correlation path: feed the true km/h back and expect a clean fit
    refpath = os.path.join(tempfile.gettempdir(), "can_cont_ref.txt")
    with open(refpath, "w", encoding="utf-8") as fh:
        for i in range(n):
            fh.write(f"{t0 + i / hz:.6f} {kmh[i]:.3f}\n")
    rc = analyze(path, reference_path=refpath, top=5)
    rhit = bool(rc) and rc[0].can_id == 0x200 and rc[0].score > 0.95
    scale_ok = rhit and abs(rc[0].scale - 0.01) < 0.002
    print(f"  {'PASS' if rhit else 'FAIL'} reference correlation finds 0x200 (r={rc[0].score:.3f})")
    print(f"  {'PASS' if scale_ok else 'FAIL'} fitted scale ~0.01 (got {rc[0].scale:.4g})")
    ok &= rhit and scale_ok

    print("\n[selftest]", "ALL PASSED" if ok else "SOME FAILED")
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Find the CAN field carrying a continuous signal (speed/RPM/temp). Read-only.",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--trace", help="CAN log to analyze")
    ap.add_argument("--reference", help="file of '<epoch> <value>' rows (e.g. GPS speed)")
    ap.add_argument("--only-ids", help="comma list of IDs to consider, e.g. 0x1A0,0x3E9")
    ap.add_argument("--exclude-ids", help="comma list of IDs to skip")
    ap.add_argument("--top", type=int, default=10, help="how many candidates to print")
    ap.add_argument("--selftest", action="store_true",
                    help="synthesize a bus with a planted speed ramp and verify recovery")
    args = ap.parse_args(argv)

    if args.selftest:
        return _selftest()
    if not args.trace:
        ap.error("--trace is required (or use --selftest)")

    cands = analyze(args.trace, reference_path=args.reference, top=args.top,
                    only_ids=_parse_id_list(args.only_ids),
                    exclude_ids=_parse_id_list(args.exclude_ids))
    if not cands:
        print("No continuous-signal candidates found. Tips:")
        print("  * drive a clear maneuver during the capture (0 -> speed -> 0)")
        print("  * pass --reference with a value trace for a correlation fit")
        return 2
    spec = f"--reference {args.reference}" if args.reference else "plausibility ranking"
    print(f"\nTop {len(cands)} continuous candidate(s)  [{spec}]:\n")
    for i, c in enumerate(cands, 1):
        print(_fmt(i, c))
        print()
    print("Next: confirm the #1 by re-driving and watching that field track your\n"
          "speed, then add it to a vehicle_profile as a `continuous` gate signal.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
