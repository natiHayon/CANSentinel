#!/usr/bin/env python3
"""
find_gears.py - locate the CAN field that encodes the GEAR SELECTOR (P/R/N/D...)
or any small HELD multi-state field (drive mode, wiper setting, ...).

This is a READ-ONLY analysis tool. It reads a recorded log; it never opens a CAN
interface and never transmits.

Why a separate finder
---------------------
A gear selector is neither a button nor a whole-byte enum: it is a small
bit-field - usually a nibble (2-4 bits) inside a byte - that takes a few STABLE
values, each HELD for as long as you stay in that gear. find_events.py looks at
single bits (toggle) or whole bytes (value enum), so it steps over it.

For CANSentinel this is the immobilizer's other STANDSTILL gate: "gear in P/N".
The geometry it finds (ID / byte / bit / width and the held codes) tells you
which value is Park.

How to use it
-------------
Shift through the gears during the capture (e.g. P -> R -> N -> D -> ... -> P),
then:
  (A) --positions N   you visited N distinct gear positions - the field that
                      holds exactly N stable levels is ranked first.
  (B) (none)          rank by "looks like a held multi-state selector".

Examples
--------
  python find_gears.py --trace shift_through.json --positions 4
  python find_gears.py --trace shift_through.json
  python find_gears.py --selftest
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass

import numpy as np

from find_events import load_log, build_series, _parse_id_list, _byte_is_counter


@dataclass
class GearCandidate:
    can_id: int
    byte: int
    bit: int               # start bit within the byte (LSB = 0)
    width: int             # 2..4
    values: list[int]      # the distinct held codes (one per gear position)
    n_dwells: int          # how many held segments (shifts + 1)
    change_frac: float
    score: float = 0.0
    detail: str = ""


def _fields(payload: np.ndarray):
    """Yield (byte, start_bit, width, series) for every 2-4 bit sub-field."""
    F, L = payload.shape
    for b in range(L):
        if _byte_is_counter(payload[:, b]):
            continue                                       # counter bits mimic a held selector
        col = payload[:, b].astype(np.int64)
        for w in (2, 3, 4):
            mask = (1 << w) - 1
            for s in range(0, 8 - w + 1):
                yield b, s, w, (col >> s) & mask


def analyze(trace: str, positions: int | None = None, top: int = 10,
            only_ids: set[int] | None = None,
            exclude_ids: set[int] | None = None) -> list[GearCandidate]:
    frames = load_log(trace)
    series = build_series(frames)

    raw: dict[tuple, GearCandidate] = {}   # series signature -> narrowest-width candidate
    for cid, s in series.items():
        if only_ids and cid not in only_ids:
            continue
        if exclude_ids and cid in exclude_ids:
            continue
        for b, bit, w, field in _fields(s.payload):
            vals = np.unique(field)
            d = int(vals.size)
            if d < 2 or d > 8:
                continue                                   # constant, or not a small enum
            n = field.size
            trans = int(np.count_nonzero(np.diff(field) != 0))
            change_frac = trans / max(1, n - 1)
            if change_frac > 0.25:
                continue                                   # not latched -> noise/counter/pulse

            heldscore = 1.0 / (1.0 + 25.0 * change_frac)   # fewer changes -> more "held"
            d_pref = 1.0 if 2 <= d <= 6 else 0.5           # gears live here
            dwells = trans + 1
            # a clean selector dwells ~ once per visit; punish excessive flicker
            runpenalty = 1.0 / (1.0 + 0.1 * max(0, dwells - 2 * d))
            score = heldscore * d_pref * runpenalty
            if positions:
                score *= 1.0 / (1.0 + abs(d - positions))

            c = GearCandidate(
                can_id=cid, byte=b, bit=bit, width=w,
                values=[int(x) for x in vals], n_dwells=dwells,
                change_frac=change_frac, score=score,
                detail=(f"{d} held levels [" + ", ".join(f"0x{x:X}" for x in vals) + "], "
                        f"{dwells} dwells, changes {change_frac * 100:.2f}%"))

            # collapse identical series (a width-4 field whose top bit is always 0
            # reads the same as width-3) - keep the narrowest, best-scoring one
            sig = (cid, b, field.tobytes())
            cur = raw.get(sig)
            if cur is None or c.width < cur.width or (c.width == cur.width and c.score > cur.score):
                raw[sig] = c

    out = sorted(raw.values(), key=lambda c: c.score, reverse=True)
    return out[:top]


def _fmt(rank: int, c: GearCandidate) -> str:
    start = c.byte * 8 + c.bit
    return (f"  #{rank}  ID 0x{c.can_id:X}  byte {c.byte}, bit {c.bit}, width {c.width}  "
            f"(cantools start-bit {start})\n"
            f"        score {c.score:.3f}   {c.detail}")


# ---------------------------------------------------------------------------
# Self-test: synthesize a bus with a planted gear nibble, confirm recovery
# ---------------------------------------------------------------------------

def _selftest() -> int:
    import os
    import tempfile

    rng = np.random.default_rng(0)
    t0 = 1_700_000_000.0
    dur, hz = 30.0, 20.0
    n = int(dur * hz)
    # a shift sequence holding each gear for a stretch. Codes P=1 R=2 N=3 D=5
    # (5 avoids a width-2 alias showing the same 4 distinct values).
    seq = [1, 2, 3, 5, 3, 2, 1, 5]
    hold = max(1, n // len(seq))
    gear = np.resize(np.concatenate([np.full(hold, g) for g in seq]), n)

    rows = []

    def emit(t, cid, d):
        rows.append((f"{t:.6f}", f"{cid:03X}", bytes(d).hex()))

    counter = 0
    for i in range(n):
        t = t0 + i / hz
        # 0x100 noise: rolling counter + random + static
        d = bytearray(8)
        d[0] = counter & 0xFF
        counter += 1
        d[1] = int(rng.integers(0, 256))
        d[2] = 0xA5
        emit(t, 0x100, d)
        # 0x3C0: gear selector held in byte3 bits 0-2 (rest of the byte idle)
        d2 = bytearray(8)
        d2[3] = int(gear[i]) & 0x7
        emit(t, 0x3C0, d2)

    path = os.path.join(tempfile.gettempdir(), "can_gear_selftest.csv")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("epoch;id;data\n")
        for rrow in sorted(rows, key=lambda x: float(x[0])):
            fh.write(";".join(rrow) + "\n")
    print(f"[selftest] synthesized bus -> {path}")

    ok = True
    cands = analyze(path, positions=4, top=8)
    if not cands:
        print("  FAIL: no candidates"); return 1
    top = cands[0]
    print(f"  top = ID 0x{top.can_id:X} byte {top.byte} bit {top.bit} width {top.width} "
          f"values {['0x%X' % v for v in top.values]} score {top.score:.3f}")
    hit = (top.can_id == 0x3C0 and top.byte == 3 and top.bit == 0
           and top.width == 3 and len(top.values) == 4)
    print(f"  {'PASS' if hit else 'FAIL'} recover 4-level gear on 0x3C0 byte 3 bits 0-2 (--positions 4)")
    ok &= hit

    no_noise = all(c.can_id != 0x100 for c in cands[:3])
    print(f"  {'PASS' if no_noise else 'FAIL'} counter/noise 0x100 kept out of top 3")
    ok &= no_noise

    cands2 = analyze(path, top=8)   # no count given
    in_top3 = any(c.can_id == 0x3C0 and c.byte == 3 for c in cands2[:3])
    print(f"  {'PASS' if in_top3 else 'FAIL'} gear still in top 3 without --positions")
    ok &= in_top3

    print("\n[selftest]", "ALL PASSED" if ok else "SOME FAILED")
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Find the gear selector / a small held multi-state CAN field. Read-only.",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--trace", help="CAN log to analyze")
    ap.add_argument("--positions", type=int, help="how many distinct positions you shifted through")
    ap.add_argument("--only-ids", help="comma list of IDs to consider, e.g. 0x3C0")
    ap.add_argument("--exclude-ids", help="comma list of IDs to skip")
    ap.add_argument("--top", type=int, default=10, help="how many candidates to print")
    ap.add_argument("--selftest", action="store_true",
                    help="synthesize a bus with a planted gear nibble and verify recovery")
    args = ap.parse_args(argv)

    if args.selftest:
        return _selftest()
    if not args.trace:
        ap.error("--trace is required (or use --selftest)")

    cands = analyze(args.trace, positions=args.positions, top=args.top,
                    only_ids=_parse_id_list(args.only_ids),
                    exclude_ids=_parse_id_list(args.exclude_ids))
    if not cands:
        print("No gear-like fields found. Tips:")
        print("  * shift through the gears during the capture (P -> R -> N -> D -> ...)")
        print("  * pass --positions N with how many gears you used")
        return 2
    spec = f"--positions {args.positions}" if args.positions else "held-selector ranking"
    print(f"\nTop {len(cands)} gear candidate(s)  [{spec}]:\n")
    for i, c in enumerate(cands, 1):
        print(_fmt(i, c))
        print()
    print("Next: confirm which held value is Park, then add the field to a\n"
          "vehicle_profile as the gear gate (idle = the Park code).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
