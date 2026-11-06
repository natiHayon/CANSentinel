#!/usr/bin/env python3
"""
checksum_counter.py - find the rolling COUNTER and crack the CHECKSUM algorithm
on a CAN message, so a transmitted frame is ACCEPTED by the target ECU.

Why you need this
-----------------
When the immobilizer transmits a data-inhibit frame, most protected ECUs reject
it unless two things are right:
  * the ROLLING COUNTER (a byte or nibble that increments every message) has the
    next expected value, and
  * the CHECKSUM / CRC byte matches, computed over the payload by the exact
    algorithm that ECU uses.
Send a stale counter or a wrong CRC and the frame is ignored or sets a fault.
This tool reverse-engineers both from a recorded log and prints the exact recipe,
then proves it by reproducing the real checksum on (ideally) 100% of frames.

It is READ-ONLY (it only analyzes a log). The recovered recipe becomes part of a
vehicle profile's inhibit "method", so the device can compute a valid counter +
checksum for each frame it sends.

What it detects
---------------
  * COUNTER: a full byte or a 4-bit nibble that increments +1 per frame (any
    modulus / wrap), reported with its location.
  * CHECKSUM, trying in order:
      - sum8:  chk = (sum(other bytes) + k) & 0xFF          (k solved directly)
      - xor8:  chk = (xor(other bytes) ^ k)                 (k solved directly)
      - CRC-8: a curated set of automotive variants (AUTOSAR 0x2F, SAE-J1850
               0x1D, CRC-8 0x07, Maxim 0x31, CDMA2000 0x9B, DVB-S2 0xD5, ...),
               each over the other bytes.
    It reports the model with the highest match rate and the exact formula.

Usage
-----
  python checksum_counter.py --trace log.csv --id 0x1A0
  python checksum_counter.py --trace log.csv            # scan every ID
  python checksum_counter.py --selftest                 # no hardware
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass

import numpy as np

from find_events import load_log, build_series, IdSeries


# ---------------------------------------------------------------------------
# Bit reflection helpers
# ---------------------------------------------------------------------------

def _reflect8(b: int) -> int:
    r = 0
    for i in range(8):
        if b & (1 << i):
            r |= 1 << (7 - i)
    return r


_REFLECT = np.array([_reflect8(i) for i in range(256)], dtype=np.int64)


# ---------------------------------------------------------------------------
# Vectorized CRC-8 across all frames at once
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class CrcSpec:
    name: str
    poly: int
    init: int
    refin: bool
    refout: bool
    xorout: int


# Curated automotive / common CRC-8 variants.
CRC8_VARIANTS = [
    CrcSpec("CRC-8/AUTOSAR", 0x2F, 0xFF, False, False, 0xFF),
    CrcSpec("CRC-8/SAE-J1850", 0x1D, 0xFF, False, False, 0xFF),
    CrcSpec("CRC-8/SAE-J1850-ZERO", 0x1D, 0x00, False, False, 0x00),
    CrcSpec("CRC-8 (base)", 0x07, 0x00, False, False, 0x00),
    CrcSpec("CRC-8/ITU", 0x07, 0x00, False, False, 0x55),
    CrcSpec("CRC-8/ROHC", 0x07, 0xFF, True, True, 0x00),
    CrcSpec("CRC-8/MAXIM", 0x31, 0x00, True, True, 0x00),
    CrcSpec("CRC-8/CDMA2000", 0x9B, 0xFF, False, False, 0x00),
    CrcSpec("CRC-8/DARC", 0x39, 0x00, True, True, 0x00),
    CrcSpec("CRC-8/DVB-S2", 0xD5, 0x00, False, False, 0x00),
    CrcSpec("CRC-8/WCDMA", 0x9B, 0x00, True, True, 0x00),
    CrcSpec("CRC-8/LTE", 0x9B, 0x00, False, False, 0x00),
]


def crc8_vec(cols: list[np.ndarray], spec: CrcSpec) -> np.ndarray:
    """CRC-8 over the given byte-columns, computed for every frame at once.

    cols: list of int64 arrays (one per data byte, in order), length F each.
    returns: int64 array (F,) of CRC values.
    """
    n = cols[0].shape[0]
    crc = np.full(n, spec.init, dtype=np.int64)
    for col in cols:
        b = _REFLECT[col] if spec.refin else col
        crc ^= b
        for _ in range(8):
            carry = (crc & 0x80) != 0
            crc = (crc << 1) & 0xFF
            crc[carry] ^= spec.poly
    if spec.refout:
        crc = _REFLECT[crc]
    return (crc ^ spec.xorout) & 0xFF


# ---------------------------------------------------------------------------
# Counter detection
# ---------------------------------------------------------------------------

@dataclass
class CounterHit:
    byte: int
    part: str          # 'byte' | 'low-nibble' | 'high-nibble'
    modulus: int
    match: float


def _counter_of_series(vals: np.ndarray, modulus: int) -> float:
    """Fraction of consecutive deltas equal to +1 (mod modulus)."""
    if vals.shape[0] < 3:
        return 0.0
    d = np.diff(vals) % modulus
    return float(np.mean(d == 1))


def detect_counters(payload: np.ndarray) -> list[CounterHit]:
    F, L = payload.shape
    hits: list[CounterHit] = []
    for b in range(L):
        col = payload[:, b].astype(np.int64)
        # full byte
        m = _counter_of_series(col, 256)
        if m >= 0.95 and len(np.unique(col)) > 8:
            hits.append(CounterHit(b, "byte", 256, m))
            continue
        # nibbles
        low = col & 0x0F
        high = (col >> 4) & 0x0F
        ml = _counter_of_series(low, 16)
        mh = _counter_of_series(high, 16)
        if ml >= 0.95 and len(np.unique(low)) > 4:
            hits.append(CounterHit(b, "low-nibble", 16, ml))
        if mh >= 0.95 and len(np.unique(high)) > 4:
            hits.append(CounterHit(b, "high-nibble", 16, mh))
    return hits


# ---------------------------------------------------------------------------
# Checksum detection
# ---------------------------------------------------------------------------

@dataclass
class ChecksumHit:
    byte: int
    algo: str
    formula: str
    match: float


def _solve_const_models(chk: np.ndarray, others: list[np.ndarray], c: int) -> list[ChecksumHit]:
    """sum8 and xor8 with a directly-solved constant k."""
    hits = []
    idxs = [j for j in range(len(others) + 1) if j != c]
    # sum8
    s = np.zeros_like(chk)
    for col in others:
        s = (s + col) & 0xFF
    k = (chk - s) & 0xFF
    if np.ptp(k) == 0:
        hits.append(ChecksumHit(c, "sum8",
                                f"byte{c} = (sum(other bytes) + 0x{int(k[0]):02X}) & 0xFF",
                                1.0))
    else:
        match = float(np.mean(k == np.bincount(k).argmax()))
        if match >= 0.8:
            hits.append(ChecksumHit(c, "sum8~",
                                    f"byte{c} ~ (sum(other bytes) + const) & 0xFF (unstable)",
                                    match))
    # xor8
    x = np.zeros_like(chk)
    for col in others:
        x ^= col
    k = chk ^ x
    if np.ptp(k) == 0:
        hits.append(ChecksumHit(c, "xor8",
                                f"byte{c} = xor(other bytes) ^ 0x{int(k[0]):02X}", 1.0))
    return hits


def detect_checksum(payload: np.ndarray, counters: set[int],
                    sample: int = 3000) -> list[ChecksumHit]:
    F, L = payload.shape
    # sample frames for the search; the winner is re-verified on ALL frames
    if F > sample:
        idx = np.linspace(0, F - 1, sample).astype(int)
        P = payload[idx].astype(np.int64)
    else:
        P = payload.astype(np.int64)

    best_per_byte: dict[int, ChecksumHit] = {}
    for c in range(L):
        col = P[:, c]
        if np.ptp(col) == 0:
            continue  # constant byte is not a checksum
        if c in counters:
            continue
        others = [P[:, j] for j in range(L) if j != c]
        if not others:
            continue
        cands = _solve_const_models(col, others, c)
        # CRC-8 variants over the other bytes (index order)
        for spec in CRC8_VARIANTS:
            pred = crc8_vec(others, spec)
            match = float(np.mean(pred == col))
            if match >= 0.8:
                byte_list = ",".join(str(j) for j in range(L) if j != c)
                cands.append(ChecksumHit(
                    c, spec.name,
                    f"byte{c} = {spec.name} over bytes[{byte_list}]  "
                    f"(poly=0x{spec.poly:02X} init=0x{spec.init:02X} "
                    f"refin={int(spec.refin)} refout={int(spec.refout)} "
                    f"xorout=0x{spec.xorout:02X})",
                    match))
        if cands:
            best = max(cands, key=lambda h: h.match)
            best_per_byte[c] = best

    # re-verify each byte's winner on the FULL log
    verified = []
    for c, hit in best_per_byte.items():
        full_others = [payload[:, j].astype(np.int64) for j in range(L) if j != c]
        full_chk = payload[:, c].astype(np.int64)
        if hit.algo == "sum8":
            s = np.zeros_like(full_chk)
            for col in full_others:
                s = (s + col) & 0xFF
            k = (full_chk - s) & 0xFF
            hit.match = float(np.mean(k == k[0]))
        elif hit.algo == "xor8":
            x = np.zeros_like(full_chk)
            for col in full_others:
                x ^= col
            k = full_chk ^ x
            hit.match = float(np.mean(k == k[0]))
        else:
            spec = next((s for s in CRC8_VARIANTS if s.name == hit.algo), None)
            if spec is not None:
                hit.match = float(np.mean(crc8_vec(full_others, spec) == full_chk))
        verified.append(hit)
    verified.sort(key=lambda h: h.match, reverse=True)
    return verified


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

def analyze_id(cid: int, s: IdSeries) -> None:
    print(f"ID 0x{cid:X}  ({s.n} frames, DLC {s.length})")
    counters = detect_counters(s.payload)
    cset = {h.byte for h in counters}
    if counters:
        for h in counters:
            print(f"  counter : byte {h.byte} {h.part} -> increments 0..{h.modulus-1} "
                  f"(+1/frame, {h.match*100:.0f}% of frames)")
    else:
        print("  counter : none detected")

    checks = detect_checksum(s.payload, cset)
    strong = [h for h in checks if h.match >= 0.999]
    if strong:
        for h in strong:
            print(f"  checksum: {h.formula}")
            print(f"            [{h.algo}]  CONFIRMED  match {h.match*100:.2f}% of {s.n} frames")
    elif checks and checks[0].match >= 0.8:
        h = checks[0]
        print(f"  checksum: best guess {h.formula}")
        print(f"            [{h.algo}]  match {h.match*100:.1f}% (NOT confirmed - may need a data-id "
              f"constant, a byte subset, or inclusion of the CAN ID)")
    else:
        print("  checksum: none matched the tried models (sum8 / xor8 / common CRC-8).")
        print("            Next: the CRC may include a per-message 'data id' constant "
              "(AUTOSAR E2E) or cover only a subset of bytes.")
    print()


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------

def _selftest() -> int:
    import tempfile, os
    rng = np.random.default_rng(3)
    t0 = 1_700_000_000.0
    rows = []

    def emit(t, cid, d):
        rows.append((f"{t:.6f}", f"{cid:03X}", bytes(d).hex()))

    # ID 0x1A0: byte6 high-nibble = counter; byte7 = sum8 of bytes0..6 (+0x00)
    cnt = 0
    t = t0
    for _ in range(1500):
        d = bytearray(8)
        for j in range(6):
            d[j] = int(rng.integers(0, 256))
        d[6] = (cnt % 16) << 4
        cnt += 1
        d[7] = sum(d[0:7]) & 0xFF
        emit(t, 0x1A0, d)
        t += 0.01

    # ID 0x2C0: byte0 = counter byte; byte7 = CRC-8/AUTOSAR over bytes0..6
    spec = next(s for s in CRC8_VARIANTS if s.name == "CRC-8/AUTOSAR")
    cnt = 0
    t = t0
    for _ in range(1500):
        d = bytearray(8)
        d[0] = cnt & 0xFF
        cnt += 1
        for j in range(1, 7):
            d[j] = int(rng.integers(0, 256))
        cols = [np.array([d[j]], dtype=np.int64) for j in range(7)]
        d[7] = int(crc8_vec(cols, spec)[0])
        emit(t, 0x2C0, d)
        t += 0.01

    path = os.path.join(tempfile.gettempdir(), "chk_selftest.csv")
    rows.sort(key=lambda x: float(x[0]))
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("epoch;id;data\n")
        for r in rows:
            fh.write(";".join(r) + "\n")
    print(f"[selftest] synthesized -> {path}\n")

    frames = load_log(path)
    series = build_series(frames)
    ok = True

    print("[selftest] ID 0x1A0 expect: counter byte6 high-nibble, checksum byte7 sum8")
    s = series[0x1A0]
    counters = detect_counters(s.payload)
    checks = detect_checksum(s.payload, {h.byte for h in counters})
    c_ok = any(h.byte == 6 and h.part == "high-nibble" for h in counters)
    k_ok = any(h.byte == 7 and h.algo == "sum8" and h.match >= 0.999 for h in checks)
    analyze_id(0x1A0, s)
    print(f"  -> counter {'PASS' if c_ok else 'FAIL'}, checksum {'PASS' if k_ok else 'FAIL'}\n")
    ok &= c_ok and k_ok

    print("[selftest] ID 0x2C0 expect: counter byte0, checksum byte7 CRC-8/AUTOSAR")
    s = series[0x2C0]
    counters = detect_counters(s.payload)
    checks = detect_checksum(s.payload, {h.byte for h in counters})
    c_ok = any(h.byte == 0 for h in counters)
    k_ok = any(h.byte == 7 and h.algo == "CRC-8/AUTOSAR" and h.match >= 0.999 for h in checks)
    analyze_id(0x2C0, s)
    print(f"  -> counter {'PASS' if c_ok else 'FAIL'}, checksum {'PASS' if k_ok else 'FAIL'}\n")
    ok &= c_ok and k_ok

    print("[selftest]", "ALL PASSED" if ok else "SOME FAILED")
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Find the rolling counter and crack the checksum of a CAN message (read-only).",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--trace", help="CAN log (webCAN / candump / generic)")
    ap.add_argument("--id", help="target CAN ID (hex, e.g. 0x1A0); omit to scan all")
    ap.add_argument("--selftest", action="store_true", help="synthesize and verify (no hardware)")
    args = ap.parse_args(argv)

    if args.selftest:
        return _selftest()
    if not args.trace:
        ap.error("--trace is required (or use --selftest)")

    frames = load_log(args.trace)
    series = build_series(frames)
    if args.id:
        cid = int(args.id, 16)
        if cid not in series:
            print(f"ID 0x{cid:X} not in log.")
            return 2
        analyze_id(cid, series[cid])
    else:
        for cid in sorted(series):
            analyze_id(cid, series[cid])
    return 0


if __name__ == "__main__":
    sys.exit(main())
