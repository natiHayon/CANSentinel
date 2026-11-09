"""Default TX deny-list for all general-purpose transmit paths.

This mirrors the device TX-guard deny list (`_decisions.md` §5 / `HARDWARE_DESIGN.md`
§10.2): the unit never sends diagnostic/UDS or network-management frames, so digital
immobilization is done only by targeted application-frame injection. Enforcing the same
rule in the bench tool keeps it a faithful proxy of the device guard and inside the
default boundary. The user-requested fixed captured lock test in BusManager is
the sole bench exception: four constant 0x7E0 payloads, gated and one-shot.
It does not change the production device policy or enable arbitrary diagnostic TX.

The check is enforced at the single transmit chokepoint (`BusManager.send`) and again at
the TX-test gate (`TxTestEngine.validate_profile`). The captured test has its own gate.

Network-management IDs are vehicle-specific and their ranges overlap legitimate
application IDs on many buses, so NM is NOT hard-coded here: pass the profile's configured
NM ranges via ``nm_ranges`` when they are known.
"""
from __future__ import annotations

from typing import Iterable, Optional, Tuple

# 11-bit diagnostic (ISO 15765-4) request/response range.
DIAG_11BIT_LO = 0x700
DIAG_11BIT_HI = 0x7FF

# 29-bit UDS ranges, matched on the top two identifier bytes:
#   0x18DAxxxx = physical addressing, 0x18DBxxxx = functional addressing.
UDS_29BIT_MASK = 0x1FFF0000
UDS_29BIT_PHYS = 0x18DA0000
UDS_29BIT_FUNC = 0x18DB0000

NmRanges = Optional[Iterable[Tuple[int, int]]]


def is_denied(can_id: int, is_extended: bool, nm_ranges: NmRanges = None) -> bool:
    """True if this ID is denied on general-purpose transmit paths.

    ``nm_ranges`` is an optional iterable of inclusive ``(lo, hi)`` id ranges for
    vehicle-specific network-management IDs (from the profile).
    """
    if is_extended:
        if (can_id & UDS_29BIT_MASK) in (UDS_29BIT_PHYS, UDS_29BIT_FUNC):
            return True
    else:
        if DIAG_11BIT_LO <= can_id <= DIAG_11BIT_HI:
            return True
    if nm_ranges:
        for lo, hi in nm_ranges:
            if lo <= can_id <= hi:
                return True
    return False


def reason(can_id: int, is_extended: bool, nm_ranges: NmRanges = None) -> str:
    """A human-readable reason an id is denied (empty string if it is allowed)."""
    if is_extended and (can_id & UDS_29BIT_MASK) in (UDS_29BIT_PHYS, UDS_29BIT_FUNC):
        return "29-bit UDS id (0x18DA**/0x18DB**) is on the non-overridable deny list"
    if not is_extended and DIAG_11BIT_LO <= can_id <= DIAG_11BIT_HI:
        return "11-bit diagnostic id (0x700-0x7FF) is on the non-overridable deny list"
    if nm_ranges:
        for lo, hi in nm_ranges:
            if lo <= can_id <= hi:
                return "network-management id is on the deny list"
    return ""


# ---------------------------------------------------------------------------
# Standalone self-test:  python -m canlink.denylist
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    _fails = 0

    def check(label: str, ok: bool) -> None:
        global _fails
        print(f"{'PASS' if ok else 'FAIL'}: {label}")
        if not ok:
            _fails += 1

    check("0x7DF (11-bit diag) denied", is_denied(0x7DF, False))
    check("0x700 boundary denied", is_denied(0x700, False))
    check("0x7FF boundary denied", is_denied(0x7FF, False))
    check("0x6FF just below range allowed", not is_denied(0x6FF, False))
    check("0x333 defrost status allowed", not is_denied(0x333, False))
    check("0x18DAF110 (29-bit UDS phys) denied", is_denied(0x18DAF110, True))
    check("0x18DB33F1 (29-bit UDS func) denied", is_denied(0x18DB33F1, True))
    check("0x18FF0000 (other 29-bit) allowed", not is_denied(0x18FF0000, True))
    check("0x700 as 29-bit id allowed (not UDS range)", not is_denied(0x700, True))
    check("NM range honoured when configured", is_denied(0x510, False, nm_ranges=[(0x500, 0x5FF)]))
    check("outside NM range allowed", not is_denied(0x400, False, nm_ranges=[(0x500, 0x5FF)]))
    check("reason() names the diag rule", "deny list" in reason(0x7DF, False))
    check("reason() empty for allowed id", reason(0x333, False) == "")

    print("-" * 40)
    print("ALL PASS" if _fails == 0 else f"{_fails} FAILED")
    raise SystemExit(1 if _fails else 0)
