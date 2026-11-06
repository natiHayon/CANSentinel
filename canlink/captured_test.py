"""Fixed, user-requested bench replay; never a general diagnostic TX exemption."""
from .frame import CanFrame

PROFILE = "igla_lock_capture_20261015_v1"
NAME = "IGLA lock — captured sequence test"
# Original request only, from action_20261015_150937.json. Sender and meaning
# remain inferred; recorded changing fields may no longer be accepted.
FRAMES = (
    (0.000, bytes.fromhex("FF FE 01 01 00 00 50 00")),
    (15.685, bytes.fromhex("FF FE 19 36 15 3A 50 01")),
    (19.966, bytes.fromhex("FF FE B5 9A 83 E9 50 02")),
    (52.766, bytes.fromhex("FF FE 01 01 00 00 50 03")),
)


def preview(profile):
    if profile != PROFILE:
        raise ValueError("Unknown captured test profile")
    return [{**CanFrame(0x7E0, data, direction="tx").to_dict(), "offset_ms": offset}
            for offset, data in FRAMES]


def mapping():
    return {"name": NAME, "kind": "actuator", "replay_profile": PROFILE,
            "can_id": 0x7E0, "id_hex": "0x7E0", "is_extended": False,
            "is_fd": False, "data": [], "actuation": "unverified"}
