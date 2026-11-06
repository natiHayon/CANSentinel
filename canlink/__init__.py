"""canlink - vendored live subset (CAN connection, frames, bus reader).

Only the read/capture layer is vendored into CANSentinel: the CanFrame value
object, the BusManager reader thread, the pluggable drivers (including a
hardware-free simulator), the TX denylist and JSON config storage. Capture
*analysis* stays in CANSentinel's own find_* tools.

This package imports with just the standard library - the Waveshare / python-can
drivers load their SDK or serial libraries lazily inside open(), so importing
canlink never requires hardware, and the simulator needs nothing at all.
"""
__all__ = ["frame", "bus", "storage", "drivers", "denylist"]
