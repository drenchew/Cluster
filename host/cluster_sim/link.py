"""Serial link to the Arduino cluster driver.

Wire format must stay byte-for-byte identical to ``struct HostState`` in
``src/Protocol.h``:

    0xAA 0x55  <len=28>  <28 payload bytes>  <xor over len+payload>

The firmware owns CAN timing. We only push setpoints; if this process dies the
Arduino keeps the 0x130 heartbeat running and falls back to its own demo sweep,
so the cluster never resets just because the GUI closed.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass

try:
    import serial  # pyserial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - reported by run.py with install hint
    serial = None
    list_ports = None

SYNC0 = 0xAA
SYNC1 = 0x55

# rpm, speed, fuelDl, coolantC, outsideC, rangeKm,
# flags, lights, turn, gearPos, gearNum, gearMode, dimmer, pedalPct,
# flags2, hour, minute, second, day, month, year
_FMT = "<HHHhhHBBBBBBBBBBBBBBH"
PAYLOAD_LEN = struct.calcsize(_FMT)
assert PAYLOAD_LEN == 28, PAYLOAD_LEN

# HostFlags in src/Protocol.h
HF_KL15 = 0x01
HF_ENGINE = 0x02
HF_HANDBRAKE = 0x04
HF_REVERSE = 0x08

# HostFlags2 in src/Protocol.h. These three drive check-control symbols on
# 0x592 rather than bits in a telltale frame - the E9x cluster has no "light
# the DSC lamp" signal, so a numbered symbol is raised and cleared instead.
HF2_TC_LAMP = 0x01       # "DTC" traction-mode lamp
HF2_DSC_LAMP = 0x02      # DSC triangle - flashed while actually intervening
HF2_DSC_OFF = 0x04       # DSC switched off, solid
HF2_CLOCK = 0x08         # the clock fields are valid

# cfg::LIGHT_* in include/Config.h
LIGHT_SIDE = 0x01
LIGHT_HIGH = 0x02
LIGHT_LOW = 0x04
LIGHT_FOGF = 0x20
LIGHT_FOGR = 0x40

# GearPos in src/ClusterCan.h
GEAR_OFF, GEAR_P, GEAR_R, GEAR_N, GEAR_D = 0, 1, 2, 4, 8
MODE_NORMAL, MODE_SPORT, MODE_MANUAL = 0, 1, 2


@dataclass
class Setpoints:
    """One frame of cluster state, in physical units."""

    rpm: int = 0
    speed_kmh: int = 0
    fuel_l: float = 30.0
    coolant_c: int = 90
    outside_c: int = 20
    range_km: int = 500
    kl15: bool = True
    engine: bool = True
    handbrake: bool = False
    reverse: bool = False
    lights: int = LIGHT_SIDE | LIGHT_LOW
    turn_left: bool = False
    turn_right: bool = False
    gear_pos: int = GEAR_P
    gear_num: int = 0
    gear_mode: int = MODE_NORMAL
    dimmer: int = 200
    pedal_pct: int = 0

    # ---- DSC / traction control -------------------------------------------
    tc_lamp: bool = False      # traction mode (DTC) enabled
    dsc_lamp: bool = False     # intervening right now; flash this yourself
    dsc_off: bool = False      # stability control switched off

    # ---- clock -------------------------------------------------------------
    # None leaves the cluster's clock alone (the firmware keeps running
    # whatever it last had). A datetime - or anything with the usual
    # attributes, e.g. time.localtime() wrapped by from_struct_time - sets it.
    clock: object | None = None

    def pack(self) -> bytes:
        flags = (
            (HF_KL15 if self.kl15 else 0)
            | (HF_ENGINE if self.engine else 0)
            | (HF_HANDBRAKE if self.handbrake else 0)
            | (HF_REVERSE if self.reverse else 0)
        )
        flags2 = (
            (HF2_TC_LAMP if self.tc_lamp else 0)
            | (HF2_DSC_LAMP if self.dsc_lamp else 0)
            | (HF2_DSC_OFF if self.dsc_off else 0)
        )
        c = self.clock
        if c is None:
            hour = minute = second = 0
            day = month = 1
            year = 2000
        else:
            flags2 |= HF2_CLOCK
            hour, minute, second = c.hour, c.minute, c.second
            day, month, year = c.day, c.month, c.year

        turn = (0x01 if self.turn_left else 0) | (0x02 if self.turn_right else 0)
        payload = struct.pack(
            _FMT,
            _clamp(self.rpm, 0, 7000),
            _clamp(self.speed_kmh, 0, 400),
            _clamp(round(self.fuel_l * 10), 0, 800),
            _clamp(self.coolant_c, -40, 150),
            _clamp(self.outside_c, -40, 80),
            _clamp(self.range_km, 0, 2000),
            flags,
            self.lights & 0xFF,
            turn,
            self.gear_pos & 0xFF,
            _clamp(self.gear_num, 0, 8),
            self.gear_mode & 0xFF,
            _clamp(self.dimmer, 0, 255),
            _clamp(self.pedal_pct, 0, 100),
            flags2,
            _clamp(hour, 0, 23),
            _clamp(minute, 0, 59),
            _clamp(second, 0, 59),
            _clamp(day, 1, 31),
            _clamp(month, 1, 12),
            _clamp(year, 2000, 2200),
        )
        checksum = PAYLOAD_LEN
        for b in payload:
            checksum ^= b
        return bytes([SYNC0, SYNC1, PAYLOAD_LEN]) + payload + bytes([checksum])


def _clamp(v, lo, hi) -> int:
    return int(max(lo, min(hi, round(v))))


@dataclass
class Clock:
    """The fields ``Setpoints.clock`` needs, from whatever source.

    ``datetime.datetime`` already has these attributes, so the sim just passes
    one; this exists for the game clock, which is a count of in-game minutes
    and has no datetime to offer.
    """

    hour: int = 0
    minute: int = 0
    second: int = 0
    day: int = 1
    month: int = 1
    year: int = 2000

    @classmethod
    def from_struct_time(cls, t) -> "Clock":
        return cls(t.tm_hour, t.tm_min, t.tm_sec, t.tm_mday, t.tm_mon, t.tm_year)

    def hhmm(self) -> str:
        return f"{self.hour:02d}:{self.minute:02d}"


def available_ports() -> list[tuple[str, str]]:
    if list_ports is None:
        return []
    return [(p.device, p.description) for p in list_ports.comports()]


def guess_port() -> str | None:
    """Pick the most likely Arduino port.

    Prefers descriptions that name an Arduino or the usual USB-serial bridges
    (CH340/FTDI/CP210x clones are what most Uno boards actually ship with).
    """
    ports = available_ports()
    if not ports:
        return None
    hints = ("arduino", "ch340", "ch341", "usb-serial", "ftdi", "cp210", "usb serial")
    for device, desc in ports:
        if any(h in desc.lower() for h in hints):
            return device
    return ports[0][0]


class ClusterLink:
    """Non-blocking, self-reconnecting serial writer.

    Every failure path is swallowed on purpose: a bench rig should keep
    simulating and showing its dashboard with the cable unplugged, and the
    Arduino is designed to cope with us disappearing.
    """

    def __init__(self, port: str | None, baud: int = 115200, reconnect_s: float = 2.0):
        if serial is None:
            raise RuntimeError("pyserial is not installed - pip install pyserial")
        self.port = port
        self.baud = baud
        self.reconnect_s = reconnect_s
        self._ser: serial.Serial | None = None
        self._next_try = 0.0
        self.frames_sent = 0
        self.last_error: str | None = None

    @property
    def connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def _open(self) -> None:
        now = time.monotonic()
        if now < self._next_try:
            return
        self._next_try = now + self.reconnect_s
        port = self.port or guess_port()
        if port is None:
            self.last_error = "no serial ports found"
            return
        try:
            # The Uno resets when DTR is asserted on open, which costs about a
            # second of bootloader time. That is fine here - the cluster is
            # driven by whatever the firmware last had, and it comes back up
            # into the demo sweep.
            self._ser = serial.Serial(port, self.baud, timeout=0, write_timeout=0)
            self.port = port
            self.last_error = None
        except Exception as exc:  # pragma: no cover - hardware dependent
            self._ser = None
            self.last_error = f"{port}: {exc}"

    def send(self, sp: Setpoints) -> bool:
        if not self.connected:
            self._open()
            if not self.connected:
                return False
        try:
            self._ser.write(sp.pack())
            self.frames_sent += 1
            return True
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.last_error = str(exc)
            self.close()
            return False

    def read_text(self) -> str:
        """Drain whatever the firmware printed, for the GUI log pane."""
        if not self.connected:
            return ""
        try:
            n = self._ser.in_waiting
            if not n:
                return ""
            return self._ser.read(n).decode("ascii", "replace")
        except Exception:
            self.close()
            return ""

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
