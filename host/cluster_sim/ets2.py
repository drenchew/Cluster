"""ETS2 / ATS telemetry source for the cluster bench.

The in-game SCS SDK plugin (``scs-sdk-plugin`` and its forks) publishes the
whole ``scsTelemetryMap_t`` struct into one 32 KiB shared-memory block:

    Linux   /dev/shm/SCSTelemetry      (native Linux game build)
    Windows Local\\SCSTelemetry        (native Windows game build)

The struct layout below mirrors ``scs-telemetry/inc/scs-telemetry-common.hpp``
at PLUGIN_REVID 12, byte for byte, and ``verify_layout()`` re-checks the
documented zone offsets every time we load - a plugin that changes the
contract fails loudly instead of feeding garbage into the CAN bus.

Proton / Wine caveat: a Windows game build puts the mapping inside its Wine
prefix, where no native Linux process can see it. This reader only works with
the *native* Linux build of the game (or a bridge that copies the block into
/dev/shm). See README.

The cluster itself is a separate question from the game: everything the jig
wants (speed, rpm, fuel, coolant, gear, blinkers, park brake, lights) is in
this one struct, and :func:`to_setpoints` turns it into a :class:`Setpoints`
frame for :mod:`cluster_sim.link`.
"""

from __future__ import annotations

import ctypes
import glob
import math
import os
import sys
import time
from dataclasses import dataclass

from . import link as lk

MAP_SIZE = 32 * 1024          # SCS_PLUGIN_MMF_SIZE
SHM_NAME = "SCSTelemetry"     # POSIX name; Windows wants the Local\ prefix
BYTES_UPTO_ZONE10 = 4000      # offset of the first zone we treat as opaque

# The struct below mirrors PLUGIN_REVID 12. Revision 13 only appends a zone
# and keeps every revision-12 offset, so it reads fine too. Anything else may
# have moved a field under us - verify_layout() cannot catch that, because it
# only checks our own struct against our own constants. The plugin publishes
# its revision in the block, so check it at runtime instead.
EXPECTED_REVID = 12
SUPPORTED_REVIDS = (12, 13)

# The shared-memory object's name is NOT standard across plugin forks. The
# struct layout is the same everywhere, but the name is not:
#   truckermudgeon / upstream   /dev/shm/SCSTelemetry
#   ETS2LA's controller plugin  /dev/shm/SCS/...
#   SimuLogistics fork          /dev/shm/SimuLogisticsTelemetry
# So try the known ones, then fall back to scanning for anything that is the
# right size and looks like telemetry. --shm-name overrides all of it.
SHM_CANDIDATES = (
    "SCSTelemetry",
    "SCS/SCSTelemetry",
    "SimuLogisticsTelemetry",
    "TSGPSTelemetry",
)
SHM_GLOBS = ("/dev/shm/*[Tt]elemetry*", "/dev/shm/SCS/*")

if sys.byteorder != "little":  # pragma: no cover - no BE targets exist
    raise RuntimeError("ETS2 shared memory is little-endian only")


# ---------------------------------------------------------------------------
# scsTelemetryMap_t, zones 1..9 verbatim; everything after offset 4000 is
# job/trailer/substance data this jig never reads, so it stays a byte tail.
# ---------------------------------------------------------------------------

class _ScsValues(ctypes.Structure):
    _fields_ = [
        ("telemetry_plugin_revision", ctypes.c_uint),
        ("version_major", ctypes.c_uint),
        ("version_minor", ctypes.c_uint),
        ("game", ctypes.c_uint),
        ("telemetry_version_game_major", ctypes.c_uint),
        ("telemetry_version_game_minor", ctypes.c_uint),
    ]


class _CommonUi(ctypes.Structure):
    _fields_ = [("time_abs", ctypes.c_uint)]


class _ConfigUi(ctypes.Structure):
    _fields_ = [
        ("gears", ctypes.c_uint),
        ("gears_reverse", ctypes.c_uint),
        ("retarderStepCount", ctypes.c_uint),
        ("truckWheelCount", ctypes.c_uint),
        ("selectorCount", ctypes.c_uint),
        ("time_abs_delivery", ctypes.c_uint),
        ("maxTrailerCount", ctypes.c_uint),
        ("unitCount", ctypes.c_uint),
        ("plannedDistanceKm", ctypes.c_uint),
    ]


class _TruckUi(ctypes.Structure):
    _fields_ = [
        ("shifterSlot", ctypes.c_uint),
        ("retarderBrake", ctypes.c_uint),
        ("lightsAuxFront", ctypes.c_uint),
        ("lightsAuxRoof", ctypes.c_uint),
        ("truck_wheelSubstance", ctypes.c_uint * 16),
        ("hshifterPosition", ctypes.c_uint * 32),
        ("hshifterBitmask", ctypes.c_uint * 32),
    ]


class _GameplayUi(ctypes.Structure):
    _fields_ = [
        ("jobDeliveredDeliveryTime", ctypes.c_uint),
        ("jobStartingTime", ctypes.c_uint),
        ("jobFinishedTime", ctypes.c_uint),
    ]


class _CommonI(ctypes.Structure):
    _fields_ = [("restStop", ctypes.c_int)]


class _TruckI(ctypes.Structure):
    _fields_ = [
        ("gear", ctypes.c_int),              # >0 forward, <0 reverse, 0 N
        ("gearDashboard", ctypes.c_int),
        ("hshifterResulting", ctypes.c_int * 32),
    ]


class _GameplayI(ctypes.Structure):
    _fields_ = [("jobDeliveredEarnedXp", ctypes.c_int)]


class _CommonF(ctypes.Structure):
    _fields_ = [("scale", ctypes.c_float)]


class _ConfigF(ctypes.Structure):
    _fields_ = [
        ("fuelCapacity", ctypes.c_float),
        ("fuelWarningFactor", ctypes.c_float),
        ("adblueCapacity", ctypes.c_float),
        ("adblueWarningFactor", ctypes.c_float),
        ("airPressureWarning", ctypes.c_float),
        ("airPressurEmergency", ctypes.c_float),
        ("oilPressureWarning", ctypes.c_float),
        ("waterTemperatureWarning", ctypes.c_float),
        ("batteryVoltageWarning", ctypes.c_float),
        ("engineRpmMax", ctypes.c_float),
        ("gearDifferential", ctypes.c_float),
        ("cargoMass", ctypes.c_float),
        ("truckWheelRadius", ctypes.c_float * 16),
        ("gearRatiosForward", ctypes.c_float * 24),
        ("gearRatiosReverse", ctypes.c_float * 8),
        ("unitMass", ctypes.c_float),
    ]


class _TruckF(ctypes.Structure):
    _fields_ = [
        ("speed", ctypes.c_float),           # m/s
        ("engineRpm", ctypes.c_float),
        ("userSteer", ctypes.c_float),
        ("userThrottle", ctypes.c_float),
        ("userBrake", ctypes.c_float),
        ("userClutch", ctypes.c_float),
        ("gameSteer", ctypes.c_float),
        ("gameThrottle", ctypes.c_float),
        ("gameBrake", ctypes.c_float),
        ("gameClutch", ctypes.c_float),
        ("cruiseControlSpeed", ctypes.c_float),
        ("airPressure", ctypes.c_float),
        ("brakeTemperature", ctypes.c_float),
        ("fuel", ctypes.c_float),            # litres
        ("fuelAvgConsumption", ctypes.c_float),
        ("fuelRange", ctypes.c_float),
        ("adblue", ctypes.c_float),
        ("oilPressure", ctypes.c_float),
        ("oilTemperature", ctypes.c_float),
        ("waterTemperature", ctypes.c_float),
        ("batteryVoltage", ctypes.c_float),
        ("lightsDashboard", ctypes.c_float),
        ("wearEngine", ctypes.c_float),
        ("wearTransmission", ctypes.c_float),
        ("wearCabin", ctypes.c_float),
        ("wearChassis", ctypes.c_float),
        ("wearWheels", ctypes.c_float),
        ("truckOdometer", ctypes.c_float),   # metres
        ("routeDistance", ctypes.c_float),
        ("routeTime", ctypes.c_float),
        ("speedLimit", ctypes.c_float),
        ("truck_wheelSuspDeflection", ctypes.c_float * 16),
        ("truck_wheelVelocity", ctypes.c_float * 16),
        ("truck_wheelSteering", ctypes.c_float * 16),
        ("truck_wheelRotation", ctypes.c_float * 16),
        ("truck_wheelLift", ctypes.c_float * 16),
        ("truck_wheelLiftOffset", ctypes.c_float * 16),
    ]


class _GameplayF(ctypes.Structure):
    _fields_ = [
        ("jobDeliveredCargoDamage", ctypes.c_float),
        ("jobDeliveredDistanceKm", ctypes.c_float),
        ("refuelAmount", ctypes.c_float),
    ]


class _JobF(ctypes.Structure):
    _fields_ = [("cargoDamage", ctypes.c_float)]


class _ConfigB(ctypes.Structure):
    _fields_ = [
        ("truckWheelSteerable", ctypes.c_bool * 16),
        ("truckWheelSimulated", ctypes.c_bool * 16),
        ("truckWheelPowered", ctypes.c_bool * 16),
        ("truckWheelLiftable", ctypes.c_bool * 16),
        ("isCargoLoaded", ctypes.c_bool),
        ("specialJob", ctypes.c_bool),
    ]


class _TruckB(ctypes.Structure):
    _fields_ = [
        ("parkBrake", ctypes.c_bool),
        ("motorBrake", ctypes.c_bool),
        ("airPressureWarning", ctypes.c_bool),
        ("airPressureEmergency", ctypes.c_bool),
        ("fuelWarning", ctypes.c_bool),
        ("adblueWarning", ctypes.c_bool),
        ("oilPressureWarning", ctypes.c_bool),
        ("waterTemperatureWarning", ctypes.c_bool),
        ("batteryVoltageWarning", ctypes.c_bool),
        ("electricEnabled", ctypes.c_bool),
        ("engineEnabled", ctypes.c_bool),
        ("wipers", ctypes.c_bool),
        ("blinkerLeftActive", ctypes.c_bool),    # logical enable -> cluster blinks
        ("blinkerRightActive", ctypes.c_bool),
        ("blinkerLeftOn", ctypes.c_bool),        # lamp state, 2 Hz in game
        ("blinkerRightOn", ctypes.c_bool),
        ("lightsParking", ctypes.c_bool),
        ("lightsBeamLow", ctypes.c_bool),
        ("lightsBeamHigh", ctypes.c_bool),
        ("lightsBeacon", ctypes.c_bool),
        ("lightsBrake", ctypes.c_bool),
        ("lightsReverse", ctypes.c_bool),
        ("lightsHazard", ctypes.c_bool),
        ("cruiseControl", ctypes.c_bool),
        ("truck_wheelOnGround", ctypes.c_bool * 16),
        ("shifterToggle", ctypes.c_bool * 2),
        ("differentialLock", ctypes.c_bool),
        ("liftAxle", ctypes.c_bool),
        ("liftAxleIndicator", ctypes.c_bool),
        ("trailerLiftAxle", ctypes.c_bool),
        ("trailerLiftAxleIndicator", ctypes.c_bool),
    ]


class _GameplayB(ctypes.Structure):
    _fields_ = [
        ("jobDeliveredAutoparkUsed", ctypes.c_bool),
        ("jobDeliveredAutoloadUsed", ctypes.c_bool),
    ]


class _ConfigFv(ctypes.Structure):
    _fields_ = [
        ("cabinPositionX", ctypes.c_float), ("cabinPositionY", ctypes.c_float),
        ("cabinPositionZ", ctypes.c_float),
        ("headPositionX", ctypes.c_float), ("headPositionY", ctypes.c_float),
        ("headPositionZ", ctypes.c_float),
        ("truckHookPositionX", ctypes.c_float),
        ("truckHookPositionY", ctypes.c_float),
        ("truckHookPositionZ", ctypes.c_float),
        ("truckWheelPositionX", ctypes.c_float * 16),
        ("truckWheelPositionY", ctypes.c_float * 16),
        ("truckWheelPositionZ", ctypes.c_float * 16),
    ]


class _TruckFv(ctypes.Structure):
    _fields_ = [
        ("lv_accelerationX", ctypes.c_float), ("lv_accelerationY", ctypes.c_float),
        ("lv_accelerationZ", ctypes.c_float),
        ("av_accelerationX", ctypes.c_float), ("av_accelerationY", ctypes.c_float),
        ("av_accelerationZ", ctypes.c_float),
        ("accelerationX", ctypes.c_float), ("accelerationY", ctypes.c_float),
        ("accelerationZ", ctypes.c_float),
        ("aa_accelerationX", ctypes.c_float), ("aa_accelerationY", ctypes.c_float),
        ("aa_accelerationZ", ctypes.c_float),
        ("cabinAVX", ctypes.c_float), ("cabinAVY", ctypes.c_float),
        ("cabinAVZ", ctypes.c_float),
        ("cabinAAX", ctypes.c_float), ("cabinAAY", ctypes.c_float),
        ("cabinAAZ", ctypes.c_float),
    ]


class _TruckFp(ctypes.Structure):
    _fields_ = [
        ("cabinOffsetX", ctypes.c_float), ("cabinOffsetY", ctypes.c_float),
        ("cabinOffsetZ", ctypes.c_float),
        ("cabinOffsetrotationX", ctypes.c_float),
        ("cabinOffsetrotationY", ctypes.c_float),
        ("cabinOffsetrotationZ", ctypes.c_float),
        ("headOffsetX", ctypes.c_float), ("headOffsetY", ctypes.c_float),
        ("headOffsetZ", ctypes.c_float),
        ("headOffsetrotationX", ctypes.c_float),
        ("headOffsetrotationY", ctypes.c_float),
        ("headOffsetrotationZ", ctypes.c_float),
    ]


class _TruckDp(ctypes.Structure):
    _fields_ = [
        ("coordinateX", ctypes.c_double), ("coordinateY", ctypes.c_double),
        ("coordinateZ", ctypes.c_double),
        ("rotationX", ctypes.c_double), ("rotationY", ctypes.c_double),
        ("rotationZ", ctypes.c_double),
    ]


_S64 = ctypes.c_char * 64


class _ConfigS(ctypes.Structure):
    _fields_ = [
        ("truckBrandId", _S64), ("truckBrand", _S64), ("truckId", _S64),
        ("truckName", _S64), ("cargoId", _S64), ("cargo", _S64),
        ("cityDstId", _S64), ("cityDst", _S64), ("compDstId", _S64),
        ("compDst", _S64), ("citySrcId", _S64), ("citySrc", _S64),
        ("compSrcId", _S64), ("compSrc", _S64),
        ("shifterType", ctypes.c_char * 16),
        ("truckLicensePlate", _S64),
        ("truckLicensePlateCountryId", _S64),
        ("truckLicensePlateCountry", _S64),
        ("jobMarket", ctypes.c_char * 32),
    ]


class _GameplayS(ctypes.Structure):
    _fields_ = [
        ("fineOffence", ctypes.c_char * 32),
        ("ferrySourceName", _S64), ("ferryTargetName", _S64),
        ("ferrySourceId", _S64), ("ferryTargetId", _S64),
        ("trainSourceName", _S64), ("trainTargetName", _S64),
        ("trainSourceId", _S64), ("trainTargetId", _S64),
    ]


class ScsTelemetry(ctypes.Structure):
    """The shared-memory map, zones 1..9 exact plus an opaque tail."""

    _fields_ = [
        ("sdkActive", ctypes.c_bool),
        ("placeHolder", ctypes.c_char * 3),
        ("paused", ctypes.c_bool),
        ("placeHolder2", ctypes.c_char * 3),
        ("time", ctypes.c_ulonglong),            # frame timestamp, advances live
        ("simulatedTime", ctypes.c_ulonglong),
        ("renderTime", ctypes.c_ulonglong),
        ("multiplayerTimeOffset", ctypes.c_longlong),

        ("scs_values", _ScsValues),
        ("common_ui", _CommonUi),
        ("config_ui", _ConfigUi),
        ("truck_ui", _TruckUi),
        ("gameplay_ui", _GameplayUi),
        ("buffer_ui", ctypes.c_char * 48),

        ("common_i", _CommonI),
        ("truck_i", _TruckI),
        ("gameplay_i", _GameplayI),
        ("buffer_i", ctypes.c_char * 56),

        ("common_f", _CommonF),
        ("config_f", _ConfigF),
        ("truck_f", _TruckF),
        ("gameplay_f", _GameplayF),
        ("job_f", _JobF),
        ("buffer_f", ctypes.c_char * 28),

        ("config_b", _ConfigB),
        ("truck_b", _TruckB),
        ("gameplay_b", _GameplayB),
        ("buffer_b", ctypes.c_char * 25),

        ("config_fv", _ConfigFv),
        ("truck_fv", _TruckFv),
        ("buffer_fv", ctypes.c_char * 60),

        ("truck_fp", _TruckFp),
        ("buffer_fp", ctypes.c_char * 152),

        ("truck_dp", _TruckDp),
        ("buffer_dp", ctypes.c_char * 52),

        ("config_s", _ConfigS),
        ("gameplay_s", _GameplayS),
        ("buffer_s", ctypes.c_char * 20),

        ("tail", ctypes.c_ubyte * (MAP_SIZE - BYTES_UPTO_ZONE10)),
    ]


_OFFSETS = {
    "time": 8,
    "scs_values": 40,
    "truck_i": 504,
    "truck_i.gear": 504,
    "config_f": 704,
    "config_f.fuelCapacity": 704,
    "truck_f": 948,
    "truck_f.speed": 948,
    "truck_f.engineRpm": 952,
    "truck_f.fuel": 1000,
    "truck_f.waterTemperature": 1024,
    "truck_f.truckOdometer": 1056,
    "config_b": 1500,
    "truck_b": 1566,
    "truck_b.parkBrake": 1566,
    "truck_b.engineEnabled": 1576,
    "truck_b.blinkerLeftActive": 1578,
    "truck_b.lightsBeamLow": 1583,
    "config_s": 2300,
}


def _offset(name: str) -> int:
    """Byte offset of ``name`` (``truck_f.speed`` style) in the map."""
    cls = ScsTelemetry
    parts = name.split(".")
    total = 0
    for i, part in enumerate(parts):
        total += getattr(cls, part).offset
        if i < len(parts) - 1:
            cls = dict(cls._fields_)[part]
    return total


def verify_layout() -> None:
    """Fail loudly if the plugin's struct contract moved under us."""
    if ctypes.sizeof(ScsTelemetry) != MAP_SIZE:
        raise RuntimeError(
            f"scsTelemetryMap_t is {ctypes.sizeof(ScsTelemetry)} bytes, "
            f"expected {MAP_SIZE} - check scs-telemetry-common.hpp")
    bad = [f"{k} at {_offset(k)} (want {v})"
           for k, v in _OFFSETS.items() if _offset(k) != v]
    if bad:
        raise RuntimeError("shared-memory layout mismatch: " + "; ".join(bad))


# ---------------------------------------------------------------------------
# Sources
# ---------------------------------------------------------------------------

class ShmSource:
    """Poll the game's shared-memory block.

    Returns ``None`` from :meth:`poll` while the game is not running, has not
    produced a telemetry frame for ``stale_s``, or the mapping does not exist
    yet. The firmware's own host watchdog covers the gap: stop sending for
    1.5 s and the cluster falls back to its demo sweep instead of holding
    whatever we last pushed.
    """

    def __init__(self, stale_s: float = 0.75, name: str | None = None):
        verify_layout()
        self._want = name
        self._stale_s = stale_s
        self._mm = None
        self._next_open = 0.0
        self._last_stamp = -1
        self._last_move = 0.0
        self.name: str | None = None      # whichever candidate actually opened
        self.last_error: str | None = None
        self.revid: int | None = None     # plugin revision, once we have seen one
        self.revid_warning: str | None = None

    # -- plumbing ----------------------------------------------------------
    def _open(self) -> None:
        now = time.monotonic()
        if now < self._next_open:
            return
        self._next_open = now + 1.0
        tried = []
        for candidate in shm_candidates(self._want):
            try:
                self._mm = _open_mapping(candidate)
            except (FileNotFoundError, OSError) as exc:
                tried.append(f"{candidate}: {exc.__class__.__name__}")
                continue
            self.name = candidate
            self.last_error = None
            self._last_stamp = -1
            self._last_move = 0.0
            return
        self._mm = None
        self.name = None
        self.last_error = ("no telemetry block found - tried "
                           + ", ".join(t.split(":")[0] for t in tried))

    def close(self) -> None:
        if self._mm is not None:
            self._mm.close()
            self._mm = None

    # -- data --------------------------------------------------------------
    def poll(self) -> ScsTelemetry | None:
        if self._mm is None:
            self._open()
            if self._mm is None:
                return None
        try:
            raw = bytes(self._mm[:MAP_SIZE])
        except (OSError, ValueError) as exc:      # game exited, block gone
            self.close()
            self.last_error = str(exc)
            return None
        if len(raw) < BYTES_UPTO_ZONE10:
            self.close()
            self.last_error = f"short mapping ({len(raw)} bytes)"
            return None
        if len(raw) < MAP_SIZE:
            # Everything this jig reads lives in the first 4000 bytes, but
            # from_buffer_copy demands the full 32768 and raises ValueError
            # otherwise - which took down the whole runner. Pad the tail.
            raw = raw.ljust(MAP_SIZE, b"\x00")

        frame = ScsTelemetry.from_buffer_copy(raw)
        stamp = int(frame.time)
        if stamp != self._last_stamp:
            self._last_stamp = stamp
            self._last_move = time.monotonic()
        if not frame.sdkActive or (time.monotonic() - self._last_move) > self._stale_s:
            return None
        self._check_revid(frame)
        return frame

    def _check_revid(self, frame: ScsTelemetry) -> None:
        """Compare the plugin's own revision against what this struct mirrors."""
        rev = int(frame.scs_values.telemetry_plugin_revision)
        if rev == self.revid:
            return
        self.revid = rev
        if rev in SUPPORTED_REVIDS:
            self.revid_warning = None
        else:
            self.revid_warning = (
                f"plugin reports revision {rev}, this reader mirrors "
                f"{EXPECTED_REVID} - fields may be misaligned. Use a "
                f"v1.12.x plugin, or check scs-telemetry-common.hpp.")


class MockSource:
    """A scripted drive, so the bench can be exercised with no game running.

    45-second loop: 6 s parked with the handbrake on, 8 s pulling away through
    the gears, cruising at 90 km/h with the left indicator on from 18-26 s,
    then braking back to a stop. Fuel drains and coolant warms as it goes.
    """

    def __init__(self):
        verify_layout()
        self._t0 = time.monotonic()
        self.last_error: str | None = None

    def close(self) -> None:
        pass

    def poll(self) -> ScsTelemetry:
        t = (time.monotonic() - self._t0) % 45.0
        f = ScsTelemetry()
        f.sdkActive = True
        f.time = int((time.monotonic() - self._t0) * 1000)
        f.scs_values.game = 1

        cruise = 90.0 / 3.6
        if t < 6.0:                                   # parked
            speed, gear, park = 0.0, 0, True
        elif t < 14.0:                                # pulling away
            speed, gear, park = cruise * (t - 6.0) / 8.0, min(6, int((t - 6.0)) + 1), False
        elif t < 38.0:                                # cruising
            speed, gear, park = cruise, 6, False
        else:                                         # braking to a stop
            speed, gear, park = max(0.0, cruise * (42.0 - t) / 4.0), 2, False

        f.truck_f.speed = speed
        f.truck_f.engineRpm = 600.0 + speed * 45.0
        f.truck_f.gameThrottle = 0.0 if speed < 1.0 else 0.55
        f.truck_f.fuel = 38.0 - t * 0.02
        f.truck_f.fuelRange = 420.0
        f.truck_f.waterTemperature = 72.0 + min(20.0, t * 0.45)
        f.truck_f.truckOdometer = 128_540.0
        f.config_f.fuelCapacity = 60.0
        f.truck_i.gear = gear
        f.truck_b.parkBrake = park
        f.truck_b.electricEnabled = True
        f.truck_b.engineEnabled = True
        f.truck_b.lightsParking = True
        f.truck_b.lightsBeamLow = True
        f.truck_b.blinkerLeftActive = 18.0 <= t < 26.0

        # Enough wheel data for wheel_slip() to work on, with a deliberate
        # slip while pulling away so the DSC lamp has something to do.
        f.config_ui.truckWheelCount = 6
        spin = 1.35 if 6.5 < t < 9.0 else 1.0
        for i in range(6):
            f.config_f.truckWheelRadius[i] = 0.5
            f.config_b.truckWheelPowered[i] = i in (2, 3)
            rot = speed / 0.5
            f.truck_f.truck_wheelVelocity[i] = rot * (spin if i in (2, 3) else 1.0)
        f.truck_b.differentialLock = 10.0 <= t < 13.0

        # Game clock: start at 06:00 and run an in-game minute a second, which
        # is roughly the game's own rate.
        f.common_ui.time_abs = 6 * 60 + int(time.monotonic() - self._t0)
        return f


def shm_candidates(name: str | None = None) -> list[str]:
    """Names to try, best first.

    An explicit name wins outright. Otherwise the known fork names come first,
    then anything in /dev/shm that is plausibly a telemetry block - the name
    is not standardised across plugin forks, so guessing one and failing
    silently is the worst outcome.
    """
    if name:
        return [name]
    out = list(SHM_CANDIDATES)
    if not sys.platform.startswith("win"):
        for pattern in SHM_GLOBS:
            for path in sorted(glob.glob(pattern)):
                try:
                    if os.path.getsize(path) < BYTES_UPTO_ZONE10:
                        continue
                except OSError:
                    continue
                found = os.path.relpath(path, "/dev/shm")
                if found not in out:
                    out.append(found)
    return out


def list_shm() -> list[tuple[str, int]]:
    """Everything in /dev/shm with its size, for --probe."""
    if sys.platform.startswith("win"):
        return []
    found = []
    for path in sorted(glob.glob("/dev/shm/*") + glob.glob("/dev/shm/*/*")):
        try:
            found.append((os.path.relpath(path, "/dev/shm"), os.path.getsize(path)))
        except OSError:
            pass
    return found


# ---------------------------------------------------------------------------
# Steam install discovery - "where do I put the plugin, and will it work?"
# ---------------------------------------------------------------------------

# Steam can live in several places, and which one it is matters: the Flatpak
# build sandboxes /dev/shm, so a telemetry block created inside it is NOT
# visible to this process without an explicit override.
STEAM_ROOTS = (
    ("native", "~/.local/share/Steam"),
    ("native", "~/.steam/steam"),
    ("native", "~/.steam/root"),
    ("native", "~/.steam/debian-installation"),
    ("flatpak", "~/.var/app/com.valvesoftware.Steam/.local/share/Steam"),
    ("snap", "~/snap/steam/common/.local/share/Steam"),
    ("windows", "C:/Program Files (x86)/Steam"),
)

GAME_DIRS = (
    ("ETS2", "Euro Truck Simulator 2"),
    ("ATS", "American Truck Simulator"),
)

FLATPAK_SHM_FIX = (
    "flatpak override --user --device=shm com.valvesoftware.Steam")


def _vdf_paths(vdf_text: str) -> list[str]:
    """Pull every "path" value out of libraryfolders.vdf.

    A full VDF parser is overkill - the file's only interesting content here
    is the library roots, and they are all plain quoted strings.
    """
    import re
    return re.findall(r'"path"\s*"([^"]+)"', vdf_text)


def steam_libraries() -> list[tuple[str, str]]:
    """(kind, path) for every Steam library root we can find."""
    seen, out = set(), []
    for kind, root in STEAM_ROOTS:
        base = os.path.expanduser(root)
        if not os.path.isdir(base):
            continue
        for lib in [base] + _extra_libraries(base):
            real = os.path.realpath(lib)
            if real not in seen and os.path.isdir(os.path.join(real, "steamapps")):
                seen.add(real)
                out.append((kind, real))
    return out


def _extra_libraries(base: str) -> list[str]:
    vdf = os.path.join(base, "steamapps", "libraryfolders.vdf")
    try:
        with open(vdf, "r", encoding="utf-8", errors="replace") as fh:
            return _vdf_paths(fh.read())
    except OSError:
        return []


def find_games() -> list[dict]:
    """Locate ETS2/ATS installs and report whether each can actually work."""
    found = []
    for kind, lib in steam_libraries():
        for tag, folder in GAME_DIRS:
            path = os.path.join(lib, "steamapps", "common", folder)
            if not os.path.isdir(path):
                continue
            native = os.path.join(path, "bin", "linux_x64")
            win = os.path.join(path, "bin", "win_x64")
            has_native = os.path.isdir(native) and bool(os.listdir(native))
            plug_dir = os.path.join(native if has_native else win, "plugins")
            plugins = []
            if os.path.isdir(plug_dir):
                plugins = [p for p in sorted(os.listdir(plug_dir))
                           if p.endswith((".so", ".dll"))]
            found.append({
                "game": tag,
                "path": path,
                "steam": kind,
                "has_native": has_native,
                "has_windows": os.path.isdir(win) and bool(os.listdir(win)),
                "plugin_dir": plug_dir,
                "plugins": plugins,
            })
    return found


def setup_report() -> int:
    """Print what is installed, where the plugin goes, and what will break.

    Returns the number of problems that would stop telemetry working.
    """
    problems = 0
    libs = steam_libraries()
    if not libs:
        print("No Steam install found in any of the usual places:")
        for _, root in STEAM_ROOTS:
            print(f"    {root}")
        print("  Pass --shm-name once the game is running, or install Steam.")
        return 1

    print("Steam libraries:")
    for kind, lib in libs:
        print(f"  [{kind}] {lib}")

    games = find_games()
    if not games:
        print("\nNo ETS2 or ATS install found. Install it from Steam first.")
        return 1

    for g in games:
        print(f"\n{g['game']}  ({g['steam']} Steam)")
        print(f"  install   {g['path']}")

        if g["has_native"]:
            print("  build     native Linux  (good)")
        elif g["has_windows"]:
            problems += 1
            print("  build     WINDOWS ONLY - telemetry cannot work")
            print("            bin/linux_x64 is missing or empty, so Steam is")
            print("            running this under Proton. The shared memory")
            print("            then lives inside the Wine prefix where this")
            print("            process cannot see it.")
            print("            Fix: Steam -> right-click the game -> Properties")
            print("            -> Compatibility -> UNTICK 'Force the use of a")
            print("            specific Steam Play compatibility tool', then let")
            print("            Steam re-download the Linux binaries.")
        else:
            problems += 1
            print("  build     no binaries found - verify the game files")

        print(f"  plugins   {g['plugin_dir']}")
        if not os.path.isdir(g["plugin_dir"]):
            print("            (does not exist yet - create it)")
            print(f"            mkdir -p '{g['plugin_dir']}'")
        if g["plugins"]:
            print(f"  installed {', '.join(g['plugins'])}")
        else:
            problems += 1
            print("  installed NOTHING - copy the SCS SDK plugin .so here:")
            print(f"            cp scs-telemetry.so '{g['plugin_dir']}/'")

        if g["steam"] == "flatpak":
            problems += 1
            print("  WARNING   Flatpak Steam sandboxes /dev/shm, so the game's")
            print("            telemetry block is invisible to this process.")
            print(f"            Fix: {FLATPAK_SHM_FIX}")
            print("            then fully restart Steam.")
        elif g["steam"] == "snap":
            print("  NOTE      Snap Steam may also confine /dev/shm; if --probe")
            print("            shows nothing while the game runs, that is why.")

    print("\nTelemetry blocks visible right now:")
    entries = list_shm()
    if not entries:
        print("  none (start the game, then re-run --probe)")
    for name, size in entries:
        mark = "  <- right size" if size >= BYTES_UPTO_ZONE10 else ""
        print(f"  {size:>9}  {name}{mark}")

    print(f"\n{problems} problem(s) to fix" if problems else
          "\nEverything looks ready.")
    return problems


def _open_mapping(name: str):
    """Open the game's mapping read-only; FileNotFoundError if no game."""
    if sys.platform.startswith("win"):
        import mmap

        k32 = ctypes.windll.kernel32
        # Declare the signatures: ctypes defaults a return to c_int, which
        # truncates a 64-bit HANDLE on Win64 and can turn a valid handle into
        # a bogus one (or a false "not found").
        k32.OpenFileMappingW.restype = ctypes.c_void_p
        k32.OpenFileMappingW.argtypes = (ctypes.c_uint32, ctypes.c_int,
                                         ctypes.c_wchar_p)
        k32.CloseHandle.restype = ctypes.c_int
        k32.CloseHandle.argtypes = (ctypes.c_void_p,)

        FILE_MAP_READ = 0x0004
        full = name if "\\" in name else f"Local\\{name}"
        # Probe without creating: mmap's tagname path would happily mint a
        # fresh all-zero block when the game is not running, which is exactly
        # the state we must not mistake for "game present but not framing".
        probe = k32.OpenFileMappingW(FILE_MAP_READ, 0, full)
        if not probe:
            raise FileNotFoundError(f"no mapping {full} (is the game running?)")
        k32.CloseHandle(probe)
        # Python's mmap() takes a *file* handle on Windows, not a mapping
        # handle (it calls CreateFileMapping itself), so reopen by name.
        return mmap.mmap(-1, MAP_SIZE, tagname=full, access=mmap.ACCESS_READ)
    else:
        import mmap as mmap_mod

        path = name if name.startswith("/") else f"/dev/shm/{name}"
        fd = os.open(path, os.O_RDONLY)
        try:
            size = os.fstat(fd).st_size
            if size < BYTES_UPTO_ZONE10:
                raise OSError(f"{path} is only {size} bytes")
            return mmap_mod.mmap(fd, min(size, MAP_SIZE), prot=mmap_mod.PROT_READ)
        finally:
            os.close(fd)


# ---------------------------------------------------------------------------
# Mapping game -> cluster
# ---------------------------------------------------------------------------

# ---- traction control -----------------------------------------------------
# The game publishes each wheel's angular velocity and radius, so the slip of
# the driven wheels can be measured the way a real DSC does rather than
# guessed from the throttle: surface speed against ground speed.
TC_SLIP_ON = 0.08          # fraction of slip that counts as an intervention
TC_SLIP_FULL = 0.35        # and where it is unambiguous
TC_MIN_THROTTLE = 0.05     # a locked wheel under braking is not wheelspin


@dataclass
class MapOptions:
    """How much of the truck to squeeze onto a car cluster."""

    rpm_scale: float = 1.0     # >1 spreads a 2100 rpm truck engine over the dial
    speed_scale: float = 1.0
    tank_l: float = 60.0       # car-tank size the fuel gauge is calibrated for
    lights_on: bool = False    # force side + low beam regardless of the game
    # "game" puts the in-game clock on the cluster (what you want while
    # driving a 03:00 delivery), "system" the PC's, "off" leaves the cluster's
    # own clock running.
    clock: str = "game"


def wheel_slip(f: ScsTelemetry) -> float:
    """Worst driven-wheel slip, 0 = hooked up.

    Surface speed of a powered wheel against the truck's ground speed. The
    denominator is floored so a launch from rest - where the ground speed is
    near zero and any wheel rotation is infinite slip - gives a usable number
    instead of exploding.

    ``truck.wheel.angular_velocity`` is in *rotations per second*, not
    radians - the SDK header says so and the game agrees: a free-rolling
    0.506 m wheel at 91.6 km/h reports 8.005, and 25.44 / (2*pi * 0.506) is
    8.00. So one turn of the wheel covers its circumference, 2*pi*r, and the
    2*pi here is not optional. Leaving it out scales surface speed to 16% of
    the truth, which makes slip permanently negative, which clamps to zero -
    the lamp simply never lights, with nothing to show you why.
    """
    speed = abs(_num(f.truck_f.speed))
    count = int(f.config_ui.truckWheelCount or 0)
    worst = 0.0
    for i in range(min(16, count if count > 0 else 16)):
        if not f.config_b.truckWheelPowered[i]:
            continue
        radius = _num(f.config_f.truckWheelRadius[i])
        if radius <= 0.05:
            continue
        rev_per_s = abs(_num(f.truck_f.truck_wheelVelocity[i]))
        surface = rev_per_s * 2.0 * math.pi * radius
        worst = max(worst, (surface - speed) / max(speed, 2.0))
    return max(0.0, worst)


def game_clock(f: ScsTelemetry) -> lk.Clock:
    """The in-game time of day, with today's date for the fields it lacks.

    ``common_ui.time_abs`` is minutes on the game's own calendar, which does
    not line up with any real year - so the hour and minute come from the game
    and the date from the host. The cluster shows a clock, not a calendar.
    """
    minutes = int(f.common_ui.time_abs) % (24 * 60)
    now = time.localtime()
    return lk.Clock(hour=minutes // 60, minute=minutes % 60, second=0,
                    day=now.tm_mday, month=now.tm_mon, year=now.tm_year)


def _num(x, default: float = 0.0) -> float:
    """A float from the map that is safe to round.

    The game publishes NaN and inf in places - fuelRange before a route is
    set, any float during a load screen - and ``int(round(nan))`` raises
    ValueError, which killed the runner mid-drive. Everything read out of the
    shared block goes through here.
    """
    try:
        v = float(x)
    except (TypeError, ValueError):
        return default
    return v if math.isfinite(v) else default


def to_setpoints(f: ScsTelemetry, opt: MapOptions | None = None) -> lk.Setpoints:
    opt = opt or MapOptions()
    sp = lk.Setpoints()

    sp.speed_kmh = int(round(abs(_num(f.truck_f.speed)) * 3.6 * opt.speed_scale))
    sp.rpm = int(round(max(0.0, _num(f.truck_f.engineRpm)) * opt.rpm_scale))

    # The gauge is a car tank, the truck has a 400-800 l tank: send the fill
    # *fraction* against a car-sized tank, otherwise the needle pins at full.
    cap = _num(f.config_f.fuelCapacity)
    fuel = _num(f.truck_f.fuel)
    frac = (fuel / cap) if cap > 1.0 else 0.5
    sp.fuel_l = max(0.0, min(1.0, frac)) * opt.tank_l

    sp.coolant_c = int(round(_num(f.truck_f.waterTemperature, 20.0)))
    sp.range_km = int(round(max(0.0, _num(f.truck_f.fuelRange))))

    sp.kl15 = bool(f.truck_b.electricEnabled)
    sp.engine = bool(f.truck_b.engineEnabled)
    sp.handbrake = bool(f.truck_b.parkBrake)

    gear = int(f.truck_i.gear)
    if gear < 0:
        sp.gear_pos, sp.gear_num, sp.reverse = lk.GEAR_R, 0, True
    elif gear == 0:
        sp.gear_pos, sp.gear_num, sp.reverse = lk.GEAR_N, 0, False
    else:
        sp.gear_pos, sp.gear_num, sp.reverse = lk.GEAR_D, gear, False

    # Blinker *enable*, not the 2 Hz lamp state: the cluster does its own
    # flashing from 0x1F6, so feeding the lamp would restart it every frame.
    # Hazards are ORed in explicitly rather than trusted to set both blinker
    # flags, which not every fork does.
    hazard = bool(f.truck_b.lightsHazard)
    sp.turn_left = bool(f.truck_b.blinkerLeftActive) or hazard
    sp.turn_right = bool(f.truck_b.blinkerRightActive) or hazard

    lights = 0
    if opt.lights_on or f.truck_b.lightsParking:
        lights |= lk.LIGHT_SIDE
    if opt.lights_on or f.truck_b.lightsBeamLow:
        lights |= lk.LIGHT_LOW
    if f.truck_b.lightsBeamHigh:
        lights |= lk.LIGHT_HIGH
    sp.lights = lights

    throttle = max(0.0, min(1.0, _num(f.truck_f.gameThrottle)))
    sp.pedal_pct = int(round(throttle * 100))

    # Traction control. The game has no DSC switch, so the lamps say what the
    # truck is doing: the DSC triangle for wheels that are actually slipping
    # under power (the caller flashes it, as a real intervention flashes), and
    # the DTC lamp for the inter-axle diff lock, which is the game's own
    # traction aid being engaged.
    slip = wheel_slip(f)
    sp.dsc_lamp = slip > TC_SLIP_ON and throttle > TC_MIN_THROTTLE
    sp.tc_lamp = bool(f.truck_b.differentialLock)

    if opt.clock == "game":
        sp.clock = game_clock(f)
    elif opt.clock == "system":
        sp.clock = lk.Clock.from_struct_time(time.localtime())
    return sp


# ---------------------------------------------------------------------------
# Self test - no game, no hardware
# ---------------------------------------------------------------------------

def selftest() -> None:
    verify_layout()

    f = ScsTelemetry()
    f.sdkActive = True
    f.time = 123456
    f.truck_f.speed = 25.0                    # 90 km/h
    f.truck_f.engineRpm = 1500.0
    f.truck_f.fuel = 30.0
    f.truck_f.waterTemperature = 88.0
    f.truck_f.fuelRange = 480.0
    f.config_f.fuelCapacity = 60.0
    f.truck_i.gear = 6
    f.truck_b.electricEnabled = True
    f.truck_b.engineEnabled = True
    f.truck_b.lightsBeamLow = True
    f.truck_b.blinkerRightActive = True
    f.common_ui.time_abs = 7 * 60 + 42          # 07:42 in game
    f.config_ui.truckWheelCount = 6
    for i in range(6):
        f.config_f.truckWheelRadius[i] = 0.5
        f.config_b.truckWheelPowered[i] = i in (2, 3)
    # Driven wheels turning 25% faster than the road. The channel is
    # rotations per second, so a 0.5 m wheel covers 2*pi*0.5 = 3.1416 m per
    # turn and 25 m/s needs 7.9577 rev/s; a quarter over that is 9.9472.
    free = 25.0 / (2.0 * math.pi * 0.5)
    for i in range(6):
        f.truck_f.truck_wheelVelocity[i] = (free * 1.25) if i in (2, 3) else free
    f.truck_f.gameThrottle = 0.8

    raw = ctypes.string_at(ctypes.byref(f), ctypes.sizeof(f))
    back = ScsTelemetry.from_buffer_copy(raw)
    assert back.time == 123456
    assert abs(back.truck_f.speed - 25.0) < 1e-6
    assert back.truck_b.engineEnabled and back.truck_b.blinkerRightActive
    assert back.truck_i.gear == 6

    assert abs(wheel_slip(back) - 0.25) < 0.01, wheel_slip(back)

    sp = to_setpoints(back)
    assert sp.speed_kmh == 90, sp.speed_kmh
    assert sp.rpm == 1500
    assert abs(sp.fuel_l - 30.0) < 1e-6
    assert sp.coolant_c == 88
    assert (sp.gear_pos, sp.gear_num) == (lk.GEAR_D, 6)
    assert not sp.reverse and sp.turn_right and not sp.turn_left
    assert sp.lights & lk.LIGHT_LOW
    assert len(sp.pack()) == lk.PAYLOAD_LEN + 4   # 2 sync + len + checksum

    # Slipping driven wheels under power light the intervention lamp; the game
    # clock reaches the cluster as 07:42.
    assert sp.dsc_lamp and not sp.tc_lamp
    assert (sp.clock.hour, sp.clock.minute) == (7, 42), sp.clock
    assert to_setpoints(back, MapOptions(clock="off")).clock is None

    lifted = ScsTelemetry.from_buffer_copy(raw)
    lifted.truck_f.gameThrottle = 0.0
    assert not to_setpoints(lifted).dsc_lamp     # coasting is not wheelspin

    rev = ScsTelemetry()
    rev.truck_i.gear = -1
    assert to_setpoints(rev).gear_pos == lk.GEAR_R
    assert to_setpoints(rev).reverse

    print("ets2 selftest OK  (layout, roundtrip, mapping, slip, clock)")
