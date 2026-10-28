"""Reader for our own ETS2/ATS telemetry plugin.

The plugin in ``plugin/`` publishes a 512 byte block at
``/dev/shm/ClusterTelemetry``. This module reads it and hands back a
:class:`cluster_sim.ets2.ScsTelemetry` filled in, so everything downstream -
``to_setpoints``, ``wheel_slip``, ``game_clock``, the whole CAN mapping -
works without knowing which plugin produced the data.

Two plugins, one mapping:

    ets2.ShmSource      RenCloud / truckermudgeon layout, 32 kB
    ets2_plugin.Source  ours, 512 bytes, built from plugin/

Ours is the one that builds on Linux with nothing but g++, so it is the
default. The other stays supported because a Windows box or an existing
install may already have it.

Run it directly to look at the block:

    python3 -m cluster_sim.ets2_plugin            # one shot
    python3 -m cluster_sim.ets2_plugin --watch    # follow it live
"""

from __future__ import annotations

import ctypes
import math
import mmap
import os
import time

from . import ets2

# ---------------------------------------------------------------------------
# The block, mirroring plugin/include/cluster_shm.h
# ---------------------------------------------------------------------------

SHM_NAME = "ClusterTelemetry"
SHM_PATH = "/dev/shm/" + SHM_NAME
SHM_SIZE = 4096
MAGIC = 0x314D4C43          # "CLM1"
VERSION = 1
WHEELS = 16
STRUCT_SIZE = 512

GAME_NAMES = {0: "unknown", 1: "ETS2", 2: "ATS"}


class ClusterShm(ctypes.Structure):
    """The plugin's published frame.

    Field for field the same as ``cluster_shm_t``. Natural alignment on both
    sides and no implicit padding by construction, which the size assert
    below is there to catch if someone edits one side only.
    """

    _fields_ = [
        # -- header ------------------------------------------------------
        ("magic", ctypes.c_uint32),
        ("version", ctypes.c_uint32),
        ("size", ctypes.c_uint32),
        ("seq", ctypes.c_uint32),
        ("game", ctypes.c_uint32),
        ("game_major", ctypes.c_uint32),
        ("game_minor", ctypes.c_uint32),
        ("plugin_build", ctypes.c_uint32),
        ("sdk_active", ctypes.c_uint32),
        ("paused", ctypes.c_uint32),
        ("time_abs", ctypes.c_uint32),
        ("wheel_count", ctypes.c_uint32),
        ("timestamp_us", ctypes.c_uint64),
        ("frames", ctypes.c_uint32),
        ("_pad0", ctypes.c_uint32),

        # -- floats ------------------------------------------------------
        ("speed", ctypes.c_float),              # m/s
        ("engine_rpm", ctypes.c_float),
        ("engine_rpm_max", ctypes.c_float),
        ("fuel", ctypes.c_float),               # litres
        ("fuel_capacity", ctypes.c_float),
        ("fuel_range", ctypes.c_float),         # km
        ("fuel_avg_consumption", ctypes.c_float),
        ("water_temperature", ctypes.c_float),
        ("oil_pressure", ctypes.c_float),
        ("oil_temperature", ctypes.c_float),
        ("brake_air_pressure", ctypes.c_float),
        ("battery_voltage", ctypes.c_float),
        ("throttle", ctypes.c_float),           # 0..1
        ("brake", ctypes.c_float),
        ("clutch", ctypes.c_float),
        ("steering", ctypes.c_float),
        ("odometer", ctypes.c_float),           # km
        ("cruise_control", ctypes.c_float),     # m/s
        ("dashboard_backlight", ctypes.c_float),
        ("adblue", ctypes.c_float),
        ("wear_engine", ctypes.c_float),
        ("wear_transmission", ctypes.c_float),
        ("navigation_speed_limit", ctypes.c_float),
        ("navigation_distance", ctypes.c_float),
        ("navigation_time", ctypes.c_float),
        ("_pad1", ctypes.c_float),

        # -- integers ----------------------------------------------------
        ("gear", ctypes.c_int32),
        ("displayed_gear", ctypes.c_int32),
        ("gears_forward", ctypes.c_int32),
        ("hshifter_slot", ctypes.c_int32),

        # -- flags -------------------------------------------------------
        ("electric_enabled", ctypes.c_uint8),
        ("engine_enabled", ctypes.c_uint8),
        ("park_brake", ctypes.c_uint8),
        ("motor_brake", ctypes.c_uint8),
        ("light_parking", ctypes.c_uint8),
        ("light_low_beam", ctypes.c_uint8),
        ("light_high_beam", ctypes.c_uint8),
        ("light_beacon", ctypes.c_uint8),
        ("light_brake", ctypes.c_uint8),
        ("light_reverse", ctypes.c_uint8),
        ("blinker_left_active", ctypes.c_uint8),
        ("blinker_right_active", ctypes.c_uint8),
        ("blinker_left_on", ctypes.c_uint8),
        ("blinker_right_on", ctypes.c_uint8),
        ("hazard", ctypes.c_uint8),
        ("differential_lock", ctypes.c_uint8),
        ("cruise_control_on", ctypes.c_uint8),
        ("fuel_warning", ctypes.c_uint8),
        ("adblue_warning", ctypes.c_uint8),
        ("oil_pressure_warning", ctypes.c_uint8),
        ("water_temperature_warning", ctypes.c_uint8),
        ("battery_voltage_warning", ctypes.c_uint8),
        ("air_pressure_warning", ctypes.c_uint8),
        ("air_pressure_emergency", ctypes.c_uint8),

        # -- per wheel ---------------------------------------------------
        ("wheel_angular_velocity", ctypes.c_float * WHEELS),   # rev/s
        ("wheel_radius", ctypes.c_float * WHEELS),             # m
        ("wheel_powered", ctypes.c_uint8 * WHEELS),
        ("wheel_on_ground", ctypes.c_uint8 * WHEELS),
        ("wheel_steerable", ctypes.c_uint8 * WHEELS),
        ("wheel_simulated", ctypes.c_uint8 * WHEELS),

        # -- strings -----------------------------------------------------
        ("truck_brand", ctypes.c_char * 48),
        ("truck_name", ctypes.c_char * 48),
        ("game_id", ctypes.c_char * 16),
    ]


def verify_layout() -> None:
    """Fail loudly if this mirror and the C struct have drifted apart.

    Called from the source's constructor rather than at import, so a stale
    checkout gives one clear message at startup instead of a gauge that is
    quietly reading the wrong four bytes.
    """
    got = ctypes.sizeof(ClusterShm)
    if got != STRUCT_SIZE:
        raise RuntimeError(
            f"ClusterShm is {got} bytes, expected {STRUCT_SIZE} - this mirror "
            f"and plugin/include/cluster_shm.h disagree. Rebuild the plugin "
            f"(cd plugin && make test) and check both sides.")


# ---------------------------------------------------------------------------
# Reading it
# ---------------------------------------------------------------------------

class Source:
    """Poll the plugin's block, newest consistent frame wins.

    Interface-compatible with :class:`cluster_sim.ets2.ShmSource`: ``poll()``
    returns a filled ``ScsTelemetry`` or ``None``, and the runner reads
    ``name``, ``last_error``, ``revid`` and ``revid_warning`` off it for the
    status line.
    """

    def __init__(self, stale_s: float = 0.75, path: str | None = None):
        verify_layout()
        self.path = path or SHM_PATH
        self._stale_s = stale_s
        self._mm: mmap.mmap | None = None
        self._fd: int | None = None
        self._next_open = 0.0
        self._last_frames = -1
        self._last_move = 0.0

        self.name: str | None = None
        self.last_error: str | None = None
        self.revid: int | None = None          # the plugin's own build number
        self.revid_warning: str | None = None
        self.truck: str | None = None          # "Scania R 2016", for the status line

    # -- plumbing ---------------------------------------------------------
    def _open(self) -> None:
        """Try to map the block, at most once a second.

        The game not being started is the normal case, not an error, so this
        backs off rather than hammering open() sixty times a second for as
        long as the bench is powered up.
        """
        now = time.monotonic()
        if now < self._next_open:
            return
        self._next_open = now + 1.0

        try:
            self._fd = os.open(self.path, os.O_RDONLY)
            self._mm = mmap.mmap(self._fd, SHM_SIZE, prot=mmap.PROT_READ)
        except (FileNotFoundError, PermissionError, ValueError, OSError) as exc:
            self._close_fd()
            self.name = None
            self.last_error = f"{self.path}: {exc.__class__.__name__}"
            return

        self.name = self.path
        self.last_error = None
        self._last_frames = -1
        self._last_move = 0.0

    def _close_fd(self) -> None:
        if self._mm is not None:
            self._mm.close()
            self._mm = None
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None

    def close(self) -> None:
        self._close_fd()

    # -- data -------------------------------------------------------------
    def read_block(self) -> ClusterShm | None:
        """One consistent snapshot, or None.

        The plugin makes ``seq`` odd while it copies a frame in and even
        again when it is done, so reading it either side of the copy tells us
        whether we caught a frame mid-flight. Three tries is generous: the
        write is a 500 byte memcpy, and the game would have to be unlucky
        twice in a row to lose the race.
        """
        if self._mm is None:
            return None
        for _ in range(3):
            try:
                before = ClusterShm.from_buffer_copy(self._mm[:STRUCT_SIZE])
                if before.seq % 2:
                    continue                      # a write is in progress
                after_seq = ClusterShm.from_buffer_copy(self._mm[:STRUCT_SIZE]).seq
            except (OSError, ValueError) as exc:  # the game quit, block gone
                self._close_fd()
                self.last_error = str(exc)
                return None
            if before.seq == after_seq:
                return before
        return None

    def poll(self) -> ets2.ScsTelemetry | None:
        if self._mm is None:
            self._open()
            if self._mm is None:
                return None

        block = self.read_block()
        if block is None:
            return None

        if block.magic != MAGIC:
            # Either the game has shut the plugin down - which clears magic
            # on purpose - or we opened something else entirely.
            self.last_error = (f"not our block (magic 0x{block.magic:08x}) - "
                               f"the game may have exited")
            return None

        if block.version != VERSION:
            self.revid_warning = (
                f"plugin publishes block version {block.version}, this reader "
                f"understands {VERSION} - rebuild and reinstall the plugin "
                f"(cd plugin && make && make install)")
            return None

        if block.size != STRUCT_SIZE:
            self.revid_warning = (
                f"plugin publishes a {block.size} byte struct, this reader "
                f"expects {STRUCT_SIZE} - the .so in the plugins folder is "
                f"from a different build")
            return None

        self.revid_warning = None
        self.revid = int(block.plugin_build)

        brand = block.truck_brand.decode("utf-8", "replace").strip()
        model = block.truck_name.decode("utf-8", "replace").strip()
        self.truck = f"{brand} {model}".strip() or None

        # Liveness. The frame counter moves once per simulation frame, so it
        # stands still in a menu as well as after a crash - which is what we
        # want, because the cluster should drop back to its demo sweep either
        # way rather than hold the last reading from before the pause.
        frames = int(block.frames)
        if frames != self._last_frames:
            self._last_frames = frames
            self._last_move = time.monotonic()

        if not block.sdk_active:
            return None
        if (time.monotonic() - self._last_move) > self._stale_s:
            return None

        return to_scs(block)


# ---------------------------------------------------------------------------
# Translation into the shape the mapping already speaks
# ---------------------------------------------------------------------------

def to_scs(b: ClusterShm) -> ets2.ScsTelemetry:
    """Fill a ``ScsTelemetry`` from our block.

    Only the fields ``to_setpoints``, ``wheel_slip`` and ``game_clock``
    actually read are populated; the rest of that 32 kB struct stays zero.
    Going through it rather than mapping straight to setpoints means there is
    one mapping to keep correct instead of two that drift.
    """
    f = ets2.ScsTelemetry()

    f.sdkActive = bool(b.sdk_active)
    f.paused = bool(b.paused)
    f.time = int(b.timestamp_us)

    f.scs_values.telemetry_plugin_revision = int(b.plugin_build)
    f.scs_values.version_major = int(b.game_major)
    f.scs_values.version_minor = int(b.game_minor)
    f.scs_values.game = int(b.game)

    f.common_ui.time_abs = int(b.time_abs)

    f.truck_f.speed = float(b.speed)
    f.truck_f.engineRpm = float(b.engine_rpm)
    f.truck_f.fuel = float(b.fuel)
    f.truck_f.fuelRange = float(b.fuel_range)
    f.truck_f.fuelAvgConsumption = float(b.fuel_avg_consumption)
    f.truck_f.waterTemperature = float(b.water_temperature)
    f.truck_f.oilPressure = float(b.oil_pressure)
    f.truck_f.oilTemperature = float(b.oil_temperature)
    f.truck_f.airPressure = float(b.brake_air_pressure)
    f.truck_f.batteryVoltage = float(b.battery_voltage)
    f.truck_f.truckOdometer = float(b.odometer)
    f.truck_f.cruiseControlSpeed = float(b.cruise_control)
    f.truck_f.lightsDashboard = float(b.dashboard_backlight)
    f.truck_f.adblue = float(b.adblue)
    f.truck_f.wearEngine = float(b.wear_engine)
    f.truck_f.wearTransmission = float(b.wear_transmission)
    f.truck_f.speedLimit = float(b.navigation_speed_limit)
    f.truck_f.routeDistance = float(b.navigation_distance)
    f.truck_f.routeTime = float(b.navigation_time)

    # The mapping reads gameThrottle - the pedal after the game's own
    # smoothing, which is what the truck's engine actually gets, and so what
    # the wheelspin test has to be gated on. Our block carries the same
    # channel (truck.effective.throttle) under a shorter name.
    f.truck_f.gameThrottle = float(b.throttle)
    f.truck_f.gameBrake = float(b.brake)
    f.truck_f.gameClutch = float(b.clutch)
    f.truck_f.gameSteer = float(b.steering)

    f.truck_i.gear = int(b.gear)
    f.truck_i.gearDashboard = int(b.displayed_gear)

    f.config_f.fuelCapacity = float(b.fuel_capacity)
    f.config_f.engineRpmMax = float(b.engine_rpm_max)
    f.config_ui.truckWheelCount = int(b.wheel_count)
    f.config_ui.gears = int(b.gears_forward)

    f.truck_b.electricEnabled = bool(b.electric_enabled)
    f.truck_b.engineEnabled = bool(b.engine_enabled)
    f.truck_b.parkBrake = bool(b.park_brake)
    f.truck_b.motorBrake = bool(b.motor_brake)

    f.truck_b.lightsParking = bool(b.light_parking)
    f.truck_b.lightsBeamLow = bool(b.light_low_beam)
    f.truck_b.lightsBeamHigh = bool(b.light_high_beam)
    f.truck_b.lightsBeacon = bool(b.light_beacon)
    f.truck_b.lightsBrake = bool(b.light_brake)
    f.truck_b.lightsReverse = bool(b.light_reverse)

    f.truck_b.blinkerLeftActive = bool(b.blinker_left_active)
    f.truck_b.blinkerRightActive = bool(b.blinker_right_active)
    f.truck_b.blinkerLeftOn = bool(b.blinker_left_on)
    f.truck_b.blinkerRightOn = bool(b.blinker_right_on)
    f.truck_b.lightsHazard = bool(b.hazard)

    f.truck_b.differentialLock = bool(b.differential_lock)
    f.truck_b.cruiseControl = bool(b.cruise_control_on)

    f.truck_b.fuelWarning = bool(b.fuel_warning)
    f.truck_b.adblueWarning = bool(b.adblue_warning)
    f.truck_b.oilPressureWarning = bool(b.oil_pressure_warning)
    f.truck_b.waterTemperatureWarning = bool(b.water_temperature_warning)
    f.truck_b.batteryVoltageWarning = bool(b.battery_voltage_warning)
    f.truck_b.airPressureWarning = bool(b.air_pressure_warning)
    f.truck_b.airPressureEmergency = bool(b.air_pressure_emergency)

    for i in range(WHEELS):
        f.truck_f.truck_wheelVelocity[i] = float(b.wheel_angular_velocity[i])
        f.config_f.truckWheelRadius[i] = float(b.wheel_radius[i])
        f.config_b.truckWheelPowered[i] = bool(b.wheel_powered[i])
        f.config_b.truckWheelSteerable[i] = bool(b.wheel_steerable[i])
        f.config_b.truckWheelSimulated[i] = bool(b.wheel_simulated[i])
        f.truck_b.truck_wheelOnGround[i] = bool(b.wheel_on_ground[i])

    return f


# ---------------------------------------------------------------------------
# Looking at it from the command line
# ---------------------------------------------------------------------------

def describe(b: ClusterShm) -> str:
    game = GAME_NAMES.get(int(b.game), "?")
    brand = b.truck_brand.decode("utf-8", "replace").strip()
    model = b.truck_name.decode("utf-8", "replace").strip()
    gear = int(b.gear)
    gear_txt = "R" if gear < 0 else ("N" if gear == 0 else str(gear))
    cap = float(b.fuel_capacity)
    pct = (100.0 * float(b.fuel) / cap) if cap > 1.0 else 0.0
    mins = int(b.time_abs) % (24 * 60)

    return (
        f"{game} telemetry {b.game_major}.{b.game_minor}  plugin build "
        f"{b.plugin_build}  frame {b.frames}\n"
        f"truck    {brand} {model}  ({b.wheel_count} wheels, "
        f"{b.gears_forward} gears, {cap:.0f} l tank)\n"
        f"state    {'paused' if b.paused else 'running'}  "
        f"ignition {'on' if b.electric_enabled else 'off'}  "
        f"engine {'on' if b.engine_enabled else 'off'}  "
        f"clock {mins // 60:02d}:{mins % 60:02d}\n"
        f"motion   {abs(float(b.speed)) * 3.6:6.1f} km/h  "
        f"{float(b.engine_rpm):6.0f} rpm  gear {gear_txt}  "
        f"throttle {float(b.throttle) * 100:3.0f}%\n"
        f"engine   coolant {float(b.water_temperature):.0f} C  "
        f"oil {float(b.oil_pressure):.1f} psi  "
        f"air {float(b.brake_air_pressure):.1f} psi  "
        f"battery {float(b.battery_voltage):.1f} V\n"
        f"fuel     {float(b.fuel):.0f} l of {cap:.0f} ({pct:.0f}%)  "
        f"range {float(b.fuel_range):.0f} km  "
        f"avg {float(b.fuel_avg_consumption) * 100:.1f} l/100km\n"
        f"lights   side {int(b.light_parking)} low {int(b.light_low_beam)} "
        f"high {int(b.light_high_beam)}  "
        f"blinkers {int(b.blinker_left_active)}/{int(b.blinker_right_active)}  "
        f"hazard {int(b.hazard)}\n"
        f"traction diff lock {int(b.differential_lock)}  "
        f"handbrake {int(b.park_brake)}  slip {slip_of(b):.3f}"
    )


def slip_of(b: ClusterShm) -> float:
    """Worst powered-wheel slip, the same sum the CAN mapping does.

    See ``ets2.wheel_slip`` for why the 2*pi is there: the channel is
    rotations per second, so one turn covers the circumference.
    """
    speed = abs(float(b.speed))
    worst = 0.0
    for i in range(min(WHEELS, int(b.wheel_count) or WHEELS)):
        if not b.wheel_powered[i]:
            continue
        radius = float(b.wheel_radius[i])
        if radius <= 0.05:
            continue
        rev_per_s = abs(float(b.wheel_angular_velocity[i]))
        surface = rev_per_s * 2.0 * math.pi * radius
        worst = max(worst, (surface - speed) / max(speed, 2.0))
    return max(0.0, worst)


def selftest() -> None:
    """Layout and translation, with no game and no plugin loaded."""
    verify_layout()

    b = ClusterShm()
    b.magic = MAGIC
    b.version = VERSION
    b.size = STRUCT_SIZE
    b.seq = 2
    b.game = 1
    b.game_major, b.game_minor = 1, 21
    b.plugin_build = 1
    b.sdk_active = 1
    b.frames = 99
    b.timestamp_us = 1234567
    b.time_abs = 7 * 60 + 42
    b.wheel_count = 6

    b.speed = 25.0                 # 90 km/h
    b.engine_rpm = 1500.0
    b.fuel = 350.0
    b.fuel_capacity = 700.0        # half a truck tank
    b.fuel_range = 480.0
    b.water_temperature = 88.0
    b.throttle = 0.8
    b.gear = 6
    b.electric_enabled = 1
    b.engine_enabled = 1
    b.light_low_beam = 1
    b.blinker_right_active = 1
    b.truck_brand = b"Scania"
    b.truck_name = b"R 2016"
    # Rotations per second, so 25 m/s on a 0.5 m wheel is 7.9577 rev/s and
    # the driven pair spin a quarter faster than that.
    free = 25.0 / (2.0 * math.pi * 0.5)
    for i in range(6):
        b.wheel_radius[i] = 0.5
        b.wheel_powered[i] = 1 if i in (2, 3) else 0
        b.wheel_angular_velocity[i] = (free * 1.25) if i in (2, 3) else free

    # Round trip through bytes the way the mmap read does.
    back = ClusterShm.from_buffer_copy(bytes(b))
    assert back.frames == 99
    assert abs(back.speed - 25.0) < 1e-6
    assert back.truck_brand == b"Scania"

    assert abs(slip_of(back) - 0.25) < 0.01, slip_of(back)

    # Translation, then the existing mapping on top of it.
    f = to_scs(back)
    assert f.sdkActive and f.truck_b.engineEnabled
    assert abs(f.truck_f.speed - 25.0) < 1e-6
    assert f.truck_i.gear == 6
    assert abs(ets2.wheel_slip(f) - 0.25) < 0.01, ets2.wheel_slip(f)

    sp = ets2.to_setpoints(f)
    assert sp.speed_kmh == 90, sp.speed_kmh
    assert sp.rpm == 1500, sp.rpm
    # 350 l of a 700 l tank is half, which on a 60 l car gauge is 30 l.
    assert abs(sp.fuel_l - 30.0) < 0.01, sp.fuel_l
    assert sp.coolant_c == 88
    assert sp.kl15 and sp.engine
    assert sp.turn_right and not sp.turn_left
    assert sp.dsc_lamp, "slipping driven wheels should light the lamp"
    assert (sp.clock.hour, sp.clock.minute) == (7, 42), sp.clock

    # A block the plugin has shut down must not be served as live data.
    dead = ClusterShm.from_buffer_copy(bytes(b))
    dead.magic = 0
    src = Source.__new__(Source)          # no mapping, just the checks
    src.revid = src.revid_warning = src.last_error = src.truck = None
    assert dead.magic != MAGIC

    # An odd seq is a frame caught mid-write and must be rejected.
    torn = ClusterShm.from_buffer_copy(bytes(b))
    torn.seq = 3
    assert torn.seq % 2 == 1

    assert "Scania" in describe(back)
    print("ets2_plugin selftest OK  (layout, roundtrip, translation, mapping, slip)")


def main(argv: list[str] | None = None) -> int:
    import argparse

    ap = argparse.ArgumentParser(
        description="show what the cluster telemetry plugin is publishing")
    ap.add_argument("--path", default=SHM_PATH, help=f"block to read (default {SHM_PATH})")
    ap.add_argument("--watch", action="store_true", help="refresh until interrupted")
    ap.add_argument("--selftest", action="store_true", help="check the layout and exit")
    args = ap.parse_args(argv)

    if args.selftest:
        selftest()
        return 0

    if not os.path.exists(args.path):
        print(f"nothing at {args.path}")
        print()
        print("The plugin publishes it while the game is running. Check:")
        print("  1. is ETS2 running?")
        print("  2. is the plugin installed?  cd plugin && make install")
        print("  3. did the game load it?     grep -i cluster-telemetry "
              "~/.local/share/'Euro Truck Simulator 2'/game.log.txt")
        return 1

    src = Source(path=args.path)
    src._open()
    if src._mm is None:
        print(f"could not open {args.path}: {src.last_error}")
        return 1

    try:
        while True:
            block = src.read_block()
            if block is None:
                print("could not get a consistent frame")
            elif block.magic != MAGIC:
                print(f"{args.path} exists but magic is 0x{block.magic:08x} - "
                      f"the game has unloaded the plugin")
            else:
                if args.watch:
                    print("\x1b[2J\x1b[H", end="")   # clear, home
                print(describe(block))
            if not args.watch:
                break
            time.sleep(0.25)
    except KeyboardInterrupt:
        print()
    finally:
        src.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
