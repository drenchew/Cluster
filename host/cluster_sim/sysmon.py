"""Reads Linux system metrics and maps them onto the instrument cluster.

The cluster becomes a system monitor:

    speedometer        network download rate
    tachometer         memory in use
    fuel gauge         battery charge

This cluster carries four needles: road speed, engine speed, fuel level and
instant consumption. The consumption needle is not driven: it reads economy,
distance divided by fuel, so it depends on road speed as well as fuel rate,
and road speed here is the download rate. It therefore cannot show processor
load on its own. Processor load reaches the driver through the check engine
telltale instead. There is no coolant temperature needle on this variant;
the value is still transmitted, because the cluster expects a plausible engine
temperature, and it drives the overheat telltale and the hidden service menu
rather than a dial.

Everything is read from ``/proc`` and ``/sys``, so the only dependency is the
same ``pyserial`` the rest of the host tooling needs.

Rate metrics are differences between two samples, so :meth:`SystemMetrics.
sample` returns ``None`` until it has been called twice. Values are smoothed
before they reach the dials: a download rate measured over a tenth of a second
is far too noisy to drive a needle directly.

Run the module directly to see what it reads:

    python3 -m cluster_sim.sysmon
    python3 -m cluster_sim.sysmon --selftest
"""

from __future__ import annotations

import glob
import os
import time
from dataclasses import dataclass, field

from . import link as lk

# ---------------------------------------------------------------------------
# Sources
# ---------------------------------------------------------------------------

PROC_MEMINFO = "/proc/meminfo"
PROC_STAT = "/proc/stat"
PROC_NET_DEV = "/proc/net/dev"
SYS_NET = "/sys/class/net"
SYS_HWMON = "/sys/class/hwmon"
SYS_POWER = "/sys/class/power_supply"
SYS_THERMAL = "/sys/class/thermal"

LOOPBACK = "lo"
BITS_PER_BYTE = 8
BITS_PER_MEGABIT = 1_000_000
KIB = 1024
MILLIDEGREES_PER_DEGREE = 1000.0

# Preferred processor temperature sensors, most specific first. A machine may
# expose a dozen thermal zones, most of which measure a chipset or the chassis
# rather than the processor package.
HWMON_PREFERRED = ("coretemp", "k10temp", "zenpower")
HWMON_PREFERRED_LABELS = ("package id 0", "tdie", "tctl")
THERMAL_PREFERRED = ("x86_pkg_temp", "tcpu", "cpu-thermal", "acpitz")


def _read_text(path: str) -> str | None:
    """File contents, or None if it cannot be read.

    Sysfs nodes disappear when hardware is removed and some are unreadable
    without privileges, so every access here is allowed to fail quietly.
    """
    try:
        with open(path, "r") as handle:
            return handle.read()
    except (OSError, ValueError):
        return None


def default_interfaces() -> list[str]:
    """Every network interface that is up, excluding the loopback."""
    found = []
    for path in sorted(glob.glob(os.path.join(SYS_NET, "*"))):
        name = os.path.basename(path)
        if name == LOOPBACK:
            continue
        state = _read_text(os.path.join(path, "operstate"))
        if state is None or state.strip() == "up":
            found.append(name)
    return found


def find_battery_path() -> str | None:
    """Directory of the first battery the kernel reports, or None.

    A desktop exposes only mains supplies, so the absence of a battery is a
    normal condition rather than an error.
    """
    found = None
    for path in sorted(glob.glob(os.path.join(SYS_POWER, "*"))):
        kind = (_read_text(os.path.join(path, "type")) or "").strip()
        if kind == "Battery" and found is None:
            found = path
    return found


def find_cpu_temperature_path() -> str | None:
    """Best available processor temperature sensor, or None.

    Returns the path of a file holding millidegrees Celsius.
    """
    # hwmon first: it labels its inputs, so the package sensor can be picked
    # out rather than guessed at.
    for chip in sorted(glob.glob(os.path.join(SYS_HWMON, "hwmon*"))):
        name = (_read_text(os.path.join(chip, "name")) or "").strip().lower()
        if name not in HWMON_PREFERRED:
            continue
        for label_path in sorted(glob.glob(os.path.join(chip, "temp*_label"))):
            label = (_read_text(label_path) or "").strip().lower()
            if label in HWMON_PREFERRED_LABELS:
                candidate = label_path.replace("_label", "_input")
                if os.path.exists(candidate):
                    return candidate
        inputs = sorted(glob.glob(os.path.join(chip, "temp*_input")))
        if inputs:
            return inputs[0]

    # Otherwise fall back to a thermal zone whose type looks like a processor.
    zones = []
    for zone in sorted(glob.glob(os.path.join(SYS_THERMAL, "thermal_zone*"))):
        kind = (_read_text(os.path.join(zone, "type")) or "").strip().lower()
        zones.append((kind, os.path.join(zone, "temp")))
    for preferred in THERMAL_PREFERRED:
        for kind, path in zones:
            if kind == preferred:
                return path
    return zones[0][1] if zones else None


# ---------------------------------------------------------------------------
# One observation
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Reading:
    """One set of measurements, in physical units."""

    download_bytes_per_s: float = 0.0
    upload_bytes_per_s: float = 0.0
    memory_used: float = 0.0        # 0..1 of total
    cpu_load: float = 0.0           # 0..1 across all cores
    cpu_celsius: float | None = None
    memory_total_bytes: int = 0
    disk_free: float = 0.0          # 0..1 of the filesystem
    disk_total_bytes: int = 0
    battery_present: bool = False
    battery_charge: float = 0.0     # 0..1 of full charge
    battery_charging: bool = False


@dataclass
class _CpuSample:
    idle: int = 0
    total: int = 0


class SystemMetrics:
    """Samples the kernel's counters and turns them into rates.

    One instance holds the previous counter values, so the same object must be
    reused between calls; a fresh one always reports nothing on its first
    sample.
    """

    def __init__(self, interfaces: list[str] | None = None,
                 temperature_path: str | None = None,
                 disk_path: str = "/",
                 battery_path: str | None = None):
        self._disk_path = disk_path
        self._battery_path = (battery_path if battery_path is not None
                              else find_battery_path())
        self._interfaces = interfaces if interfaces else default_interfaces()
        self._temperature_path = (
            temperature_path if temperature_path is not None else find_cpu_temperature_path())
        self._last_cpu = _CpuSample()
        self._last_net: tuple[int, int] | None = None
        self._last_time: float | None = None

    @property
    def interfaces(self) -> list[str]:
        return list(self._interfaces)

    @property
    def temperature_path(self) -> str | None:
        return self._temperature_path

    @property
    def battery_path(self) -> str | None:
        return self._battery_path

    # -- individual sources ----------------------------------------------
    def _memory(self) -> tuple[float, int]:
        """(fraction in use, total bytes).

        Uses MemAvailable rather than MemFree: the kernel counts reclaimable
        page cache as free for practical purposes, and MemFree alone reports a
        healthy machine as almost full.
        """
        text = _read_text(PROC_MEMINFO)
        total_kib = 0
        available_kib = 0
        if text is not None:
            for line in text.splitlines():
                parts = line.split()
                if len(parts) >= 2:
                    if parts[0] == "MemTotal:":
                        total_kib = int(parts[1])
                    elif parts[0] == "MemAvailable:":
                        available_kib = int(parts[1])
        used = 0.0
        if total_kib > 0:
            used = max(0.0, min(1.0, 1.0 - (available_kib / total_kib)))
        return used, total_kib * KIB

    def _cpu(self) -> float:
        """Processor load since the previous call, 0..1."""
        text = _read_text(PROC_STAT)
        load = 0.0
        if text is not None:
            for line in text.splitlines():
                parts = line.split()
                if parts and parts[0] == "cpu":
                    fields = [int(value) for value in parts[1:]]
                    # user nice system idle iowait irq softirq steal ...
                    total = sum(fields)
                    idle = fields[3] + (fields[4] if len(fields) > 4 else 0)
                    delta_total = total - self._last_cpu.total
                    delta_idle = idle - self._last_cpu.idle
                    if self._last_cpu.total > 0 and delta_total > 0:
                        load = max(0.0, min(1.0, 1.0 - (delta_idle / delta_total)))
                    self._last_cpu = _CpuSample(idle=idle, total=total)
                    break
        return load

    def _network_counters(self) -> tuple[int, int]:
        """Cumulative (received, transmitted) bytes over the chosen interfaces."""
        text = _read_text(PROC_NET_DEV)
        received = 0
        transmitted = 0
        if text is not None:
            for line in text.splitlines():
                if ":" not in line:
                    continue
                name, _, rest = line.partition(":")
                if name.strip() not in self._interfaces:
                    continue
                fields = rest.split()
                if len(fields) >= 9:
                    received += int(fields[0])
                    transmitted += int(fields[8])
        return received, transmitted

    def _disk(self) -> tuple[float, int]:
        """(fraction free, total bytes) for the chosen filesystem.

        Uses the blocks available to an unprivileged user rather than the
        total free blocks, which is what the filesystem will actually let a
        program write and therefore what "space left" means in practice.
        """
        free = 0.0
        total = 0
        try:
            stats = os.statvfs(self._disk_path)
            total = stats.f_blocks * stats.f_frsize
            if total > 0:
                free = max(0.0, min(1.0,
                                    (stats.f_bavail * stats.f_frsize) / total))
        except OSError:
            pass
        return free, total

    def _battery(self) -> tuple[bool, float, bool]:
        """(present, charge fraction, charging).

        The driver reports a ready made percentage, which is preferred because
        it already accounts for whichever of the charge or energy counters the
        hardware provides. The counters are read only as a fallback.
        """
        present = False
        charge = 0.0
        charging = False

        if self._battery_path is not None:
            percent = _read_text(os.path.join(self._battery_path, "capacity"))
            if percent is not None and percent.strip():
                try:
                    charge = max(0.0, min(1.0, int(percent.strip()) / 100.0))
                    present = True
                except ValueError:
                    present = False
            if not present:
                for now_name, full_name in (("charge_now", "charge_full"),
                                            ("energy_now", "energy_full")):
                    now = _read_text(os.path.join(self._battery_path, now_name))
                    full = _read_text(os.path.join(self._battery_path, full_name))
                    if now and full and not present:
                        try:
                            total = int(full.strip())
                            if total > 0:
                                charge = max(0.0, min(1.0, int(now.strip()) / total))
                                present = True
                        except ValueError:
                            pass
            status = (_read_text(os.path.join(self._battery_path, "status")) or "").strip()
            charging = status in ("Charging", "Full")

        return present, charge, charging

    def _temperature(self) -> float | None:
        value = None
        if self._temperature_path is not None:
            text = _read_text(self._temperature_path)
            if text is not None and text.strip():
                try:
                    value = int(text.strip()) / MILLIDEGREES_PER_DEGREE
                except ValueError:
                    value = None
        return value

    # -- public ------------------------------------------------------------
    def sample(self) -> Reading | None:
        """One reading, or None on the first call.

        The network rate is a difference between two counter values, so the
        first call only establishes a baseline.
        """
        now = time.monotonic()
        received, transmitted = self._network_counters()
        load = self._cpu()
        used, total = self._memory()

        reading = None
        if self._last_net is not None and self._last_time is not None:
            elapsed = now - self._last_time
            if elapsed > 0.0:
                # Counters are 64 bit on any supported kernel, but they do
                # reset when an interface is removed, so a negative difference
                # is treated as no traffic rather than a vast burst.
                down = max(0, received - self._last_net[0]) / elapsed
                up = max(0, transmitted - self._last_net[1]) / elapsed
                disk_free, disk_total = self._disk()
                battery_present, battery_charge, battery_charging = self._battery()
                reading = Reading(download_bytes_per_s=down,
                                  upload_bytes_per_s=up,
                                  memory_used=used,
                                  cpu_load=load,
                                  cpu_celsius=self._temperature(),
                                  memory_total_bytes=total,
                                  disk_free=disk_free,
                                  disk_total_bytes=disk_total,
                                  battery_present=battery_present,
                                  battery_charge=battery_charge,
                                  battery_charging=battery_charging)

        self._last_net = (received, transmitted)
        self._last_time = now
        return reading


# ---------------------------------------------------------------------------
# Mapping onto the dials
# ---------------------------------------------------------------------------

@dataclass
class MapOptions:
    """Full-scale values for each dial, and the warning thresholds."""

    # The speedometer face reads to 260 km/h, so that is full scale. The rate
    # it corresponds to has to be chosen: a gigabit link would otherwise keep
    # the needle against its stop on a 100 Mbit/s connection, and vice versa.
    max_download_mbit: float = 100.0
    max_speed_kmh: int = 260

    # The diesel tachometer reads to 5500 rpm.
    max_rpm: int = 5500

    # Fuel demand sent to the consumption gauge. Left at zero: that gauge
    # reads economy, so it divides by road speed, which this mode uses for the
    # download rate, and the needle shows a ratio of the two rather than
    # processor load. The firmware support remains, so raising this drives the
    # gauge again for anyone who wants that ratio.
    max_consumption_percent: int = 0

    # The fuel gauge shows the battery charge, which is the natural fit: both
    # are a reserve being consumed, and the cluster's own low fuel warning
    # then means a low battery, which is worth having rather than a nuisance.
    # That warning appears near 8 litres of 60, so it fires at roughly
    # 13 percent charge.
    #
    # A machine with no battery falls back to free filesystem space, so the
    # needle still shows a reserve rather than sitting at empty.
    tank_litres: float = 60.0

    # Raise this to keep the needle clear of that warning, at the cost of the
    # bottom of the dial. Zero allows it to act as a low battery alarm.
    fuel_floor_litres: float = 0.0

    # Charge below this lights the charging fault telltale, which is the
    # cluster's own symbol for a battery that is not keeping up. Suppressed
    # while the machine is on mains, since a low battery that is charging is
    # not a fault.
    warn_battery_charge: float = 0.15

    # Processor temperature is reported as the coolant temperature unchanged,
    # since both are temperatures in degrees Celsius. This variant of the
    # cluster has no coolant needle, so the value reaches the driver through
    # the overheat telltale and the hidden service menu; it is still worth
    # sending, because the cluster expects a plausible engine temperature.
    # A machine with no readable sensor reports this instead.
    fallback_celsius: int = 60

    # Telltales. Each is a fraction of full scale except the temperature.
    #
    # Memory uses the tyre pressure symbol rather than the fuel reserve one.
    # The fuel gauge already displays processor load here, so a reserve
    # telltale beside it would appear to refer to that gauge; and the cluster
    # raises the reserve warning itself, so the symbol is not ours to assign.
    # Tyre pressure is free because no tyre pressure data is transmitted, and
    # it reads as a pressure warning, which is what memory pressure is.
    warn_cpu_load: float = 0.90
    warn_memory_used: float = 0.90
    warn_cpu_celsius: float = 85.0

    # Smoothing time constant in seconds. Without it the needles jitter on
    # every sample, because a download rate measured over a tenth of a second
    # varies far more than the quantity being displayed.
    smoothing_seconds: float = 0.45

    lights_on: bool = True
    show_clock: bool = True


class _Smoother:
    """Exponential moving average with a time based constant.

    Using elapsed time rather than a fixed weight keeps the response the same
    whether the caller samples at 5 Hz or 50 Hz.
    """

    def __init__(self, seconds: float):
        self._seconds = max(0.0, seconds)
        self._value: float | None = None
        self._last: float | None = None

    def update(self, target: float, now: float) -> float:
        # The previous timestamp is compared against None rather than tested
        # for truth: a monotonic clock legitimately reads 0.0, which would
        # otherwise be mistaken for "no previous sample" and disable
        # smoothing altogether.
        if self._value is None or self._seconds <= 0.0 or self._last is None:
            self._value = target
        else:
            elapsed = max(0.0, now - self._last)
            weight = 1.0 - pow(0.5, elapsed / self._seconds) if elapsed > 0.0 else 0.0
            self._value += (target - self._value) * weight
        self._last = now
        return self._value


@dataclass
class Mapper:
    """Turns readings into setpoints, keeping the smoothing state."""

    options: MapOptions = field(default_factory=MapOptions)

    def __post_init__(self) -> None:
        self._download = _Smoother(self.options.smoothing_seconds)
        self._memory = _Smoother(self.options.smoothing_seconds)
        self._cpu = _Smoother(self.options.smoothing_seconds)
        # Temperature already changes slowly, so it is passed through.
        self._temperature = _Smoother(0.0)

    def to_setpoints(self, reading: Reading, now: float | None = None) -> lk.Setpoints:
        opt = self.options
        moment = time.monotonic() if now is None else now

        full_scale_bytes = opt.max_download_mbit * BITS_PER_MEGABIT / BITS_PER_BYTE
        download_fraction = 0.0
        if full_scale_bytes > 0.0:
            download_fraction = reading.download_bytes_per_s / full_scale_bytes

        download = self._download.update(_clamp_unit(download_fraction), moment)
        memory = self._memory.update(_clamp_unit(reading.memory_used), moment)
        cpu = self._cpu.update(_clamp_unit(reading.cpu_load), moment)

        celsius = reading.cpu_celsius
        if celsius is None:
            celsius = float(opt.fallback_celsius)
        celsius = self._temperature.update(celsius, moment)

        sp = lk.Setpoints()
        sp.speed_kmh = int(round(download * opt.max_speed_kmh))
        sp.rpm = int(round(memory * opt.max_rpm))
        # Zero by default; see max_consumption_percent.
        sp.consumption_pct = int(round(cpu * opt.max_consumption_percent))

        # Battery charge drives the fuel gauge, falling back to free
        # filesystem space on a machine that has no battery.
        reserve = (reading.battery_charge if reading.battery_present
                   else reading.disk_free)
        usable = max(0.0, opt.tank_litres - opt.fuel_floor_litres)
        sp.fuel_l = opt.fuel_floor_litres + _clamp_unit(reserve) * usable
        sp.coolant_c = int(round(celsius))

        # The gauge must not be damped here. The firmware models a real tank
        # by default, where a full scale change takes over a minute, which
        # would hide the load entirely. Requesting it per frame is what lets
        # this mode and the vehicle modes share one firmware build.
        sp.fuel_direct = True

        # The cluster must believe the engine is running: with the ignition on
        # and the crankshaft at rest it reports a stall and raises a group of
        # telltales. This also means the tachometer cannot read below the idle
        # floor the firmware applies, so a nearly idle machine still shows a
        # little over 800 rpm.
        sp.kl15 = True
        sp.engine = True

        # Never engaged. The cluster latches its brake warning if the
        # handbrake is applied above about 20 km/h and will not extinguish it
        # until it sees a clean release at standstill, and road speed here is
        # the download rate, which goes well past that. There is also nothing
        # on a system monitor that a handbrake would mean.
        sp.handbrake = False
        sp.gear_pos = lk.GEAR_D
        sp.gear_num = 0
        sp.pedal_pct = int(round(cpu * 100))

        if opt.lights_on:
            sp.lights = lk.LIGHT_SIDE | lk.LIGHT_LOW

        warn = 0
        if cpu >= opt.warn_cpu_load:
            warn |= lk.WARN_CHECK_ENGINE
        if memory >= opt.warn_memory_used:
            warn |= lk.WARN_TYRE
        if celsius >= opt.warn_cpu_celsius:
            warn |= lk.WARN_COOLANT
        if (reading.battery_present and not reading.battery_charging
                and reading.battery_charge <= opt.warn_battery_charge):
            warn |= lk.WARN_BATTERY
        sp.warn = warn

        if opt.show_clock:
            sp.clock = lk.Clock.from_struct_time(time.localtime())
        return sp


def _clamp_unit(value: float) -> float:
    return max(0.0, min(1.0, value))


def describe(reading: Reading, sp: lk.Setpoints, opt: MapOptions) -> str:
    """One line summarising a reading and what it put on the dials."""
    down_mbit = reading.download_bytes_per_s * BITS_PER_BYTE / BITS_PER_MEGABIT
    up_mbit = reading.upload_bytes_per_s * BITS_PER_BYTE / BITS_PER_MEGABIT
    temperature = ("%.0fC" % reading.cpu_celsius) if reading.cpu_celsius is not None else "--"
    if reading.battery_present:
        reserve = (f"batt {reading.battery_charge * 100:3.0f}%"
                   f"{'+' if reading.battery_charging else '-'}")
    else:
        reserve = f"disk {reading.disk_free * 100:3.0f}%"
    return (f"down {down_mbit:7.2f} Mbit/s -> {sp.speed_kmh:3d} km/h   "
            f"ram {reading.memory_used * 100:5.1f}% -> {sp.rpm:4d} rpm   "
            f"cpu {reading.cpu_load * 100:5.1f}%   "
            f"{reserve} -> {sp.fuel_l:4.1f} l   "
            f"temp {temperature} -> {sp.coolant_c:3d}C   "
            f"up {up_mbit:5.2f} Mbit/s   warn 0x{sp.warn:02X}")


# ---------------------------------------------------------------------------
# Self test
# ---------------------------------------------------------------------------

# The level at which the cluster raises its own low fuel warning, from the
# vehicle simulator's figure for a real E9x. Used by the self test to prove the
# mapping can never reach it.
RESERVE_WARNING_LITRES = 8.0


def selftest() -> None:
    """Checks the mapping with synthetic readings, so no hardware is needed."""
    opt = MapOptions(max_download_mbit=100.0, smoothing_seconds=0.0)
    mapper = Mapper(opt)

    full_scale = opt.max_download_mbit * BITS_PER_MEGABIT / BITS_PER_BYTE

    # Half the rated download, half the memory, a quarter of the processor,
    # three quarters of the filesystem free.
    half = Reading(download_bytes_per_s=full_scale / 2,
                   memory_used=0.5, cpu_load=0.25, cpu_celsius=55.0,
                   disk_free=0.75, battery_present=True, battery_charge=0.5,
                   battery_charging=True)
    sp = mapper.to_setpoints(half, now=0.0)
    assert sp.speed_kmh == 130, sp.speed_kmh
    assert sp.rpm == 2750, sp.rpm
    assert sp.consumption_pct == 0, "the consumption gauge is not driven"
    # Battery takes the fuel gauge, so half a charge is half a tank. The
    # filesystem figure is deliberately different, to catch a mix-up.
    assert abs(sp.fuel_l - 30.0) < 0.01, sp.fuel_l
    assert sp.coolant_c == 55, sp.coolant_c
    assert sp.warn == 0, hex(sp.warn)
    assert sp.fuel_direct, "the gauge shows a measurement, not a tank level"
    assert not sp.handbrake, "the handbrake must never be engaged in this mode"
    assert sp.kl15 and sp.engine, "the cluster must see a running engine"

    # Full scale on every dial, and a link faster than the rated speed must
    # peg the needle rather than wrap around.
    mapper = Mapper(opt)
    loaded = Reading(download_bytes_per_s=full_scale * 4,
                     memory_used=1.0, cpu_load=1.0, cpu_celsius=95.0,
                     disk_free=1.0, battery_present=True, battery_charge=1.0,
                     battery_charging=True)
    sp = mapper.to_setpoints(loaded, now=0.0)
    assert sp.speed_kmh == opt.max_speed_kmh, sp.speed_kmh
    assert sp.rpm == opt.max_rpm, sp.rpm
    assert sp.consumption_pct == 0, sp.consumption_pct
    assert abs(sp.fuel_l - opt.tank_litres) < 0.01, sp.fuel_l
    assert sp.warn & lk.WARN_CHECK_ENGINE
    assert sp.warn & lk.WARN_TYRE, "memory pressure uses the tyre symbol"
    assert sp.warn & lk.WARN_COOLANT
    assert not (sp.warn & lk.WARN_FUEL), \
        "the reserve symbol belongs to the cluster, not to this mapping"

    # An idle machine sits at the bottom of the needles it drives.
    mapper = Mapper(opt)
    idle = Reading(cpu_celsius=40.0, battery_present=True, battery_charge=0.5)
    sp = mapper.to_setpoints(idle, now=0.0)
    assert sp.speed_kmh == 0 and sp.rpm == 0, (sp.speed_kmh, sp.rpm)
    assert sp.consumption_pct == 0, sp.consumption_pct

    # Processor load must never reach the fuel gauge, whatever it does. That
    # is the whole reason it moved to the consumption gauge: the cluster
    # raises its own reserve warning from the fuel level, and this host has no
    # way to clear a warning the cluster raised itself.
    for percent in range(0, 101):
        level = mapper.to_setpoints(
            Reading(cpu_load=percent / 100.0, battery_present=True,
                    battery_charge=0.5, cpu_celsius=40.0), now=0.0).fuel_l
        assert abs(level - 30.0) < 0.01, (percent, level)

    # The fuel gauge follows the battery, and a nearly flat one is allowed to
    # reach the reserve warning, where it means something.
    mapper = Mapper(opt)
    charged = mapper.to_setpoints(
        Reading(battery_present=True, battery_charge=0.9, cpu_celsius=40.0), now=0.0)
    flat = mapper.to_setpoints(
        Reading(battery_present=True, battery_charge=0.05, cpu_celsius=40.0), now=0.0)
    assert charged.fuel_l > RESERVE_WARNING_LITRES, charged.fuel_l
    assert flat.fuel_l < RESERVE_WARNING_LITRES, flat.fuel_l

    # A flat battery on mains is charging, not failing, so the telltale stays
    # out; the same charge on battery power lights it.
    mapper = Mapper(opt)
    on_mains = mapper.to_setpoints(
        Reading(battery_present=True, battery_charge=0.05, battery_charging=True,
                cpu_celsius=40.0), now=0.0)
    assert not (on_mains.warn & lk.WARN_BATTERY), "charging is not a fault"
    mapper = Mapper(opt)
    on_battery = mapper.to_setpoints(
        Reading(battery_present=True, battery_charge=0.05, battery_charging=False,
                cpu_celsius=40.0), now=0.0)
    assert on_battery.warn & lk.WARN_BATTERY, "a flat battery should warn"

    # A machine with no battery falls back to the filesystem, so the needle
    # still shows a reserve rather than reading empty.
    mapper = Mapper(opt)
    desktop = mapper.to_setpoints(
        Reading(battery_present=False, disk_free=0.5, cpu_celsius=40.0), now=0.0)
    assert abs(desktop.fuel_l - 30.0) < 0.01, desktop.fuel_l
    assert not (desktop.warn & lk.WARN_BATTERY), "no battery is not a flat battery"

    # A machine with no temperature sensor must not read as frozen.
    mapper = Mapper(opt)
    sp = mapper.to_setpoints(Reading(cpu_celsius=None), now=0.0)
    assert sp.coolant_c == opt.fallback_celsius, sp.coolant_c

    # Every frame must still fit the wire format the firmware expects.
    assert len(sp.pack()) == lk.PAYLOAD_LEN + 4

    # Smoothing must approach the target rather than jump to it, and must
    # settle on it eventually.
    smoothed = Mapper(MapOptions(smoothing_seconds=0.5))
    first = smoothed.to_setpoints(loaded, now=0.0)
    assert first.rpm == opt.max_rpm, "the first sample seeds the average"
    down = smoothed.to_setpoints(idle, now=0.05)
    assert 0 < down.rpm < opt.max_rpm, down.rpm
    for step in range(2, 120):
        down = smoothed.to_setpoints(idle, now=0.05 * step)
    assert down.rpm < 20, down.rpm

    # The readers must tolerate a machine that exposes none of these files.
    metrics = SystemMetrics(interfaces=["definitely-not-a-nic"], temperature_path=None)
    assert metrics.sample() is None, "the first sample only sets a baseline"
    second = metrics.sample()
    assert second is not None
    assert second.download_bytes_per_s == 0.0

    # Whatever the inputs, the handbrake stays released: engaging it above
    # about 20 km/h latches the cluster's brake warning.
    for speed_fraction in (0.0, 0.25, 1.0):
        probe = Mapper(opt).to_setpoints(
            Reading(download_bytes_per_s=full_scale * speed_fraction,
                    battery_present=True, battery_charge=0.5, cpu_celsius=40.0),
            now=0.0)
        assert not probe.handbrake, speed_fraction

    # Opting back in must still drive the gauge, since the firmware support
    # and the wire field both remain.
    opted_in = Mapper(MapOptions(max_download_mbit=100.0, smoothing_seconds=0.0,
                                 max_consumption_percent=100))
    assert opted_in.to_setpoints(
        Reading(cpu_load=0.4, battery_present=True, battery_charge=0.5,
                cpu_celsius=40.0), now=0.0).consumption_pct == 40

    assert "down" in describe(half, sp, opt)
    print("sysmon selftest OK  (mapping, full scale, idle, smoothing, "
          "battery, fallback, missing sensors)")


def main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(
        description="show the system metrics that would drive the cluster")
    parser.add_argument("--selftest", action="store_true",
                        help="check the mapping and exit")
    parser.add_argument("--iface", action="append",
                        help="network interface to count (repeatable)")
    parser.add_argument("--max-mbit", type=float, default=100.0,
                        help="download rate that reads as full scale")
    parser.add_argument("--interval", type=float, default=0.5,
                        help="seconds between samples")
    args = parser.parse_args(argv)

    if args.selftest:
        selftest()
        return 0

    metrics = SystemMetrics(interfaces=args.iface)
    mapper = Mapper(MapOptions(max_download_mbit=args.max_mbit))

    print(f"interfaces: {', '.join(metrics.interfaces) or 'none found'}")
    print(f"temperature: {metrics.temperature_path or 'no sensor found'}")
    print(f"full scale: {args.max_mbit:.0f} Mbit/s -> "
          f"{mapper.options.max_speed_kmh} km/h")
    print()

    try:
        while True:
            reading = metrics.sample()
            if reading is not None:
                print(describe(reading, mapper.to_setpoints(reading), mapper.options),
                      flush=True)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
