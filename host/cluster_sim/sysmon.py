"""Reads Linux system metrics and maps them onto the instrument cluster.

The cluster becomes a system monitor:

    speedometer     network download rate
    tachometer      memory in use
    fuel gauge      processor load
    coolant gauge   processor temperature

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
                 temperature_path: str | None = None):
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
                reading = Reading(download_bytes_per_s=down,
                                  upload_bytes_per_s=up,
                                  memory_used=used,
                                  cpu_load=load,
                                  cpu_celsius=self._temperature(),
                                  memory_total_bytes=total)

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

    # The fuel gauge is calibrated in litres by the firmware, so processor load
    # is expressed as a fraction of this tank size.
    tank_litres: float = 60.0

    # Processor temperature goes to the coolant gauge unchanged, since both are
    # temperatures in degrees Celsius. A machine with no readable sensor shows
    # this instead, which keeps the needle off the cold stop.
    fallback_celsius: int = 60

    # Telltales. Each is a fraction of full scale except the temperature.
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
        sp.fuel_l = cpu * opt.tank_litres
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
            warn |= lk.WARN_FUEL
        if celsius >= opt.warn_cpu_celsius:
            warn |= lk.WARN_COOLANT
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
    return (f"down {down_mbit:7.2f} Mbit/s -> {sp.speed_kmh:3d} km/h   "
            f"ram {reading.memory_used * 100:5.1f}% -> {sp.rpm:4d} rpm   "
            f"cpu {reading.cpu_load * 100:5.1f}% -> {sp.fuel_l:4.1f} l   "
            f"temp {temperature} -> {sp.coolant_c:3d}C   "
            f"up {up_mbit:6.2f} Mbit/s   warn 0x{sp.warn:02X}")


# ---------------------------------------------------------------------------
# Self test
# ---------------------------------------------------------------------------

def selftest() -> None:
    """Checks the mapping with synthetic readings, so no hardware is needed."""
    opt = MapOptions(max_download_mbit=100.0, smoothing_seconds=0.0)
    mapper = Mapper(opt)

    full_scale = opt.max_download_mbit * BITS_PER_MEGABIT / BITS_PER_BYTE

    # Half the rated download, half the memory, a quarter of the processor.
    half = Reading(download_bytes_per_s=full_scale / 2,
                   memory_used=0.5, cpu_load=0.25, cpu_celsius=55.0)
    sp = mapper.to_setpoints(half, now=0.0)
    assert sp.speed_kmh == 130, sp.speed_kmh
    assert sp.rpm == 2750, sp.rpm
    assert abs(sp.fuel_l - 15.0) < 0.01, sp.fuel_l
    assert sp.coolant_c == 55, sp.coolant_c
    assert sp.warn == 0, hex(sp.warn)
    assert sp.fuel_direct, "load on the fuel gauge requires the damping off"
    assert sp.kl15 and sp.engine, "the cluster must see a running engine"

    # Full scale on every dial, and a link faster than the rated speed must
    # peg the needle rather than wrap around.
    mapper = Mapper(opt)
    loaded = Reading(download_bytes_per_s=full_scale * 4,
                     memory_used=1.0, cpu_load=1.0, cpu_celsius=95.0)
    sp = mapper.to_setpoints(loaded, now=0.0)
    assert sp.speed_kmh == opt.max_speed_kmh, sp.speed_kmh
    assert sp.rpm == opt.max_rpm, sp.rpm
    assert abs(sp.fuel_l - opt.tank_litres) < 0.01, sp.fuel_l
    assert sp.warn & lk.WARN_CHECK_ENGINE
    assert sp.warn & lk.WARN_FUEL
    assert sp.warn & lk.WARN_COOLANT

    # An idle machine sits at the bottom of every dial.
    mapper = Mapper(opt)
    idle = Reading(cpu_celsius=40.0)
    sp = mapper.to_setpoints(idle, now=0.0)
    assert sp.speed_kmh == 0 and sp.rpm == 0, (sp.speed_kmh, sp.rpm)
    assert sp.fuel_l == 0.0

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

    assert "down" in describe(half, sp, opt)
    print("sysmon selftest OK  (mapping, full scale, idle, smoothing, missing sensors)")


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
