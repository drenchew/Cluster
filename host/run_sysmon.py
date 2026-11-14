#!/usr/bin/env python3
"""Drive the E9x cluster from this machine's own vital signs.

    python3 run_sysmon.py --selftest        mapping checks, no hardware
    python3 run_sysmon.py --no-serial       print what would be sent
    python3 run_sysmon.py                   drive the cluster
    python3 run_sysmon.py --list            serial ports and exit

The dials read:

    speedometer        network download rate  (full scale --max-mbit)
    tachometer         memory in use          (full scale 5500 rpm)
    consumption gauge  processor load         (0 to 100 percent demand)
    fuel gauge         battery charge         (a full tank is a full battery)
    on-board clock     this machine's clock

Those are the four needles this cluster has. Processor temperature is also
sent, as the coolant temperature, but this variant has no needle for it: it
appears as the overheat telltale and in the hidden service menu.

Telltales light for sustained processor load, memory nearly exhausted and a
hot processor.

Processor load is on the consumption gauge rather than the fuel gauge: the
cluster raises its own low fuel warning from the level it is sent, so an idle
machine on the fuel gauge would light the reserve telltale and sound the
chime, and a warning the cluster raised itself cannot be cleared from here.
The consumption gauge has no warning at either end.

Battery charge takes the fuel gauge, which is the natural fit: both are a
reserve being consumed, and the cluster's own low fuel warning then means a
low battery. A machine with no battery falls back to free filesystem space so
the needle still shows a reserve. Raise --fuel-floor-l to suppress the
warning.

The fuel gauge damping is switched off per frame, because the gauge is showing
a live measurement rather than the level in a tank. The firmware obeys that
request, so the same build serves this mode and the vehicle telemetry modes
and nothing needs reflashing to switch between them.

Setpoints are pushed at 50 Hz and the firmware owns the CAN timing, so
stopping this program never resets the cluster: it falls back to its own demo
sweep after 1.5 s.
"""

from __future__ import annotations

import argparse
import sys
import time

from cluster_sim import link as lk
from cluster_sim import sysmon

SEND_HZ = 50.0
SAMPLE_HZ = 10.0
STATUS_PERIOD_S = 1.0
IDLE_STATUS_PERIOD_S = 5.0


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="E9x cluster as a system monitor")
    parser.add_argument("--port", help="serial port (default: auto-detect)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--list", action="store_true", help="list serial ports and exit")
    parser.add_argument("--selftest", action="store_true",
                        help="check the mapping, then exit")
    parser.add_argument("--no-serial", action="store_true",
                        help="read and print metrics without opening the port")

    parser.add_argument("--iface", action="append",
                        help="network interface to count; repeat for several "
                             "(default: every interface that is up)")
    parser.add_argument("--max-mbit", type=float, default=100.0,
                        help="download rate that reads as full scale (default 100)")
    parser.add_argument("--max-speed", type=int, default=260,
                        help="speedometer full scale in km/h (default 260)")
    parser.add_argument("--max-rpm", type=int, default=5500,
                        help="tachometer full scale (default 5500)")
    parser.add_argument("--tank-l", type=float, default=60.0,
                        help="fuel gauge full scale in litres (default 60)")
    parser.add_argument("--fuel-floor-l", type=float, default=0.0,
                        help="litres the fuel gauge reads at zero charge. "
                             "Raise it to keep clear of the cluster's own "
                             "reserve warning near 8 l (default 0, which lets "
                             "that warning act as a low battery alarm)")
    parser.add_argument("--disk", default="/",
                        help="filesystem shown on the fuel gauge (default /)")
    parser.add_argument("--smoothing", type=float, default=0.45,
                        help="needle smoothing half-life in seconds (default 0.45)")
    parser.add_argument("--no-clock", action="store_true",
                        help="leave the cluster clock alone")
    parser.add_argument("--rate", type=float, default=SEND_HZ,
                        help="frames per second to the jig")
    parser.add_argument("--sample-hz", type=float, default=SAMPLE_HZ,
                        help="how often to read the kernel counters")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="one status line per second")
    return parser


def _stamp() -> str:
    return time.strftime("%H:%M:%S")


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)

    if args.list:
        ports = lk.available_ports()
        if not ports:
            print("no serial ports found")
        for device, description in ports:
            print(f"{device}\t{description}")
        return 0

    if args.selftest:
        sysmon.selftest()
        return 0

    metrics = sysmon.SystemMetrics(interfaces=args.iface, disk_path=args.disk)
    mapper = sysmon.Mapper(sysmon.MapOptions(
        max_download_mbit=args.max_mbit,
        max_speed_kmh=args.max_speed,
        max_rpm=args.max_rpm,
        tank_litres=args.tank_l,
        fuel_floor_litres=args.fuel_floor_l,
        smoothing_seconds=args.smoothing,
        show_clock=not args.no_clock,
    ))

    link = None if args.no_serial else lk.ClusterLink(args.port, args.baud)
    port_text = "disabled" if args.no_serial else (
        args.port or lk.guess_port() or "auto")

    print(f"interfaces : {', '.join(metrics.interfaces) or 'none found'}")
    print(f"temperature: {metrics.temperature_path or 'no sensor found'}")
    print(f"battery    : {metrics.battery_path or 'none, using filesystem instead'}")
    print(f"full scale : {args.max_mbit:.0f} Mbit/s -> {args.max_speed} km/h, "
          f"memory -> {args.max_rpm} rpm, load -> consumption gauge, "
          f"charge -> {args.fuel_floor_l:.0f}..{args.tank_l:.0f} l")
    print(f"serial     : {port_text}")
    print()
    if not metrics.interfaces:
        print("no network interface is up, so the speedometer will stay at zero",
              file=sys.stderr)

    send_period = 1.0 / max(1.0, args.rate)
    sample_period = 1.0 / max(0.5, args.sample_hz)
    next_send = time.monotonic()
    next_sample = time.monotonic()
    next_status = 0.0
    reading = sysmon.Reading()
    have_reading = False
    frames = 0

    try:
        while True:
            now = time.monotonic()

            if now >= next_sample:
                next_sample = now + sample_period
                fresh = metrics.sample()
                if fresh is not None:
                    reading = fresh
                    have_reading = True

            if now >= next_send:
                next_send = now + send_period
                # The mapper is called every frame even between samples: its
                # smoothing is time based, so the needles continue easing
                # towards the latest reading instead of stepping once per
                # sample.
                sp = mapper.to_setpoints(reading, now)
                if link is not None and link.send(sp):
                    frames += 1

                if now >= next_status:
                    live = have_reading
                    next_status = now + (STATUS_PERIOD_S if args.verbose
                                         else IDLE_STATUS_PERIOD_S)
                    if link is None:
                        link_state = "disabled"
                    elif link.connected:
                        link_state = "ok"
                    elif link.last_error:
                        link_state = link.last_error
                    else:
                        link_state = "not opened yet"
                    status = (sysmon.describe(reading, sp, mapper.options) if live
                              else "waiting for a second sample")
                    print(f"{_stamp()}  {status}   [{frames} frames, link {link_state}]",
                          flush=True)
                    frames = 0

            # Sleep until the next thing is due rather than spinning.
            delay = min(next_send, next_sample) - time.monotonic()
            if delay > 0:
                time.sleep(delay)
    except KeyboardInterrupt:
        pass
    finally:
        if link is not None:
            link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
