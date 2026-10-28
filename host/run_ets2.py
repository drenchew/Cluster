#!/usr/bin/env python3
"""Drive the E90 cluster bench jig from ETS2 / ATS telemetry.

    python3 run_ets2.py --selftest        layout + mapping checks, no game
    python3 run_ets2.py --source mock     scripted drive, no game needed
    python3 run_ets2.py --port /dev/ttyUSB0
    python3 run_ets2.py --list            serial ports and exit

Needs a telemetry plugin loaded in the game and the *native* Linux build of
the game (or a native Windows build) - a Proton run keeps the shared memory
inside its Wine prefix where this process cannot see it. See README.

Two plugins work. Ours, in plugin/, is the default and the one that builds
on Linux with nothing but g++:

    cd plugin && make && make install

--source shm reads RenCloud's 32 kB layout instead, for a machine that
already has that plugin installed.

Same link as run.py: we push setpoints at 50 Hz and the firmware owns CAN
timing, so quitting here (or losing the game) never resets the cluster - it
falls back to its own demo sweep after 1.5 s.
"""

from __future__ import annotations

import argparse
import sys
import time

from cluster_sim import ets2
from cluster_sim import ets2_plugin
from cluster_sim import link as lk

SEND_HZ = 50.0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="E90 cluster bench - ETS2 telemetry")
    ap.add_argument("--port", help="serial port (default: auto-detect)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--source", default="plugin", choices=("plugin", "shm", "mock"),
                    help="plugin = our plugin's block (default), "
                         "shm = RenCloud's 32 kB layout, mock = scripted drive")
    ap.add_argument("--selftest", action="store_true",
                    help="check the struct layout and mapping, then exit")
    ap.add_argument("--probe", action="store_true",
                    help="list what is in /dev/shm and which block we would use")
    ap.add_argument("--setup", action="store_true",
                    help="find the Steam/ETS2 install, say where the plugin "
                         "goes and what would stop telemetry working")
    ap.add_argument("--shm-name",
                    help="block to read, as a name or an absolute path. With "
                         "--source plugin the default is "
                         "/dev/shm/ClusterTelemetry; with --source shm the "
                         "known fork names are tried, then /dev/shm is scanned")
    ap.add_argument("--no-serial", action="store_true",
                    help="read telemetry and print it, but do not open the port")
    ap.add_argument("--rate", type=float, default=SEND_HZ, help="frames per second to the jig")
    ap.add_argument("--rpm-scale", type=float, default=1.0,
                    help="multiply engine rpm (a 2100 rpm truck engine on a car dial)")
    ap.add_argument("--speed-scale", type=float, default=1.0)
    ap.add_argument("--tank-l", type=float, default=60.0,
                    help="car-tank size the fuel gauge reads as full-scale")
    ap.add_argument("--lights-on", action="store_true",
                    help="force side + low beam regardless of the game")
    ap.add_argument("--clock", default="game", choices=("game", "system", "off"),
                    help="what to set the cluster clock to: the in-game time "
                         "(default), the PC clock, or leave it alone")
    ap.add_argument("-v", "--verbose", action="store_true", help="one status line per second")
    args = ap.parse_args(argv)

    if args.list:
        ports = lk.available_ports()
        if not ports:
            print("no serial ports found")
        for device, desc in ports:
            print(f"{device}\t{desc}")
        return 0

    if args.selftest:
        ets2.selftest()
        ets2_plugin.selftest()
        return 0

    if args.setup:
        return 0 if ets2.setup_report() == 0 else 1

    if args.probe:
        entries = ets2.list_shm()
        if not entries:
            print("nothing in /dev/shm (or not a Linux host)")
        for name, size in entries:
            mark = "  <- right size" if size >= ets2.BYTES_UPTO_ZONE10 else ""
            print(f"{size:>9}  {name}{mark}")
        print()
        print(f"our plugin publishes {ets2_plugin.SHM_PATH} "
              f"({ets2_plugin.STRUCT_SIZE} bytes in a "
              f"{ets2_plugin.SHM_SIZE} byte mapping)")
        import os as _os
        print("  present" if _os.path.exists(ets2_plugin.SHM_PATH)
              else "  not there - is the game running with the plugin installed?")
        print()
        print("--source shm would try these, in order:")
        for c in ets2.shm_candidates(args.shm_name):
            print(f"  {c}")
        return 0

    if args.source == "mock":
        source = ets2.MockSource()
    elif args.source == "shm":
        source = ets2.ShmSource(name=args.shm_name)
    else:
        source = ets2_plugin.Source(path=args.shm_name)
    opts = ets2.MapOptions(
        rpm_scale=args.rpm_scale,
        speed_scale=args.speed_scale,
        tank_l=args.tank_l,
        lights_on=args.lights_on,
        clock=args.clock,
    )

    link = None
    if not args.no_serial:
        try:
            link = lk.ClusterLink(args.port, args.baud)
        except RuntimeError as exc:
            print(exc, file=sys.stderr)
            return 2

    interval = 1.0 / max(1.0, args.rate)
    next_send = time.monotonic()
    next_status = time.monotonic()
    live_prev: bool | None = None
    frames = 0
    warned: str | None = None
    status = "starting"

    port_txt = "disabled" if args.no_serial else (
        args.port or lk.guess_port() or "auto")
    print(f"source: {args.source}   serial: {port_txt}")
    print("waiting for telemetry (Ctrl-C to quit)")
    try:
        while True:
            now = time.monotonic()
            if now < next_send:
                time.sleep(min(0.004, next_send - now))
                continue
            next_send = max(now, next_send + interval)

            frame = source.poll()
            if frame is not None:
                sp = ets2.to_setpoints(frame, opts)
                # A real DSC flashes its lamp while it is intervening. The
                # mapping reports the intervention; the blinking belongs here,
                # where the clock is - the firmware only knows on and off.
                if sp.dsc_lamp:
                    sp.dsc_lamp = int(now * 4.5) % 2 == 0
                if link is not None:
                    link.send(sp)
                frames += 1
                status = _describe(sp)
                live = True
                # A plugin whose struct revision we do not mirror would feed
                # plausible-looking nonsense, so say it once and loudly.
                warn = getattr(source, "revid_warning", None)
                if warn and warn != warned:
                    warned = warn
                    print(f"WARNING: {warn}", file=sys.stderr, flush=True)
            else:
                live = False
                err = source.last_error or "game not producing frames"
                status = f"no telemetry ({err}) - cluster runs its own demo sweep"

            # One line on every state change, then a slow tick: a second while
            # live (with -v), five seconds while waiting for the game.
            if live != live_prev or now >= next_status:
                period = (1.0 if args.verbose else 2.0) if live else 5.0
                next_status = now + period
                live_prev = live
                # The link only opens on the first frame we actually have to
                # send, so before the game speaks we have not touched the port
                # at all - say that rather than crying "down".
                if link is None:
                    link_state = "disabled"
                elif link.connected:
                    link_state = "ok"
                elif link.last_error:
                    link_state = link.last_error
                else:
                    link_state = "not opened yet"
                extra = ""
                name = getattr(source, "name", None)
                rev = getattr(source, "revid", None)
                if live and name:
                    extra = f", {name}"
                    if rev is not None:
                        extra += f" rev{rev}"
                print(f"{_stamp()}  {status}   "
                      f"[{frames} frames, link {link_state}{extra}]", flush=True)
                frames = 0
    except KeyboardInterrupt:
        pass
    finally:
        source.close()
        if link is not None:
            link.close()
    return 0


def _describe(sp: lk.Setpoints) -> str:
    gear = {lk.GEAR_R: "R", lk.GEAR_N: "N", lk.GEAR_D: "D", lk.GEAR_P: "P"}.get(sp.gear_pos, "?")
    extra = ""
    if sp.clock is not None:
        extra += f"  {sp.clock.hhmm()}"
    if sp.dsc_lamp:
        extra += "  DSC"
    if sp.tc_lamp:
        extra += "  DTC"
    return (f"{sp.speed_kmh:3d} km/h  {sp.rpm:4d} rpm  {gear}{sp.gear_num or ''}  "
            f"park {'on ' if sp.handbrake else 'off'}  fuel {sp.fuel_l:4.1f} l  "
            f"{sp.coolant_c:3d} C  lights 0x{sp.lights:02X}{extra}")


def _stamp() -> str:
    return time.strftime("%H:%M:%S")


if __name__ == "__main__":
    raise SystemExit(main())
