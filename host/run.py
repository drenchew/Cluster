#!/usr/bin/env python3
"""Drive the E90 cluster bench jig from the keyboard.

    python run.py                  # auto-detect the Arduino port
    python run.py --port COM6
    python run.py --list           # show serial ports and exit
    python run.py --no-serial      # sim + GUI + audio only, no hardware
    python run.py --audio synth    # auto | synth | samples | off

The simulator runs at the display refresh rate and pushes setpoints to the
firmware at 50 Hz. The firmware owns CAN timing, so quitting this program is
safe: the Arduino keeps its 0x130 heartbeat alive and falls back to its own
demo sweep rather than letting the cluster reset.
"""

from __future__ import annotations

import argparse
import sys
import time

from cluster_sim import audio as au
from cluster_sim import link as lk
from cluster_sim.vehicle import Vehicle

SEND_HZ = 50.0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="E90 cluster bench simulator")
    ap.add_argument("--port", help="serial port (default: auto-detect)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--no-serial", action="store_true", help="run without hardware")
    ap.add_argument("--audio", default="auto", choices=("auto", "synth", "samples", "off"))
    ap.add_argument("--volume", type=float, default=0.6)
    args = ap.parse_args(argv)

    if args.list:
        ports = lk.available_ports()
        if not ports:
            print("no serial ports found")
        for device, desc in ports:
            print(f"{device}\t{desc}")
        return 0

    try:
        import pygame  # noqa: F401
    except ImportError:
        print("pygame is required for the GUI:\n  pip install -r requirements.txt",
              file=sys.stderr)
        return 2

    from cluster_sim.gui import Dashboard

    link = None
    port_label = "no serial"
    if not args.no_serial:
        try:
            link = lk.ClusterLink(args.port, args.baud)
            port_label = args.port or (lk.guess_port() or "auto")
        except RuntimeError as exc:
            print(f"{exc}\nContinuing without hardware.", file=sys.stderr)

    engine = au.make_engine(args.audio, args.volume)
    engine.start()
    if not engine.available and args.audio != "off":
        print(f"audio unavailable: {au.audio_status()}", file=sys.stderr)

    vehicle = Vehicle()
    dash = Dashboard(vehicle, engine, link, port_label)

    send_interval = 1.0 / SEND_HZ
    next_send = time.monotonic()
    try:
        while dash.running:
            dt = dash.tick()
            dash.handle_events()
            vehicle.step(dt)
            # Pushes rpm/boost/fuelling plus the one-shot shift, wastegate and
            # start events, so the audio hears what the engine is doing rather
            # than inferring it from the pedal.
            au.drive_from_vehicle(engine, vehicle)

            now = time.monotonic()
            if link is not None and now >= next_send:
                # Catch up without bursting if a frame hitched.
                next_send = max(now, next_send + send_interval)
                link.send(dash.setpoints())
            dash.pump_log()
            dash.draw(dt)
    except KeyboardInterrupt:
        pass
    finally:
        engine.stop()
        if link is not None:
            link.close()
        dash.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
