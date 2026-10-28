#!/usr/bin/env python3
"""What resistor makes the cluster's on-board computer read a given temperature.

The outside temperature on an E9x KOMBI does **not** come off the CAN bus. The
cluster measures it itself, through a resistive sensor wired across connector
**pins 4 and 5**. With nothing there the input reads open circuit and the
on-board computer pegs at the bottom of its scale - that is the "-40" (same
number in C and F, which is the giveaway) you see on a bench rig.

So the fix is a resistor, not a frame. The sensor is an NTC thermistor,
5 kOhm at 25 C with a beta of 3950:

    R(T) = 5000 * exp(3950 * (1/(T+273.15) - 1/298.15))   ohms

    python3 ambient_resistor.py              # the whole table
    python3 ambient_resistor.py --temp 18    # what to wire for 18 C
    python3 ambient_resistor.py --ohms 10000 # what 10k will read

Note the cluster filters this input hard - a change takes a minute or two to
work through to the display, and it will not move at all with KL15 off. A
trimpot or a digital potentiometer (see the USE_AD5272_AMBIENT option in
veikkos/e90-can-cluster) is the comfortable way to play with it.
"""

from __future__ import annotations

import argparse
import math

R0 = 5000.0        # ohms at 25 C
T0 = 298.15        # 25 C in kelvin
BETA = 3950.0

# E12, the series every junk-box resistor belongs to.
E12 = (1.0, 1.2, 1.5, 1.8, 2.2, 2.7, 3.3, 3.9, 4.7, 5.6, 6.8, 8.2)


def ohms_for(celsius: float) -> float:
    return R0 * math.exp(BETA * (1.0 / (celsius + 273.15) - 1.0 / T0))


def celsius_for(ohms: float) -> float:
    if ohms <= 0.0:
        return float("inf")
    return 1.0 / (math.log(ohms / R0) / BETA + 1.0 / T0) - 273.15


def nearest_e12(ohms: float) -> float:
    """Closest E12 value, in ohms, searched a decade either side."""
    best = None
    for decade in (1e1, 1e2, 1e3, 1e4, 1e5):
        for m in E12:
            v = m * decade
            if best is None or abs(math.log(v / ohms)) < abs(math.log(best / ohms)):
                best = v
    return best


def _fmt(ohms: float) -> str:
    return f"{ohms / 1000.0:.2f} kOhm" if ohms >= 1000.0 else f"{ohms:.0f} Ohm"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="E9x cluster outside-temperature sensor (pins 4-5)")
    ap.add_argument("--temp", type=float, help="target temperature, degC")
    ap.add_argument("--ohms", type=float, help="resistance you have, ohms")
    args = ap.parse_args(argv)

    if args.temp is not None:
        want = ohms_for(args.temp)
        pick = nearest_e12(want)
        print(f"{args.temp:+.0f} C  wants {_fmt(want)}")
        print(f"          nearest E12 {_fmt(pick)} reads "
              f"{celsius_for(pick):+.1f} C / {celsius_for(pick) * 9 / 5 + 32:.0f} F")
        return 0

    if args.ohms is not None:
        c = celsius_for(args.ohms)
        print(f"{_fmt(args.ohms)} reads {c:+.1f} C / {c * 9 / 5 + 32:.0f} F")
        return 0

    print("  degC    degF   resistance   nearest E12 (and what it reads)")
    for t in range(-20, 41, 5):
        want = ohms_for(t)
        pick = nearest_e12(want)
        print(f"  {t:+4d}   {t * 9 / 5 + 32:5.0f}   {_fmt(want):>10}   "
              f"{_fmt(pick):>10}  -> {celsius_for(pick):+5.1f} C")
    print("\n10 kOhm across pins 4 and 5 is the usual bench choice: about 10 C,")
    print("which is warm enough to clear the cold-weather (snowflake) warning.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
