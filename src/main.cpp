/**
 * @file main.cpp
 * @brief Entry points for the Arduino runtime.
 *
 * BMW E9x instrument cluster driver. The board presents a vehicle to the
 * cluster over K-CAN so that the dials, telltales and on-board computer can be
 * operated on a bench. Setpoints arrive either from a host simulator over USB
 * or from the typed console; when neither is present a local sweep keeps the
 * cluster supplied with plausible data.
 *
 * All structure lives in Application, which constructs and connects the
 * objects involved. These two functions exist only because the Arduino
 * runtime requires them.
 *
 * Console commands:
 *   r 3000   engine speed, rpm      s 100   road speed, km/h
 *   p 50     accelerator, percent   f 35    fuel, litres
 *   c 90     coolant, degrees C     t 20    outside air, degrees C
 *   l 5      lighting bitmask       b 200   instrument dimmer, 0..255
 *   g 8      gear: 1 P, 2 R, 4 N, 8 D
 *   n 1      indicators: 0 off, 1 left, 2 right, 3 hazard
 *   h 1      handbrake              e 0|1   engine running
 *   k 0|1    terminal 15            d       resume the demonstration sweep
 *   o 0|1|2  distance counter: frozen, alive, integrating
 *   x 0..3   extra speed frames: 1 adds 0x1A0, 2 adds 0x0CE
 *   w 1435   set the clock, HHMM or HHMMSS
 *   u 0..7   stability telltales: 1 DTC, 2 triangle, 4 disabled
 *   m 0..255 warning telltales: 1 check engine, 2 oil, 4 battery,
 *            8 coolant, 16 fuel, 32 AdBlue, 64 brake, 128 tyre pressure
 *   v 0..250 cruise control set speed, km/h; 0 switches it off
 *   y 357 1  display or clear any check control symbol by identifier
 *   i 0..100 instant consumption demand, drives the mpg needle
 *   j <n>    force the injected-fuel rate to n counts per 100 ms,
 *            negative to return to the demand percentage
 *   z <n>    0x19E byte 5, the brake system status byte
 *   a 0|1    fuel gauge damping: 1 behaves like a tank, 0 follows
 *            the setpoint at once for a live measurement
 *   ?        print driver and link state
 */

#include "Application.h"

namespace {
Application application;
}  // namespace

void setup() {
    application.begin();
}

void loop() {
    application.run();
}
