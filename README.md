# BMW E9x instrument cluster bench jig

Drives a BMW E90/E91/E92/E93 instrument cluster (tested against
**A2C53117692**, the diesel KOMBI) from an Arduino Uno over K-CAN, with a PC
simulator that lets you drive it from the keyboard like a game.

```
 keyboard ──► host/run.py ──USB serial──► Arduino Uno ──SPI──► MCP2515 ──K-CAN──► cluster
              vehicle sim                 frame builder         100 kbit/s
              engine audio                + CAN timing
              dashboard
```

## Hardware

| Uno | HW-184 (MCP2515) | note |
|-----|------------------|------|
| D10 | CS               | hardware SS |
| D13 | SCK              | hardware SPI, do not move |
| D11 | SI               | hardware SPI, do not move |
| D12 | SO               | hardware SPI, do not move |
| D2  | INT              | diagnostics only, TX does not need it |
| 5V  | VCC              | |
| GND | GND              | |

Cluster connector: **CAN-H → pin 6, CAN-L → pin 7, +12 V → pin 9, GND → pin 18.**

Two things that bite people:

- **The cluster is on K-CAN at 100 kbit/s**, not the 500 kbit/s PT-CAN. At
  500k it never answers.
- **Bus termination.** The HW-184 carries a 120 Ω terminator. K-CAN in the car
  uses distributed termination inside the modules instead, and at least one
  bench build only stopped resetting after the shield's terminator was
  removed. If you get unexplained resets, check this.

## Firmware

```
pio run                 # build
pio run -t upload       # flash
pio device monitor      # 115200 baud
```

The firmware owns CAN timing and the whole cluster state. The PC only supplies
setpoints, so closing the GUI or pulling the USB cable cannot reset the
cluster — the 0x130 heartbeat keeps running and the local demo sweep takes
over within 1.5 s.

### Typed commands

Work on the same port as the GUI, for poking at one thing in isolation. They
are ignored while the GUI is connected.

```
r 3000   rpm                s 100   speed km/h          p 50   pedal %
f 35     fuel litres        c 90    coolant degC        t 20   outside degC
l 5      lights mask        b 200   dimmer 0..255       g 8    gear 1P 2R 4N 8D
n 1      turn: 0 off, 1 left, 2 right, 3 hazard
h 0|1    handbrake          e 0|1   engine running      k 0|1  terminal 15
o 0|1|2  0x1A6 distance accumulator: frozen / alive / distance
x 0..3   extra speed frames: +1 = 0x1A0, +2 = 0x0CE (both on by default)
d        resume demo sweep  ?       print state
```

### If the speedometer needle will not move

The speedometer was the one thing on this jig that never responded, and the
reason is now pinned down: **0x1A6 bytes 0–5 are not a speed value.** They are
three copies of a *cumulative distance accumulator* that the cluster
differentiates to get speed. The jig used to write a constant speed word into
them, the cluster differentiated a constant, got 0, and parked the needle —
while every other gauge, fed by direct-value frames, worked normally.

Three frames carry speed, and all three are sent by default:

| frame | field | rate | note |
|---|---|---|---|
| 0x1A0 | 12-bit V_VEH, 0.1 km/h/count | 100 ms | direct value, checksummed |
| 0x1A6 | 16-bit distance accumulator ×3 | 100 ms | cluster differentiates it |
| 0x0CE | 4 × signed 16-bit wheel speed, 0.636 km/h/count | 100 ms | DSC wheel speeds |

All three come from the DSC in a real car, and a real capture has them side by
side. Nothing below needs a reflash:

```
s 60        hold 60 km/h, then watch the needle after each step below
o 2         accumulator tracks distance      (the default, and what a car does)
o 1         accumulator +1 per frame         (alive-counter test, needle reads 0)
o 0         accumulator frozen               (proves whether the needle reads it)
x 0         drop 0x1A0 and 0x0CE, leaving 0x1A6 alone
x 3         0x1A6 + 0x1A0 + 0x0CE              (the default)
x 1         0x1A6 + 0x1A0 only
x 2         0x1A6 + 0x0CE wheel speeds only
```

## PC simulator

```
cd host
pip install -r requirements.txt
python run.py                 # auto-detects the Arduino port
python run.py --port COM6
python run.py --list          # list serial ports
python run.py --no-serial     # sim + GUI + audio, no hardware
python run.py --audio off
```

`pygame-ce` is in requirements rather than `pygame` because it is the fork
that ships wheels for current Python (3.13/3.14). It imports as `pygame`, so
nothing in the code changes.

### Controls

| key | |
|---|---|
| `W` / `Up` | throttle |
| `S` / `Down` | brake |
| `Space` | handbrake |
| `I` | terminal 15 (ignition) |
| `Enter` | hold to crank the starter |
| `K` | kill the engine |
| `A` / `Left`, `D` / `Right` | indicators |
| `Z` | hazards |
| `G` | shift lever: P → R → N → D |
| `M` | auto / manual gearbox |
| `E` / `Q` | shift up / down (manual) |
| `1`–`6` | select a gear directly |
| `L` | lights: off → side → low → high |
| `F` / `V` | front / rear fog |
| `[` / `]` | dimmer down / up |
| `-` / `=` | fuel down / up |
| `P` | sport shift map |
| `Tab` | show / hide help |
| `Esc` | quit |

To get moving from cold: hold `Enter` until the engine catches (ignition is
already on), `G` for `D`, then `W`. The handbrake starts **released** and the
lever starts in `P`, so a freshly started bench shows no park-brake telltale
and the car sits still — `Space` pulls it again if you want the gong.

### Vehicle model

Tuned to the 2.0 diesel the cluster came behind: 800 rpm idle, torque plateau
from 1750–2750, 4600 rpm fuel cut, 6-speed with a torque-converter launch.
The converter matters — without a slipping phase, selecting a gear at a
standstill locks the engine to stationary wheels and stalls it instantly.

### Engine audio

Default is **procedural pulse-train synthesis**, not samples. A 4-stroke fires
each cylinder once per two crank revolutions, so a 4-cylinder fires twice per
revolution and the firing frequency is `rpm/30` Hz — 27 Hz at idle, 150 Hz at
4500. Engine noise is a train of exhaust pressure pulses coloured by the
resonance of the exhaust tract rather than a sustained harmonic tone, so that
is what `audio.py` builds: an impulse per firing event through a resonator,
plus injector clatter and load-dependent turbo whine. Phase is continuous
across buffers, so pitch glides instead of stepping.

**On real samples:** I looked, and there is no realistic multi-rpm engine
sample set available under a license that allows redistribution. The one CC0
"racing car engine loops" pack on OpenGameArt is pitch-shifted from a car
*seatbelt warning chime*, not an engine recording — so shipping it would have
been worse than synthesis. If you obtain a sample set yourself, drop looping
mono or stereo WAVs into `host/samples/` named by the rpm they were recorded
at and the sample back end is picked up automatically:

```
host/samples/rpm_0800.wav
host/samples/rpm_1500.wav
host/samples/rpm_2500.wav
host/samples/rpm_3500.wav
```

They get resampled and crossfaded by rpm. Force the choice with
`--audio samples` or `--audio synth`.

## Frames

Confidence column: **OK** means corroborated by a working bench capture and/or
a DBC signal definition; **?** means the layout is a documented guess and the
group defaults to *off* in `include/Config.h`. Enable those one at a time.

| ID | rate | what | conf |
|---|---|---|---|
| 0x130 | 100 ms | terminal status — keeps the cluster awake | OK |
| 0x0AA | 20 ms | engine rpm (B4..B5 = rpm × 4) + pedal | OK |
| 0x1A6 | 100 ms | cumulative distance accumulator ×3 + time counter | OK |
| 0x1B4 | 100 ms | speed + handbrake (B5: 0x30 off, 0x32 on; B1 `0xC0`, B2 `0xE2`) | OK |
| 0x34F | event + 5 s | handbrake status, DLC 2: `FD FF` released / `FE FF` on | OK |
| 0x21A | 100 ms | illumination / backlight + light telltales | OK |
| 0x1F6 | 200 ms | turn signals | OK |
| 0x26E | 200 ms | ignition status | OK |
| 0x1D0 | 200 ms | coolant temperature (B0 = degC + 48) | OK |
| 0x349 | 200 ms | fuel level, DLC 5, 160 counts/litre | OK |
| 0x19E / 0x0C0 / 0x0D7 | 200 ms | DSC / ABS / airbag counters | OK |
| 0x1A0 | 100 ms | "Speed K-CAN" — direct speed, **on by default** | OK |
| 0x0CE | 100 ms | DSC wheel speeds — four signed 16-bit LE, **on by default** | OK |
| 0x1D2 | 200 ms | gear display P R N D, D1..D6 | ? |
| 0x202 / 0x2C0 | 200 ms | dimmer level / LCD illumination | ? |
| 0x3B0 | 200 ms | reverse gear | ? |
| 0x366 | 1 s | outside temperature + range | ? |

### Four things that will bite you if you change them

**0x130 byte 4 must stay constant.** Its high nibble is a sequence number and
its low nibble a check value derived from it, so incrementing one alone yields
a byte the cluster rejects. It then reads terminal status as implausible and
re-runs its power-on sequence — gong and every telltale — in a loop. Both
payloads reported working keep B4 fixed: `0xFE` (raw capture) or `0xE2`
(e46canbus blog). What must not stall is the *frame*, not that byte.

**Never command 0 rpm while terminal 15 is on.** The cluster reads it as the
engine stalling and answers like a real car: gong plus the charge,
oil-pressure, brake and DSC telltales. `setRPM()` floors at
`cfg::ENGINE_IDLE_RPM`; use `e 0` to simulate a stall deliberately.

**0x1A6 bytes 0–5 must not hold a constant.** The cluster treats the field as a
rate source: freeze the accumulator (or move the bytes 6–7 time counter) and it
reads the frame as stale, or derives 0 km/h and drops the needle. `cfg::ODO_MODE`
picks what gets integrated:

| mode | bytes 0–5 | speedo | odometer |
|---|---|---|---|
| `ODO_FROZEN` | held at 0 | **dead** | untouched |
| `ODO_ALIVE` | +1 per frame (no distance) | reads the +1/frame rate, ~0 km/h | untouched |
| `ODO_DISTANCE` (default) | `km/h × 13.0 counts/s` | correct | **accrues real mileage, not reversible** |

Switch live with `o 0`/`o 1`/`o 2` — no reflash — to see the difference.

**0x1D0 byte 2 is a live-alive nibble plus an engine state, not a counter.**
Bits 0–3 count 0…15 and bits 4–5 are the engine state: 0 off, 1 starting,
2 running, 3 invalid. Sending a raw `cnt1D0_++` rolls the state nibble through
*off* and *invalid* for a quarter of every 256 frames while the cluster is also
being told "terminal 15 on, engine running", which is the stall response above:
gong plus charge / oil / brake / DSC telltales. The state nibble is pinned to
`0x20` while `engineOn_` and only the low nibble counts.

**Never send a DSC frame with its counter frozen.** 0x19E (DSC status) was a
constant payload with no changing byte anywhere in it; a sender whose status
never once changes reads as dead, and the ABS / DSC / tyre-pressure telltales
stay lit. Its byte 7 now counts, and the payload matches the working veikkos
one — the old `B5 = 0x43` never occurs in 1154 real frames (real values:
`0x00 0x41 0x61 0x40`).

### Why 13.0 counts/s per km/h, and the +400 time tick

Measured from a 68.7 s / 1154-frame capture of a real E90 M3 on K-CAN:

- Bytes 0–5 climb monotonically 0 → 8793 while speed goes 0 → 15.9 km/h, at
  **12.3–13.7 counts per km/h per second**, reading `00 00` at a standstill.
  The three copies drift independently (copy3 − copy1 goes −1 → +658), so they
  are three views of one accumulator, not three wheel speeds.
- Bytes 6–7 are a **12-bit time counter**: the mean per-frame delta is
  exactly 400.0, it wraps at 4096, and B7's high nibble is always `0xF`.
  It ticks while the car is parked — it is not distance.

`cfg::ODO_COUNTS_PER_KMH_PER_SEC = 13.0f` is the rate the working
[veikkos/e90-can-cluster](https://github.com/veikkos/e90-can-cluster) driver
uses for a speedo that reads true; its odometer-exact figure is 13.6.
`cfg::TICK_1A6_STEP = 400` reproduces the time counter.

**Cadence matters as much as the number.** The real cadence is 100 ms, so the
scheduler puts 0x1A6 on slots 7 and 17 only (0x1A0 on 1 and 11, 0x0AA keeps its
20 ms slots). Sending the accumulator faster makes the derived speed scale
proportionally — the old 20 ms rate would have read 5× too fast even with the
right integrator.

## Layout

```
include/Config.h          every tunable, with confidence notes per frame group
src/ClusterCan.{h,cpp}    frame builders + 10 ms TX scheduler
src/Protocol.{h,cpp}      binary host link + host-timeout watchdog
src/Demo.{h,cpp}          local sweep when no host is connected
src/main.cpp              wiring, text console
E90_Cluster/              standalone ESP32 sketch (same fixes applied)
host/run.py               simulator entry point
host/cluster_sim/link.py      serial framing, must match src/Protocol.h
host/cluster_sim/vehicle.py   engine, gearbox, chassis
host/cluster_sim/audio.py     engine synthesis + sample back end
host/cluster_sim/gui.py       dashboard + keyboard
```

The wire format is `0xAA 0x55 <len=20> <20 bytes> <xor>`, little-endian. The
layout lives in `struct HostState` (`src/Protocol.h`) and `_FMT`
(`host/cluster_sim/link.py`); a `static_assert` fails the firmware build if
the struct ever stops matching `cfg::HOST_PAYLOAD`.

## Sources

Byte layouts came from these, cross-checked against each other:

- [e46canbus — E90 CAN bus](https://e46canbus.blogspot.com/2015/11/e90-can-bus.html) — 0x130 terminal-15 on/off payloads
- [Arduino Forum — Controlling BMW E90 instrument cluster](https://forum.arduino.cc/t/controlling-bmw-e90-instrument-cluster/670728) — working 0x130/0x21A/0x1F6/0x1B4 captures, termination fix
- [bmw-e90.dbc](https://gist.github.com/nberlette/0ed4967da74d626da377e1b1cff70989) — signal definitions, scaling, enums
- [E90 K-CAN notes](https://gist.github.com/MorGuux/1f93228d5dde65fc7f81d78ddf405f99) — ID to module map
- [loopybunny — K-CAN](http://www.loopybunny.co.uk/CarPC/k_can.html) — frame rates and IDs from a real bus
- [veikkos/e90-can-cluster](https://github.com/veikkos/e90-can-cluster) — a working driver for this exact cluster: 0x1A6 accumulator at 13.0 counts/s per km/h, 100 ms cadence, +400 time tick, time-based 0x1A0 alive nibble. Its `e90_m3_dump_veikkos.txt` is the capture analysed above
- [E65 ReverseEngineering — 0x1A0](https://github.com/HeinrichG-V12/E65_ReverseEngineering/blob/main/docs/0x1A0.md) — 0x1A0 bit fields and checksum
