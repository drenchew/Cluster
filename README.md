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
w 1435   set the clock, HHMM or HHMMSS       (0x39E)
u 0..7   traction lamps: +1 DTC, +2 DSC triangle, +4 DSC off  (0x592)
y 357 1  raise (1) or clear (0) any check-control symbol by id (0x592)
d        resume demo sweep  ?       print state
```

Every five seconds the firmware also prints what the **cluster** is saying
about itself, which is the quickest way to see whether the clock took and what
the thermometer really reads:

```
kombi: outside 10.5C clock 14:32:07
```

That line comes from 0x2CA and 0x2F8, which the KOMBI transmits — see
[The on-board computer](#the-on-board-computer-temperature-and-clock).

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
py -3 -m pip install -r requirements.txt
py -3 run.py                  # auto-detects the Arduino port
py -3 run.py --port COM6
py -3 run.py --list           # list serial ports
py -3 run.py --no-serial      # sim + GUI + audio, no hardware
py -3 run.py --audio off
```

Use `py -3`, not `python`: on a stock Windows install `python` is the Microsoft
Store alias, which drops into an interactive prompt instead of running your
script.

`pygame-ce` is in requirements rather than `pygame` because it is the fork
that ships wheels for current Python (3.13/3.14). It imports as `pygame`, so
nothing in the code changes.

**Why pygame and not a GUI toolkit:** a cluster needs 60 fps hand-drawn dials
and per-frame held-key state, which is what immediate-mode SDL2 is for. Qt or
Dear PyGui would mean fighting a widget framework to custom-draw every gauge
anyway, and a web UI adds a server and keyboard-focus problems for no gain.
The work that makes it look like an instrument panel is in `gauges.py`: dial
faces are rendered once and cached so only the needle is redrawn, needles are
rotated with `rotozoom` (which filters as it rotates) instead of being drawn
as rotated polygons, and lit elements get an additive downscale-blur glow.
That costs about 6 ms a frame, leaving plenty of the 16.7 ms budget.

Needles run through a second-order spring (`NeedleMotion`) rather than chasing
their target, so they lag and overshoot ~3% like a real stepper. It integrates
in fixed 1/240 s substeps — a single 1/60 s explicit-Euler step on a spring
that stiff is already marginal, and one dropped frame makes it diverge.

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
| `C` | DSC: on → DTC (traction mode) → off |
| `[` / `]` | dimmer down / up (visibly dims the panel, as the real one does) |
| `-` / `=` | fuel down / up, 5 l a press |
| `R` | hold to refuel, 36 l/min |
| `X` | fuel drain: real time ↔ ×25 so the needle visibly moves |
| `;` / `'` | outside temperature down / up |
| `,` / `.` | road gradient down / up |
| `T` | reset trip |
| `P` | sport shift map |
| `Tab` | controls overlay |
| `Esc` | quit |

To get moving from cold: hold `Enter` until the engine catches (ignition is
already on), `G` for `D`, then `W`. The handbrake starts **released** and the
lever starts in `P`, so a freshly started bench shows no park-brake telltale
and the car sits still — `Space` pulls it again if you want the gong.

### Vehicle model

A turbocharged 6-speed making **332 hp at 5440 rpm and 500 Nm from 2000–4000**,
idling at 800 with the fuel cut at **5500**. `peak_power_hp()` reports the
figures straight off `TORQUE_CURVE`, so retuning the curve and re-running the
tests tells you what you actually built. What gives it the character:

**Turbo lag.** Only `NA_TORQUE_FRAC` (36%) of peak torque is available off
boost. Stab the throttle at 1200 rpm and that is all you get; the rest arrives
over the next half second as the turbo spools, and below `BOOST_THRESHOLD_RPM`
it never arrives at all. Boost is an *air limit* on fuelling, not a multiplier
on it — part throttle is not air-limited and behaves normally, while full
throttle off boost is. Multiplying throttle by boost instead double-counts:
30% throttle at 30% boost came out as 17% of the curve, which left the car
barely able to accelerate and the gearbox stuck in first.

**Overrun fuel cut.** Lift off above idle and injection stops outright, so the
engine becomes a brake. The audio reads the fuelling signal directly, which is
why the note hollows out the instant you lift (measured at 0.47× the on-power
level at the same rpm).

**Torque interrupt and rev matching.** An upshift cuts fuel and lets the engine
fall to the next gear's speed; a downshift blips it up to meet the road. Both
are a slew toward a computed target over the shift time, so the tacho needle
dips or flares instead of jumping. Plus kickdown above 82% throttle, a shift
lockout that kills hunting, and shift points that rise with throttle.

**Torque-converter launch.** Without a slipping phase, selecting a gear at a
standstill locks the engine to stationary wheels and stalls it instantly. The
converter passes torque in proportion to slip squared, which is also what lets
it creep at idle.

**Pops and bangs.** Back off a committed throttle and unburnt mixture carries
into a hot exhaust and ignites there — one bang on the lift, then crackle that
fades over about half a second. It needs all three of fuelling cut, revs above
`POP_MIN_RPM`, and the engine having been on boost a moment earlier, so it
only happens when you actually lift off something, not every time you breathe
off the pedal at 1500 rpm.

**Traction limit.** Drive force is capped at what the rear axle can take
(`TYRE_MU × mass × g × REAR_WEIGHT_FRAC` ≈ 8 kN) and the surplus spins the
engine instead of the car. First gear at the torque peak asks for 14.5 kN, so
with 500 Nm this now bites on a launch — it did not at the original 340 Nm.
A spinning tyre also grips *less* than a hooked-up one (`TYRE_SPIN_MU_LOSS`,
0.3), which is what makes wheelspin cost time rather than being free.

**Road gradient** (`,` / `.`) up to ±20%, which is the quickest way to load the
engine without needing road speed.

### Traction control

`C` cycles the three positions of the real car's console button:

| mode | slip allowed | lamp |
|---|---|---|
| **DSC** | 4% | DSC triangle flashes while intervening |
| **DTC** (traction mode) | 30% | `DTC` lit steady, triangle still flashes |
| **DSC off** | — no intervention at all | `DSC OFF` lit steady |

What intervenes is the engine, not the brakes: the controller ramps a fuelling
cut in at `TC_CUT_RATE` (up to 85%) while the driveline is asking the tyres for
more than they have, and hands it back slower than it takes it away
(`TC_RELEASE_RATE`) so the two do not see-saw at the threshold. Because the cut
lands on `fuelling`, the **audio hears it** — a traction-limited launch comes
out as the surging note a real one has, not a flat full-throttle pull.

It acts on the slip measured the frame before, like a real system reacting to
wheel-speed sensors rather than predicting. Measured over a standing start:

| mode | peak slip | slip-seconds | 0–50 km/h |
|---|---|---|---|
| DSC | 0.95 | 0.13 | 4.48 s |
| DTC | 0.97 | 0.61 | 4.52 s |
| off | 1.00 | 3.10 | 5.11 s |

On the cluster these are **check-control symbols on 0x592**, not telltale bits:
the E9x KOMBI has no "light the DSC lamp" signal, so the host raises symbol
357 (`DTC`), 215 (DSC triangle) or 36 (`DSC OFF`) by number. The flashing is
done on the host — the firmware only knows on and off, and the host is the end
that knows an intervention started.

### Fuel

The tank is 60 l (`FUEL_TANK_L`), the reserve warning comes up at 8 l, and the
level runs **at the real burn rate** by default: a cruise is about 0.1 l a
minute, so the needle genuinely barely moves. `X` switches to a ×25 bench
drain when you want to watch it fall, `-`/`=` jump the level 5 l at a time, and
`R` holds a 36 l/min pump against it. The l/100km readout always uses the true
rate, whichever drain is selected.

Fuel is burned from the power the engine is making — crank kW over the energy
in a litre of diesel times the thermal efficiency — rather than from pedal
travel, so load decides consumption. That lands a 90 km/h cruise at 6.6
l/100km, 130 km/h at 9.0, idle at 0.9 l/h and full throttle near 65 l/h. The
old pedal-position formula read 1.5 l/100km at every cruise.

**The gauge is not linear, and the two senders disagree.** The tank is a saddle
tank with a sender in each half, and 0x349 carries both as 16-bit counts. The
flat 160 counts/litre this jig used to send is only right for the left sender
near full; the real breakpoints (from the veikkos driver) are

| fill | 100% | 87.5% | 75% | 50% | 25% | 0% |
|---|---|---|---|---|---|---|
| left | 9700 | 8200 | 6250 | 3600 | 1950 | 625 |
| right | 8400 | 5400 | 4600 | 3350 | 2200 | 950 |

which `ClusterCan::fuelCounts()` interpolates (`cfg::FUEL_CURVE_REAL = false`
goes back to the flat scaling if you want to compare them on the bench). The
firmware also slews what it sends at 1.5%/s (`FUEL_SLEW_FRAC_PER_S`): one
keypress moves the setpoint 5 l, no real sender can do that, and a step that
large reads to the cluster as a refuelling event.

### The on-board computer: temperature and clock

Two things a fresh bench cluster gets wrong, and neither is a missing frame
you forgot to send.

**“−40” is an open circuit, not a CAN value.** The outside temperature does
not come off the bus at all — the cluster measures it itself, through an NTC
across connector **pins 4 and 5**. With nothing wired there the input reads
open and the display pegs at the bottom of its scale, which is −40 in both
units (that the C and F numbers agree is the giveaway). The sensor is 5 kΩ at
25 °C with β = 3950:

```
cd host
python3 ambient_resistor.py              # the whole table
python3 ambient_resistor.py --temp 18    # 6.8 kΩ reads +18.2 C / 65 F
python3 ambient_resistor.py --ohms 10000 # 10 k reads +10.2 C / 50 F
```

10 kΩ across pins 4 and 5 is the usual bench choice: about 10 °C, warm enough
to clear the cold-weather warning. A trimpot lets you play with it; a digital
potentiometer (AD5272, as in the veikkos driver) lets the host drive it. Note
the cluster filters this input hard — a change takes a minute or two to reach
the display and does not move at all with KL15 off.

The `t` console command and `;`/`'` in the GUI only change what goes out on
0x366, which is **off by default** because the DBC names the KOMBI as that
frame's sender; the GUI's own readout shows both units so you can compare it
against what the cluster displays.

**No clock means nothing ever told it the time.** In a car the CAS, iDrive or
GPS sets it; on a bench nothing does, and a cluster that has never been given
the time shows no clock at all. `0x39E` is the frame that sets it, and the jig
now sends it once a second — the simulator from the PC clock, `run_ets2.py`
from the in-game clock by default (`--clock game|system|off`), and the
firmware keeps the clock running off `millis()` between host updates so
unplugging the USB cable does not freeze it. `w 1435` sets it by hand.

The cluster then broadcasts what it believes on **0x2F8**, and its measured
temperature on **0x2CA**, both of which the firmware reads back and prints —
so you can confirm the clock took without squinting at the LCD. Never
*transmit* either: the KOMBI is their sender, and two nodes putting different
data behind one identifier collide and produce bus errors.

If the clock is set but the display still will not show it, that is the
on-board computer's display mode rather than the bus. The BC menus are
navigated by pulling **pin 16** to ground through 1 kΩ (enter), 2 kΩ (up) and
3 kΩ (down).

### Engine audio

Default is **procedural pulse-train synthesis**, not samples. A 4-stroke fires
each cylinder once per two crank revolutions, so a 4-cylinder fires twice per
revolution and the firing frequency is `rpm/30` Hz — 27 Hz at idle, 150 Hz at
4500. Engine noise is a train of exhaust pressure pulses coloured by the
resonance of the exhaust tract rather than a sustained harmonic tone, so that
is what `audio.py` builds: an impulse per firing event through a resonator,
plus injector clatter. Phase is continuous across buffers, so pitch glides
instead of stepping, and the output is stereo.

Everything above the base note is driven by the simulation rather than guessed
from rpm, which is what makes it track what the engine is actually doing:

| layer | driven by |
|---|---|
| combustion level | fuelling — so it hollows out on the overrun |
| turbo whine | boost, at ~19× crank speed — it lags the pedal like the turbo |
| wastegate flutter | a lift while on boost |
| starter motor | cranking, before the engine catches |
| shift thunk | each gearchange |
| transmission whine | road speed |
| tyre scrabble | the traction limit being exceeded |
| pops and bangs | the overrun crackle level, as individual cracks |

The crackle is added to the *excitation* rather than the output, so the
exhaust resonator colours it like everything else — the pops come out of the
exhaust rather than out of nowhere. Each crack is a few milliseconds of noise
with a hard decay, scattered by a Poisson draw so they never fall into a
rhythm.

`drive_from_vehicle()` in `audio.py` is the single call that feeds all of it.

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

## Game telemetry: ETS2 / ATS

`host/run_ets2.py` drives the same jig from Euro Truck Simulator 2 (or American
Truck Simulator) with no game-side hardware and no new wire protocol: it reads
the game's shared memory and pushes the usual 50 Hz setpoints at `link.py`.

### Just drive

```sh
./drive.sh
```

One command, from any state. It finds the game across every Steam library,
rebuilds and reinstalls the plugin if a game update wiped it, opens up the
serial port (asking for sudo only if it has to), runs the layout selftests,
and then hands over to the bridge. `./drive.sh --check` reports without
changing anything; `--launch` starts the game too; `--ats` does American
Truck Simulator. Anything after `--` is passed to `run_ets2.py`:

```sh
./drive.sh --check
./drive.sh --launch
./drive.sh -- --rpm-scale 1.5 --tank-l 70
```

The rest of this section is what it does for you, and what to do when it
cannot.

#### Serial port permission, permanently

The jig is an Arduino on a CH340 adapter, and `/dev/ttyUSB0` is created
`root:dialout` mode 660 every time it is plugged in. A `chmod` is lost on the
next replug; the permanent fix is to be in the group:

```sh
sudo usermod -aG dialout $USER
```

**Then log out and back in.** Linux stamps a session's group list at login,
so until you do, `id` will not list `dialout` even though `/etc/group` has
you in it — which looks exactly like the command not having worked. Check
the file rather than the session:

```sh
getent group dialout        # you want your name on the end of this
```

`drive.sh` recognises that in-between state and re-runs itself under
`sg dialout`, which starts a shell that does carry the group, so it works
without sudo and without logging out. Once you have logged in again that
step stops happening.

### Getting the data out of the game

The game publishes nothing by itself: a plugin has to be loaded into it, which
is then free to do what it likes with the numbers. This repo ships one, in
[`plugin/`](plugin/), and it is what `run_ets2.py` expects by default.

```sh
cd plugin
make            # build plugins/cluster-telemetry.so
make test       # load and drive it with no game and no hardware
make install    # copy it into the game's plugins folder
```

Start the game, then check both ends:

```sh
grep -i cluster-telemetry ~/.local/share/"Euro Truck Simulator 2"/game.log.txt
cd host && python3 -m cluster_sim.ets2_plugin --watch
```

It publishes a 512-byte block at `/dev/shm/ClusterTelemetry` once per
simulation frame. [`plugin/README.md`](plugin/README.md) has the channel
table, the seqlock, and how to add a field without breaking the reader;
[`plugin/include/cluster_shm.h`](plugin/include/cluster_shm.h) is the layout
itself, mirrored field for field by
[`host/cluster_sim/ets2_plugin.py`](host/cluster_sim/ets2_plugin.py).

Built and running against **ETS2 1.61.1.9**, telemetry API 1.21, native Linux
x86_64. It needs only `libc.so.6` and asks for nothing newer than
`GLIBC_2.4` — older than the game binary's own `GLIBC_2.6` — so it loads
anywhere the game does, Steam Linux Runtime container included.

### Using RenCloud's plugin instead

If a machine already has the community **SCS SDK plugin**, `--source shm`
reads that instead. It is a 32 KiB struct covering trailers, jobs and
gameplay events, which `ets2.py` mirrors in full:

```
python3 run_ets2.py --source shm
```

| fork | Linux binary | revision | shm name |
|---|---|---|---|
| [truckermudgeon](https://github.com/truckermudgeon/scs-sdk-plugin) | `build-ubuntu-latest.zip` | 12 | `/SCSTelemetry` |
| [SimuLogistics](https://github.com/SimuLogistics/scs-sdk-plugin) | `SimuLogistics-Telemetry.so` | 13 (rev-12 offsets kept, zone appended) | `/SimuLogisticsTelemetry` |
| [RenCloud](https://github.com/RenCloud/scs-sdk-plugin) (upstream) | none — Windows only | 12 | `Local\SCSTelemetry` |

Both Linux options are in the reader's name-candidate list, so either is
picked up without `--shm-name`. Avoid the older mmap forks (jackz314, alxwk,
y1lichen) — their compatibility tables stop around ETS2 1.36.

The reader checks the plugin's own `telemetry_plugin_revision` at runtime and
warns if it is not one it mirrors. That matters because `--selftest` can only
prove the struct is self-consistent; it cannot know what the *running* plugin
publishes. Revisions 12 and 13 are accepted silently, anything else warns but
still runs.

**The object's name is not standardised across those forks.** Upstream and the
truckermudgeon fork use `/dev/shm/SCSTelemetry`; ETS2LA's controller plugin
uses a `/dev/shm/SCS/` subdirectory. So `run_ets2.py` tries the known names
and then scans `/dev/shm` for anything of the right size. If it still misses
yours:

```
python3 run_ets2.py --probe                  # what is actually in /dev/shm
python3 run_ets2.py --shm-name MyBlockName   # or an absolute path
```

### Where the plugin goes, and what will not work

| | plugins folder |
|---|---|
| Linux (native) | `<game>/bin/linux_x64/plugins/` |
| Windows (native) | `<game>\bin\win_x64\plugins\` |

Create `plugins/` if it is not there; `make install` does. On Windows the
block is a named mapping `Local\SCSTelemetry` and `--source shm` works there
too.

- **Proton / Wine: not supported.** The Windows build's mapping lives inside
  the Wine prefix where no native Linux process can see it, and the Linux
  build is a different install. A bridge running *inside* the prefix would be
  needed; this repo does not ship one.
- **macOS: not supported** — the reader expects POSIX shared memory in
  `/dev/shm`, which macOS does not have.

### From Steam (the usual case)

ETS2 ships a native Linux build on Steam, so this works — but two Steam
specifics will silently produce "no telemetry" with no other symptom. Run
the doctor first; it finds the install, says exactly where the plugin goes,
and names anything that would stop it working:

```
python3 run_ets2.py --setup
```

It walks every Steam library (including extra drives via
`steamapps/libraryfolders.vdf`) and checks:

1. **Is `bin/linux_x64` actually populated?** If Steam has forced Proton, that
   directory is empty or missing and only `bin/win_x64` exists. Telemetry
   cannot work. Fix: right-click the game → Properties → Compatibility →
   **untick** "Force the use of a specific Steam Play compatibility tool", and
   let Steam re-download the Linux binaries. (A known Steam client bug can
   also swap the Linux binaries for Windows ones on its own, with the same
   fix.)
2. **Is Steam a Flatpak?** Flatpak gives each sandbox its own `/dev/shm`, so
   the game's telemetry block is invisible outside it. Fix:
   `flatpak override --user --device=shm com.valvesoftware.Steam`, then fully
   restart Steam.

Typical Steam locations for the plugins folder:

```
~/.local/share/Steam/steamapps/common/Euro Truck Simulator 2/bin/linux_x64/plugins/
~/.var/app/com.valvesoftware.Steam/.local/share/Steam/steamapps/common/...   (Flatpak)
```

`make install` finds the first of those on its own; for the Flatpak install,
or for ATS, pass `GAME_DIR=...`.

### Running it

**Only `pyserial` is needed** — this path uses no pygame, numpy or
sounddevice, so on the Linux box `pip install pyserial` is the whole install.

```
cd host
python3 run_ets2.py --setup                 # find Steam/ETS2, check it can work
python3 run_ets2.py --selftest              # layout check: no game, no jig
python3 run_ets2.py --probe                 # what telemetry blocks exist
python3 run_ets2.py --source mock --no-serial   # scripted drive, nothing attached
python3 run_ets2.py --source mock           # scripted drive -> real cluster
python3 run_ets2.py                         # the game -> real cluster
python3 run_ets2.py --port /dev/ttyUSB0
python3 run_ets2.py --list                  # serial ports
```

`python3` on Linux (the port will be `/dev/ttyUSB0` for a CH340 or
`/dev/ttyACM0` for a genuine Uno); `py -3` on Windows.

Useful flags: `--rpm-scale` (spread a ~2500 rpm truck engine over the car
dial), `--tank-l` (car-tank size the fuel gauge reads as full scale),
`--speed-scale`, `--lights-on` (force side + low beam), `--shm-name`,
`--no-serial`, `-v` (one status line per second instead of on change only),
`--rate` (frames per second to the jig).

On Linux you will likely need to be in the `dialout` group to open the port:
`sudo usermod -aG dialout $USER`, then log out and back in.

The status line is the whole diagnostic surface:

```
18:49:14  90 km/h  1500 rpm D6  park off  fuel 36.4 l  84 C  lights 0x05  [100 frames, link ok]
18:49:19  no telemetry (no mapping Local\SCSTelemetry (is the game running?)) - cluster runs its own demo sweep  [link not opened yet]
```

`link ok` / `link <error>` comes straight from `ClusterLink`, so a flaky cable
shows up there rather than as a frozen cluster. The serial port is only opened
on the first frame we actually send, hence `not opened yet` while the game is
not producing telemetry.

### What the truck becomes

| game channel | cluster |
|---|---|
| `truck.speed` (m/s) | speedometer × 3.6 |
| `truck.engineRpm` | tachometer × `--rpm-scale` |
| `truck.fuel / config.fuelCapacity` | fuel gauge as a **fill fraction** × `--tank-l` |
| `truck.waterTemperature` | coolant gauge, 0x1D0 |
| `truck.gear` (<0 R, 0 N, >0 D) | gear position + number |
| `truck.parkBrake` | handbrake telltale, 0x34F + 0x1B4 |
| `truck.blinkerLeft/RightActive` | turn signals, 0x1F6 |
| `truck.lightsParking / BeamLow / BeamHigh` | light telltales, 0x21A |
| `truck.electricEnabled / engineEnabled` | KL15 + engine-on state |
| `truck.gameThrottle` | pedal readout in the telemetry rail |
| `truck.wheelVelocity × wheelRadius` vs `truck.speed` | driven-wheel slip → flashing DSC triangle, 0x592 |
| `truck.differentialLock` | `DTC` lamp — the game's own traction aid, 0x592 |
| `common.time_abs` (in-game minutes) | the clock, 0x39E (`--clock game|system|off`) |

Three of those are deliberate compromises:

- **Fuel is a fraction, not litres.** The truck carries 400–800 l; sent raw it
  would pin a car gauge. The firmware also clamps to 800 deci-litres.
- **The tachometer idles at 800.** A truck idles near 600 rpm, but the
  firmware floors rpm at `cfg::ENGINE_IDLE_RPM` (800) while the engine is on
  (`ClusterCan.cpp`), so below idle the needle simply sits at 800.
- **Blinker *enable*, not lamp.** `truck.lblinker` is the logical switch and
  `truck.light.lblinker` is the 2 Hz lamp state. Feeding the lamp would
  re-kick the cluster's own 0x1F6 flasher every frame.

Traction control has no switch in the game, so the lamps report what the truck
is doing instead of what a driver selected. `wheel_slip()` measures the worst
powered wheel's surface speed against ground speed — the way a real DSC does,
rather than guessing from the throttle — and anything past 8% slip under power
flashes the triangle. The in-game date is not a real one, so only the hour and
minute come from the game; the date comes from the PC.

### When the game goes quiet

`ShmSource.poll()` returns `None` when the mapping is gone, the SDK is not
active, or `frame.time` has not advanced for 0.75 s (paused game, main menu,
alt-tabbed replay). Nothing is sent, the firmware's 1.5 s host watchdog trips,
and the cluster drops into its own demo sweep — it never resets and never
holds a stale value. Same story on Ctrl-C: `run_ets2.py` closes the link and
the jig carries on by itself.

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
| 0x349 | 200 ms | fuel level, DLC 5, two non-linear senders | OK |
| 0x19E / 0x0C0 / 0x0D7 | 200 ms | DSC / ABS / airbag counters | OK |
| 0x1A0 | 100 ms | "Speed K-CAN" — direct speed, **on by default** | OK |
| 0x0CE | 100 ms | DSC wheel speeds — four signed 16-bit LE, **on by default** | OK |
| 0x39E | 1 s | time + date — **sets the cluster clock** | OK |
| 0x592 | event + 200 ms | check-control symbol by id: DTC / DSC / DSC OFF | OK |
| 0x1D2 | 200 ms | gear display P R N D, D1..D6 | ? |
| 0x202 / 0x2C0 | 200 ms | dimmer level / LCD illumination | ? |
| 0x3B0 | 200 ms | reverse gear | ? |
| 0x366 | 1 s | outside temperature + range (KOMBI is the DBC sender) | ? |

Received, never transmitted — these are the cluster's own:

| ID | what |
|---|---|
| 0x2CA | the outside temperature it measures on pins 4–5, `B0 × 0.5 − 40` °C |
| 0x2F8 | its clock: `hour, minute, second, day, month<<4, year LE` |


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
E90_Cluster/              standalone ESP32 sketch (older: no clock, symbols
                          or fuel curve - the Uno build is the current one)
drive.sh                  one command: fix whatever is broken, then drive
host/run.py               simulator entry point
host/run_ets2.py          ETS2 / ATS entry point
host/ambient_resistor.py  temperature <-> resistor for the pins 4-5 NTC

host/cluster_sim/link.py        serial framing, must match src/Protocol.h
host/cluster_sim/vehicle.py     engine, turbo, gearbox, driveline, chassis
host/cluster_sim/audio.py       engine synthesis + sample back end
host/cluster_sim/gauges.py      dials, needles, glow, telltale icons
host/cluster_sim/gui.py         cluster layout, telemetry rail, keyboard
host/cluster_sim/ets2.py        scsTelemetryMap_t reader + game -> cluster mapping
host/cluster_sim/ets2_plugin.py reader for our plugin's block, same mapping

plugin/src/cluster_telemetry.cpp  the ETS2/ATS plugin itself (builds with g++)
plugin/include/cluster_shm.h      the published block - the C/Python contract
plugin/test/host_test.cpp         loads the .so and drives it with no game
plugin/scs_sdk/                   official SCS SDK 1.16 headers, MIT, vendored
```

The wire format is `0xAA 0x55 <len=28> <28 bytes> <xor>`, little-endian. The
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
- [veikkos/e90-can-cluster](https://github.com/veikkos/e90-can-cluster) — a working driver for this exact cluster: 0x1A6 accumulator at 13.0 counts/s per km/h, 100 ms cadence, +400 time tick, time-based 0x1A0 alive nibble, the 0x39E clock layout, the 0x592 symbol frame and its id list, the two fuel-sender curves, and the pins 4–5 ambient NTC (5 kΩ at 25 °C, β 3950). Its `e90_m3_dump_veikkos.txt` is the capture analysed above
- [E65 ReverseEngineering — 0x1A0](https://github.com/HeinrichG-V12/E65_ReverseEngineering/blob/main/docs/0x1A0.md) — 0x1A0 bit fields and checksum
- [truckermudgeon/scs-sdk-plugin](https://github.com/truckermudgeon/scs-sdk-plugin) — the plugin that publishes `scsTelemetryMap_t`; its `scs-telemetry/inc/scs-telemetry-common.hpp` is the layout `host/cluster_sim/ets2.py` mirrors
- [SCS Software — Telemetry SDK](https://modding.scssoft.com/wiki/SCS_SDK) — channel units and semantics (`gear`, blinker enable vs. lamp)
