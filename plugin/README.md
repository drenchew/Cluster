# cluster-telemetry — ETS2/ATS plugin for the E90 cluster rig

Publishes the truck's state into shared memory so `host/run_ets2.py` can turn
it into CAN frames for the instrument cluster.

```sh
cd plugin
make            # build plugins/cluster-telemetry.so
make test       # load it and drive it with no game running
make install    # copy it into the game's plugins folder
```

Then start ETS2 and check it came up:

```sh
grep -i cluster-telemetry ~/.local/share/"Euro Truck Simulator 2"/game.log.txt
cd ../host && python3 -m cluster_sim.ets2_plugin --watch
```

## Why this exists

The usual choice is [RenCloud's scs-sdk-plugin][rencloud], which is
Windows-first; Linux support lives in [a fork][tm] and has to be built with
CMake. Its shared block is 32 kB of trailers, jobs and gameplay events, and
the host has to mirror that struct exactly or every gauge reads the wrong
four bytes.

The cluster needs about thirty numbers. This publishes those, in 512 bytes,
from one source file that builds with nothing but `g++`. `host/cluster_sim/`
reads both, so an existing RenCloud install still works via
`run_ets2.py --source shm`.

## How it works

The game loads any `.so` it finds in its plugins folder and calls
`scs_telemetry_init`. We register for ~80 telemetry channels, each with its
destination in a staging struct as the callback's context pointer, plus five
events. Channel callbacks fill the staging copy; `frame_end` publishes it as
one block.

Every channel is registered `each_frame | no_value`, so the game calls us
once per frame whatever the value did, and with a null value when a channel
has nothing to say. Zeroing on null is what stops a gauge freezing at
whatever it read before a load screen.

### The seqlock

A reader in another process can catch the block mid-write. `seq` is made odd
before the copy and even after it, so a reader that sees the same even value
either side of its own read knows it got one whole frame:

```python
before = read()                 # reject if before.seq is odd
if before.seq == read_seq():    # unchanged - the frame is good
```

`magic`, `version` and `size` sit ahead of `seq` and are written once at
init, so the per-frame copy starts after the lock rather than clobbering it.

### Shutdown

`scs_telemetry_shutdown` zeroes `magic` and unlinks the block, so a host that
still has it mapped sees "no plugin" rather than replaying the last frame of
a game that has quit. A destructor repeats it in case the game `dlclose()`s
us without calling shutdown.

## The block

`/dev/shm/ClusterTelemetry`, 512 bytes in a 4 kB mapping. The layout lives in
[`include/cluster_shm.h`](include/cluster_shm.h) and is mirrored field for
field by [`host/cluster_sim/ets2_plugin.py`](../host/cluster_sim/ets2_plugin.py).
Both sides assert the struct is 512 bytes, so editing one and not the other
fails at startup instead of silently misreading.

To add a field: append it (never insert), mind the alignment, bump
`CLUSTER_SHM_VERSION`, add it to the Python mirror, and run `make test`
followed by `python3 -m cluster_sim.ets2_plugin --selftest`.

Setting `CLUSTER_SHM_NAME=/SomeOtherName` in the environment moves the block.
`make test` uses it to publish to `/ClusterTelemetryTest`, so the test can
run while a game is playing without its shutdown unlinking the live block.
Only a plain `/name` is accepted; anything else falls back to the default.
Read a moved block with `python3 -m cluster_sim.ets2_plugin --path ...`.

Wheel arrays are 16 wide. `wheel_count` says how many are real; the rest are
zero, and the config handler clears them on a truck change so a 6-wheeler
cannot inherit axles from the truck before it — which would read as
permanent wheelspin.

## What it reads, and what the cluster does with it

| Block field | Channel | Cluster |
|---|---|---|
| `speed` | `truck.speed` | speedometer (0x1A0) |
| `engine_rpm` | `truck.engine.rpm` | tachometer (0x0AA) |
| `fuel`, `fuel_capacity` | `truck.fuel.amount`, config | fuel gauge as a *fraction* of a 60 l car tank (0x349) |
| `water_temperature` | `truck.water.temperature` | coolant gauge (0x1D0) |
| `fuel_range` | `truck.fuel.range` | range on the on-board computer (0x366) |
| `gear` | `truck.engine.gear` | gear display (0x1D2) |
| `electric_enabled` | `truck.electric.enabled` | KL15 / terminal status (0x130) |
| `park_brake` | `truck.brake.parking` | handbrake lamp (0x1B4) |
| `light_parking`, `light_low_beam`, `light_high_beam` | `truck.light.*` | lights (0x21A) |
| `blinker_*_active` | `truck.lblinker`, `truck.rblinker` | indicators (0x1F6) — the stalk, not the lamp, because the cluster does its own flashing |
| `hazard` | `truck.hazard.warning` | both indicators |
| `differential_lock` | `truck.differential_lock` | DTC lamp (0x592) |
| `wheel_angular_velocity`, `wheel_radius`, `wheel_powered` | `truck.wheel.*` + config | wheelspin → DSC triangle (0x592) |
| `time_abs` | `game.time` | cluster clock (0x39E) |
| `throttle` | `truck.effective.throttle` | gates the wheelspin lamp, so coasting is not an intervention |

Traction control is the interesting one: the game has no DSC switch, so the
lamps report what the truck is doing. Surface speed of a powered wheel
against ground speed gives slip; slip under throttle flashes the DSC
triangle, and the game's own diff lock lights DTC.

### `wheel.angular_velocity` is in rotations per second

Despite the name, it is **not** radians. The SDK header says rotations, and
the game agrees — measured on a free-rolling wheel at a steady 91.6 km/h:

```
wheel 0 free  r=0.506 m  w=8.005  w*r/v=0.1591
```

`0.1591` is `1/2π`. Cross-check: 25.44 m/s ÷ (2π × 0.506 m) = 8.00 rev/s.

So **surface speed is `value * 2*pi * radius`**. Getting this wrong is
quiet rather than loud: omitting the 2π scales surface speed to 16% of the
truth, slip goes permanently negative, the `max(0.0, ...)` clamps it to zero,
and the lamp simply never lights with nothing to show you why. It read
`0.000` through full-throttle wheelspin against a held handbrake until this
was fixed; the same moment now reads `1.219`.

Both `ets2.wheel_slip` and the test fixtures encode the corrected form, and
`make test` asserts a 0.25 slip that only comes out right with the 2π in
place.

## Compatibility

Built and running against **ETS2 1.61.1.9**, telemetry API 1.21, native
Linux x86_64.

There is no game-version gate: the plugin accepts telemetry API 1.00 and
1.01 — layout-identical, and stable for a decade — and records the game's
version in the block for the host to report. A channel the running game does
not have costs that one gauge, not the whole plugin, so a future patch
degrades rather than breaks.

The only shared library it needs is `libc.so.6`, and the highest symbol
version it asks for is `GLIBC_2.4`, which is older than the game binary's own
`GLIBC_2.6`. It will load anywhere the game itself loads, Steam Linux Runtime
container included. `-static-libstdc++` and `-fno-exceptions` are what keep
libstdc++ out of it.

For American Truck Simulator, or a non-Steam copy:

```sh
make install GAME_DIR="$HOME/.local/share/Steam/steamapps/common/American Truck Simulator"
```

## Troubleshooting

**Nothing at `/dev/shm/ClusterTelemetry`.** Check the game log — it records
both the plugin coming up and any refusal:

```sh
grep -iE "cluster-telemetry|plugin" ~/.local/share/"Euro Truck Simulator 2"/game.log.txt
```

**The game must be the native Linux build.** Under Proton the block is
created inside the Wine prefix, where the host process cannot see it.

**`make install` can't find the game.** Pass `GAME_DIR=...`.

**Reader complains about the version or size.** The `.so` in the plugins
folder is from a different build than the Python mirror: `make && make
install`.

**The block vanished while the game was still running.** Almost certainly
`make test`: it loads the real plugin, so it uses the real block name, and
its shutdown unlinks it — deleting the name out from under a game that is
still writing to the mapping. The test now refuses to start if the block
already exists, so this should only bite an older build. Restart the game.

**`make install` while the game is open** is safe: it renames the new file
over the old one rather than writing in place, so the running game keeps the
inode it has mapped. It still runs the *old* code until you restart the game
or type `sdk reload` in its console.

The game also has console commands for plugin work — `sdk reinit` and
`sdk unload` / `sdk reload` — so the plugin can be rebuilt without restarting
the game.

## Licence

`scs_sdk/` is the official SCS SDK 1.16, MIT licensed, vendored unchanged
with its notice in `scs_sdk/sdk_license.txt`. Everything in `src/`,
`include/` and `test/` is part of this project.

[rencloud]: https://github.com/RenCloud/scs-sdk-plugin
[tm]: https://github.com/truckermudgeon/scs-sdk-plugin
