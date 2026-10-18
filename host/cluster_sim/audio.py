"""Engine audio.

``SynthEngine`` (default)
    Pulse-train synthesis. A 4-stroke fires each cylinder once every two crank
    revolutions, so a 4-cylinder fires twice per revolution and the firing
    frequency is rpm/30 Hz - 27 Hz at an 800 rpm idle, 150 Hz at 4500. Engine
    noise is not a sustained harmonic tone, it is a train of exhaust pressure
    pulses coloured by the resonance of the exhaust tract, so that is what this
    builds: an impulse per firing event convolved with the decaying sinusoidal
    impulse response of a resonator.

    Layered on top of that, driven by the sim rather than guessed from rpm:

    * **Fuelling, not throttle, sets the combustion level.** On the overrun the
      injectors stop, so the combustion layer drops away and what is left is
      mechanical - which is exactly how a diesel hollows out when you lift.
    * **Boost drives the turbo whine**, so it spools up and bleeds off with the
      real lag instead of tracking the pedal instantly.
    * **Wastegate dump** on a lift at boost.
    * **Starter motor** while cranking, and a catch as it fires.
    * **Shift transients**, one per gearchange.
    * **Transmission whine** from road speed, quiet but it fills out cruise.
    * **Tyre scrabble** when the rear axle is past its traction limit.

    Rendering is block-based with overlap-save so there are no seams, and the
    firing phase is continuous across blocks so pitch glides smoothly instead
    of stepping per buffer. Output is stereo with slight decorrelation.

``SampleEngine``
    Crossfades and resamples real looping recordings from ``host/samples/``
    (``rpm_0800.wav``, ``rpm_1500.wav``, ... - the number is the rpm the loop
    was recorded at). Use this if you have a sample set; see README.md. None is
    shipped, because no realistic multi-rpm engine pack exists under a license
    that allows redistribution - the one CC0 "racing engine" pack on
    OpenGameArt is a pitch-shifted seatbelt warning chime, not an engine.

Both are optional: if numpy or sounddevice are missing, ``make_engine``
returns a ``NullEngine`` and everything else still runs.
"""

from __future__ import annotations

import math
import wave
from pathlib import Path

try:
    import numpy as np
except ImportError:
    np = None

try:
    import sounddevice as sd
except Exception:  # pragma: no cover - PortAudio may be absent entirely
    sd = None

SAMPLE_RATE = 44100
BLOCK = 512
CHANNELS = 2
CYLINDERS = 4
STROKES = 4

SAMPLES_DIR = Path(__file__).resolve().parent.parent / "samples"


def firing_hz(rpm: float) -> float:
    """Cylinder firing frequency: rpm/60 * cylinders / (strokes/2)."""
    return rpm / 60.0 * CYLINDERS / (STROKES / 2)


class NullEngine:
    """Stand-in when audio is unavailable or disabled."""

    available = False
    name = "off"

    def start(self) -> None: ...
    def stop(self) -> None: ...
    def update(self, **kw) -> None: ...
    def event(self, **kw) -> None: ...


class _StreamEngine:
    """Shared output-stream plumbing and parameter smoothing."""

    available = True
    name = "base"

    def __init__(self, volume: float = 0.6):
        self.volume = volume
        self._stream = None
        # Targets written by the main thread, read by the audio thread. Plain
        # floats, so reads and writes are atomic enough for this.
        self._rpm = 0.0
        self._load = 0.0
        self._running = False
        self._boost = 0.0
        self._fuelling = 0.0
        self._speed = 0.0
        self._cranking = False
        self._wheelspin = 0.0
        self._pop = 0.0
        # One-shot event accumulators, drained by the audio thread.
        self._ev_bov = 0.0
        self._ev_pop = 0.0
        self._ev_shift = 0.0
        self._ev_start = 0.0
        # Smoothed values owned by the audio thread.
        self._cur_rpm = 0.0
        self._cur_load = 0.0
        self._cur_gain = 0.0
        self._cur_boost = 0.0
        self._cur_fuel = 0.0
        self._cur_speed = 0.0
        self._cur_pop = 0.0

    def start(self) -> None:
        if sd is None:
            return
        self._stream = sd.OutputStream(
            samplerate=SAMPLE_RATE,
            blocksize=BLOCK,
            channels=CHANNELS,
            dtype="float32",
            callback=self._callback,
        )
        self._stream.start()

    def stop(self) -> None:
        if self._stream is not None:
            try:
                self._stream.stop()
                self._stream.close()
            except Exception:
                pass
            self._stream = None

    def update(self, rpm=0.0, load=0.0, running=False, boost=0.0, fuelling=0.0,
               speed_kmh=0.0, cranking=False, wheelspin=0.0, pop=0.0) -> None:
        self._rpm = float(rpm)
        self._load = float(max(0.0, min(1.0, load)))
        self._running = bool(running)
        self._boost = float(max(0.0, min(1.0, boost)))
        self._fuelling = float(max(0.0, min(1.0, fuelling)))
        self._speed = float(abs(speed_kmh))
        self._cranking = bool(cranking)
        self._wheelspin = float(max(0.0, min(1.0, wheelspin)))
        self._pop = float(max(0.0, min(1.0, pop)))

    def event(self, bov=0.0, shift=0, start=False, pop=0.0) -> None:
        """Fire one-shot transients. Accumulated so none are lost to a frame."""
        if bov:
            self._ev_bov = max(self._ev_bov, float(bov))
        if pop:
            self._ev_pop = max(self._ev_pop, float(pop))
        if shift:
            self._ev_shift = 1.0
        if start:
            self._ev_start = 1.0

    def _smooth(self, n: int):
        dt = n / SAMPLE_RATE
        rpm0 = self._cur_rpm
        # ~25 ms time constant: fast enough to feel connected to the throttle,
        # slow enough that frame-rate jitter in the sim does not warble.
        self._cur_rpm += (self._rpm - self._cur_rpm) * min(1.0, dt / 0.025)
        self._cur_load += (self._load - self._cur_load) * min(1.0, dt / 0.05)
        self._cur_boost += (self._boost - self._cur_boost) * min(1.0, dt / 0.04)
        self._cur_fuel += (self._fuelling - self._cur_fuel) * min(1.0, dt / 0.03)
        self._cur_speed += (self._speed - self._cur_speed) * min(1.0, dt / 0.08)
        # Pops follow their target almost immediately - the whole point is
        # that they are abrupt.
        self._cur_pop += (self._pop - self._cur_pop) * min(1.0, dt / 0.01)
        g0 = self._cur_gain
        target_gain = self.volume if (self._running or self._cranking) else 0.0
        self._cur_gain += (target_gain - self._cur_gain) * min(1.0, dt / 0.08)
        return rpm0, self._cur_rpm, self._cur_load, g0, self._cur_gain

    def _callback(self, outdata, frames, time_info, status):  # pragma: no cover
        try:
            out = self.render(frames)
            if out.ndim == 1:
                outdata[:, 0] = out
                if CHANNELS > 1:
                    outdata[:, 1] = out
            else:
                outdata[:] = out
        except Exception:
            outdata.fill(0.0)

    def render(self, n: int):
        raise NotImplementedError


class SynthEngine(_StreamEngine):
    """Pulse-train + resonator diesel model."""

    name = "synth"

    # Exhaust resonator: decaying sinusoids, which gives the pulse train body
    # and bark without a per-sample IIR loop.
    RESONANCES = ((92.0, 0.016), (210.0, 0.009), (640.0, 0.004))
    IR_LEN = 1024
    # Measured so the loudest operating point peaks near 0.85 at volume 1.0,
    # leaving the soft limiter with nothing to do under normal use.
    MAKEUP = 4.5
    # Each firing pulse decays over its own firing cycle, so a pulse lasts less
    # wall-clock time as revs rise and carries less energy into the resonator -
    # measured output fell about 4x from idle to redline. This exponent flattens
    # that back out so the engine does not fade as it revs.
    LEVEL_COMP_REF_RPM = 1800.0
    LEVEL_COMP_EXP = 0.6

    def __init__(self, volume: float = 0.6):
        super().__init__(volume)
        self._phase = 0.0          # firing-cycle phase, in cycles
        self._turbo_phase = 0.0
        self._trans_phase = 0.0
        self._starter_phase = 0.0
        self._ir = self._build_ir() if np is not None else None
        self._tail = np.zeros(self.IR_LEN - 1, dtype=np.float32) if np is not None else None
        self._rng = np.random.default_rng(7) if np is not None else None
        # Decaying one-shot envelopes.
        self._bov_env = 0.0
        self._shift_env = 0.0
        self._start_env = 0.0
        self._bang_env = 0.0

    def _build_ir(self):
        t = np.arange(self.IR_LEN, dtype=np.float32) / SAMPLE_RATE
        ir = np.zeros(self.IR_LEN, dtype=np.float32)
        for freq, tau in self.RESONANCES:
            ir += (np.exp(-t / tau) * np.sin(2.0 * math.pi * freq * t)).astype(np.float32)
        # Normalise by total absolute weight, not by peak: convolving with a
        # 1024-tap kernel whose peak is 1 multiplies the signal by roughly the
        # sum of its taps, which drove the output to hard clipping.
        ir /= float(np.sum(np.abs(ir))) or 1.0
        return ir

    def _ramp(self, n, a, b):
        return np.linspace(a, b, n, dtype=np.float32)

    def _osc(self, n, freq, phase_attr):
        """Phase-continuous sine at a fixed frequency for this block."""
        ph = getattr(self, phase_attr)
        step = freq / SAMPLE_RATE
        phases = ph + np.arange(1, n + 1, dtype=np.float32) * step
        setattr(self, phase_attr, float(phases[-1] % 1.0))
        return np.sin(2.0 * math.pi * phases).astype(np.float32)

    def render(self, n: int):
        rpm0, rpm1, load, g0, g1 = self._smooth(n)
        boost, fuel, speed = self._cur_boost, self._cur_fuel, self._cur_speed
        noise = self._rng.standard_normal(n).astype(np.float32)

        # ---- cranking -----------------------------------------------------
        if self._cranking and not self._running:
            # Starter motor: a whirring tone plus the lumpy compression beat of
            # the engine being turned over.
            whir = self._osc(n, 115.0, "_starter_phase") * 0.22
            lump = self._osc(n, max(rpm1, 60.0) / 30.0, "_turbo_phase") * 0.18
            out = (whir + lump + noise * 0.05) * self._ramp(n, g0, g1)
            return self._stereo(np.tanh(out).astype(np.float32), n)

        # ---- firing phase, ramped so pitch glides -------------------------
        f0 = firing_hz(max(rpm0, 1.0))
        f1 = firing_hz(max(rpm1, 1.0))
        inc = self._ramp(n, f0, f1) / SAMPLE_RATE
        phase = self._phase + np.cumsum(inc)
        self._phase = float(phase[-1] % 1.0)
        frac = phase % 1.0

        # ---- combustion: one impulse per firing event ---------------------
        # Sharper under load, which is what takes a diesel from a soft chuff at
        # idle to a hard bark on boost.
        sharp = 9.0 + 22.0 * load
        pulse = np.exp(-sharp * frac).astype(np.float32)
        pulse -= pulse.mean()

        # Cylinder-to-cylinder variation: a perfectly regular train sounds
        # synthetic. Keyed to the firing index so it repeats every 4 cylinders.
        cyl = (phase.astype(np.int64) % CYLINDERS).astype(np.float32)
        pulse *= 1.0 + 0.10 * np.sin(cyl * 2.3987)

        # Injector clatter: broadband noise gated by the firing pulse.
        clatter = noise * np.exp(-26.0 * frac).astype(np.float32)

        # Fuelling, not throttle, scales combustion. On the overrun injection
        # stops outright, so only the mechanical layers remain - which is what
        # makes the note hollow out the instant you lift.
        combustion = pulse * (0.22 + 1.60 * fuel) + clatter * (0.06 + 0.52 * fuel)
        # Pumping and valvetrain noise is there regardless of fuelling, but it
        # has to stay well below the combustion layer or the overrun is barely
        # quieter than being on power.
        mech = pulse * 0.13 + clatter * 0.06
        excite = combustion + mech + self._crackle(n, noise)

        # ---- resonator, overlap-save so blocks join seamlessly ------------
        padded = np.concatenate((self._tail, excite))
        wet = np.convolve(padded, self._ir, mode="valid").astype(np.float32)
        self._tail = padded[-(self.IR_LEN - 1):]

        comp = (max(rpm1, 200.0) / self.LEVEL_COMP_REF_RPM) ** self.LEVEL_COMP_EXP
        comp = min(2.5, max(0.5, comp))
        out = wet * (self.MAKEUP * comp)

        # ---- turbo whine, driven by boost not throttle --------------------
        # The compressor spins roughly 19x crank speed, and it only sings when
        # it is actually making boost - so it lags the pedal exactly as the
        # turbo does.
        turbo_f = max(rpm1, 1.0) / 60.0 * 19.0
        out += self._osc(n, turbo_f, "_turbo_phase") * (0.075 * boost * boost)

        # ---- transmission whine from road speed ---------------------------
        if speed > 4.0:
            trans_f = 48.0 + speed * 7.5
            out += self._osc(n, trans_f, "_trans_phase") * 0.013

        # ---- intake rush --------------------------------------------------
        out += noise * (0.012 + 0.040 * fuel) * min(1.0, rpm1 / 3000.0)

        # ---- one-shot transients ------------------------------------------
        out += self._transients(n, noise, boost)

        # ---- tyre scrabble when past the traction limit -------------------
        if self._wheelspin > 0.01:
            out += noise * (0.09 * self._wheelspin)

        out *= self._ramp(n, g0, g1)
        # Soft limit rather than hard clip: tanh is bounded by construction, so
        # a transient can never crack, and it saturates like a speaker would.
        return self._stereo(np.tanh(out).astype(np.float32), n)

    def _crackle(self, n, noise):
        """Overrun pops and bangs, fed through the exhaust resonator.

        Individual cracks are a few milliseconds of noise with a hard decay,
        scattered at random through the block by a Poisson draw so they never
        fall into an audible rhythm. Adding them to the excitation rather than
        to the output means the resonator colours them like everything else -
        they come out of the exhaust, not out of nowhere.
        """
        out = np.zeros(n, dtype=np.float32)
        if self._ev_pop > 0.0:
            self._bang_env = max(self._bang_env, self._ev_pop)
            self._ev_pop = 0.0

        dt = n / SAMPLE_RATE

        # The first bang on the lift: bigger, lower, one hit.
        if self._bang_env > 0.01:
            dur = min(n, int(0.035 * SAMPLE_RATE))
            env = np.exp(-np.linspace(0.0, 1.0, dur, dtype=np.float32) * 5.0)
            out[:dur] += noise[:dur] * env * (1.6 * self._bang_env)
            self._bang_env *= math.exp(-dt / 0.06)
        else:
            self._bang_env = 0.0

        # The crackle that follows, while the overrun lasts.
        level = self._cur_pop
        if level > 0.01:
            expected = 90.0 * level * n / SAMPLE_RATE
            count = int(min(10, self._rng.poisson(expected)))
            for _ in range(count):
                pos = int(self._rng.integers(0, n))
                dur = min(n - pos, int(0.005 * SAMPLE_RATE))
                if dur < 8:
                    continue
                # Hard decay keeps each one a crack rather than a puff, and
                # the amplitude has to clear the mechanical floor that is all
                # that remains on the overrun - at the first tuning the pops
                # only lifted the level 25%, which was inaudible in practice.
                env = np.exp(-np.linspace(0.0, 1.0, dur, dtype=np.float32) * 9.0)
                amp = (1.1 + 1.8 * float(self._rng.random())) * level
                out[pos:pos + dur] += noise[pos:pos + dur] * env * amp
        return out

    def _transients(self, n, noise, boost):
        """Wastegate dump, shift thunk and start catch, as decaying envelopes."""
        out = np.zeros(n, dtype=np.float32)
        dt = n / SAMPLE_RATE

        if self._ev_bov > 0.0:
            self._bov_env = max(self._bov_env, self._ev_bov)
            self._ev_bov = 0.0
        if self._ev_shift > 0.0:
            self._shift_env = 1.0
            self._ev_shift = 0.0
        if self._ev_start > 0.0:
            self._start_env = 1.0
            self._ev_start = 0.0

        if self._bov_env > 0.001:
            # Pressure dumping past the wastegate: filtered hiss, fading fast.
            env = self._ramp(n, self._bov_env, self._bov_env * math.exp(-dt / 0.13))
            out += noise * env * 0.28
            self._bov_env *= math.exp(-dt / 0.13)
        else:
            self._bov_env = 0.0

        if self._shift_env > 0.001:
            # A short low thunk through the driveline.
            env = self._ramp(n, self._shift_env, self._shift_env * math.exp(-dt / 0.05))
            out += self._osc(n, 70.0, "_trans_phase") * env * 0.20
            out += noise * env * 0.05
            self._shift_env *= math.exp(-dt / 0.05)
        else:
            self._shift_env = 0.0

        if self._start_env > 0.001:
            env = self._ramp(n, self._start_env, self._start_env * math.exp(-dt / 0.25))
            out += noise * env * 0.10
            self._start_env *= math.exp(-dt / 0.25)
        else:
            self._start_env = 0.0

        return out

    def _stereo(self, mono, n):
        """Widen to stereo with a small decorrelation between the channels."""
        if CHANNELS == 1:
            return mono
        out = np.empty((n, 2), dtype=np.float32)
        out[:, 0] = mono
        # One-sample shift plus a touch of level difference is enough to stop
        # it sounding like a point source in the middle of your head.
        out[1:, 1] = mono[:-1] * 0.97
        out[0, 1] = mono[0] * 0.97
        return out


class SampleEngine(_StreamEngine):
    """Real looping recordings, resampled and crossfaded by rpm."""

    name = "samples"

    def __init__(self, loops, volume: float = 0.6):
        super().__init__(volume)
        self._loops = sorted(loops, key=lambda it: it[0])
        self._pos = [0.0] * len(self._loops)

    @classmethod
    def load(cls, directory: Path = SAMPLES_DIR, volume: float = 0.6):
        """Load ``rpm_<rpm>.wav`` loops. Returns None if there are none."""
        if np is None or not directory.is_dir():
            return None
        loops = []
        for path in sorted(directory.glob("rpm_*.wav")):
            stem = path.stem.split("_", 1)[1]
            try:
                rpm = float(stem)
            except ValueError:
                continue
            data = _read_wav_mono(path)
            if data is not None and data.size > 64:
                loops.append((rpm, data))
        if not loops:
            return None
        return cls(loops, volume)

    def _pick(self, rpm: float):
        rpms = [r for r, _ in self._loops]
        if rpm <= rpms[0]:
            return 0, 0, 0.0
        if rpm >= rpms[-1]:
            last = len(rpms) - 1
            return last, last, 0.0
        for i in range(len(rpms) - 1):
            if rpm <= rpms[i + 1]:
                span = rpms[i + 1] - rpms[i]
                return i, i + 1, 0.0 if span <= 0 else (rpm - rpms[i]) / span
        last = len(rpms) - 1
        return last, last, 0.0

    def _play(self, idx: int, n: int, rpm: float):
        base_rpm, data = self._loops[idx]
        ratio = max(0.25, min(4.0, rpm / max(base_rpm, 1.0)))
        start = self._pos[idx]
        idxs = start + np.arange(n, dtype=np.float64) * ratio
        self._pos[idx] = float(idxs[-1] + ratio) % data.size
        i0 = np.floor(idxs).astype(np.int64) % data.size
        i1 = (i0 + 1) % data.size
        frac = (idxs - np.floor(idxs)).astype(np.float32)
        return data[i0] * (1.0 - frac) + data[i1] * frac

    def render(self, n: int):
        rpm0, rpm1, load, g0, g1 = self._smooth(n)
        rpm = max(rpm1, 1.0)
        lo, hi, blend = self._pick(rpm)
        out = self._play(lo, n, rpm) * (1.0 - blend)
        if hi != lo:
            out = out + self._play(hi, n, rpm) * blend
        # Fuelling shapes level, since a recording at a given rpm is one load.
        out = out * (0.5 + 0.5 * self._cur_fuel)
        out = out * np.linspace(g0, g1, n, dtype=np.float32)
        out = np.clip(out.astype(np.float32), -1.0, 1.0)
        if CHANNELS == 1:
            return out
        st = np.empty((n, 2), dtype=np.float32)
        st[:, 0] = out
        st[:, 1] = out
        return st


def _read_wav_mono(path: Path):
    """Read a PCM wav into a mono float32 array in -1..1."""
    try:
        with wave.open(str(path), "rb") as w:
            frames = w.readframes(w.getnframes())
            width, channels = w.getsampwidth(), w.getnchannels()
        if width == 2:
            arr = np.frombuffer(frames, dtype="<i2").astype(np.float32) / 32768.0
        elif width == 1:
            arr = (np.frombuffer(frames, dtype=np.uint8).astype(np.float32) - 128.0) / 128.0
        elif width == 4:
            arr = np.frombuffer(frames, dtype="<i4").astype(np.float32) / 2147483648.0
        else:
            return None
        if channels > 1:
            arr = arr.reshape(-1, channels).mean(axis=1)
        return np.ascontiguousarray(arr, dtype=np.float32)
    except Exception:
        return None


def make_engine(mode: str = "auto", volume: float = 0.6):
    """Build the best available engine back end.

    mode: "auto" prefers real samples when present, "synth", "samples", "off".
    """
    if mode == "off" or np is None or sd is None:
        return NullEngine()
    if mode in ("auto", "samples"):
        eng = SampleEngine.load(volume=volume)
        if eng is not None:
            return eng
        if mode == "samples":
            return NullEngine()
    return SynthEngine(volume=volume)


def audio_status() -> str:
    """Human-readable reason audio may be unavailable."""
    if np is None:
        return "numpy missing"
    if sd is None:
        return "sounddevice missing"
    if not SAMPLES_DIR.is_dir() or not any(SAMPLES_DIR.glob("rpm_*.wav")):
        return "synth (no samples/ loops found)"
    return "samples"


def drive_from_vehicle(engine, v) -> None:
    """Feed one simulation frame to the audio engine, events included."""
    engine.update(rpm=v.rpm, load=v.load, running=v.running, boost=v.boost,
                  fuelling=v.fuelling, speed_kmh=v.speed_kmh,
                  cranking=v.cranking, wheelspin=v.wheelspin, pop=v.pop)
    if v.ev_bov or v.ev_shift or v.ev_start or v.ev_pop:
        engine.event(bov=v.ev_bov, shift=v.ev_shift, start=v.ev_start,
                     pop=v.ev_pop)
