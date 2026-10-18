"""Engine audio.

Two back ends:

``SynthEngine`` (default)
    Pulse-train synthesis. A 4-stroke engine fires each cylinder once every
    two crank revolutions, so a 4-cylinder fires twice per revolution and the
    firing frequency is rpm/30 Hz - 27 Hz at an 800 rpm idle, 150 Hz at 4500.
    Engine noise is not a sustained harmonic tone, it is a train of exhaust
    pressure pulses coloured by the resonance of the exhaust tract, so that is
    what this builds: an impulse per firing event, convolved with the decaying
    sinusoidal impulse response of a resonator, plus injector clatter and
    turbo whine that track load. Rendering is block-based with overlap-save so
    there are no seams, and the firing phase is continuous across blocks so
    pitch glides smoothly instead of stepping per buffer.

``SampleEngine``
    Crossfades and resamples real looping recordings from ``host/samples/``
    (``rpm_0800.wav``, ``rpm_1500.wav``, ... - the number is the rpm the loop
    was recorded at). Use this if you have a sample set; see README.md. I did
    not ship one, because no realistic multi-rpm engine pack exists under a
    license that allows redistribution - the one CC0 "racing engine" pack on
    OpenGameArt is a pitch-shifted seatbelt warning chime, not an engine.

Both are optional: if numpy or sounddevice are missing, ``make_engine``
returns a ``NullEngine`` and everything else still runs.
"""

from __future__ import annotations

import math
import os
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
    def update(self, rpm: float, load: float, running: bool) -> None: ...


class _StreamEngine:
    """Shared output-stream plumbing."""

    available = True
    name = "base"

    def __init__(self, volume: float = 0.6):
        self.volume = volume
        self._stream = None
        # Target values written by the GUI thread, read by the audio thread.
        # Plain floats, so reads and writes are atomic enough for this.
        self._rpm = 0.0
        self._load = 0.0
        self._running = False
        # Smoothed values owned by the audio thread.
        self._cur_rpm = 0.0
        self._cur_load = 0.0
        self._cur_gain = 0.0

    def start(self) -> None:
        if sd is None:
            return
        self._stream = sd.OutputStream(
            samplerate=SAMPLE_RATE,
            blocksize=BLOCK,
            channels=1,
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

    def update(self, rpm: float, load: float, running: bool) -> None:
        self._rpm = float(rpm)
        self._load = float(max(0.0, min(1.0, load)))
        self._running = bool(running)

    def _smooth(self, n: int):
        """Advance the smoothed rpm/load/gain towards their targets.

        Returns (rpm_start, rpm_end, load, gain_start, gain_end) so the block
        can ramp rather than jump - a step in either one is audible as a click.
        """
        dt = n / SAMPLE_RATE
        rpm0 = self._cur_rpm
        # ~25 ms time constant: fast enough to feel connected to the throttle,
        # slow enough that frame-rate jitter in the sim does not warble.
        k = min(1.0, dt / 0.025)
        self._cur_rpm += (self._rpm - self._cur_rpm) * k
        self._cur_load += (self._load - self._cur_load) * min(1.0, dt / 0.05)
        g0 = self._cur_gain
        target_gain = self.volume if self._running else 0.0
        self._cur_gain += (target_gain - self._cur_gain) * min(1.0, dt / 0.08)
        return rpm0, self._cur_rpm, self._cur_load, g0, self._cur_gain

    def _callback(self, outdata, frames, time_info, status):  # pragma: no cover
        try:
            outdata[:, 0] = self.render(frames)
        except Exception:
            outdata.fill(0.0)

    def render(self, n: int):
        raise NotImplementedError


class SynthEngine(_StreamEngine):
    """Pulse-train + resonator diesel model."""

    name = "synth"

    # Exhaust resonator: two decaying sinusoids, which is enough to give the
    # pulse train a body and a bark without a per-sample IIR loop.
    RESONANCES = ((92.0, 0.016), (210.0, 0.009), (640.0, 0.004))
    IR_LEN = 1024
    # Measured so the loudest operating point peaks near 0.85 at volume 1.0,
    # leaving the soft limiter with nothing to do under normal use.
    MAKEUP = 4.5
    # Each firing pulse decays over its own firing cycle, so a pulse lasts
    # less wall-clock time as revs rise and carries less energy into the
    # resonator - measured output fell about 4x from idle to redline. This
    # exponent flattens that back out so the engine does not fade as it revs.
    LEVEL_COMP_REF_RPM = 1800.0
    LEVEL_COMP_EXP = 0.6

    def __init__(self, volume: float = 0.6):
        super().__init__(volume)
        self._phase = 0.0          # firing-cycle phase, in cycles
        self._turbo_phase = 0.0
        self._tail = np.zeros(self.IR_LEN - 1, dtype=np.float32) if np is not None else None
        self._ir = self._build_ir() if np is not None else None
        self._rng = np.random.default_rng(7) if np is not None else None

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

    def render(self, n: int):
        rpm0, rpm1, load, g0, g1 = self._smooth(n)

        # Firing phase, ramped across the block so pitch glides continuously.
        f0 = firing_hz(max(rpm0, 1.0))
        f1 = firing_hz(max(rpm1, 1.0))
        inc = np.linspace(f0, f1, n, dtype=np.float32) / SAMPLE_RATE
        phase = self._phase + np.cumsum(inc)
        self._phase = float(phase[-1] % 1.0)

        frac = phase % 1.0

        # One impulse per firing event: a sharp exponential decay over the
        # firing cycle. Sharper under load, which is what makes a diesel go
        # from a soft chuff at idle to a hard bark on boost.
        sharp = 9.0 + 22.0 * load
        pulse = np.exp(-sharp * frac).astype(np.float32)
        pulse -= pulse.mean()

        # Cylinder-to-cylinder variation: real engines are never even, and a
        # perfectly regular train sounds synthetic. Keyed to the firing index
        # so it repeats every 4 cylinders instead of being pure noise.
        cyl = (phase.astype(np.int64) % CYLINDERS).astype(np.float32)
        pulse *= 1.0 + 0.10 * np.sin(cyl * 2.3987)

        # Diesel injector clatter: broadband noise gated by the firing pulse.
        noise = self._rng.standard_normal(n).astype(np.float32)
        clatter = noise * np.exp(-26.0 * frac).astype(np.float32)
        excite = pulse * (1.0 + 0.55 * load) + clatter * (0.22 + 0.30 * load)

        # Resonator, via overlap-save so blocks join seamlessly.
        padded = np.concatenate((self._tail, excite))
        wet = np.convolve(padded, self._ir, mode="valid").astype(np.float32)
        self._tail = padded[-(self.IR_LEN - 1):]

        # Turbo whine: the compressor spins far faster than the crank, and it
        # only sings when it is actually making boost.
        turbo_f = max(rpm1, 1.0) / 60.0 * 19.0
        self._turbo_phase = (self._turbo_phase + turbo_f * n / SAMPLE_RATE) % 1.0
        tp = np.linspace(0.0, turbo_f * n / SAMPLE_RATE, n, dtype=np.float32) + self._turbo_phase
        whine = np.sin(2.0 * math.pi * tp).astype(np.float32) * (0.05 * load * load)

        # Intake rush, rising with revs and load.
        rush = noise * (0.015 + 0.035 * load) * min(1.0, rpm1 / 3000.0)

        comp = (max(rpm1, 200.0) / self.LEVEL_COMP_REF_RPM) ** self.LEVEL_COMP_EXP
        comp = min(2.5, max(0.5, comp))

        out = wet * (self.MAKEUP * comp) + whine + rush
        out *= np.linspace(g0, g1, n, dtype=np.float32)
        # Soft limit rather than hard clip: tanh is bounded by construction, so
        # a transient can never crack, and it saturates like a speaker would.
        return np.tanh(out).astype(np.float32)


class SampleEngine(_StreamEngine):
    """Real looping recordings, resampled and crossfaded by rpm."""

    name = "samples"

    def __init__(self, loops: list[tuple[float, "np.ndarray"]], volume: float = 0.6):
        super().__init__(volume)
        self._loops = sorted(loops, key=lambda it: it[0])
        self._pos = [0.0] * len(self._loops)

    @classmethod
    def load(cls, directory: Path = SAMPLES_DIR, volume: float = 0.6):
        """Load ``rpm_<rpm>.wav`` loops. Returns None if there are none."""
        if np is None or not directory.is_dir():
            return None
        loops: list[tuple[float, np.ndarray]] = []
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
        """Return (lower_index, upper_index, blend 0..1)."""
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
        """Read n samples from loop idx, resampled to the requested rpm."""
        base_rpm, data = self._loops[idx]
        ratio = max(0.25, min(4.0, rpm / max(base_rpm, 1.0)))
        start = self._pos[idx]
        idxs = start + np.arange(n, dtype=np.float64) * ratio
        self._pos[idx] = float(idxs[-1] + ratio) % data.size
        # Linear interpolation with wraparound keeps the loop seamless.
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
        # Load shapes level, since a recording at a given rpm is one load only.
        out = out * (0.55 + 0.45 * load)
        out *= np.linspace(g0, g1, n, dtype=np.float32)
        return np.clip(out.astype(np.float32), -1.0, 1.0)


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
