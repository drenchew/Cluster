"""Drawing primitives for the cluster: dials, needles, glow, telltale icons.

Two things make the difference between a diagram and something that reads like
an instrument cluster:

* **Cache the static parts.** A dial face - bezel, ticks, numerals, red zone -
  never changes, so it is rendered once onto a Surface and blitted. Only the
  needle is redrawn per frame. That is what buys the headroom to anti-alias
  everything and still hold 60 fps.
* **Anti-alias the needle.** A needle is a long thin diagonal, which is the
  worst case for aliasing. Each is drawn once into a small horizontal Surface
  and then rotated with ``rotozoom``, which filters as it rotates, instead of
  being drawn as a rotated polygon with hard edges.

Glow is a downscale-blur-upscale blit in ``BLEND_ADD``: cheap, and it is what
stops lit telltales and the needle tip looking like flat coloured shapes.
"""

from __future__ import annotations

import math

import pygame

# ---------------------------------------------------------------------------
# Palette - a dark BMW-ish cluster: near-black faces, amber-orange needles,
# cold white numerals.
# ---------------------------------------------------------------------------
BG = (9, 10, 13)
FACE_OUTER = (38, 41, 49)
FACE_INNER = (15, 16, 21)
BEZEL = (58, 63, 74)
BEZEL_HI = (92, 99, 114)
TICK = (206, 212, 224)
TICK_MINOR = (108, 115, 130)
NUMERAL = (226, 231, 241)
LABEL = (122, 130, 146)
NEEDLE = (236, 86, 38)
NEEDLE_HUB = (176, 182, 196)
REDZONE = (150, 32, 28)
LCD_BG = (16, 26, 22)
LCD_FG = (150, 226, 180)

GOOD = (96, 206, 134)
WARN = (238, 186, 58)
BAD = (224, 72, 66)
BLUE = (96, 150, 240)
AMBER = (240, 162, 48)
OFF = (38, 41, 48)


def lerp(a, b, t):
    return a + (b - a) * t


def mix(c1, c2, t):
    return tuple(int(lerp(c1[i], c2[i], t)) for i in range(3))


# ---------------------------------------------------------------------------
# Glow
# ---------------------------------------------------------------------------
def add_glow(dest: pygame.Surface, src: pygame.Surface, pos, strength=1.0,
             downscale=6):
    """Additively blit a blurred copy of ``src`` onto ``dest``.

    The blur is a downscale followed by a smooth upscale - a box blur in two
    scalings, which is far cheaper than a real kernel and indistinguishable at
    this size.
    """
    if strength <= 0.0:
        return
    w, h = src.get_size()
    sw, sh = max(1, w // downscale), max(1, h // downscale)
    small = pygame.transform.smoothscale(src, (sw, sh))
    big = pygame.transform.smoothscale(small, (w, h))
    if strength < 1.0:
        big.set_alpha(int(255 * strength))
    dest.blit(big, pos, special_flags=pygame.BLEND_ADD)


def radial_face(radius: int, outer=FACE_OUTER, inner=FACE_INNER) -> pygame.Surface:
    """A dial face with a soft radial gradient, drawn as concentric circles."""
    size = radius * 2
    surf = pygame.Surface((size, size), pygame.SRCALPHA)
    steps = 26
    for i in range(steps, 0, -1):
        t = i / steps
        r = int(radius * t)
        pygame.draw.circle(surf, mix(inner, outer, t), (radius, radius), r)
    return surf


# ---------------------------------------------------------------------------
# Needle
# ---------------------------------------------------------------------------
class Needle:
    """A pre-rendered needle, rotated per frame.

    Drawn pointing right (0 degrees) into a surface wide enough to hold it,
    then ``rotozoom``-ed. The pivot is at ``tail`` pixels from the left, so
    rotating about the surface centre puts the pivot on the dial centre.
    """

    def __init__(self, length: int, tail: int, width: int, colour=NEEDLE):
        self.length = length
        self.tail = tail
        span = (length + tail) * 2
        self.surf = pygame.Surface((span, span), pygame.SRCALPHA)
        cx = cy = span // 2
        # A tapered needle: wide at the hub, a point at the tip.
        pts = [
            (cx - tail, cy - max(1, width // 3)),
            (cx + length, cy - 1),
            (cx + length, cy + 1),
            (cx - tail, cy + max(1, width // 3)),
        ]
        pygame.draw.polygon(self.surf, colour, pts)
        # A brighter core line keeps it crisp once rotozoom has filtered it.
        pygame.draw.line(self.surf, mix(colour, (255, 255, 255), 0.45),
                         (cx - tail, cy), (cx + length, cy), max(1, width // 3))

    def blit(self, dest: pygame.Surface, centre, angle_deg: float,
             glow: float = 0.55):
        rot = pygame.transform.rotozoom(self.surf, -angle_deg, 1.0)
        rect = rot.get_rect(center=centre)
        if glow > 0.0:
            add_glow(dest, rot, rect.topleft, glow, downscale=5)
        dest.blit(rot, rect.topleft)


# ---------------------------------------------------------------------------
# Dial
# ---------------------------------------------------------------------------
class Dial:
    """A round gauge: cached face, live needle.

    ``start_deg``/``sweep_deg`` are screen angles measured clockwise from the
    3 o'clock position, matching pygame's y-down coordinates.
    """

    def __init__(self, radius: int, vmax: float, major: float, minor_per_major=2,
                 label="", value_scale=1.0, redline=None, fonts=None,
                 start_deg=144.0, sweep_deg=252.0, decimals=0,
                 numerals=True, end_labels=None, vmin=0.0):
        self.radius = radius
        self.vmin = float(vmin)
        self.vmax = float(vmax)
        self.major = float(major)
        self.minor_per_major = minor_per_major
        self.label = label
        self.value_scale = value_scale
        self.redline = redline
        self.start = start_deg
        self.sweep = sweep_deg
        self.decimals = decimals
        # Small dials have no room for numerals, and a real cluster does not
        # print numbers on fuel or temperature anyway - it marks the ends.
        self.numerals = numerals
        self.end_labels = end_labels
        self.fonts = fonts
        self.face = self._render_face()
        self.needle = Needle(int(radius * 0.80), int(radius * 0.13),
                             max(4, radius // 16))

    # -- geometry -----------------------------------------------------------
    def angle_for(self, value: float) -> float:
        span = self.vmax - self.vmin
        frac = 0.0 if span <= 0 else (value - self.vmin) / span
        frac = max(0.0, min(1.0, frac))
        return self.start + self.sweep * frac

    def _pt(self, cx, cy, ang_deg, r):
        a = math.radians(ang_deg)
        return (cx + math.cos(a) * r, cy + math.sin(a) * r)

    # -- static face --------------------------------------------------------
    def _render_face(self) -> pygame.Surface:
        r = self.radius
        size = r * 2 + 8
        surf = pygame.Surface((size, size), pygame.SRCALPHA)
        c = size // 2

        # Bezel: two rings, the outer one lighter at the top so it reads as a
        # lit metal rim rather than a flat outline.
        pygame.draw.circle(surf, BEZEL, (c, c), r + 3)
        pygame.draw.circle(surf, BEZEL_HI, (c, c), r + 3, 2)
        surf.blit(radial_face(r), (c - r, c - r))

        # Red zone as a filled annulus.
        if self.redline is not None and self.redline < self.vmax:
            a0 = self.angle_for(self.redline)
            a1 = self.start + self.sweep
            steps = 40
            inner, outer = r * 0.80, r * 0.95
            pts_out, pts_in = [], []
            for i in range(steps + 1):
                a = a0 + (a1 - a0) * i / steps
                pts_out.append(self._pt(c, c, a, outer))
                pts_in.append(self._pt(c, c, a, inner))
            pygame.draw.polygon(surf, REDZONE, pts_out + pts_in[::-1])

        # Step by the major interval and label with the actual value at each
        # tick. Dividing the range into N equal parts instead gave a 0..260
        # speedo labelled 0 / 43 / 87 / 130 - the labels have to be multiples
        # of `major`, so vmax should be one too.
        step = self.major / self.minor_per_major
        n_ticks = int(round((self.vmax - self.vmin) / step))
        f_num = self.fonts["numeral"] if self.fonts else None
        for i in range(n_ticks + 1):
            value = self.vmin + i * step
            a = self.angle_for(value)
            is_major = (i % self.minor_per_major) == 0
            r_out = r * 0.95
            r_in = r * (0.82 if is_major else 0.88)
            pygame.draw.line(surf, TICK if is_major else TICK_MINOR,
                             self._pt(c, c, a, r_in), self._pt(c, c, a, r_out),
                             3 if is_major else 1)
            if is_major and f_num is not None and self.numerals:
                txt = f"{value * self.value_scale:.{self.decimals}f}"
                img = f_num.render(txt, True, NUMERAL)
                surf.blit(img, img.get_rect(center=self._pt(c, c, a, r * 0.68)))

        if self.end_labels and self.fonts:
            for text, value in zip(self.end_labels, (self.vmin, self.vmax)):
                img = self.fonts["numeral"].render(text, True, NUMERAL)
                a = self.angle_for(value)
                surf.blit(img, img.get_rect(center=self._pt(c, c, a, r * 0.64)))

        if self.label and self.fonts:
            img = self.fonts["label"].render(self.label, True, LABEL)
            surf.blit(img, img.get_rect(center=(c, c + int(r * 0.42))))
        return surf

    # -- per frame ----------------------------------------------------------
    def draw(self, dest: pygame.Surface, centre, value: float,
             readout: str | None = None):
        face_rect = self.face.get_rect(center=centre)
        dest.blit(self.face, face_rect.topleft)
        self.needle.blit(dest, centre, self.angle_for(value))
        pygame.draw.circle(dest, NEEDLE_HUB, centre, max(6, self.radius // 14))
        pygame.draw.circle(dest, BEZEL, centre, max(6, self.radius // 14), 2)
        if readout is not None and self.fonts:
            img = self.fonts["readout"].render(readout, True, NUMERAL)
            dest.blit(img, img.get_rect(
                center=(centre[0], centre[1] + int(self.radius * 0.62))))


# ---------------------------------------------------------------------------
# Needle dynamics
# ---------------------------------------------------------------------------
class NeedleMotion:
    """Second-order needle response.

    A real cluster needle is a stepper driving a mass against a spring, so it
    lags, overshoots a little and settles. A plain exponential chase looks
    artificially smooth by comparison, especially on a fast tacho rise.
    """

    # Explicit Euler on a spring this stiff needs w*h well under 1 to stay
    # stable, and w is ~21 rad/s - so a single 1/60 s step is already marginal
    # and a dropped frame makes it diverge outright. Integrate in fixed
    # substeps instead, which also keeps the overshoot accurate rather than
    # letting integration error damp it away.
    MAX_SUBSTEP = 1.0 / 240.0

    def __init__(self, freq=3.4, damping=0.72):
        self.value = 0.0
        self.vel = 0.0
        self.freq = freq
        self.damping = damping

    def step(self, target: float, dt: float) -> float:
        dt = max(1e-5, min(dt, 0.25))
        n = int(dt / self.MAX_SUBSTEP) + 1
        h = dt / n
        w = 2.0 * math.pi * self.freq
        k = w * w
        c = 2.0 * self.damping * w
        for _ in range(n):
            accel = k * (target - self.value) - c * self.vel
            self.vel += accel * h
            self.value += self.vel * h
        return self.value


# ---------------------------------------------------------------------------
# Bars and telltales
# ---------------------------------------------------------------------------
def bar(dest, font, x, y, w, label, value, vmax, colour, text=None):
    dest.blit(font.render(label, True, LABEL), (x, y - 1))
    track = pygame.Rect(x + 86, y + 2, w, 11)
    pygame.draw.rect(dest, (22, 24, 29), track, border_radius=3)
    frac = 0.0 if vmax <= 0 else max(0.0, min(1.0, value / vmax))
    if frac > 0:
        fill = pygame.Rect(track.x, track.y, max(2, int(track.w * frac)), track.h)
        pygame.draw.rect(dest, colour, fill, border_radius=3)
    pygame.draw.rect(dest, (52, 56, 66), track, 1, border_radius=3)
    if text is not None:
        dest.blit(font.render(text, True, NUMERAL), (track.right + 10, y - 1))


def _icon_turn(surf, r, colour, left: bool):
    # Smaller than it looks like it should be: this is the only solid-filled
    # icon in the row, so at the same nominal size as the 2px-outline icons it
    # carries far more visual weight and reads as oversized next to them.
    w, h = surf.get_size()
    cx, cy = w // 2, h // 2
    d = -1 if left else 1
    pts = [(cx + d * 4, cy - 6), (cx + d * 4, cy + 6), (cx - d * 6, cy)]
    pygame.draw.polygon(surf, colour, pts)


def _icon_beam(surf, colour, high: bool):
    w, h = surf.get_size()
    cx, cy = w // 2, h // 2
    pygame.draw.ellipse(surf, colour, pygame.Rect(cx - 11, cy - 8, 13, 16), 2)
    for i in range(3):
        y = cy - 5 + i * 5
        if high:
            pygame.draw.line(surf, colour, (cx + 3, y), (cx + 12, y), 2)
        else:
            pygame.draw.line(surf, colour, (cx + 3, y), (cx + 11, y + 3), 2)


def _icon_park(surf, colour):
    w, h = surf.get_size()
    cx, cy = w // 2, h // 2
    pygame.draw.circle(surf, colour, (cx, cy), 10, 2)
    for i in range(8):
        a = math.radians(i * 45)
        pygame.draw.line(surf, colour,
                         (cx + math.cos(a) * 10, cy + math.sin(a) * 10),
                         (cx + math.cos(a) * 13, cy + math.sin(a) * 13), 2)
    f = pygame.font.SysFont("consolas,monospace", 11, bold=True)
    img = f.render("P", True, colour)
    surf.blit(img, img.get_rect(center=(cx, cy)))


def _icon_battery(surf, colour):
    w, h = surf.get_size()
    cx, cy = w // 2, h // 2
    body = pygame.Rect(cx - 12, cy - 7, 24, 14)
    pygame.draw.rect(surf, colour, body, 2)
    pygame.draw.rect(surf, colour, pygame.Rect(cx - 8, cy - 10, 4, 3))
    pygame.draw.rect(surf, colour, pygame.Rect(cx + 4, cy - 10, 4, 3))
    pygame.draw.line(surf, colour, (cx - 7, cy), (cx - 2, cy), 2)
    pygame.draw.line(surf, colour, (cx + 2, cy - 3), (cx + 2, cy + 3), 2)
    pygame.draw.line(surf, colour, (cx - 1, cy), (cx + 5, cy), 2)


def _icon_oil(surf, colour):
    w, h = surf.get_size()
    cx, cy = w // 2, h // 2
    pts = [(cx - 13, cy + 4), (cx - 4, cy + 4), (cx - 1, cy - 2),
           (cx + 2, cy + 4), (cx + 9, cy + 4)]
    pygame.draw.lines(surf, colour, False, pts, 2)
    pygame.draw.circle(surf, colour, (cx + 7, cy - 4), 3)
    pygame.draw.line(surf, colour, (cx - 13, cy + 4), (cx - 13, cy - 1), 2)


def _icon_dsc(surf, colour):
    w, h = surf.get_size()
    cx, cy = w // 2, h // 2
    pygame.draw.circle(surf, colour, (cx - 1, cy - 2), 7, 2)
    pygame.draw.arc(surf, colour, pygame.Rect(cx - 13, cy + 1, 12, 9),
                    math.radians(200), math.radians(340), 2)
    pygame.draw.arc(surf, colour, pygame.Rect(cx + 2, cy + 1, 12, 9),
                    math.radians(200), math.radians(340), 2)


def _icon_abs(surf, colour):
    f = pygame.font.SysFont("consolas,monospace", 12, bold=True)
    img = f.render("ABS", True, colour)
    w, h = surf.get_size()
    surf.blit(img, img.get_rect(center=(w // 2, h // 2)))
    pygame.draw.circle(surf, colour, (w // 2, h // 2), 13, 1)


def _icon_fog(surf, colour, rear: bool):
    w, h = surf.get_size()
    cx, cy = w // 2, h // 2
    pygame.draw.ellipse(surf, colour, pygame.Rect(cx - 12, cy - 8, 11, 16), 2)
    for i in range(3):
        y = cy - 5 + i * 5
        pygame.draw.line(surf, colour, (cx - 1, y), (cx + 11, y), 2)
    pygame.draw.line(surf, colour, (cx + 2, cy - 9), (cx + 2, cy + 9), 1)
    if rear:
        pygame.draw.line(surf, colour, (cx + 7, cy - 9), (cx + 7, cy + 9), 1)


def _icon_seat(surf, colour):
    w, h = surf.get_size()
    cx, cy = w // 2, h // 2
    pygame.draw.circle(surf, colour, (cx - 2, cy - 7), 4)
    pygame.draw.line(surf, colour, (cx - 2, cy - 3), (cx - 2, cy + 7), 3)
    pygame.draw.line(surf, colour, (cx - 2, cy + 7), (cx + 8, cy + 7), 3)
    pygame.draw.line(surf, colour, (cx - 9, cy - 1), (cx + 6, cy + 9), 2)


def _icon_glow(surf, colour):
    """Diesel pre-heat coil."""
    w, h = surf.get_size()
    cx, cy = w // 2, h // 2
    pts = []
    for i in range(25):
        t = i / 24
        pts.append((cx - 11 + t * 22, cy + math.sin(t * math.pi * 3.0) * 6))
    pygame.draw.lines(surf, colour, False, pts, 2)


ICONS = {
    "turn_l": lambda s, c: _icon_turn(s, 10, c, True),
    "turn_r": lambda s, c: _icon_turn(s, 10, c, False),
    "low": lambda s, c: _icon_beam(s, c, False),
    "high": lambda s, c: _icon_beam(s, c, True),
    "park": _icon_park,
    "batt": _icon_battery,
    "oil": _icon_oil,
    "dsc": _icon_dsc,
    "abs": _icon_abs,
    "fog_f": lambda s, c: _icon_fog(s, c, False),
    "fog_r": lambda s, c: _icon_fog(s, c, True),
    "belt": _icon_seat,
    "glow": _icon_glow,
}


class TelltaleBank:
    """Renders each telltale icon once per colour and caches it."""

    SIZE = (38, 30)

    def __init__(self):
        self._cache: dict[tuple[str, tuple[int, int, int]], pygame.Surface] = {}

    def get(self, name: str, colour) -> pygame.Surface:
        key = (name, colour)
        hit = self._cache.get(key)
        if hit is None:
            surf = pygame.Surface(self.SIZE, pygame.SRCALPHA)
            drawer = ICONS.get(name)
            if drawer is not None:
                drawer(surf, colour)
            self._cache[key] = surf
            hit = surf
        return hit

    # Solid icons need less glow than outlined ones or they bloom into blobs.
    GLOW = {"turn_l": 0.5, "turn_r": 0.5}

    def draw(self, dest, name: str, pos, on: bool, colour):
        img = self.get(name, colour if on else OFF)
        if on:
            add_glow(dest, img, pos, self.GLOW.get(name, 0.9), downscale=4)
        dest.blit(img, pos)
