"""Pygame dashboard and keyboard control.

Draws what the cluster is being told, so you can see whether a disagreement is
the simulator's fault or the cluster's. Needles are tweened towards their
targets rather than snapped, which also matches how the real gauges move.
"""

from __future__ import annotations

import math

import pygame

from . import link as lk
from .vehicle import LIMITER_RPM, SEL_D, SEL_N, SEL_P, SEL_R, Vehicle

WIDTH, HEIGHT = 1060, 660
FPS = 60

BG = (16, 18, 22)
PANEL = (26, 29, 35)
EDGE = (52, 57, 66)
TEXT = (226, 230, 238)
DIM = (128, 136, 150)
ACCENT = (240, 120, 60)
GOOD = (90, 200, 130)
WARN = (240, 190, 70)
BAD = (230, 90, 90)
NEEDLE = (236, 240, 248)

TACH_MAX = 5000
SPEED_MAX = 260

HELP = [
    ("W / Up", "throttle"),
    ("S / Down", "brake"),
    ("Space", "handbrake"),
    ("I", "terminal 15 (ignition)"),
    ("Enter", "hold to crank the starter"),
    ("K", "kill engine"),
    ("A / Left", "left indicator"),
    ("D / Right", "right indicator"),
    ("Z", "hazards"),
    ("G", "shift lever: P / R / N / D"),
    ("M", "auto / manual gearbox"),
    ("E / Q", "shift up / down (manual)"),
    ("1-6", "select a gear directly"),
    ("L", "lights: off / side / low / high"),
    ("F", "front fog"),
    ("V", "rear fog"),
    ("[ / ]", "dimmer down / up"),
    ("- / =", "fuel down / up"),
    ("P", "sport shift map"),
    ("Tab", "show / hide this help"),
    ("Esc", "quit"),
]


class Dashboard:
    def __init__(self, vehicle: Vehicle, engine, link, port_label: str):
        pygame.init()
        pygame.display.set_caption("E90 cluster bench - simulator")
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("consolas,menlo,monospace", 17)
        self.small = pygame.font.SysFont("consolas,menlo,monospace", 14)
        self.big = pygame.font.SysFont("consolas,menlo,monospace", 30, bold=True)

        self.vehicle = vehicle
        self.engine = engine
        self.link = link
        self.port_label = port_label

        self.show_help = True
        self.running = True
        self.log: list[str] = []
        self._log_tail = ""

        # Tweened needle positions.
        self._n_rpm = 0.0
        self._n_speed = 0.0
        # Indicator blink.
        self._blink_t = 0.0
        self._blink_on = False

    # -- input --------------------------------------------------------------
    def handle_events(self) -> None:
        c = self.vehicle.controls
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                self.running = False
            elif ev.type == pygame.KEYDOWN:
                self._keydown(ev.key, c)
            elif ev.type == pygame.KEYUP:
                if ev.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                    c.starter = False

        keys = pygame.key.get_pressed()
        c.throttle = 1.0 if (keys[pygame.K_w] or keys[pygame.K_UP]) else 0.0
        c.brake = 1.0 if (keys[pygame.K_s] or keys[pygame.K_DOWN]) else 0.0
        c.turn_left = keys[pygame.K_a] or keys[pygame.K_LEFT]
        c.turn_right = keys[pygame.K_d] or keys[pygame.K_RIGHT]

    def _keydown(self, key: int, c) -> None:
        v = self.vehicle
        if key == pygame.K_ESCAPE:
            self.running = False
        elif key == pygame.K_TAB:
            self.show_help = not self.show_help
        elif key == pygame.K_SPACE:
            c.handbrake = not c.handbrake
        elif key == pygame.K_i:
            c.ignition = not c.ignition
            if not c.ignition:
                v.stall()
        elif key in (pygame.K_RETURN, pygame.K_KP_ENTER):
            c.starter = True
        elif key == pygame.K_k:
            v.stall()
        elif key == pygame.K_z:
            c.hazard = not c.hazard
        elif key == pygame.K_m:
            c.auto = not c.auto
            c.gear = v.gear if v.gear > 0 else 1
            c.selector = SEL_D
        elif key == pygame.K_e:
            v.shift_up()
        elif key == pygame.K_q:
            v.shift_down()
        elif key == pygame.K_g:
            v.cycle_selector()
        elif key == pygame.K_l:
            c.lights_step = (c.lights_step + 1) % 4
        elif key == pygame.K_f:
            c.fog_front = not c.fog_front
        elif key == pygame.K_v:
            c.fog_rear = not c.fog_rear
        elif key == pygame.K_p:
            c.sport = not c.sport
        elif key == pygame.K_LEFTBRACKET:
            c.dimmer = max(0, c.dimmer - 25)
        elif key == pygame.K_RIGHTBRACKET:
            c.dimmer = min(255, c.dimmer + 25)
        elif key in (pygame.K_MINUS, pygame.K_KP_MINUS):
            v.fuel_l = max(0.0, v.fuel_l - 5.0)
        elif key in (pygame.K_EQUALS, pygame.K_PLUS, pygame.K_KP_PLUS):
            v.fuel_l = min(60.0, v.fuel_l + 5.0)
        elif pygame.K_1 <= key <= pygame.K_6:
            c.auto = False
            c.selector = SEL_D
            c.gear = key - pygame.K_0

    # -- setpoints ----------------------------------------------------------
    def setpoints(self) -> lk.Setpoints:
        v = self.vehicle
        c = v.controls

        lights = 0
        if c.lights_step >= 1:
            lights |= lk.LIGHT_SIDE
        if c.lights_step >= 2:
            lights |= lk.LIGHT_LOW
        if c.lights_step >= 3:
            lights |= lk.LIGHT_HIGH
        if c.fog_front:
            lights |= lk.LIGHT_FOGF
        if c.fog_rear:
            lights |= lk.LIGHT_FOGR

        left = c.turn_left or c.hazard
        right = c.turn_right or c.hazard

        # The cluster shows the lever position, not whichever ratio happens to
        # be engaged - so a mid-shift neutral does not blink the display.
        pos = {SEL_P: lk.GEAR_P, SEL_R: lk.GEAR_R,
               SEL_N: lk.GEAR_N, SEL_D: lk.GEAR_D}[c.selector]
        num = v.gear if (c.selector == SEL_D and v.gear > 0) else 0

        mode = lk.MODE_NORMAL
        if not c.auto:
            mode = lk.MODE_MANUAL
        elif c.sport:
            mode = lk.MODE_SPORT

        return lk.Setpoints(
            rpm=int(v.rpm),
            speed_kmh=int(abs(v.speed_kmh)),
            fuel_l=v.fuel_l,
            coolant_c=int(v.coolant_c),
            outside_c=int(v.outside_c),
            range_km=int(v.range_km),
            kl15=c.ignition,
            engine=v.running,
            handbrake=c.handbrake,
            reverse=v.gear < 0,
            lights=lights,
            turn_left=left and self._blink_on,
            turn_right=right and self._blink_on,
            gear_pos=pos,
            gear_num=num,
            gear_mode=mode,
            dimmer=c.dimmer,
            pedal_pct=int(c.throttle * 100),
        )

    # -- drawing ------------------------------------------------------------
    def draw(self, dt: float) -> None:
        v = self.vehicle
        c = v.controls

        self._blink_t += dt
        if self._blink_t >= 0.34:
            self._blink_t = 0.0
            self._blink_on = not self._blink_on

        k = min(1.0, dt * 9.0)
        self._n_rpm += (v.rpm - self._n_rpm) * k
        self._n_speed += (abs(v.speed_kmh) - self._n_speed) * k

        s = self.screen
        s.fill(BG)

        self._gauge(s, (238, 236), 150, self._n_rpm, TACH_MAX, 1000, "rpm x1000", 0.001,
                    redline=LIMITER_RPM)
        self._gauge(s, (556, 236), 150, self._n_speed, SPEED_MAX, 20, "km/h", 1.0)

        self._draw_gear(s, 397, 372)
        self._draw_bars(s, 740, 60)
        self._draw_telltales(s, 740, 250)
        self._draw_status(s, 740, 372)
        self._draw_log(s, 24, 470)
        if self.show_help:
            self._draw_help(s)

        pygame.display.flip()

    def _gauge(self, s, centre, radius, value, vmax, step, label, scale, redline=None):
        cx, cy = centre
        start, sweep = math.radians(144.0), math.radians(252.0)
        pygame.draw.circle(s, PANEL, centre, radius)
        pygame.draw.circle(s, EDGE, centre, radius, 2)

        if redline is not None:
            a0 = start + sweep * (redline / vmax)
            pts = [centre]
            steps = 24
            for i in range(steps + 1):
                a = a0 + (start + sweep - a0) * i / steps
                pts.append((cx + math.cos(a) * radius, cy + math.sin(a) * radius))
            pygame.draw.polygon(s, (70, 30, 30), pts)

        ticks = int(vmax / step)
        for i in range(ticks + 1):
            a = start + sweep * (i / ticks)
            inner = radius - (16 if i % 2 == 0 else 9)
            p0 = (cx + math.cos(a) * inner, cy + math.sin(a) * inner)
            p1 = (cx + math.cos(a) * (radius - 3), cy + math.sin(a) * (radius - 3))
            pygame.draw.line(s, TEXT if i % 2 == 0 else DIM, p0, p1, 2)
            if i % 2 == 0:
                txt = self.small.render(f"{i * step * scale:g}", True, DIM)
                tp = (cx + math.cos(a) * (radius - 32), cy + math.sin(a) * (radius - 32))
                s.blit(txt, txt.get_rect(center=tp))

        frac = max(0.0, min(1.0, value / vmax))
        a = start + sweep * frac
        tip = (cx + math.cos(a) * (radius - 20), cy + math.sin(a) * (radius - 20))
        tail = (cx - math.cos(a) * 18, cy - math.sin(a) * 18)
        pygame.draw.line(s, ACCENT, tail, tip, 4)
        pygame.draw.circle(s, NEEDLE, centre, 9)
        pygame.draw.circle(s, EDGE, centre, 9, 2)

        lab = self.small.render(label, True, DIM)
        s.blit(lab, lab.get_rect(center=(cx, cy + 52)))
        val = self.big.render(f"{value:.0f}", True, TEXT)
        s.blit(val, val.get_rect(center=(cx, cy + 86)))

    def _draw_gear(self, s, x, y):
        v = self.vehicle
        c = v.controls
        if c.selector != SEL_D:
            text = v.selector_name
        elif v.gear <= 0:
            text = "D-"
        else:
            prefix = "M" if not c.auto else ("S" if c.sport else "D")
            text = f"{prefix}{v.gear}"
        box = pygame.Rect(x, y, 90, 62)
        pygame.draw.rect(s, PANEL, box, border_radius=8)
        pygame.draw.rect(s, EDGE, box, 2, border_radius=8)
        g = self.big.render(text, True, ACCENT if v.gear != 0 else DIM)
        s.blit(g, g.get_rect(center=box.center))

    def _draw_bars(self, s, x, y):
        v = self.vehicle
        rows = [
            ("fuel", v.fuel_l, 60.0, "l", GOOD if v.fuel_l > 8 else BAD),
            ("coolant", v.coolant_c, 130.0, "C", GOOD if v.coolant_c < 110 else BAD),
            ("range", v.range_km, 1000.0, "km", DIM),
            ("throttle", v.controls.throttle * 100, 100.0, "%", ACCENT),
            ("brake", v.controls.brake * 100, 100.0, "%", BAD),
            ("dimmer", v.controls.dimmer, 255.0, "", WARN),
        ]
        for i, (name, val, vmax, unit, colour) in enumerate(rows):
            ry = y + i * 30
            s.blit(self.small.render(name, True, DIM), (x, ry))
            bar = pygame.Rect(x + 78, ry + 2, 150, 12)
            pygame.draw.rect(s, PANEL, bar, border_radius=3)
            frac = max(0.0, min(1.0, val / vmax))
            if frac > 0:
                fill = pygame.Rect(bar.x, bar.y, int(bar.w * frac), bar.h)
                pygame.draw.rect(s, colour, fill, border_radius=3)
            pygame.draw.rect(s, EDGE, bar, 1, border_radius=3)
            s.blit(self.small.render(f"{val:.0f}{unit}", True, TEXT), (x + 238, ry))

    def _draw_telltales(self, s, x, y):
        v = self.vehicle
        c = v.controls
        left = (c.turn_left or c.hazard) and self._blink_on
        right = (c.turn_right or c.hazard) and self._blink_on
        lamps = [
            ("<", left, GOOD),
            (">", right, GOOD),
            ("P!", c.handbrake, BAD),
            ("BATT", c.ignition and not v.running, BAD),
            ("OIL", c.ignition and not v.running, BAD),
            ("side", c.lights_step >= 1, WARN),
            ("low", c.lights_step >= 2, GOOD),
            ("HIGH", c.lights_step >= 3, (110, 160, 240)),
            ("fogF", c.fog_front, GOOD),
            ("fogR", c.fog_rear, WARN),
        ]
        s.blit(self.small.render("telltales", True, DIM), (x, y - 20))
        for i, (name, on, colour) in enumerate(lamps):
            bx = x + (i % 5) * 56
            by = y + (i // 5) * 34
            box = pygame.Rect(bx, by, 50, 28)
            pygame.draw.rect(s, colour if on else PANEL, box, border_radius=5)
            pygame.draw.rect(s, EDGE, box, 1, border_radius=5)
            col = (20, 20, 24) if on else DIM
            t = self.small.render(name, True, col)
            s.blit(t, t.get_rect(center=box.center))

    def _draw_status(self, s, x, y):
        v = self.vehicle
        c = v.controls
        conn = self.link is not None and self.link.connected
        rows = [
            ("link", f"{self.port_label} {'up' if conn else 'down'}", GOOD if conn else BAD),
            ("frames", str(self.link.frames_sent if self.link else 0), DIM),
            ("audio", self.engine.name, GOOD if self.engine.available else DIM),
            ("terminal 15", "on" if c.ignition else "off", GOOD if c.ignition else DIM),
            ("engine", "running" if v.running else "stopped", GOOD if v.running else BAD),
            ("gearbox", f"{v.selector_name} {'auto' if c.auto else 'manual'}", DIM),
            ("odometer", f"{v.odo_m / 1000.0:.2f} km (sim only)", DIM),
        ]
        for i, (name, val, colour) in enumerate(rows):
            ry = y + i * 22
            s.blit(self.small.render(name, True, DIM), (x, ry))
            s.blit(self.small.render(val, True, colour), (x + 110, ry))
        if self.link is not None and self.link.last_error:
            err = self.small.render(self.link.last_error[:46], True, BAD)
            s.blit(err, (x, y + len(rows) * 22 + 6))

    def _draw_log(self, s, x, y):
        s.blit(self.small.render("firmware says", True, DIM), (x, y - 20))
        box = pygame.Rect(x, y, 660, 160)
        pygame.draw.rect(s, PANEL, box, border_radius=6)
        pygame.draw.rect(s, EDGE, box, 1, border_radius=6)
        for i, ln in enumerate(self.log[-8:]):
            s.blit(self.small.render(ln[:88], True, DIM), (x + 10, y + 8 + i * 18))

    def _draw_help(self, s):
        w, h = 300, 2 + len(HELP) * 20
        box = pygame.Rect(WIDTH - w - 16, HEIGHT - h - 16, w, h)
        panel = pygame.Surface((box.w, box.h), pygame.SRCALPHA)
        panel.fill((20, 22, 28, 235))
        s.blit(panel, box.topleft)
        pygame.draw.rect(s, EDGE, box, 1, border_radius=6)
        for i, (key, what) in enumerate(HELP):
            ry = box.y + 4 + i * 20
            s.blit(self.small.render(key, True, ACCENT), (box.x + 10, ry))
            s.blit(self.small.render(what, True, DIM), (box.x + 112, ry))

    # -- firmware console ---------------------------------------------------
    def pump_log(self) -> None:
        if self.link is None:
            return
        text = self.link.read_text()
        if not text:
            return
        self._log_tail += text
        while "\n" in self._log_tail:
            line, self._log_tail = self._log_tail.split("\n", 1)
            line = line.strip()
            if line:
                self.log.append(line)
        del self.log[:-64]

    def tick(self) -> float:
        return self.clock.tick(FPS) / 1000.0

    def quit(self) -> None:
        pygame.quit()
