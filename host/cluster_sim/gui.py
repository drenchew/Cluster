"""The cluster window: an E9x-style instrument panel plus a telemetry rail.

Layout mirrors the real thing - tacho left, speedo right, fuel and coolant as
small dials between them, an LCD strip underneath for gear, odometer, trip and
outside temperature, and telltale rows top and bottom. The right-hand rail is
the part a real car does not have: live boost, throttle and brake, the link
state and whatever the firmware is printing, so a disagreement between the sim
and the cluster is visible at a glance.

Needles run through ``NeedleMotion`` rather than chasing their target directly,
so they lag and overshoot the way a stepper-driven needle does.
"""

from __future__ import annotations

import math

import pygame

from . import gauges as g
from . import link as lk
from .vehicle import (GEAR_RATIOS, LIMITER_RPM, REDLINE_RPM, SEL_D, SEL_N,
                      SEL_P, SEL_R, Vehicle)

WIDTH, HEIGHT = 1280, 720
FPS = 60

# Ranges are multiples of their major interval so the numerals come out round.
TACH_MAX = 6000      # major 1000 -> 0 1 2 3 4 5 6, limiter at 5.5
SPEED_MAX = 240      # major 40   -> 0 40 80 120 160 200 240
FUEL_MAX = 60        # major 30   -> 0 30 60
# Coolant spans 50..130 like the real gauge, so a normal 90 C sits dead centre
# instead of three-quarters up an alarming-looking 0..120 scale.
TEMP_MIN, TEMP_MAX = 50, 130

HELP = [
    ("W / Up", "throttle"),
    ("S / Down", "brake"),
    ("Enter", "hold to crank the starter"),
    ("I", "terminal 15 (ignition)"),
    ("K", "kill the engine"),
    ("Space", "handbrake"),
    ("G", "shift lever  P / R / N / D"),
    ("M", "auto / manual gearbox"),
    ("E / Q", "shift up / down (manual)"),
    ("1-6", "select a gear directly"),
    ("P", "sport shift map"),
    ("A / Left", "left indicator"),
    ("D / Right", "right indicator"),
    ("Z", "hazards"),
    ("L", "lights  off / side / low / high"),
    ("F / V", "front / rear fog"),
    ("[ / ]", "dimmer down / up"),
    ("- / =", "fuel down / up"),
    (", / .", "gradient down / up"),
    ("T", "reset trip"),
    ("Tab", "show / hide this help"),
    ("Esc", "quit"),
]


class Dashboard:
    def __init__(self, vehicle: Vehicle, engine, link, port_label: str):
        pygame.init()
        pygame.display.set_caption("E90 cluster bench - simulator")
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        self.clock = pygame.time.Clock()

        mono = "consolas,dejavusansmono,menlo,monospace"
        self.fonts = {
            "numeral": pygame.font.SysFont(mono, 17, bold=True),
            "label": pygame.font.SysFont(mono, 13),
            "readout": pygame.font.SysFont(mono, 20, bold=True),
            "small": pygame.font.SysFont(mono, 13),
            "lcd": pygame.font.SysFont(mono, 19, bold=True),
            "lcd_big": pygame.font.SysFont(mono, 34, bold=True),
            "gear": pygame.font.SysFont(mono, 44, bold=True),
        }

        self.vehicle = vehicle
        self.engine = engine
        self.link = link
        self.port_label = port_label

        self.show_help = False
        self.running = True
        self.log: list[str] = []
        self._log_tail = ""

        # Radii and centres chosen so the small dials clear the big ones: the
        # tacho ends at 393 and the fuel dial starts at 402.
        self.tach = g.Dial(radius=158, vmax=TACH_MAX, major=1000,
                           minor_per_major=4, label="1/min x1000",
                           value_scale=0.001, redline=LIMITER_RPM,
                           fonts=self.fonts)
        self.speedo = g.Dial(radius=158, vmax=SPEED_MAX, major=40,
                             minor_per_major=4, label="km/h",
                             fonts=self.fonts)
        self.fuel = g.Dial(radius=64, vmax=FUEL_MAX, major=30,
                           minor_per_major=3, label="fuel", fonts=self.fonts,
                           start_deg=150.0, sweep_deg=120.0,
                           numerals=False, end_labels=("E", "F"))
        self.temp = g.Dial(radius=64, vmin=TEMP_MIN, vmax=TEMP_MAX, major=40,
                           minor_per_major=2, label="temp", fonts=self.fonts,
                           redline=118, start_deg=270.0, sweep_deg=120.0,
                           numerals=False, end_labels=("C", "H"))

        self.n_rpm = g.NeedleMotion(freq=3.6, damping=0.70)
        self.n_speed = g.NeedleMotion(freq=2.4, damping=0.85)
        self.n_fuel = g.NeedleMotion(freq=0.5, damping=1.0)
        self.n_temp = g.NeedleMotion(freq=0.5, damping=1.0)

        self.bank = g.TelltaleBank()
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
        elif key == pygame.K_t:
            v.reset_trip()
        elif key == pygame.K_LEFTBRACKET:
            c.dimmer = max(0, c.dimmer - 25)
        elif key == pygame.K_RIGHTBRACKET:
            c.dimmer = min(255, c.dimmer + 25)
        elif key in (pygame.K_MINUS, pygame.K_KP_MINUS):
            v.fuel_l = max(0.0, v.fuel_l - 5.0)
        elif key in (pygame.K_EQUALS, pygame.K_PLUS, pygame.K_KP_PLUS):
            v.fuel_l = min(60.0, v.fuel_l + 5.0)
        elif key == pygame.K_COMMA:
            c.grade_pct = max(-20.0, c.grade_pct - 2.0)
        elif key == pygame.K_PERIOD:
            c.grade_pct = min(20.0, c.grade_pct + 2.0)
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

        self._blink_t += dt
        if self._blink_t >= 0.34:
            self._blink_t = 0.0
            self._blink_on = not self._blink_on

        rpm = self.n_rpm.step(v.rpm, dt)
        speed = self.n_speed.step(abs(v.speed_kmh), dt)
        fuel = self.n_fuel.step(v.fuel_l, dt)
        temp = self.n_temp.step(v.coolant_c, dt)

        s = self.screen
        s.fill(g.BG)

        # Dim the whole panel with the cluster dimmer, as the real backlight
        # would - it is a control the user has, so it should visibly do this.
        panel = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)

        self.tach.draw(panel, (232, 292), max(0.0, rpm), f"{v.rpm:4.0f}")
        self.speedo.draw(panel, (708, 292), max(0.0, speed), f"{abs(v.speed_kmh):3.0f}")
        self.fuel.draw(panel, (470, 206), max(0.0, fuel))
        self.temp.draw(panel, (470, 378), max(0.0, temp))

        self._draw_lcd(panel, 350, 460)
        self._draw_telltales(panel)
        self._draw_telltales_lower(panel, 120, 600)

        dim = max(0.35, v.controls.dimmer / 255.0)
        if dim < 0.999:
            panel.fill((int(255 * dim),) * 3 + (255,),
                       special_flags=pygame.BLEND_RGBA_MULT)
        s.blit(panel, (0, 0))

        self._draw_rail(s, 940, 36)
        if self.show_help:
            self._draw_help(s)
        else:
            hint = self.fonts["small"].render("Tab  controls", True, g.LABEL)
            s.blit(hint, (20, HEIGHT - 26))

        pygame.display.flip()

    # -- LCD strip ----------------------------------------------------------
    def _draw_lcd(self, s, x, y):
        v = self.vehicle
        c = v.controls
        box = pygame.Rect(x, y, 240, 118)
        pygame.draw.rect(s, g.LCD_BG, box, border_radius=6)
        pygame.draw.rect(s, (40, 60, 50), box, 1, border_radius=6)

        if c.selector != SEL_D:
            gear_txt = v.selector_name
        elif v.shifting:
            gear_txt = "--"
        elif v.gear <= 0:
            gear_txt = "D"
        else:
            prefix = "M" if not c.auto else ("S" if c.sport else "D")
            gear_txt = f"{prefix}{v.gear}"
        img = self.fonts["gear"].render(gear_txt, True, g.LCD_FG)
        s.blit(img, img.get_rect(center=(box.x + 46, box.y + 36)))

        f = self.fonts["lcd"]
        rows = [
            (f"{v.odo_m / 1000.0:8.1f} km", box.y + 62),
            (f"trip {v.trip_m / 1000.0:5.1f} km", box.y + 84),
        ]
        for txt, ry in rows:
            s.blit(f.render(txt, True, g.LCD_FG), (box.x + 12, ry))

        s.blit(f.render(f"{v.outside_c:4.0f} C", True, g.LCD_FG),
               (box.x + 148, box.y + 14))
        cons = f"{v.l_per_100km:4.1f}" if v.l_per_100km > 0.1 else " --.-"
        s.blit(f.render(f"{cons} l", True, g.LCD_FG), (box.x + 148, box.y + 36))

    # -- telltales ----------------------------------------------------------
    def _draw_telltales(self, s):
        v = self.vehicle
        c = v.controls
        left = (c.turn_left or c.hazard) and self._blink_on
        right = (c.turn_right or c.hazard) and self._blink_on
        # Top row sits in the gap above the two big dials, indicators pushed
        # out to the sides as they are on the real cluster.
        y = 56
        self.bank.draw(s, "turn_l", (352, y), left, g.GOOD)
        self.bank.draw(s, "turn_r", (562, y), right, g.GOOD)
        self.bank.draw(s, "high", (412, y), c.lights_step >= 3, g.BLUE)
        self.bank.draw(s, "low", (452, y), c.lights_step >= 2, g.GOOD)
        self.bank.draw(s, "glow", (492, y),
                       c.ignition and not v.running and v.coolant_c < 40, g.AMBER)

    def _draw_telltales_lower(self, s, x, y):
        v = self.vehicle
        c = v.controls
        key_on_engine_off = c.ignition and not v.running
        lamps = [
            ("park", c.handbrake, g.BAD),
            ("batt", key_on_engine_off, g.BAD),
            ("oil", key_on_engine_off, g.BAD),
            ("abs", not v.running, g.AMBER),
            ("dsc", v.wheelspin > 0.05 or not v.running, g.AMBER),
            ("belt", True, g.BAD),
            ("fog_f", c.fog_front, g.GOOD),
            ("fog_r", c.fog_rear, g.AMBER),
        ]
        for i, (name, on, colour) in enumerate(lamps):
            self.bank.draw(s, name, (x + i * 76, y), bool(on), colour)

    # -- telemetry rail -----------------------------------------------------
    def _draw_rail(self, s, x, y):
        v = self.vehicle
        c = v.controls
        f = self.fonts["small"]
        rail = pygame.Rect(x - 14, y - 14, WIDTH - x, 520)
        pygame.draw.rect(s, (15, 16, 20), rail, border_radius=8)
        pygame.draw.rect(s, (44, 48, 57), rail, 1, border_radius=8)

        s.blit(self.fonts["readout"].render("telemetry", True, g.LABEL), (x, y))
        ry = y + 34
        bars = [
            ("throttle", c.throttle * 100, 100.0, g.NEEDLE, f"{c.throttle*100:3.0f}%"),
            ("brake", c.brake * 100, 100.0, g.BAD, f"{c.brake*100:3.0f}%"),
            ("boost", v.boost * 100, 100.0, g.BLUE, f"{v.boost*100:3.0f}%"),
            ("fuelling", v.fuelling * 100, 100.0, g.WARN, f"{v.fuelling*100:3.0f}%"),
            ("slip", v.wheelspin * 100, 100.0, g.BAD, f"{v.wheelspin*100:3.0f}%"),
            ("fuel", v.fuel_l, 60.0, g.GOOD, f"{v.fuel_l:4.1f}l"),
            ("coolant", v.coolant_c, 130.0,
             g.GOOD if v.coolant_c < 110 else g.BAD, f"{v.coolant_c:3.0f}C"),
            ("dimmer", c.dimmer, 255.0, g.WARN, f"{c.dimmer:3d}"),
        ]
        for label, val, vmax, colour, text in bars:
            g.bar(s, f, x, ry, 150, label, val, vmax, colour, text)
            ry += 24

        ry += 10
        conn = self.link is not None and self.link.connected
        rows = [
            ("link", f"{self.port_label} {'up' if conn else 'down'}",
             g.GOOD if conn else g.BAD),
            ("frames", str(self.link.frames_sent if self.link else 0), g.LABEL),
            ("audio", self.engine.name,
             g.GOOD if self.engine.available else g.LABEL),
            ("terminal 15", "on" if c.ignition else "off",
             g.GOOD if c.ignition else g.LABEL),
            ("engine", ("cranking" if v.cranking else
                        "running" if v.running else "stopped"),
             g.GOOD if v.running else g.BAD),
            ("gearbox", f"{v.selector_name} {'auto' if c.auto else 'man'}"
                        f"{' sport' if c.sport else ''}", g.LABEL),
            ("gradient", f"{c.grade_pct:+.0f}%", g.LABEL),
            ("accel", f"{v.accel_ms2:+5.2f} m/s2", g.LABEL),
        ]
        for label, val, colour in rows:
            s.blit(f.render(label, True, g.LABEL), (x, ry))
            s.blit(f.render(val, True, colour), (x + 108, ry))
            ry += 20

        if self.link is not None and self.link.last_error:
            s.blit(f.render(self.link.last_error[:34], True, g.BAD), (x, ry))
            ry += 20

        ry += 8
        s.blit(f.render("firmware", True, g.LABEL), (x, ry))
        ry += 18
        box = pygame.Rect(x, ry, WIDTH - x - 28, HEIGHT - ry - 24)
        pygame.draw.rect(s, (11, 12, 15), box, border_radius=5)
        pygame.draw.rect(s, (40, 44, 52), box, 1, border_radius=5)
        lines = max(1, box.h // 17)
        for i, ln in enumerate(self.log[-lines:]):
            s.blit(f.render(ln[:38], True, g.LABEL), (box.x + 7, box.y + 5 + i * 17))

    # -- help ---------------------------------------------------------------
    def _draw_help(self, s):
        """Centred modal overlay.

        It used to sit bottom-left, where it covered the tacho and three of
        the telltales - so it is a dimmed-backdrop modal now, off by default.
        """
        f = self.fonts["small"]
        backdrop = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
        backdrop.fill((0, 0, 0, 170))
        s.blit(backdrop, (0, 0))

        cols = 2
        rows = (len(HELP) + cols - 1) // cols
        col_w, row_h = 300, 21
        w = cols * col_w + 36
        h = rows * row_h + 56
        box = pygame.Rect((WIDTH - w) // 2, (HEIGHT - h) // 2, w, h)
        panel = pygame.Surface((box.w, box.h), pygame.SRCALPHA)
        panel.fill((14, 15, 20, 246))
        s.blit(panel, box.topleft)
        pygame.draw.rect(s, (62, 68, 80), box, 1, border_radius=8)

        title = self.fonts["readout"].render("controls", True, g.NUMERAL)
        s.blit(title, (box.x + 18, box.y + 14))
        for i, (key, what) in enumerate(HELP):
            cx = box.x + 18 + (i // rows) * col_w
            ry = box.y + 48 + (i % rows) * row_h
            s.blit(f.render(key, True, g.NEEDLE), (cx, ry))
            s.blit(f.render(what, True, g.LABEL), (cx + 96, ry))

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
