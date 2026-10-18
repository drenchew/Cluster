"""Vehicle model: engine, gearbox, chassis.

Tuned to behave like the 2.0 litre diesel the A2C53117692 cluster came behind
(320d / 120d family): ~800 rpm idle, torque plateau from 1750-2750, and a
4800 rpm limiter that arrives well before anything interesting happens up top.

Everything is in SI internally; the GUI and link convert at the edges.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

# ---------------------------------------------------------------------------
# Engine
# ---------------------------------------------------------------------------
IDLE_RPM = 800.0
REDLINE_RPM = 4800.0
LIMITER_RPM = 4600.0        # fuel cut, so the needle stops short of redline
STALL_RPM = 350.0           # below this the engine actually dies
CRANK_RPM = 300.0           # starter cranking speed
ENGINE_INERTIA = 0.26       # kg m^2, flywheel + rotating assembly

# Torque curve as (rpm, Nm at full fuelling). A diesel plateau, not a peak.
TORQUE_CURVE = [
    (0, 120.0),
    (800, 210.0),
    (1250, 290.0),
    (1750, 340.0),
    (2750, 340.0),
    (3500, 300.0),
    (4200, 230.0),
    (4800, 150.0),
    (5200, 0.0),
]

# ---------------------------------------------------------------------------
# Drivetrain
# ---------------------------------------------------------------------------
GEAR_RATIOS = [4.11, 2.32, 1.54, 1.18, 1.00, 0.85]   # 6-speed manual
REVERSE_RATIO = 3.85
FINAL_DRIVE = 2.47
DRIVETRAIN_EFF = 0.90
WHEEL_RADIUS = 0.316        # 225/45 R17, metres

# Clutch / torque converter. Without a slipping phase, selecting a gear at a
# standstill locks the engine to stationary wheels and stalls it instantly -
# so the car could never move off. A real converter absorbs torque roughly in
# proportion to the square of the slip speed, which is also what lets an
# automatic creep at idle instead of stalling.
CONVERTER_K = 7.5e-5        # Nm per (rpm of slip)^2
LOCKUP_SLIP = 120.0         # rpm; below this the clutch is treated as locked

# ---------------------------------------------------------------------------
# Chassis
# ---------------------------------------------------------------------------
MASS = 1500.0               # kg
CD_A = 0.68                 # drag coefficient * frontal area, m^2
AIR_DENSITY = 1.2
ROLL_COEFF = 0.013
BRAKE_TORQUE = 2600.0       # Nm at the wheels, full pedal
ENGINE_BRAKE_COEFF = 0.018  # Nm per rpm when off throttle
FRICTION_COEFF = 0.009      # internal engine drag, Nm per rpm

# Shift lever positions
SEL_P, SEL_R, SEL_N, SEL_D = 0, 1, 2, 3
SELECTOR_NAMES = ("P", "R", "N", "D")

# Automatic shift points
SHIFT_UP_RPM = 2600.0
SHIFT_DOWN_RPM = 1300.0
SHIFT_TIME = 0.35           # seconds of clutch-out per shift


def torque_at(rpm: float) -> float:
    """Full-fuelling crank torque, linearly interpolated from TORQUE_CURVE."""
    if rpm <= TORQUE_CURVE[0][0]:
        return TORQUE_CURVE[0][1]
    for (r0, t0), (r1, t1) in zip(TORQUE_CURVE, TORQUE_CURVE[1:]):
        if rpm <= r1:
            span = r1 - r0
            k = 0.0 if span <= 0 else (rpm - r0) / span
            return t0 + k * (t1 - t0)
    return 0.0


@dataclass
class Controls:
    throttle: float = 0.0       # 0..1
    brake: float = 0.0          # 0..1
    handbrake: bool = False
    ignition: bool = True       # terminal 15
    starter: bool = False       # held while cranking
    # Shift lever: 0 P, 1 R, 2 N, 3 D. Defaults to P with the handbrake
    # released so a freshly started bench shows the park-brake telltale dark
    # and the car sits still; press G for D (or Space to pull the handbrake).
    selector: int = SEL_P
    gear: int = 1               # commanded gear in manual mode, 1..6
    auto: bool = True
    sport: bool = False
    lights_step: int = 1        # 0 off, 1 side, 2 low beam, 3 + high beam
    fog_front: bool = False
    fog_rear: bool = False
    turn_left: bool = False
    turn_right: bool = False
    hazard: bool = False
    dimmer: int = 200


@dataclass
class Vehicle:
    rpm: float = 0.0
    speed_ms: float = 0.0
    running: bool = False
    gear: int = 0
    coolant_c: float = 20.0
    fuel_l: float = 45.0
    outside_c: float = 18.0
    odo_m: float = 0.0
    _shift_timer: float = 0.0
    _shift_to: int = 0
    controls: Controls = field(default_factory=Controls)

    # -- derived ------------------------------------------------------------
    @property
    def speed_kmh(self) -> float:
        return self.speed_ms * 3.6

    @property
    def gear_ratio(self) -> float:
        if self.gear > 0:
            return GEAR_RATIOS[self.gear - 1] * FINAL_DRIVE
        if self.gear < 0:
            return REVERSE_RATIO * FINAL_DRIVE
        return 0.0

    @property
    def clutch_engaged(self) -> bool:
        return self.gear != 0 and self._shift_timer <= 0.0

    @property
    def load(self) -> float:
        """0..1 engine load, for the audio timbre."""
        if not self.running:
            return 0.0
        return max(self.controls.throttle, 0.12 if self.rpm > IDLE_RPM * 1.1 else 0.04)

    @property
    def range_km(self) -> float:
        return self.fuel_l * 18.0       # ~5.5 l/100 km

    # -- main step ----------------------------------------------------------
    def step(self, dt: float) -> None:
        c = self.controls
        dt = max(1e-4, min(dt, 0.1))    # a stalled frame must not explode the sim

        if not c.ignition:
            self.running = False
            self.rpm = max(0.0, self.rpm - 4000.0 * dt)
            self._roll(dt, drive_torque=0.0)
            return

        self._update_gear(dt)

        if self.running:
            self._step_running(dt)
        else:
            self._step_cranking(dt)

        self._heat(dt)

    # -- engine -------------------------------------------------------------
    def _step_cranking(self, dt: float) -> None:
        c = self.controls
        if c.starter:
            self.rpm += (CRANK_RPM - self.rpm) * min(1.0, 8.0 * dt)
            # Catches once it is spinning and there is any fuelling at all.
            if self.rpm > CRANK_RPM * 0.8:
                self.running = True
                self.rpm = IDLE_RPM * 0.75
        else:
            self.rpm = max(0.0, self.rpm - 2500.0 * dt)
        self._roll(dt, drive_torque=0.0)

    def _step_running(self, dt: float) -> None:
        c = self.controls

        # Idle governor: a diesel holds idle by adding fuel as revs sag, which
        # is why the needle does not fall to zero when you lift off in neutral.
        throttle = c.throttle
        if self.rpm < IDLE_RPM + 150.0:
            sag = (IDLE_RPM + 150.0 - self.rpm) / 400.0
            throttle = max(throttle, min(0.35, sag))
        if self.rpm > LIMITER_RPM:
            throttle = 0.0              # fuel cut

        crank_t = torque_at(self.rpm) * throttle
        crank_t -= FRICTION_COEFF * self.rpm
        if throttle <= 0.02:
            crank_t -= ENGINE_BRAKE_COEFF * self.rpm

        if self.clutch_engaged:
            ratio = self.gear_ratio
            wheel_rpm = (self.speed_ms / WHEEL_RADIUS) * 60.0 / (2.0 * math.pi)
            target_rpm = abs(wheel_rpm * ratio)
            slip = self.rpm - target_rpm

            if slip <= LOCKUP_SLIP:
                # Locked: the road sets engine speed and all crank torque goes
                # to the wheels. This is also the overrun case, which is what
                # gives engine braking when you lift off.
                self.rpm += (target_rpm - self.rpm) * min(1.0, 14.0 * dt)
                clutch_t = crank_t
            else:
                # Slipping: the converter passes a fraction of the torque and
                # the engine flares on the rest. Capacity rises with slip, so
                # the two converge and it locks up on its own.
                clutch_t = min(CONVERTER_K * slip * slip, crank_t + 400.0)
                net = crank_t - clutch_t
                self.rpm += net / ENGINE_INERTIA * 60.0 / (2.0 * math.pi) * dt

            drive_torque = clutch_t * ratio * DRIVETRAIN_EFF
            if self.gear < 0:
                drive_torque = -drive_torque
            self._roll(dt, drive_torque)
        else:
            # Free-revving: all crank torque goes into the flywheel.
            accel = crank_t / ENGINE_INERTIA            # rad/s^2
            self.rpm += accel * 60.0 / (2.0 * math.pi) * dt
            self._roll(dt, drive_torque=0.0)

        self.rpm = max(0.0, min(self.rpm, REDLINE_RPM + 200.0))
        if self.rpm < STALL_RPM:
            self.running = False

        # Fuel burn: idle draw plus a term that grows with torque and revs.
        burn = (0.35 + 2.4 * max(0.0, throttle) * (self.rpm / 2000.0)) / 3600.0
        self.fuel_l = max(0.0, self.fuel_l - burn * dt)
        if self.fuel_l <= 0.0:
            self.running = False

    # -- chassis ------------------------------------------------------------
    def _roll(self, dt: float, drive_torque: float) -> None:
        c = self.controls
        v = self.speed_ms
        drive_f = drive_torque / WHEEL_RADIUS

        resist = 0.5 * AIR_DENSITY * CD_A * v * v * (1 if v >= 0 else -1)
        resist += ROLL_COEFF * MASS * 9.81 * (1 if v > 0.01 else (-1 if v < -0.01 else 0))

        brake_f = (c.brake * BRAKE_TORQUE + (BRAKE_TORQUE if c.handbrake else 0.0)) / WHEEL_RADIUS
        if abs(v) > 0.05:
            resist += brake_f * (1 if v > 0 else -1)
            self.speed_ms = v + (drive_f - resist) / MASS * dt
            # Brakes must not drag the car backwards through zero.
            if v > 0 > self.speed_ms and drive_f <= 0:
                self.speed_ms = 0.0
            elif v < 0 < self.speed_ms and drive_f >= 0:
                self.speed_ms = 0.0
        else:
            # Standing still: only move if drive beats static brake/rolling.
            net = abs(drive_f) - brake_f - ROLL_COEFF * MASS * 9.81
            self.speed_ms = 0.0 if net <= 0 else v + (drive_f - resist) / MASS * dt

        self.speed_ms = max(-30.0, min(self.speed_ms, 90.0))
        self.odo_m += abs(self.speed_ms) * dt

    # -- gearbox ------------------------------------------------------------
    def _update_gear(self, dt: float) -> None:
        c = self.controls
        if self._shift_timer > 0.0:
            self._shift_timer -= dt
            if self._shift_timer <= 0.0:
                self.gear = self._shift_to
            return

        # P and N are neutral as far as the driveline is concerned; the
        # difference only shows up on the cluster's gear display.
        if c.selector in (SEL_P, SEL_N):
            if self.gear != 0:
                self._begin_shift(0)
            return

        if c.selector == SEL_R:
            # Do not slam it into reverse while rolling forwards.
            if self.gear != -1 and self.speed_ms < 1.0:
                self._begin_shift(-1)
            return

        # Selector is in D.
        if not c.auto:
            target = max(1, min(c.gear, len(GEAR_RATIOS)))
            if target != self.gear:
                self._begin_shift(target)
            return

        if self.gear <= 0:
            if self.speed_ms >= -0.5:
                self._begin_shift(1)
            return

        up_rpm = SHIFT_UP_RPM * (1.5 if c.sport else 1.0) + 900.0 * c.throttle
        if self.rpm > up_rpm and self.gear < len(GEAR_RATIOS):
            self._begin_shift(self.gear + 1)
        elif self.rpm < SHIFT_DOWN_RPM and self.gear > 1:
            self._begin_shift(self.gear - 1)

    def _begin_shift(self, to: int) -> None:
        to = max(-1, min(to, len(GEAR_RATIOS)))
        if to == self.gear:
            return
        self._shift_to = to
        self._shift_timer = SHIFT_TIME
        self.gear = 0           # clutch out while the shift is in progress

    # -- thermal ------------------------------------------------------------
    def _heat(self, dt: float) -> None:
        if self.running:
            target = 88.0 + 6.0 * self.load
            rate = 0.22 + 0.5 * self.load
        else:
            target = self.outside_c
            rate = 0.05
        self.coolant_c += (target - self.coolant_c) * min(1.0, rate * dt)

    # -- actions ------------------------------------------------------------
    def shift_up(self) -> None:
        if not self.controls.auto:
            self.controls.gear = min(self.controls.gear + 1, len(GEAR_RATIOS))

    def shift_down(self) -> None:
        if not self.controls.auto:
            self.controls.gear = max(self.controls.gear - 1, 1)

    def cycle_selector(self) -> None:
        """P -> R -> N -> D -> P, as the lever would go."""
        self.controls.selector = (self.controls.selector + 1) % 4

    @property
    def selector_name(self) -> str:
        return SELECTOR_NAMES[self.controls.selector]

    def stall(self) -> None:
        self.running = False
        self.rpm = 0.0
