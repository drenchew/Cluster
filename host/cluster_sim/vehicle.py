"""Vehicle model: engine, turbo, gearbox, driveline, chassis.

Tuned to behave like the 2.0 litre diesel the A2C53117692 cluster came behind
(320d / 120d family): ~800 rpm idle, torque plateau from 1750-2750, and a
4600 rpm fuel cut that arrives well before anything interesting happens up top.

What makes a diesel feel like a diesel, and what this models:

* **Turbo lag.** Only about a third of peak torque is available off boost.
  Stab the throttle at 1200 rpm and you get that third, then the rest arrives
  over the next half second as the turbo spools. Below the boost threshold it
  never arrives at all. This is the single biggest contributor to the
  character, and the torque curve alone does not capture it.
* **Overrun fuel cut.** Lift off above idle and fuelling goes to zero, so the
  engine becomes a brake rather than coasting. The audio reads the fuelling
  signal directly, which is why the note hollows out when you lift.
* **Torque interrupt and rev matching.** An upshift cuts fuel and lets the
  engine fall to the next gear's speed; a downshift blips it up to meet the
  road. Shifts are a slew towards a computed target, not a teleport.
* **Traction limit.** Drive force is capped by what the rear tyres can take,
  and the surplus torque spins the engine up instead of the car.
* **Traction control.** Slip past the threshold and DSC pulls fuelling, which
  is why a hard launch on a rear-drive car comes out as a series of surges
  rather than one long wheelspin. Three modes, as the real console switch
  gives: DSC on, DTC (traction mode, slip allowed), DSC off (no intervention).

Everything is SI internally; the GUI and the CAN link convert at the edges.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

# ---------------------------------------------------------------------------
# Engine
# ---------------------------------------------------------------------------
IDLE_RPM = 800.0
REDLINE_RPM = 5700.0
LIMITER_RPM = 5500.0        # fuel cut, so the needle stops short of redline
STALL_RPM = 350.0           # below this the engine actually dies
CRANK_RPM = 300.0           # starter cranking speed
ENGINE_INERTIA = 0.26       # kg m^2, flywheel + rotating assembly

# Torque curve as (rpm, Nm) at full fuelling AND full boost.
# A broad turbocharged plateau that holds power to the 5500 limiter: roughly
# 500 Nm from 2000-4000 and about 330 hp at 5400 (see peak_power_hp below).
TORQUE_CURVE = [
    (0, 200.0),
    (900, 330.0),
    (1400, 450.0),
    (2000, 500.0),
    (4000, 500.0),
    (5000, 470.0),
    (5500, 430.0),
    (5900, 380.0),
    (6400, 0.0),
]

# ---- Turbo ----------------------------------------------------------------
# Fraction of the curve available with no boost at all. A 2.0d makes roughly
# a third of its torque off boost; the rest waits for the turbo.
NA_TORQUE_FRAC = 0.38
BOOST_THRESHOLD_RPM = 1300.0   # nothing below here
BOOST_FULL_RPM = 2400.0        # full spool capacity by here
SPOOL_UP_TAU = 0.32            # seconds, building boost
SPOOL_DOWN_TAU = 0.55          # seconds, bleeding off
# Lifting off at boost dumps it through the wastegate - the audible flutter.
BOV_BOOST_MIN = 0.35           # boost fraction below which the dump is silent

# ---- Overrun pops and bangs ----------------------------------------------
# Lift off hard at revs and unburnt mixture carries into a hot exhaust and
# ignites there. It needs three things to be true, which is why it only
# happens when you actually back off a committed throttle: no fuelling, decent
# revs, and the engine having been loaded a moment ago.
POP_MIN_RPM = 2600.0
POP_FULL_RPM = 4300.0          # crackle is at full strength by here
POP_BOOST_MIN = 0.25           # had to be on boost before the lift
POP_DECAY_S = 0.55             # how long the crackle keeps going

# ---------------------------------------------------------------------------
# Drivetrain
# ---------------------------------------------------------------------------
GEAR_RATIOS = [4.11, 2.32, 1.54, 1.18, 1.00, 0.85]   # 6-speed
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
HANDBRAKE_TORQUE = 1400.0   # rear wheels only, so weaker than the foot brake
ENGINE_BRAKE_COEFF = 0.018  # Nm per rpm when off throttle
FRICTION_COEFF = 0.009      # internal engine drag, Nm per rpm

# Traction. Rear-drive, so only the rear axle's share of the weight holds the
# car down - and that is what limits a launch on boost.
TYRE_MU = 1.05
REAR_WEIGHT_FRAC = 0.52
# A spinning tyre grips less than one that is still hooked up - sliding
# friction is below static. Without this the sim clips drive force at the grip
# limit and a wheelspinning launch accelerates exactly as hard as a clean one,
# which leaves traction control with nothing to be worth. 0.3 puts a fully
# spinning rear axle at about 0.74 mu against 1.05 stuck.
TYRE_SPIN_MU_LOSS = 0.30

# ---- DSC / traction control -----------------------------------------------
# A real E9x has one console button: a press gives DTC (traction mode, which
# lets the car slip for a snow pull-away or a bit of fun), press-and-hold gives
# DSC off. The threshold is on ``wheelspin``, i.e. how far past the tyres'
# grip limit the driveline is asking them to go.
DSC_ON, DSC_DTC, DSC_OFF = 0, 1, 2
DSC_MODE_NAMES = ("DSC", "DTC", "off")
# None = this mode never intervenes.
TC_SLIP_LIMIT = (0.04, 0.30, None)
TC_CUT_RATE = 7.0           # how fast fuelling is pulled, fraction per second
TC_RELEASE_RATE = 2.2       # and handed back, which is slower - no see-sawing
TC_MAX_CUT = 0.85           # DSC never cuts the engine completely
TC_LAMP_HOLD = 0.35         # s the lamp stays up after the cut ends, so a
                            # 50 ms intervention is still visible as a flash

# ---- Fuel -----------------------------------------------------------------
FUEL_TANK_L = 60.0          # E90 320d is 61 l; the cluster curve is scaled to this
FUEL_RESERVE_L = 8.0        # low-fuel warning on a real E9x comes up here
REFUEL_L_PER_S = 0.6        # ~36 l/min, a European diesel pump

# Fuel is burned from the power the engine is actually making, not from pedal
# position: litres/hour = idle draw + crank kW / (energy in a litre x thermal
# efficiency). Diesel holds 9.8 kWh/l and a modern turbodiesel converts about a
# third of that to crank power, which puts a 90 km/h cruise at ~4 l/100km and
# full throttle at ~65 l/h - both about right for this car.
DIESEL_KWH_PER_L = 9.8
ENGINE_EFFICIENCY = 0.33
IDLE_L_PER_HOUR = 0.65

# A correct burn rate drains about 0.1 l a minute at a cruise, which is 0.16%
# of the gauge - real, but on a bench the needle looks frozen. The sim runs at
# the true rate by default now that the tank can be filled and set from the
# keyboard; ``Controls.fuel_fast`` multiplies the *tank* drain by this so the
# gauge visibly moves, and leaves the l/100km readout honest either way.
FUEL_BURN_SCALE = 25.0

# ---------------------------------------------------------------------------
# Gearbox
# ---------------------------------------------------------------------------
SEL_P, SEL_R, SEL_N, SEL_D = 0, 1, 2, 3
SELECTOR_NAMES = ("P", "R", "N", "D")

SHIFT_UP_RPM = 2800.0       # lifted by throttle, see _auto_shift
SHIFT_DOWN_RPM = 1250.0
KICKDOWN_THROTTLE = 0.82    # above this, drop a gear to get on boost
UPSHIFT_TIME = 0.28         # torque interrupt, seconds
DOWNSHIFT_TIME = 0.34       # longer: it has to blip the revs up first
SHIFT_LOCKOUT = 0.55        # no second shift for this long, kills hunting
SPORT_RPM_SCALE = 1.25
# An upshift point above the limiter can never be reached, so the box would
# sit bouncing off the fuel cut instead of changing gear. Always leave room.
SHIFT_UP_CEILING = LIMITER_RPM - 150.0


def torque_at(rpm: float) -> float:
    """Crank torque at full fuelling and full boost, interpolated."""
    if rpm <= TORQUE_CURVE[0][0]:
        return TORQUE_CURVE[0][1]
    for (r0, t0), (r1, t1) in zip(TORQUE_CURVE, TORQUE_CURVE[1:]):
        if rpm <= r1:
            span = r1 - r0
            k = 0.0 if span <= 0 else (rpm - r0) / span
            return t0 + k * (t1 - t0)
    return 0.0


def power_kw(rpm: float) -> float:
    """Crank power at full fuelling and full boost."""
    return torque_at(rpm) * rpm / 9549.0


def power_hp(rpm: float) -> float:
    return power_kw(rpm) / 0.7457


def peak_power_hp() -> tuple[float, float]:
    """(hp, rpm) at the power peak, searched over the usable range."""
    best = (0.0, 0.0)
    r = 500.0
    while r <= REDLINE_RPM:
        p = power_hp(r)
        if p > best[0]:
            best = (p, r)
        r += 10.0
    return best


def spool_capacity(rpm: float) -> float:
    """How much boost the turbo could make at this engine speed, 0..1."""
    if rpm <= BOOST_THRESHOLD_RPM:
        return 0.0
    span = BOOST_FULL_RPM - BOOST_THRESHOLD_RPM
    return min(1.0, (rpm - BOOST_THRESHOLD_RPM) / span)


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
    grade_pct: float = 0.0      # road gradient, percent. +uphill, -downhill
    dsc_mode: int = DSC_ON      # DSC_ON / DSC_DTC / DSC_OFF
    refuel: bool = False        # held while the pump is running
    fuel_fast: bool = False     # drain the tank FUEL_BURN_SCALE times as fast
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
    oil_c: float = 20.0
    fuel_l: float = 45.0
    outside_c: float = 18.0
    odo_m: float = 0.0
    trip_m: float = 0.0

    # ---- live engine state, read by the audio and the dashboard ----------
    boost: float = 0.0          # 0..1 fraction of full boost
    fuelling: float = 0.0       # 0..1 actual injected fuelling after cuts
    wheelspin: float = 0.0      # 0..1 how far past the traction limit we are
    tc_cut: float = 0.0         # 0..1 fuelling DSC is currently taking away
    tc_lamp: float = 0.0        # >0 while the intervention lamp should be lit
    pop: float = 0.0            # 0..1 overrun crackle intensity
    cranking: bool = False
    accel_ms2: float = 0.0
    l_per_100km: float = 0.0

    # ---- one-shot events for the audio engine ---------------------------
    # Each is set true for a single frame; the audio layer consumes them.
    ev_shift: int = 0           # +1 upshift, -1 downshift, 0 none
    ev_bov: float = 0.0         # 0..1 strength of a wastegate dump
    ev_pop: float = 0.0         # 0..1 the bang right at the lift
    ev_start: bool = False
    ev_stall: bool = False

    _shift_timer: float = 0.0
    _shift_total: float = 0.0
    _shift_to: int = 0
    _shift_from: int = 0
    _lockout: float = 0.0
    _shift_shock: float = 0.0
    _prev_boost: float = 0.0
    _fuel_torque: float = 0.0   # Nm the fuelling is producing, for _burn()
    controls: Controls = field(default_factory=Controls)

    # -- derived ------------------------------------------------------------
    @property
    def speed_kmh(self) -> float:
        return self.speed_ms * 3.6

    @property
    def gear_ratio(self) -> float:
        return self._ratio_of(self.gear)

    def _ratio_of(self, gear: int) -> float:
        if gear > 0:
            return GEAR_RATIOS[gear - 1] * FINAL_DRIVE
        if gear < 0:
            return REVERSE_RATIO * FINAL_DRIVE
        return 0.0

    @property
    def shifting(self) -> bool:
        return self._shift_timer > 0.0

    @property
    def shift_progress(self) -> float:
        if self._shift_total <= 0.0:
            return 0.0
        return 1.0 - self._shift_timer / self._shift_total

    @property
    def clutch_engaged(self) -> bool:
        return self.gear != 0 and not self.shifting

    @property
    def load(self) -> float:
        """0..1 engine load, for the audio timbre."""
        if not self.running:
            return 0.0
        return max(self.fuelling, 0.05)

    @property
    def tc_active(self) -> bool:
        """True while DSC is intervening - what flashes the lamp."""
        return self.tc_lamp > 0.0

    @property
    def low_fuel(self) -> bool:
        return self.fuel_l <= FUEL_RESERVE_L

    @property
    def fuel_frac(self) -> float:
        return max(0.0, min(1.0, self.fuel_l / FUEL_TANK_L))

    @property
    def dsc_name(self) -> str:
        return DSC_MODE_NAMES[self.controls.dsc_mode]

    @property
    def range_km(self) -> float:
        if self.l_per_100km > 0.5:
            return self.fuel_l / self.l_per_100km * 100.0
        return self.fuel_l * 18.0

    def target_rpm_for(self, gear: int) -> float:
        """Engine speed this road speed implies in the given gear."""
        ratio = self._ratio_of(gear)
        if ratio == 0.0:
            return IDLE_RPM
        wheel_rpm = (self.speed_ms / WHEEL_RADIUS) * 60.0 / (2.0 * math.pi)
        return abs(wheel_rpm * ratio)

    # -- main step ----------------------------------------------------------
    def step(self, dt: float) -> None:
        c = self.controls
        dt = max(1e-4, min(dt, 0.1))    # a stalled frame must not explode the sim

        self.ev_shift = 0
        self.ev_bov = 0.0
        self.ev_pop = 0.0
        self.ev_start = False
        self.ev_stall = False
        self._prev_boost = self.boost
        was_running = self.running
        v0 = self.speed_ms

        if self._lockout > 0.0:
            self._lockout -= dt
        if self._shift_shock > 0.0:
            self._shift_shock = max(0.0, self._shift_shock - dt * 6.0)

        self._refuel(dt)

        if not c.ignition:
            self.running = False
            self.cranking = False
            self.fuelling = 0.0
            self.rpm = max(0.0, self.rpm - 4000.0 * dt)
            self._bleed_boost(dt)
            self._roll(dt, 0.0)
        else:
            self._update_gear(dt)
            if self.running:
                self._step_running(dt)
            else:
                self._step_cranking(dt)
            self._heat(dt)

        if was_running and not self.running and c.ignition:
            self.ev_stall = True

        self.accel_ms2 = (self.speed_ms - v0) / dt

    # -- engine -------------------------------------------------------------
    def _step_cranking(self, dt: float) -> None:
        c = self.controls
        self.fuelling = 0.0
        self.cranking = bool(c.starter)
        if c.starter and self.fuel_l > 0.0:
            self.rpm += (CRANK_RPM - self.rpm) * min(1.0, 8.0 * dt)
            if self.rpm > CRANK_RPM * 0.8:
                self.running = True
                self.cranking = False
                self.rpm = IDLE_RPM * 0.75
                self.ev_start = True
        else:
            self.rpm = max(0.0, self.rpm - 2500.0 * dt)
        self._bleed_boost(dt)
        self._roll(dt, 0.0)

    def _demand(self) -> float:
        """Driver fuelling demand after the idle governor and the cuts."""
        c = self.controls
        throttle = c.throttle

        # Idle governor: a diesel holds idle by adding fuel as revs sag, which
        # is why the needle does not fall to zero when you lift off in neutral.
        if self.rpm < IDLE_RPM + 150.0:
            sag = (IDLE_RPM + 150.0 - self.rpm) / 400.0
            throttle = max(throttle, min(0.40, sag))

        if self.rpm > LIMITER_RPM:
            return 0.0                      # limiter fuel cut

        # Overrun cut: off the throttle above idle, injection stops outright.
        # That is what turns the engine into a brake, and what hollows out the
        # exhaust note the moment you lift.
        if c.throttle <= 0.02 and self.rpm > IDLE_RPM + 250.0 and self.clutch_engaged:
            return 0.0

        # Fuel starvation.
        if self.fuel_l <= 0.0:
            return 0.0
        return max(0.0, min(1.0, throttle))

    def _update_boost(self, dt: float, demand: float) -> None:
        target = spool_capacity(self.rpm) * demand
        tau = SPOOL_UP_TAU if target > self.boost else SPOOL_DOWN_TAU
        self.boost += (target - self.boost) * min(1.0, dt / tau)
        self.boost = max(0.0, min(1.0, self.boost))

    def _bleed_boost(self, dt: float) -> None:
        self.boost *= max(0.0, 1.0 - dt / SPOOL_DOWN_TAU)
        self.pop *= max(0.0, 1.0 - dt / 0.12)

    def _update_pop(self, dt: float, demand: float, lifted: bool) -> None:
        """Overrun crackle: unburnt mixture igniting in a hot exhaust.

        Only fires when all three conditions hold - fuelling cut, enough revs,
        and the engine having been loaded a moment before. That is what keeps
        it to backing off a committed throttle rather than crackling every
        time you breathe off the pedal at 1500 rpm.
        """
        overrun = demand <= 0.02 and self.rpm > POP_MIN_RPM
        if overrun and self._prev_boost > POP_BOOST_MIN:
            rev_frac = min(1.0, (self.rpm - POP_MIN_RPM) /
                           (POP_FULL_RPM - POP_MIN_RPM))
            strength = min(1.0, self._prev_boost * 1.15) * rev_frac
            if lifted:
                self.ev_pop = strength          # the first, loudest bang
            self.pop = max(self.pop, strength)

        # Fades while the overrun continues, and dies quickly once back on it.
        tau = POP_DECAY_S if overrun else 0.10
        self.pop *= max(0.0, 1.0 - dt / tau)
        if self.pop < 0.002:
            self.pop = 0.0

    def _crank_torque(self, demand: float) -> float:
        """Net torque at the crank, including boost, friction and overrun.

        Boost is an *air limit*, not a multiplier. Fuelling sets the torque;
        how much fuel can actually be burned is capped by the air the turbo is
        supplying (the smoke limit). So full throttle off boost gives
        NA_TORQUE_FRAC of the curve and climbs as it spools, while part
        throttle is not air-limited at all and behaves normally.

        Multiplying demand by the boost fraction instead double-counts: 30%
        throttle at 30% boost came out as 17% of the curve, which left the car
        barely able to accelerate and the gearbox stuck in first.
        """
        air = NA_TORQUE_FRAC + (1.0 - NA_TORQUE_FRAC) * self.boost
        self._fuel_torque = torque_at(self.rpm) * min(demand, air)
        t = self._fuel_torque
        t -= FRICTION_COEFF * self.rpm
        if demand <= 0.02:
            t -= ENGINE_BRAKE_COEFF * self.rpm
        return t

    def _step_running(self, dt: float) -> None:
        c = self.controls
        self.cranking = False

        demand = self._demand()
        # DSC acts on the slip it measured last frame - a real one is reacting
        # to wheel-speed sensors too, not predicting. Pulling fuelling here
        # rather than clipping drive force further down is what a real system
        # does, and it means the audio hears the cut: self.fuelling is what the
        # exhaust note is built from.
        self._update_traction(dt)
        if self.tc_cut > 0.0:
            demand *= 1.0 - self.tc_cut
        self._update_boost(dt, demand)
        # A wastegate dump happens when boost was up and the demand just
        # vanished. This has to be tested *after* the boost update - comparing
        # before it meant comparing boost against itself, so it never fired.
        lifted = (self._prev_boost > BOV_BOOST_MIN and demand <= 0.02
                  and self.boost < self._prev_boost)
        if lifted:
            self.ev_bov = self._prev_boost
        self._update_pop(dt, demand, lifted)
        self.fuelling = demand

        crank_t = self._crank_torque(demand)

        if self.shifting:
            self._slew_through_shift(dt)
            self._roll(dt, 0.0)             # torque interrupt: no drive
        elif self.clutch_engaged:
            self._step_in_gear(dt, crank_t)
        else:
            # Free-revving in neutral: all crank torque spins the flywheel.
            self.rpm += crank_t / ENGINE_INERTIA * 60.0 / (2.0 * math.pi) * dt
            self.wheelspin = 0.0
            self._roll(dt, 0.0)

        self.rpm = max(0.0, min(self.rpm, REDLINE_RPM + 200.0))
        if self.rpm < STALL_RPM:
            self.running = False
            self.fuelling = 0.0

        self._burn(dt, demand)

    def _step_in_gear(self, dt: float, crank_t: float) -> None:
        ratio = self.gear_ratio
        target_rpm = self.target_rpm_for(self.gear)
        slip = self.rpm - target_rpm

        if slip <= LOCKUP_SLIP:
            # Locked: the road sets engine speed and all crank torque goes to
            # the wheels. This is also the overrun case, which is what gives
            # engine braking when you lift off.
            self.rpm += (target_rpm - self.rpm) * min(1.0, 14.0 * dt)
            clutch_t = crank_t
        else:
            # Slipping: the converter passes a fraction of the torque and the
            # engine flares on the rest. Capacity rises with slip, so the two
            # converge and it locks up on its own.
            clutch_t = min(CONVERTER_K * slip * slip, crank_t + 400.0)
            self.rpm += (crank_t - clutch_t) / ENGINE_INERTIA * 60.0 / (2.0 * math.pi) * dt

        drive_torque = clutch_t * ratio * DRIVETRAIN_EFF
        if self.gear < 0:
            drive_torque = -drive_torque

        # Traction limit: the rear axle can only put down so much, and the
        # surplus spins the engine rather than the car.
        grip_f = TYRE_MU * MASS * 9.81 * REAR_WEIGHT_FRAC
        if self.wheelspin > 0.0:
            grip_f *= 1.0 - TYRE_SPIN_MU_LOSS * min(1.0, self.wheelspin)
        drive_f = drive_torque / WHEEL_RADIUS
        if abs(drive_f) > grip_f:
            excess = abs(drive_f) - grip_f
            self.wheelspin = min(1.0, excess / grip_f)
            drive_f = math.copysign(grip_f, drive_f)
            # Unloaded torque accelerates the engine instead.
            self.rpm += (excess * WHEEL_RADIUS / max(ratio, 0.1)) \
                / ENGINE_INERTIA * 60.0 / (2.0 * math.pi) * dt * 0.25
        else:
            self.wheelspin = max(0.0, self.wheelspin - dt * 4.0)

        if self._shift_shock > 0.0:
            drive_f *= 1.0 - 0.45 * self._shift_shock

        self._roll(dt, drive_f * WHEEL_RADIUS)

    def _slew_through_shift(self, dt: float) -> None:
        """Pull the engine towards the speed the incoming gear demands.

        An upshift falls (torque interrupt), a downshift is blipped up to meet
        the road. Either way it is a slew over the shift time, which is what
        makes the tacho needle dip or flare instead of jumping.
        """
        target = self.target_rpm_for(self._shift_to)
        if self._shift_to == 0:
            target = IDLE_RPM
        target = max(target, IDLE_RPM * 0.9)
        k = min(1.0, dt / max(0.04, self._shift_timer))
        self.rpm += (target - self.rpm) * k
        self.wheelspin = 0.0

    # -- DSC ----------------------------------------------------------------
    def _update_traction(self, dt: float) -> None:
        """Ramp the fuelling cut up while the tyres are past their limit.

        A ramp rather than a switch: DSC does not cut 85% of the engine the
        instant a wheel slips, it feeds the cut in and hands it back slowly,
        which is what makes a traction-limited launch surge instead of either
        bogging or spinning. Handing it back slower than it goes on is what
        stops the two fighting each other at the threshold.
        """
        self.tc_lamp = max(0.0, self.tc_lamp - dt)

        limit = TC_SLIP_LIMIT[self.controls.dsc_mode]
        if limit is None or not self.running:
            self.tc_cut = max(0.0, self.tc_cut - dt * TC_RELEASE_RATE)
            return

        if self.wheelspin > limit:
            over = min(1.0, (self.wheelspin - limit) / 0.25)
            self.tc_cut = min(TC_MAX_CUT,
                              self.tc_cut + dt * TC_CUT_RATE * (0.3 + over))
        else:
            self.tc_cut = max(0.0, self.tc_cut - dt * TC_RELEASE_RATE)

        if self.tc_cut > 0.02:
            self.tc_lamp = TC_LAMP_HOLD

    # -- fuel ---------------------------------------------------------------
    def _refuel(self, dt: float) -> None:
        if self.controls.refuel:
            self.fuel_l = min(FUEL_TANK_L, self.fuel_l + REFUEL_L_PER_S * dt)

    def _burn(self, dt: float, demand: float) -> None:
        # Burned from the power actually being made - torque x revs - so the
        # consumption answers to load rather than to pedal travel. Idling, in
        # gear uphill and flat out in sixth all come out where they should.
        power_kw = max(0.0, self._fuel_torque) * self.rpm / 9549.0
        l_per_hour = IDLE_L_PER_HOUR + power_kw / (DIESEL_KWH_PER_L
                                                   * ENGINE_EFFICIENCY)
        # The l/100km readout below always uses the true rate; only the tank
        # level is sped up, and only when asked for (Controls.fuel_fast).
        scale = FUEL_BURN_SCALE if self.controls.fuel_fast else 1.0
        self.fuel_l = max(0.0, self.fuel_l
                          - l_per_hour * scale / 3600.0 * dt)
        kmh = abs(self.speed_kmh)
        inst = (l_per_hour / kmh * 100.0) if kmh > 3.0 else 0.0
        self.l_per_100km += (min(inst, 40.0) - self.l_per_100km) * min(1.0, dt * 0.7)
        if self.fuel_l <= 0.0:
            self.running = False

    # -- chassis ------------------------------------------------------------
    def _roll(self, dt: float, drive_torque: float) -> None:
        c = self.controls
        v = self.speed_ms
        drive_f = drive_torque / WHEEL_RADIUS

        resist = 0.5 * AIR_DENSITY * CD_A * v * v * (1 if v >= 0 else -1)
        resist += ROLL_COEFF * MASS * 9.81 * (1 if v > 0.01 else (-1 if v < -0.01 else 0))
        # Gradient pulls downhill regardless of travel direction.
        resist += MASS * 9.81 * math.sin(math.atan(c.grade_pct / 100.0))

        brake_f = (c.brake * BRAKE_TORQUE
                   + (HANDBRAKE_TORQUE if c.handbrake else 0.0)) / WHEEL_RADIUS

        if abs(v) > 0.05:
            resist += brake_f * (1 if v > 0 else -1)
            self.speed_ms = v + (drive_f - resist) / MASS * dt
            # Brakes must not drag the car backwards through zero.
            if v > 0 > self.speed_ms and drive_f <= 0:
                self.speed_ms = 0.0
            elif v < 0 < self.speed_ms and drive_f >= 0:
                self.speed_ms = 0.0
        else:
            # Standing still: only move if drive beats the static resistances.
            hold = brake_f + ROLL_COEFF * MASS * 9.81
            net_push = abs(drive_f - MASS * 9.81 * math.sin(math.atan(c.grade_pct / 100.0)))
            if net_push <= hold:
                self.speed_ms = 0.0
            else:
                self.speed_ms = v + (drive_f - resist) / MASS * dt

        self.speed_ms = max(-30.0, min(self.speed_ms, 90.0))
        travelled = abs(self.speed_ms) * dt
        self.odo_m += travelled
        self.trip_m += travelled

    # -- gearbox ------------------------------------------------------------
    def _update_gear(self, dt: float) -> None:
        c = self.controls
        if self._shift_timer > 0.0:
            self._shift_timer -= dt
            if self._shift_timer <= 0.0:
                self.gear = self._shift_to
                self._shift_timer = 0.0
                self._shift_shock = 1.0
                self._lockout = SHIFT_LOCKOUT
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

        self._auto_shift()

    def _auto_shift(self) -> None:
        c = self.controls
        if self._lockout > 0.0:
            return

        scale = SPORT_RPM_SCALE if c.sport else 1.0
        # Hold gears longer the harder you are asking: part throttle shifts at
        # 2450, flat out holds past 3700.
        up_rpm = min((SHIFT_UP_RPM + 2200.0 * c.throttle) * scale,
                     SHIFT_UP_CEILING)
        down_rpm = SHIFT_DOWN_RPM * scale

        # Kickdown: a bootful at low revs should drop a gear to get on boost
        # rather than wait for the turbo.
        if (c.throttle > KICKDOWN_THROTTLE and self.gear > 1
                and self.target_rpm_for(self.gear - 1) < up_rpm * 0.95):
            self._begin_shift(self.gear - 1)
            return

        if self.rpm > up_rpm and self.gear < len(GEAR_RATIOS):
            # Never upshift into a gear that would bog below the downshift
            # point - that is what causes shift hunting.
            if self.target_rpm_for(self.gear + 1) > down_rpm * 1.05:
                self._begin_shift(self.gear + 1)
        elif self.rpm < down_rpm and self.gear > 1:
            self._begin_shift(self.gear - 1)

    def _begin_shift(self, to: int) -> None:
        to = max(-1, min(to, len(GEAR_RATIOS)))
        if to == self.gear:
            return
        upshift = to > self.gear and self.gear > 0
        self._shift_from = self.gear
        self._shift_to = to
        base = UPSHIFT_TIME if upshift else DOWNSHIFT_TIME
        if self.controls.sport:
            base *= 0.6
        self._shift_total = base
        self._shift_timer = base
        self.gear = 0           # clutch out while the shift is in progress
        if self._shift_from > 0 and to > 0:
            self.ev_shift = 1 if upshift else -1

    # -- thermal ------------------------------------------------------------
    def _heat(self, dt: float) -> None:
        if self.running:
            target = 88.0 + 7.0 * self.load
            rate = 0.22 + 0.5 * self.load
        else:
            target = self.outside_c
            rate = 0.05
        self.coolant_c += (target - self.coolant_c) * min(1.0, rate * dt)
        # Oil lags the coolant and settles a little below it, which is what the
        # capture shows on 0x1D0 B1.
        self.oil_c += (self.coolant_c - 18.0 - self.oil_c) * min(1.0, 0.12 * dt)

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

    def cycle_dsc(self) -> None:
        """DSC on -> DTC -> DSC off -> on, as the console button would."""
        self.controls.dsc_mode = (self.controls.dsc_mode + 1) % 3

    def reset_trip(self) -> None:
        self.trip_m = 0.0

    def stall(self) -> None:
        self.running = False
        self.rpm = 0.0
        self.fuelling = 0.0
        self.boost = 0.0
