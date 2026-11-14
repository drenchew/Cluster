#include "ClusterCan.h"

#include <Arduino.h>

namespace {

/// Extracts the low byte of a sixteen bit value.
uint8_t lowByte16(uint16_t value) {
    return static_cast<uint8_t>(value & frame::common::ByteMask);
}

/// Extracts the high byte of a sixteen bit value.
uint8_t highByte16(uint16_t value) {
    return static_cast<uint8_t>(value >> frame::common::ByteBits);
}

/// Advances a counter that wraps before the given modulus.
uint8_t nextCounter(uint8_t value, uint8_t modulus) {
    return static_cast<uint8_t>((value + 1) % modulus);
}

}  // namespace

ClusterCan::ClusterCan(ICanBus &bus) : bus_(bus) {
}

bool ClusterCan::begin() {
    const bool ready = bus_.begin();
    if (ready) {
        lastTickMs_ = millis();
    }
    return ready;
}

VehicleState &ClusterCan::state() {
    return state_;
}

const VehicleState &ClusterCan::state() const {
    return state_;
}

const TimeOfDay &ClusterCan::clock() const {
    return clock_;
}

const ClusterFeedback &ClusterCan::feedback() const {
    return feedback_;
}

uint16_t ClusterCan::odometerCounter() const {
    return distanceCounter_;
}

ICanBus::Statistics ClusterCan::statistics() const {
    return bus_.statistics();
}

void ClusterCan::setClock(uint8_t hour, uint8_t minute, uint8_t second, uint8_t day,
                          uint8_t month, uint16_t year) {
    clock_.set(hour, minute, second, day, month, year, millis());
}

void ClusterCan::setTractionLamps(bool tractionMode, bool intervening, bool disabled) {
    checkControl_.setLamp(CheckControl::Lamp::Dtc, tractionMode);
    checkControl_.setLamp(CheckControl::Lamp::DscTriangle, intervening);
    checkControl_.setLamp(CheckControl::Lamp::DscOff, disabled);
}

void ClusterCan::setWarnings(uint8_t mask) {
    state_.setWarningMask(mask);
    checkControl_.applyWarningMask(mask);
}

void ClusterCan::setSymbol(uint16_t identifier, bool on) {
    checkControl_.setCustomSymbol(identifier, on, bus_);
}

void ClusterCan::setCruise(uint8_t kmh) {
    state_.setCruiseKmh(kmh);
}

void ClusterCan::setFuelDamping(bool enabled) {
    fuel_.setDamping(enabled);
}

bool ClusterCan::fuelDamping() const {
    return fuel_.damping();
}

void ClusterCan::setInjectionRateOverride(float countsPer100Ms) {
    injectionRateOverride_ = countsPer100Ms;
}

float ClusterCan::injectionRateOverride() const {
    return injectionRateOverride_;
}

void ClusterCan::setBrakeStatus(uint8_t value) {
    stabilityBrakeStatus_ = value;
}

uint8_t ClusterCan::brakeStatus() const {
    return stabilityBrakeStatus_;
}

void ClusterCan::setOdometerMode(OdometerMode mode) {
    state_.setOdometerMode(mode);
    distanceRemainder_ = 0.0f;
}

void ClusterCan::clearLatchedWarnings() {
    for (uint8_t i = 0; i < cfg::symbol::LatchedAtBootCount; ++i) {
        bus_.send(CheckControl::buildMessage(cfg::symbol::LatchedAtBoot[i], false));
    }
}

void ClusterCan::clearDriverRequests() {
    setTractionLamps(false, false, false);
    setWarnings(0);
    setCruise(0);
    // The local sweep presents a vehicle, so the gauge returns to behaving
    // like one rather than keeping whatever the last host asked for.
    setFuelDamping(cfg::fuel::EnableSlew);
}

// ---------------------------------------------------------------------------
// Schedule
// ---------------------------------------------------------------------------

void ClusterCan::update() {
    const uint32_t now = millis();
    if ((now - lastTickMs_) >= cfg::schedule::TickMs) {
        lastTickMs_ = now;
        ++tickCount_;

        clock_.advanceTo(now);
        if (cfg::send::Fuel) {
            fuel_.update(state_.fuelLitres(), now);
        }

        // Engine speed is read as a direct value, so a faster cadence than
        // the rest of the schedule is accepted and keeps the needle smooth.
        if ((tickCount_ % 2) == 0) {
            sendEngineSpeed();
        }

        serviceSlot(static_cast<uint8_t>(tickCount_ % cfg::schedule::SlotsPerCycle));
        serviceAperiodic(now);
        drainReceiveQueue();
    }
}

void ClusterCan::serviceSlot(uint8_t slot) {
    switch (slot) {
        case cfg::schedule::SlotTerminalA:
        case cfg::schedule::SlotTerminalB:
            sendTerminalStatus();
            break;

        case cfg::schedule::SlotDynamicsSpeedA:
        case cfg::schedule::SlotDynamicsSpeedB:
            if (state_.usesSpeedSource(cfg::speed::SourceDynamics)) {
                sendDynamicsSpeed();
            }
            break;

        case cfg::schedule::SlotLightsA:
        case cfg::schedule::SlotLightsB:
            if (cfg::send::Lights) {
                sendLighting();
            }
            if (cfg::send::DmeStatus) {
                sendDmeStatus();
            }
            break;

        case cfg::schedule::SlotBrakeA:
        case cfg::schedule::SlotBrakeB:
            if (cfg::send::Brake) {
                sendBrake();
            }
            break;

        case cfg::schedule::SlotWheelSpeedA:
        case cfg::schedule::SlotWheelSpeedB:
            if (state_.usesSpeedSource(cfg::speed::SourceWheels)) {
                sendWheelSpeeds();
            }
            break;

        case cfg::schedule::SlotDistanceA:
            if (cfg::send::WarningSuppression) {
                sendStabilityStatus();
            }
            sendDistance();
            break;

        case cfg::schedule::SlotDistanceB:
            if (cfg::send::Dimmer) {
                sendDimmer();
            }
            sendDistance();
            break;

        case cfg::schedule::SlotIgnition:
            sendIgnitionStatus();
            break;

        case cfg::schedule::SlotCoolantA:
        case cfg::schedule::SlotCoolantB:
            if (cfg::send::Coolant) {
                sendEngineTemperature();
            }
            break;

        case cfg::schedule::SlotSuppressionCounters:
            if (cfg::send::WarningSuppression) {
                sendAbsCounter();
                sendAirbagCounter();
            }
            break;

        case cfg::schedule::SlotFuel:
            if (cfg::send::Fuel) {
                sendFuelLevel();
            }
            break;

        case cfg::schedule::SlotTurnSignals:
            if (cfg::send::TurnSignals) {
                sendTurnSignals();
            }
            break;

        case cfg::schedule::SlotGear:
            if (cfg::send::Gear) {
                sendGear();
            }
            break;

        case cfg::schedule::SlotReverseAndLcd:
            if (cfg::send::Reverse) {
                sendReverse();
            }
            if (cfg::send::LcdLight) {
                sendLcdIllumination();
            }
            break;

        default:
            break;
    }
}

void ClusterCan::serviceAperiodic(uint32_t nowMs) {
    // The park brake frame is event driven in a vehicle: it is sent when the
    // state changes, with an occasional repeat while it holds.
    const bool parkBrakeChanged = (state_.handbrake() != lastParkBrakeSent_);
    const bool parkBrakeRepeatDue =
        (tickCount_ % cfg::schedule::TicksPer5Seconds) == cfg::schedule::PhaseParkBrake;
    if (cfg::send::ParkBrake && (parkBrakeChanged || parkBrakeRepeatDue)) {
        lastParkBrakeSent_ = state_.handbrake();
        sendParkBrake();
    }

    if (cfg::send::Range &&
        (tickCount_ % cfg::schedule::TicksPer1Second) == cfg::schedule::PhaseRange) {
        sendTemperatureAndRange();
    }

    if (cfg::send::Cruise &&
        (tickCount_ % cfg::schedule::SlotsPerCycle) == cfg::schedule::PhaseCruise) {
        sendCruise();
    }

    // One symbol frame per 100 ms, so a burst of telltale changes is spread
    // over successive slots rather than filling a single tick.
    if (cfg::send::Symbols &&
        (tickCount_ % cfg::schedule::TicksPer100Ms) == cfg::schedule::PhaseSymbols) {
        checkControl_.service(bus_, nowMs);
    }

    const bool clockSlot =
        (tickCount_ % cfg::schedule::TicksPer100Ms) == cfg::schedule::PhaseClock;
    const bool clockDue = (nowMs - lastClockSendMs_) >= cfg::clock::SendPeriodMs;
    if (cfg::send::Clock && clock_.isValid() && clockSlot && clockDue) {
        lastClockSendMs_ = nowMs;
        sendTimeAndDate();
    }
}

void ClusterCan::drainReceiveQueue() {
    CanMessage message;
    while (bus_.receive(message)) {
        if (cfg::feedback::Enabled) {
            feedback_.consume(message, millis());
        }
    }
}

// ---------------------------------------------------------------------------
// Frame builders
// ---------------------------------------------------------------------------

void ClusterCan::sendTerminalStatus() {
    using namespace frame::terminal;

    const bool on = state_.terminal15();
    uint8_t packed = on ? PackedIgnitionOn : PackedIgnitionOff;
    if (UseRollingCounter && on) {
        packed = static_cast<uint8_t>((terminalCounter_ << frame::common::NibbleBits) |
                                      (packed & frame::common::NibbleMask));
        terminalCounter_ = nextCounter(terminalCounter_, frame::common::CounterModulus);
    }

    CanMessage message(cfg::canid::TerminalStatus, Length);
    message[IndexKeyStatus] = on ? KeyPresent : KeyAbsent;
    message[IndexTransponder] = Transponder;
    message[IndexTerminalStatus] = TerminalStatus;
    message[IndexSteeringLock] = SteeringLock;
    message[IndexPacked] = packed;
    bus_.send(message);
}

void ClusterCan::sendIgnitionStatus() {
    CanMessage message(cfg::canid::IgnitionStatus, frame::ignition::Length);
    for (uint8_t i = 0; i < frame::ignition::Length; ++i) {
        message[i] = frame::ignition::Payload[i];
    }
    bus_.send(message);
}

void ClusterCan::sendDmeStatus() {
    CanMessage message(cfg::canid::DmeStatus, frame::dmeStatus::Length);
    for (uint8_t i = 0; i < frame::dmeStatus::Length; ++i) {
        message[i] = frame::dmeStatus::Payload[i];
    }
    bus_.send(message);
}

void ClusterCan::sendEngineSpeed() {
    using namespace frame::engineSpeed;

    const uint16_t encodedSpeed = static_cast<uint16_t>(state_.engineSpeed() * SpeedScale);
    const uint16_t encodedPedal = static_cast<uint16_t>(state_.pedalPercent() * PedalScale);

    uint8_t flags = FlagsAllSet;
    if (state_.pedalPercent() == 0) {
        flags &= static_cast<uint8_t>(~FlagsPressedMask);
    }

    CanMessage message(cfg::canid::EngineSpeed, Length);
    message[IndexUnknown] = UnknownValue;
    message[IndexCounter] = static_cast<uint8_t>(CounterBase | engineSpeedCounter_);
    message[IndexPedalLow] = lowByte16(encodedPedal);
    message[IndexPedalHigh] = highByte16(encodedPedal);
    message[IndexSpeedLow] = lowByte16(encodedSpeed);
    message[IndexSpeedHigh] = highByte16(encodedSpeed);
    message[IndexFlags] = flags;
    message[IndexTrailer] = TrailerValue;

    engineSpeedCounter_ = nextCounter(engineSpeedCounter_, frame::common::CounterModulus);
    bus_.send(message);
}

void ClusterCan::advanceDistance(uint32_t nowMs) {
    uint32_t elapsedMs = nowMs - lastDistanceMs_;
    lastDistanceMs_ = nowMs;
    if (elapsedMs > cfg::odometer::MaxIntegrationMs) {
        elapsedMs = cfg::odometer::MaxIntegrationMs;
    }

    const uint16_t speed = state_.roadSpeed();
    if (state_.odometerMode() == OdometerMode::Distance && speed > 0) {
        const float seconds =
            static_cast<float>(elapsedMs) / static_cast<float>(cfg::clock::MillisecondsPerSecond);
        distanceRemainder_ +=
            static_cast<float>(speed) * cfg::odometer::CountsPerKmhPerSecond * seconds;
        const uint16_t whole = static_cast<uint16_t>(distanceRemainder_);
        distanceRemainder_ -= static_cast<float>(whole);
        // Allowed to wrap, as the counter does in a vehicle.
        distanceCounter_ = static_cast<uint16_t>(distanceCounter_ + whole);
    } else if (state_.odometerMode() == OdometerMode::Alive) {
        distanceCounter_ = static_cast<uint16_t>(distanceCounter_ + cfg::odometer::AliveStep);
    }

    distanceTimeCounter_ =
        static_cast<uint16_t>(distanceTimeCounter_ + cfg::odometer::TimeCounterStep);
}

void ClusterCan::sendDistance() {
    using namespace frame::distance;

    advanceDistance(millis());

    const uint8_t low = lowByte16(distanceCounter_);
    const uint8_t high = highByte16(distanceCounter_);

    CanMessage message(cfg::canid::DistanceAccumulator, Length);
    for (uint8_t copy = 0; copy < CopyCount; ++copy) {
        const uint8_t base = static_cast<uint8_t>(IndexCopyFirst + copy * 2);
        message[base] = low;
        message[base + 1] = high;
    }
    message[IndexTimeLow] = lowByte16(distanceTimeCounter_);
    message[IndexTimeHigh] = static_cast<uint8_t>(
        TimeHighBase | ((distanceTimeCounter_ >> frame::common::ByteBits) &
                        frame::common::NibbleMask));
    bus_.send(message);
}

void ClusterCan::sendDynamicsSpeed() {
    using namespace frame::dynamics;

    uint16_t counts = static_cast<uint16_t>(static_cast<float>(state_.roadSpeed()) /
                                            cfg::speed::DynamicsKmhPerCount);
    if (counts > cfg::speed::TwelveBitMax) {
        counts = cfg::speed::TwelveBitMax;
    }

    const uint8_t motion = (counts != 0) ? 1 : 0;
    const uint8_t alive = static_cast<uint8_t>(
        ((millis() % AlivePeriodMs) * AliveSteps / AlivePeriodMs) & frame::common::NibbleMask);

    CanMessage message(cfg::canid::DynamicsSpeed, Length);
    message[IndexSpeedLow] = lowByte16(counts);
    message[IndexSpeedHigh] = static_cast<uint8_t>(
        ((counts >> frame::common::ByteBits) & frame::common::NibbleMask) |
        (motion << StatusShift) | AlwaysSetBit);
    message[IndexAliveNibble] = static_cast<uint8_t>(alive << AliveShift);

    uint16_t sum = ChecksumSeed;
    for (uint8_t i = 0; i < ChecksumBytes; ++i) {
        sum = static_cast<uint16_t>(sum + message[i]);
    }
    message[IndexChecksum] =
        static_cast<uint8_t>(sum + (sum >> frame::common::ByteBits) + ChecksumFinalAdd);
    bus_.send(message);
}

void ClusterCan::sendWheelSpeeds() {
    using namespace frame::wheelSpeed;

    const uint16_t counts = static_cast<uint16_t>(static_cast<float>(state_.roadSpeed()) /
                                                  cfg::speed::WheelKmhPerCount);
    CanMessage message(cfg::canid::WheelSpeeds, Length);
    for (uint8_t wheel = 0; wheel < WheelCount; ++wheel) {
        const uint8_t base = static_cast<uint8_t>(wheel * 2);
        message[base] = lowByte16(counts);
        message[base + 1] = highByte16(counts);
    }
    bus_.send(message);
}

uint16_t ClusterCan::brakeSpeedCounts() const {
    uint16_t counts = static_cast<uint16_t>(static_cast<float>(state_.roadSpeed()) /
                                            cfg::speed::BrakeKmhPerCount);
    if (counts > cfg::speed::TwelveBitMax) {
        counts = cfg::speed::TwelveBitMax;
    }
    return counts;
}

void ClusterCan::sendBrake() {
    using namespace frame::brake;

    const uint16_t counts = brakeSpeedCounts();
    const bool engaged = state_.handbrake();

    CanMessage message(cfg::canid::BrakeAndSpeed, Length);
    message[IndexSpeedLow] = lowByte16(counts);
    message[IndexSpeedHigh] = static_cast<uint8_t>(
        SpeedHighFlags | ((counts >> frame::common::ByteBits) & frame::common::NibbleMask));
    message[IndexConstant] = ConstantValue;
    message[IndexCounterHigh] = static_cast<uint8_t>(CounterBase | brakeCounter_);
    message[IndexSpare] = frame::common::Unused;
    message[IndexHandbrake] = engaged ? HandbrakeEngaged : HandbrakeReleased;
    message[IndexHandbrakeMirror] = engaged ? MirrorEngaged : MirrorReleased;
    message[IndexCounter] = brakeCounter_;

    brakeCounter_ = nextCounter(brakeCounter_, frame::common::CounterModulus);
    bus_.send(message);
}

void ClusterCan::advanceInjection(uint32_t nowMs) {
    uint32_t elapsedMs = nowMs - lastInjectionMs_;
    lastInjectionMs_ = nowMs;
    // Clamped for the same reason as the distance accumulator: the first call
    // after power-up would otherwise charge the whole uptime to one frame.
    if (elapsedMs > cfg::odometer::MaxIntegrationMs) {
        elapsedMs = cfg::odometer::MaxIntegrationMs;
    }

    const float intervals = static_cast<float>(elapsedMs) /
                            static_cast<float>(cfg::consumption::ReferenceIntervalMs);
    const float ratePer100Ms =
        (injectionRateOverride_ >= 0.0f)
            ? injectionRateOverride_
            : (static_cast<float>(state_.consumptionPercent()) *
               cfg::consumption::CountsPerPercentPer100Ms);
    injectionRemainder_ += ratePer100Ms * intervals;
    const uint16_t whole = static_cast<uint16_t>(injectionRemainder_);
    injectionRemainder_ -= static_cast<float>(whole);
    // Allowed to wrap, as the counter in a vehicle does.
    injectionCounter_ = static_cast<uint16_t>(injectionCounter_ + whole);
}

void ClusterCan::sendEngineTemperature() {
    using namespace frame::engineTemperature;

    int16_t oil = static_cast<int16_t>(state_.coolantCelsius() - OilBelowCoolant);
    if (oil < -TemperatureOffset) {
        oil = -TemperatureOffset;
    }

    engineTemperatureCounter_ =
        static_cast<uint8_t>((engineTemperatureCounter_ + 1) & frame::common::NibbleMask);
    const uint8_t runState = state_.engineRunning() ? StateRunning : StateStopped;
    advanceInjection(millis());

    CanMessage message(cfg::canid::EngineTemperature, Length);
    message[IndexCoolant] = static_cast<uint8_t>(state_.coolantCelsius() + TemperatureOffset);
    message[IndexOil] = static_cast<uint8_t>(oil + TemperatureOffset);
    message[IndexCounterAndState] =
        static_cast<uint8_t>(engineTemperatureCounter_ | runState);
    message[IndexFillerAfterState] = FillerAfterStateValue;
    message[IndexInjectionLow] = lowByte16(injectionCounter_);
    message[IndexInjectionHigh] = highByte16(injectionCounter_);
    message[IndexDistinctFiller] = DistinctFillerValue;
    message[IndexFillerLast] = FillerLastValue;
    bus_.send(message);
}

void ClusterCan::sendStabilityStatus() {
    using namespace frame::stability;

    CanMessage message(cfg::canid::StabilityStatus, Length);
    message[1] = Payload1;
    message[IndexRotating] = static_cast<uint8_t>(
        ((stabilityCounter_ % RotatingModulus) << frame::common::NibbleBits) |
        RotatingLowNibble);
    message[3] = Payload3;
    message[4] = Payload4;
    message[IndexBrakeStatus] = stabilityBrakeStatus_;
    message[IndexCounter] = stabilityCounter_;

    ++stabilityCounter_;
    bus_.send(message);
}

void ClusterCan::sendParkBrake() {
    CanMessage message(cfg::canid::ParkBrakeStatus, frame::parkBrake::Length);
    message[0] = state_.handbrake() ? frame::parkBrake::Engaged : frame::parkBrake::Released;
    message[1] = frame::common::Filler;
    bus_.send(message);
}

void ClusterCan::sendAbsCounter() {
    CanMessage message(cfg::canid::AbsCounter, frame::absCounter::Length);
    message[0] = absCounter_;
    message[1] = frame::common::Filler;

    absCounter_ = (absCounter_ == frame::absCounter::Last)
                      ? frame::absCounter::First
                      : static_cast<uint8_t>(absCounter_ + 1);
    bus_.send(message);
}

void ClusterCan::sendAirbagCounter() {
    CanMessage message(cfg::canid::AirbagCounter, frame::airbagCounter::Length);
    message[0] = airbagCounter_;
    message[1] = frame::common::Filler;

    ++airbagCounter_;
    bus_.send(message);
}

void ClusterCan::sendFuelLevel() {
    using namespace frame::fuelLevel;

    const float litres = fuel_.displayedLitres(state_.fuelLitres());
    const float fraction = litres / cfg::fuel::TankLitres;
    const uint16_t left = FuelSender::counts(fraction, FuelSenderSide::Left);
    const uint16_t right = FuelSender::counts(fraction, FuelSenderSide::Right);

    CanMessage message(cfg::canid::FuelLevel, Length);
    message[IndexLeftLow] = lowByte16(left);
    message[IndexLeftHigh] = highByte16(left);
    message[IndexRightLow] = lowByte16(right);
    message[IndexRightHigh] = highByte16(right);
    message[IndexSpare] = frame::common::Unused;
    bus_.send(message);
}

void ClusterCan::sendTimeAndDate() {
    using namespace frame::timeAndDate;

    CanMessage message(cfg::canid::TimeAndDate, Length);
    message[IndexHour] = clock_.hour();
    message[IndexMinute] = clock_.minute();
    message[IndexSecond] = clock_.second();
    message[IndexDay] = clock_.day();
    message[IndexMonth] =
        static_cast<uint8_t>((clock_.month() << MonthShift) | MonthLowNibble);
    message[IndexYearLow] = lowByte16(clock_.year());
    message[IndexYearHigh] = highByte16(clock_.year());
    message[IndexTrailer] = TrailerValue;
    bus_.send(message);
}

void ClusterCan::sendLighting() {
    using namespace frame::lighting;

    const uint8_t mask = state_.lightMask();
    CanMessage message(cfg::canid::LightingStatus, Length);
    message[IndexMask] = mask;
    message[IndexSecondary] = (mask != 0) ? SecondaryOn : SecondaryOff;
    message[IndexSpare] = frame::common::Unused;
    bus_.send(message);
}

void ClusterCan::sendDimmer() {
    CanMessage message(cfg::canid::DimmerLevel, frame::dimmer::Length);
    message[0] = state_.dimmer();
    message[1] = frame::common::Unused;
    bus_.send(message);
}

void ClusterCan::sendLcdIllumination() {
    CanMessage message(cfg::canid::LcdIllumination, frame::lcdLight::Length);
    message[0] = state_.dimmer();
    message[1] = frame::common::Unused;
    message[2] = frame::common::Unused;
    bus_.send(message);
}

void ClusterCan::sendTurnSignals() {
    using namespace frame::turnSignals;

    const uint8_t bits = state_.turnSignalBits();
    uint8_t encoded = Base;
    if (bits != 0) {
        encoded |= ActiveBit;
        if ((bits & static_cast<uint8_t>(TurnSignal::Left)) != 0) {
            encoded |= LeftBit;
        }
        if ((bits & static_cast<uint8_t>(TurnSignal::Right)) != 0) {
            encoded |= RightBit;
        }
    }

    CanMessage message(cfg::canid::TurnSignals, Length);
    message[IndexState] = encoded;
    message[IndexConstant] = ConstantValue;
    bus_.send(message);
}

void ClusterCan::sendCruise() {
    using namespace frame::cruise;

    const bool active = state_.cruiseKmh() > 0;
    cruiseTimer_ = static_cast<uint8_t>(cruiseTimer_ + TimerStep);

    uint8_t setSpeed = SpeedWhenOff;
    if (active) {
        setSpeed = (state_.cruiseKmh() > cfg::limits::MaxCruiseKmh)
                       ? cfg::limits::MaxCruiseKmh
                       : state_.cruiseKmh();
    }
    const uint8_t changed = (setSpeed != lastCruiseSpeed_) ? ChangedFlag : ChangedNone;
    lastCruiseSpeed_ = setSpeed;

    CanMessage message(cfg::canid::CruiseControl, Length);
    message[IndexTimer] = cruiseTimer_;
    message[IndexSetSpeed] = setSpeed;
    message[IndexStatus] = active ? StatusActiveKmh : StatusOff;
    message[IndexAssistFlags] = frame::common::Unused;
    message[IndexAssistExtra] = frame::common::Unused;
    message[IndexEnableFlag] = active ? EnableActive : EnableOff;
    message[IndexChanged] = changed;
    message[IndexSpare] = frame::common::Unused;
    bus_.send(message);
}

void ClusterCan::sendGear() {
    using namespace frame::gear;

    const uint8_t position =
        static_cast<uint8_t>(state_.gearPosition()) & frame::common::NibbleMask;

    CanMessage message(cfg::canid::GearDisplay, Length);
    message[IndexPosition] = static_cast<uint8_t>(
        ((~position & frame::common::NibbleMask) << frame::common::NibbleBits) | position);
    message[IndexNumber] =
        static_cast<uint8_t>((state_.gearNumber() << NumberShift) | NumberLowNibble);
    message[IndexFiller] = frame::common::Filler;
    message[IndexCounterAndSport] = static_cast<uint8_t>(
        (gearCounter_ << CounterShift) |
        ((state_.gearMode() == GearMode::Sport) ? SportBit : NoSport));
    message[IndexMode] = static_cast<uint8_t>(state_.gearMode());
    for (uint8_t i = IndexFillerFirst; i < Length; ++i) {
        message[i] = frame::common::Filler;
    }

    gearCounter_ = nextCounter(gearCounter_, frame::common::CounterModulus);
    bus_.send(message);
}

void ClusterCan::sendReverse() {
    CanMessage message(cfg::canid::ReverseGear, frame::reverse::Length);
    message[0] = state_.reverse() ? frame::reverse::Engaged : frame::reverse::Released;
    message[1] = frame::common::Filler;
    bus_.send(message);
}

void ClusterCan::sendTemperatureAndRange() {
    using namespace frame::temperatureAndRange;

    const uint16_t miles = static_cast<uint16_t>(
        static_cast<uint32_t>(state_.rangeKm()) * MetresPerKilometre / MetresPerMile);
    const uint16_t encodedRange = static_cast<uint16_t>(miles * RangeScale);

    CanMessage message(cfg::canid::TemperatureAndRange, Length);
    message[IndexTemperature] = static_cast<uint8_t>(
        (state_.outsideCelsius() + TemperatureOffset) * TemperatureScale);
    message[IndexRangeLow] = lowByte16(encodedRange);
    message[IndexRangeHigh] = highByte16(encodedRange);
    message[IndexSpare] = frame::common::Unused;
    bus_.send(message);
}
