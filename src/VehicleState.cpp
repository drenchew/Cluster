#include "VehicleState.h"

namespace {

/// Clamps a value to an inclusive range. Declared locally rather than taken
/// from the Arduino core so that the class can be compiled off target.
template <typename T>
T clampTo(T value, T lowest, T highest) {
    T result = value;
    if (result < lowest) {
        result = lowest;
    } else if (result > highest) {
        result = highest;
    }
    return result;
}

constexpr float TenthsPerUnit = 10.0f;

}  // namespace

VehicleState::VehicleState()
    : fuelTenthLitres_(cfg::defaults::FuelTenthLitres),
      coolantCelsius_(cfg::defaults::CoolantCelsius),
      outsideCelsius_(cfg::defaults::OutsideCelsius),
      rangeKm_(cfg::defaults::RangeKm),
      dimmer_(cfg::defaults::Dimmer),
      gearPosition_(GearPosition::Park),
      terminal15_(cfg::defaults::Terminal15) {
}

void VehicleState::applyIdleFloor() {
    if (engineRunning_ && engineSpeed_ < cfg::limits::IdleRpm) {
        engineSpeed_ = cfg::limits::IdleRpm;
    }
}

void VehicleState::setEngineSpeed(uint16_t rpm) {
    engineSpeed_ = clampTo<uint16_t>(rpm, 0, cfg::limits::MaxRpm);
    applyIdleFloor();
}

void VehicleState::setRoadSpeed(uint16_t kmh) {
    roadSpeed_ = clampTo<uint16_t>(kmh, 0, cfg::limits::MaxSpeedKmh);
}

void VehicleState::setPedalPercent(uint8_t percent) {
    pedalPercent_ = clampTo<uint8_t>(percent, 0, cfg::limits::MaxPedalPercent);
}

void VehicleState::setFuelLitres(float litres) {
    const float bounded =
        clampTo<float>(litres, cfg::limits::MinFuelLitres, cfg::limits::MaxFuelLitres);
    fuelTenthLitres_ = static_cast<uint16_t>(bounded * TenthsPerUnit);
}

void VehicleState::setCoolantCelsius(int16_t celsius) {
    coolantCelsius_ =
        clampTo<int16_t>(celsius, cfg::limits::MinCoolantC, cfg::limits::MaxCoolantC);
}

void VehicleState::setOutsideCelsius(int16_t celsius) {
    outsideCelsius_ =
        clampTo<int16_t>(celsius, cfg::limits::MinOutsideC, cfg::limits::MaxOutsideC);
}

void VehicleState::setRangeKm(uint16_t km) {
    rangeKm_ = clampTo<uint16_t>(km, 0, cfg::limits::MaxRangeKm);
}

void VehicleState::setLightMask(uint8_t mask) {
    lightMask_ = mask;
}

void VehicleState::setDimmer(uint8_t level) {
    dimmer_ = level;
}

void VehicleState::setTurnSignal(bool left, bool right) {
    uint8_t bits = static_cast<uint8_t>(TurnSignal::None);
    if (left) {
        bits |= static_cast<uint8_t>(TurnSignal::Left);
    }
    if (right) {
        bits |= static_cast<uint8_t>(TurnSignal::Right);
    }
    turnSignalBits_ = bits;
}

void VehicleState::setHandbrake(bool engaged) {
    handbrake_ = engaged;
}

void VehicleState::setReverse(bool engaged) {
    reverse_ = engaged;
}

void VehicleState::setGear(GearPosition position, uint8_t number, GearMode mode) {
    gearPosition_ = position;
    gearNumber_ = clampTo<uint8_t>(number, 0, cfg::limits::MaxGearNumber);
    gearMode_ = (mode > GearMode::Manual) ? GearMode::Normal : mode;
}

void VehicleState::setCruiseKmh(uint8_t kmh) {
    cruiseKmh_ = kmh;
}

void VehicleState::setConsumptionPercent(uint8_t percent) {
    consumptionPercent_ = clampTo<uint8_t>(percent, 0, cfg::consumption::MaxPercent);
}

void VehicleState::setWarningMask(uint8_t mask) {
    warningMask_ = mask;
}

void VehicleState::setOdometerMode(OdometerMode mode) {
    odometerMode_ = (mode > OdometerMode::Distance) ? OdometerMode::Alive : mode;
}

void VehicleState::setSpeedSources(uint8_t mask) {
    speedSources_ = mask & cfg::speed::SourceMask;
}

void VehicleState::setEngineRunning(bool running) {
    engineRunning_ = running;
    if (running) {
        applyIdleFloor();
    } else {
        engineSpeed_ = 0;
    }
}

void VehicleState::setTerminal15(bool on) {
    terminal15_ = on;
}

uint16_t VehicleState::engineSpeed() const {
    return engineSpeed_;
}

uint16_t VehicleState::roadSpeed() const {
    return roadSpeed_;
}

uint8_t VehicleState::pedalPercent() const {
    return pedalPercent_;
}

uint16_t VehicleState::fuelTenthLitres() const {
    return fuelTenthLitres_;
}

float VehicleState::fuelLitres() const {
    return static_cast<float>(fuelTenthLitres_) / TenthsPerUnit;
}

int16_t VehicleState::coolantCelsius() const {
    return coolantCelsius_;
}

int16_t VehicleState::outsideCelsius() const {
    return outsideCelsius_;
}

uint16_t VehicleState::rangeKm() const {
    return rangeKm_;
}

uint8_t VehicleState::lightMask() const {
    return lightMask_;
}

uint8_t VehicleState::dimmer() const {
    return dimmer_;
}

uint8_t VehicleState::turnSignalBits() const {
    return turnSignalBits_;
}

bool VehicleState::handbrake() const {
    return handbrake_;
}

bool VehicleState::reverse() const {
    return reverse_;
}

GearPosition VehicleState::gearPosition() const {
    return gearPosition_;
}

uint8_t VehicleState::gearNumber() const {
    return gearNumber_;
}

GearMode VehicleState::gearMode() const {
    return gearMode_;
}

uint8_t VehicleState::cruiseKmh() const {
    return cruiseKmh_;
}

uint8_t VehicleState::consumptionPercent() const {
    return consumptionPercent_;
}

uint8_t VehicleState::warningMask() const {
    return warningMask_;
}

OdometerMode VehicleState::odometerMode() const {
    return odometerMode_;
}

uint8_t VehicleState::speedSources() const {
    return speedSources_;
}

bool VehicleState::engineRunning() const {
    return engineRunning_;
}

bool VehicleState::terminal15() const {
    return terminal15_;
}

bool VehicleState::usesSpeedSource(uint8_t sourceBit) const {
    return (speedSources_ & sourceBit) != 0;
}
