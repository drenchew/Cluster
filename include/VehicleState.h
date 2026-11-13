#pragma once

/**
 * @file VehicleState.h
 * @brief Setpoints describing what the cluster should display.
 */

#include <stdint.h>

#include "Config.h"

/// Shift lever position, encoded as the cluster expects it in 0x1D2.
enum class GearPosition : uint8_t {
    Off = 0,
    Park = 1,
    Reverse = 2,
    Neutral = 4,
    Drive = 8
};

/// Transmission program shown next to the gear number.
enum class GearMode : uint8_t {
    Normal = 0,
    Sport = 1,
    Manual = 2
};

/// How the 0x1A6 distance accumulator advances.
enum class OdometerMode : uint8_t {
    Frozen = 0,    ///< never advances; used to prove the counter is read
    Alive = 1,     ///< fixed step per frame, independent of speed
    Distance = 2   ///< integrates speed, as a vehicle does
};

/// Turn indicator stalk position. The cluster runs its own flash timer, so
/// this is the requested state rather than the instantaneous lamp state.
enum class TurnSignal : uint8_t {
    None = 0,
    Left = 0x01,
    Right = 0x02,
    Hazard = Left | Right
};

/**
 * @brief Holds every quantity the cluster is driven from.
 *
 * All members are private and reached through accessors so that range limits
 * and interdependent rules are enforced in one place. The most important of
 * those rules is that engine speed cannot fall below idle while the engine is
 * marked as running, because a cluster reading zero with the ignition on
 * treats the engine as stalled and raises a group of warning telltales.
 */
class VehicleState {
public:
    VehicleState();

    void setEngineSpeed(uint16_t rpm);
    void setRoadSpeed(uint16_t kmh);
    void setPedalPercent(uint8_t percent);
    void setFuelLitres(float litres);
    void setCoolantCelsius(int16_t celsius);
    void setOutsideCelsius(int16_t celsius);
    void setRangeKm(uint16_t km);
    void setLightMask(uint8_t mask);
    void setDimmer(uint8_t level);
    void setTurnSignal(bool left, bool right);
    void setHandbrake(bool engaged);
    void setReverse(bool engaged);
    void setGear(GearPosition position, uint8_t number, GearMode mode);
    void setCruiseKmh(uint8_t kmh);

    /// Instantaneous fuel demand, 0 to 100 percent, shown on the
    /// consumption gauge.
    void setConsumptionPercent(uint8_t percent);
    void setWarningMask(uint8_t mask);
    void setOdometerMode(OdometerMode mode);
    void setSpeedSources(uint8_t mask);

    /// Marks the engine as running or stopped. Stopping forces engine speed
    /// to zero, which is how a stall is deliberately simulated.
    void setEngineRunning(bool running);
    void setTerminal15(bool on);

    uint16_t engineSpeed() const;
    uint16_t roadSpeed() const;
    uint8_t pedalPercent() const;
    uint16_t fuelTenthLitres() const;
    float fuelLitres() const;
    int16_t coolantCelsius() const;
    int16_t outsideCelsius() const;
    uint16_t rangeKm() const;
    uint8_t lightMask() const;
    uint8_t dimmer() const;
    uint8_t turnSignalBits() const;
    bool handbrake() const;
    bool reverse() const;
    GearPosition gearPosition() const;
    uint8_t gearNumber() const;
    GearMode gearMode() const;
    uint8_t cruiseKmh() const;
    uint8_t consumptionPercent() const;
    uint8_t warningMask() const;
    OdometerMode odometerMode() const;
    uint8_t speedSources() const;
    bool engineRunning() const;
    bool terminal15() const;

    /// True when the named source is enabled in the speed source mask.
    bool usesSpeedSource(uint8_t sourceBit) const;

private:
    /// Reapplies the idle floor after the engine running flag changes.
    void applyIdleFloor();

    uint16_t engineSpeed_ = 0;
    uint16_t roadSpeed_ = 0;
    uint8_t pedalPercent_ = 0;
    uint16_t fuelTenthLitres_ = 0;
    int16_t coolantCelsius_ = 0;
    int16_t outsideCelsius_ = 0;
    uint16_t rangeKm_ = 0;
    uint8_t lightMask_ = 0;
    uint8_t dimmer_ = 0;
    uint8_t turnSignalBits_ = 0;
    bool handbrake_ = false;
    bool reverse_ = false;
    GearPosition gearPosition_ = GearPosition::Off;
    uint8_t gearNumber_ = 0;
    GearMode gearMode_ = GearMode::Normal;
    uint8_t cruiseKmh_ = 0;
    uint8_t consumptionPercent_ = 0;
    uint8_t warningMask_ = 0;
    OdometerMode odometerMode_ = OdometerMode::Distance;
    uint8_t speedSources_ = cfg::speed::DefaultSources;
    bool engineRunning_ = cfg::limits::EngineRunningAtBoot;
    bool terminal15_ = false;
};
