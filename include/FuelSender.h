#pragma once

/**
 * @file FuelSender.h
 * @brief Converts a fuel level into the two sender readings on 0x349.
 */

#include <stdint.h>

#include "Config.h"

/// The tank is a saddle tank with one sender in each half. They report on the
/// same frame but do not share a transfer curve.
enum class FuelSenderSide : uint8_t {
    Left = 0,
    Right = 1
};

/**
 * @brief Models the damping and non-linearity of a vehicle fuel gauge.
 *
 * Two behaviours are reproduced. First, the displayed level follows the
 * setpoint at a limited rate, because a real gauge reads a long average and a
 * step change is interpreted by the cluster as refuelling. Second, the count
 * reported for a given fill fraction follows a measured curve rather than a
 * straight line, since the gauge is deliberately slow to fall from full and
 * quick through the bottom of the tank.
 */
class FuelSender {
public:
    FuelSender() = default;

    /**
     * @brief Enables or disables the damping.
     *
     * The damping models a real tank, which is what the cluster expects from
     * a fuel gauge. It has to be switchable at run time because the gauge is
     * also used to display quantities that change far faster than a fuel
     * level, where a limited rate of change would hide the signal entirely.
     * Disabling it makes the displayed level follow the setpoint at once.
     */
    void setDamping(bool enabled);

    bool damping() const;

    /**
     * @brief Moves the displayed level towards the setpoint.
     *
     * The first call adopts the setpoint immediately so that the gauge does
     * not sweep up from empty at power-on.
     *
     * @param targetLitres Requested level.
     * @param nowMs        Current millisecond count.
     */
    void update(float targetLitres, uint32_t nowMs);

    /// Level currently being transmitted, in litres.
    float displayedLitres(float fallbackLitres) const;

    /// Sender count for a fill fraction between zero and one.
    static uint16_t counts(float fraction, FuelSenderSide side);

private:
    /// Interpolates the measured curve for one sender.
    static uint16_t interpolate(uint16_t permille, FuelSenderSide side);

    /// Negative until the first update, which marks the level as unset.
    float displayedLitres_ = -1.0f;
    uint32_t lastUpdateMs_ = 0;
    bool damping_ = cfg::fuel::EnableSlew;
};
