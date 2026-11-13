#pragma once

/**
 * @file DemoSweep.h
 * @brief Self-contained demonstration pattern for the dials.
 */

#include <stdint.h>

#include "ClusterCan.h"

/**
 * @brief Sweeps engine speed and road speed while no host is connected.
 *
 * The pattern runs from idle to a maximum and back, pauses at idle and
 * repeats. It never passes below idle: zero engine speed while the ignition
 * is reported as on is read by the cluster as a stall, which it answers with
 * an audible warning and a group of telltales.
 */
class DemoSweep {
public:
    /// Advances the sweep. Returns immediately when inactive or not yet due.
    void update(ClusterCan &cluster);

    void stop();

    /// Restarts the sweep from idle.
    void resume();

    bool isActive() const;

private:
    /// Minimum interval between updates, matching the display refresh.
    static constexpr uint32_t StepIntervalMs = 20;
    static constexpr float DefaultStepSeconds = 0.02f;
    static constexpr int8_t DirectionUp = 1;
    static constexpr int8_t DirectionDown = -1;

    /// Holds the dials at idle between sweeps. Returns true while dwelling.
    bool serviceDwell(ClusterCan &cluster, uint32_t nowMs, float idleRpm);

    /// Derives a road speed from the part of the sweep above idle.
    void applySweptSpeed(ClusterCan &cluster, float idleRpm) const;

    bool active_ = true;
    uint32_t lastUpdateMs_ = 0;
    uint32_t dwellUntilMs_ = 0;
    float engineSpeed_ = 0.0f;
    int8_t direction_ = DirectionUp;
};
