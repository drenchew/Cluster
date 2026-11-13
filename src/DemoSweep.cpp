#include "DemoSweep.h"

#include <Arduino.h>

namespace {

constexpr float MillisecondsPerSecond = 1000.0f;
constexpr float MinimumSpan = 1.0f;

}  // namespace

void DemoSweep::stop() {
    active_ = false;
}

void DemoSweep::resume() {
    active_ = true;
    lastUpdateMs_ = 0;
    dwellUntilMs_ = 0;
    engineSpeed_ = static_cast<float>(cfg::limits::IdleRpm);
    direction_ = DirectionUp;
}

bool DemoSweep::isActive() const {
    return active_;
}

bool DemoSweep::serviceDwell(ClusterCan &cluster, uint32_t nowMs, float idleRpm) {
    const bool dwelling = (dwellUntilMs_ != 0);
    if (dwelling) {
        cluster.state().setEngineSpeed(static_cast<uint16_t>(idleRpm));
        if (cfg::demo::SweepSpeed) {
            cluster.state().setRoadSpeed(0);
        }
        if (nowMs >= dwellUntilMs_) {
            dwellUntilMs_ = 0;
            engineSpeed_ = idleRpm;
            direction_ = DirectionUp;
        }
    }
    return dwelling;
}

void DemoSweep::applySweptSpeed(ClusterCan &cluster, float idleRpm) const {
    // Road speed follows the part of the sweep above idle, so the speedometer
    // reads zero whenever the engine is idling. The distance accumulator is
    // integrated from this speed, so the odometer stops advancing there too.
    const float span = static_cast<float>(cfg::demo::MaxRpm) - idleRpm;
    float fraction = (span > MinimumSpan) ? ((engineSpeed_ - idleRpm) / span) : 0.0f;
    if (fraction < 0.0f) {
        fraction = 0.0f;
    }
    cluster.state().setRoadSpeed(
        static_cast<uint16_t>(fraction * static_cast<float>(cfg::demo::MaxSpeedKmh)));
}

void DemoSweep::update(ClusterCan &cluster) {
    const uint32_t now = millis();
    const bool ready = active_ && (now >= cfg::demo::StartDelayMs) &&
                       ((now - lastUpdateMs_) >= StepIntervalMs);

    if (ready) {
        const float idleRpm = static_cast<float>(cfg::limits::IdleRpm);
        const float stepSeconds =
            (lastUpdateMs_ == 0) ? DefaultStepSeconds
                                 : (static_cast<float>(now - lastUpdateMs_) /
                                    MillisecondsPerSecond);
        lastUpdateMs_ = now;

        if (!serviceDwell(cluster, now, idleRpm)) {
            if (engineSpeed_ < idleRpm) {
                engineSpeed_ = idleRpm;
            }
            engineSpeed_ += static_cast<float>(direction_) *
                            static_cast<float>(cfg::demo::RpmPerSecond) * stepSeconds;

            if (engineSpeed_ >= static_cast<float>(cfg::demo::MaxRpm)) {
                engineSpeed_ = static_cast<float>(cfg::demo::MaxRpm);
                direction_ = DirectionDown;
            }
            if (engineSpeed_ <= idleRpm) {
                engineSpeed_ = idleRpm;
                dwellUntilMs_ = now + cfg::demo::IdleDwellMs;
            }

            cluster.state().setEngineSpeed(static_cast<uint16_t>(engineSpeed_));
            if (cfg::demo::SweepSpeed) {
                applySweptSpeed(cluster, idleRpm);
            }
        }
    }
}
