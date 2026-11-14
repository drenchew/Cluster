#include "ClusterFeedback.h"

namespace {

constexpr uint8_t TemperatureMinBytes = 1;
constexpr uint8_t ClockMinBytes = 3;
constexpr uint8_t IndexTemperature = 0;
constexpr uint8_t IndexHour = 0;
constexpr uint8_t IndexMinute = 1;
constexpr uint8_t IndexSecond = 2;

}  // namespace

void ClusterFeedback::track(uint16_t identifier) {
    bool found = false;
    for (uint8_t i = 0; i < seenUsed_ && !found; ++i) {
        if (seenId_[i] == identifier) {
            ++seenCount_[i];
            found = true;
        }
    }
    if (!found && seenUsed_ < TrackedIdentifiers) {
        seenId_[seenUsed_] = identifier;
        seenCount_[seenUsed_] = 1;
        ++seenUsed_;
    }
}

uint8_t ClusterFeedback::distinctCount() const {
    return seenUsed_;
}

uint16_t ClusterFeedback::identifierAt(uint8_t index) const {
    return (index < seenUsed_) ? seenId_[index] : 0;
}

uint16_t ClusterFeedback::countAt(uint8_t index) const {
    return (index < seenUsed_) ? seenCount_[index] : 0;
}

bool ClusterFeedback::consume(const CanMessage &message, uint32_t nowMs) {
    bool recognised = false;
    track(message.identifier());

    if (message.identifier() == cfg::canid::ClusterTemperature &&
        message.length() >= TemperatureMinBytes) {
        temperatureRaw_ = message[IndexTemperature];
        // Half degree steps offset by -40 degrees, kept in tenths because the
        // AVR print implementation has no floating point conversion.
        temperatureDeciC_ =
            static_cast<int16_t>(temperatureRaw_) * cfg::feedback::TemperatureStepDeciC +
            cfg::feedback::TemperatureOffsetDeciC;
        temperatureSeen_ = true;
        temperatureMs_ = nowMs;
        recognised = true;
    } else if (message.identifier() == cfg::canid::ClusterClock &&
               message.length() >= ClockMinBytes) {
        clockHour_ = message[IndexHour];
        clockMinute_ = message[IndexMinute];
        clockSecond_ = message[IndexSecond];
        clockSeen_ = true;
        clockMs_ = nowMs;
        recognised = true;
    }

    return recognised;
}

bool ClusterFeedback::temperatureFresh(uint32_t nowMs) const {
    return temperatureSeen_ && ((nowMs - temperatureMs_) < cfg::feedback::FreshnessMs);
}

bool ClusterFeedback::temperatureValid() const {
    return temperatureRaw_ != cfg::feedback::TemperatureUnavailable;
}

int16_t ClusterFeedback::temperatureDeciCelsius() const {
    return temperatureDeciC_;
}

bool ClusterFeedback::clockFresh(uint32_t nowMs) const {
    return clockSeen_ && ((nowMs - clockMs_) < cfg::feedback::FreshnessMs);
}

bool ClusterFeedback::clockValid() const {
    return (clockHour_ < cfg::clock::HoursPerDay) &&
           (clockMinute_ < cfg::clock::MinutesPerHour) &&
           (clockSecond_ < cfg::clock::SecondsPerMinute);
}

uint8_t ClusterFeedback::clockHour() const {
    return clockHour_;
}

uint8_t ClusterFeedback::clockMinute() const {
    return clockMinute_;
}

uint8_t ClusterFeedback::clockSecond() const {
    return clockSecond_;
}
