#pragma once

/**
 * @file ClusterFeedback.h
 * @brief Decodes the frames the instrument cluster transmits about itself.
 */

#include <stdint.h>

#include "CanMessage.h"
#include "Config.h"

/**
 * @brief Records the cluster's own broadcasts.
 *
 * The cluster is not only a display: it reports the outside temperature it
 * measures on its sensor pins and the time it currently holds. Both answer
 * questions that are otherwise awkward on a bench, namely what the
 * thermometer actually reads and whether a transmitted time was accepted.
 *
 * Neither frame may be transmitted by this firmware. The cluster is their
 * sender, and a second sender behind the same identifier would produce bus
 * error frames.
 */
class ClusterFeedback {
public:
    ClusterFeedback() = default;

    /// Decodes a received frame. Returns true if it was one of interest.
    bool consume(const CanMessage &message, uint32_t nowMs);

    /// True if a temperature reading arrived recently enough to trust.
    bool temperatureFresh(uint32_t nowMs) const;

    /// True if the cluster reported an actual reading rather than its
    /// "no sensor" placeholder.
    bool temperatureValid() const;

    /// Outside temperature in tenths of a degree Celsius.
    int16_t temperatureDeciCelsius() const;

    bool clockFresh(uint32_t nowMs) const;

    /// True if the reported time is a real time of day. A cluster that has
    /// never been given the time reports values outside the valid ranges.
    bool clockValid() const;

    uint8_t clockHour() const;
    uint8_t clockMinute() const;
    uint8_t clockSecond() const;

private:
    bool temperatureSeen_ = false;
    uint8_t temperatureRaw_ = cfg::feedback::TemperatureUnavailable;
    int16_t temperatureDeciC_ = 0;
    uint32_t temperatureMs_ = 0;

    bool clockSeen_ = false;
    uint8_t clockHour_ = 0;
    uint8_t clockMinute_ = 0;
    uint8_t clockSecond_ = 0;
    uint32_t clockMs_ = 0;
};
