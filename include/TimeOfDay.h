#pragma once

/**
 * @file TimeOfDay.h
 * @brief Calendar clock advanced from the millisecond timer.
 */

#include <stdint.h>

#include "Config.h"

/**
 * @brief Tracks wall clock time and date for the cluster display.
 *
 * The cluster has no clock of its own until one is supplied over the bus, and
 * it must not stop if the host link is lost. This class therefore keeps time
 * locally from the millisecond counter and accepts a correction whenever the
 * host sends one, treating the host as the time master.
 */
class TimeOfDay {
public:
    TimeOfDay();

    /**
     * @brief Applies an externally supplied time and date.
     *
     * Out of range fields are normalised rather than rejected. If any field
     * differs from the current value the local second boundary is realigned
     * to the moment of the update, so that the two sources cannot drift a
     * second apart and alternate between values.
     *
     * @param nowMs Current millisecond count, used to realign the tick.
     */
    void set(uint8_t hour, uint8_t minute, uint8_t second, uint8_t day,
             uint8_t month, uint16_t year, uint32_t nowMs);

    /// Advances the clock by whole seconds elapsed since the last call.
    void advanceTo(uint32_t nowMs);

    uint8_t hour() const;
    uint8_t minute() const;
    uint8_t second() const;
    uint8_t day() const;
    uint8_t month() const;
    uint16_t year() const;

    /// True once any source has supplied a time.
    bool isValid() const;

    /// Length of a month, accounting for leap years in the Gregorian rule.
    static uint8_t daysInMonth(uint8_t month, uint16_t year);

private:
    static bool isLeapYear(uint16_t year);

    /// Adds one second and propagates any resulting carry into the date.
    void addOneSecond();

    uint8_t hour_ = cfg::clock::BootHour;
    uint8_t minute_ = cfg::clock::BootMinute;
    uint8_t second_ = 0;
    uint8_t day_ = cfg::clock::BootDay;
    uint8_t month_ = cfg::clock::BootMonth;
    uint16_t year_ = cfg::clock::BootYear;
    bool valid_ = false;
    uint32_t lastSecondMs_ = 0;
};
