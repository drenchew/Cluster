#include "TimeOfDay.h"

namespace {

/// Days in each month for a common year, January first.
const uint8_t kMonthLengths[cfg::clock::MonthsPerYear] = {31, 28, 31, 30, 31, 30,
                                                          31, 31, 30, 31, 30, 31};

constexpr uint16_t LeapCycleShort = 4;
constexpr uint16_t LeapCycleCentury = 100;
constexpr uint16_t LeapCycleLong = 400;

}  // namespace

TimeOfDay::TimeOfDay() {
}

bool TimeOfDay::isLeapYear(uint16_t year) {
    const bool divisibleByFour = (year % LeapCycleShort) == 0;
    const bool divisibleByCentury = (year % LeapCycleCentury) == 0;
    const bool divisibleByFourHundred = (year % LeapCycleLong) == 0;
    return divisibleByFour && (!divisibleByCentury || divisibleByFourHundred);
}

uint8_t TimeOfDay::daysInMonth(uint8_t month, uint16_t year) {
    uint8_t days = cfg::clock::FallbackMonthDays;
    if (month >= 1 && month <= cfg::clock::MonthsPerYear) {
        days = kMonthLengths[month - 1];
        if (month == cfg::clock::FebruaryIndex && isLeapYear(year)) {
            days = cfg::clock::FebruaryLeapDays;
        }
    }
    return days;
}

void TimeOfDay::set(uint8_t hour, uint8_t minute, uint8_t second, uint8_t day,
                    uint8_t month, uint16_t year, uint32_t nowMs) {
    const bool changed = (hour != hour_) || (minute != minute_) || (second != second_) ||
                         (day != day_) || (month != month_) || (year != year_);

    hour_ = hour % cfg::clock::HoursPerDay;
    minute_ = minute % cfg::clock::MinutesPerHour;
    second_ = second % cfg::clock::SecondsPerMinute;
    day_ = (day == 0 || day > cfg::clock::FallbackMonthDays) ? 1 : day;
    month_ = (month == 0 || month > cfg::clock::MonthsPerYear) ? 1 : month;
    year_ = year;
    valid_ = true;

    if (changed) {
        lastSecondMs_ = nowMs;
    }
}

void TimeOfDay::addOneSecond() {
    ++second_;
    if (second_ >= cfg::clock::SecondsPerMinute) {
        second_ = 0;
        ++minute_;
        if (minute_ >= cfg::clock::MinutesPerHour) {
            minute_ = 0;
            ++hour_;
            if (hour_ >= cfg::clock::HoursPerDay) {
                hour_ = 0;
                ++day_;
                if (day_ > daysInMonth(month_, year_)) {
                    day_ = 1;
                    ++month_;
                    if (month_ > cfg::clock::MonthsPerYear) {
                        month_ = 1;
                        ++year_;
                    }
                }
            }
        }
    }
}

void TimeOfDay::advanceTo(uint32_t nowMs) {
    if (lastSecondMs_ == 0) {
        // First call after power-up: adopt the current instant as the second
        // boundary rather than replaying the whole uptime as elapsed seconds.
        lastSecondMs_ = nowMs;
    } else {
        while ((nowMs - lastSecondMs_) >= cfg::clock::MillisecondsPerSecond) {
            lastSecondMs_ += cfg::clock::MillisecondsPerSecond;
            addOneSecond();
        }
    }
}

uint8_t TimeOfDay::hour() const {
    return hour_;
}

uint8_t TimeOfDay::minute() const {
    return minute_;
}

uint8_t TimeOfDay::second() const {
    return second_;
}

uint8_t TimeOfDay::day() const {
    return day_;
}

uint8_t TimeOfDay::month() const {
    return month_;
}

uint16_t TimeOfDay::year() const {
    return year_;
}

bool TimeOfDay::isValid() const {
    return valid_;
}
