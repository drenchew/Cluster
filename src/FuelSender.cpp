#include "FuelSender.h"

#include <Arduino.h>

namespace {

/// Measured breakpoints: sender counts at a given fill fraction, held in
/// permille so the interpolation can be done in integer arithmetic. Stored in
/// program memory because the Uno has very little SRAM.
const uint16_t kFraction[cfg::fuel::CurvePoints] PROGMEM = {1000, 875, 750, 500, 250, 0};
const uint16_t kLeftCounts[cfg::fuel::CurvePoints] PROGMEM = {9700, 8200, 6250,
                                                              3600, 1950, 625};
const uint16_t kRightCounts[cfg::fuel::CurvePoints] PROGMEM = {8400, 5400, 4600,
                                                               3350, 2200, 950};

constexpr float FractionEmpty = 0.0f;
constexpr float FractionFull = 1.0f;
constexpr float RoundingOffset = 0.5f;
constexpr float MillisecondsPerSecond = 1000.0f;

}  // namespace

uint16_t FuelSender::interpolate(uint16_t permille, FuelSenderSide side) {
    const uint16_t *table = (side == FuelSenderSide::Right) ? kRightCounts : kLeftCounts;
    // Falls back to the empty end of the curve if no segment matches, which
    // can only happen for a fraction outside the tabulated range.
    uint16_t result = pgm_read_word(&table[cfg::fuel::CurvePoints - 1]);
    bool found = false;

    for (uint8_t i = 0; i + 1 < cfg::fuel::CurvePoints && !found; ++i) {
        const uint16_t upperFraction = pgm_read_word(&kFraction[i]);
        const uint16_t lowerFraction = pgm_read_word(&kFraction[i + 1]);
        if (permille <= upperFraction && permille >= lowerFraction) {
            const uint16_t upperCount = pgm_read_word(&table[i]);
            const uint16_t lowerCount = pgm_read_word(&table[i + 1]);
            const uint16_t span = upperFraction - lowerFraction;
            if (span == 0) {
                result = lowerCount;
            } else {
                const uint32_t rise = static_cast<uint32_t>(upperCount - lowerCount);
                result = static_cast<uint16_t>(
                    lowerCount + rise * (permille - lowerFraction) / span);
            }
            found = true;
        }
    }
    return result;
}

uint16_t FuelSender::counts(float fraction, FuelSenderSide side) {
    float bounded = fraction;
    if (bounded < FractionEmpty) {
        bounded = FractionEmpty;
    } else if (bounded > FractionFull) {
        bounded = FractionFull;
    }

    uint16_t result = 0;
    if (cfg::fuel::UseMeasuredCurve) {
        const uint16_t permille = static_cast<uint16_t>(
            bounded * cfg::fuel::PermilleFullScale + RoundingOffset);
        result = interpolate(permille, side);
    } else {
        result = static_cast<uint16_t>(bounded * cfg::fuel::TankLitres *
                                       cfg::fuel::FlatCountsPerLitre);
    }
    return result;
}

void FuelSender::update(float targetLitres, uint32_t nowMs) {
    if (!cfg::fuel::EnableSlew || displayedLitres_ < 0.0f) {
        displayedLitres_ = targetLitres;
        lastUpdateMs_ = nowMs;
    } else {
        const uint32_t elapsedMs = nowMs - lastUpdateMs_;
        // Floating point work is limited to a few times a second; the gauge
        // moves far too slowly for a finer step to be visible.
        if (elapsedMs >= cfg::fuel::SlewIntervalMs) {
            lastUpdateMs_ = nowMs;
            const float step = cfg::fuel::SlewFractionPerSecond * cfg::fuel::TankLitres *
                               (static_cast<float>(elapsedMs) / MillisecondsPerSecond);
            const float difference = targetLitres - displayedLitres_;
            if (difference > step) {
                displayedLitres_ += step;
            } else if (difference < -step) {
                displayedLitres_ -= step;
            } else {
                displayedLitres_ = targetLitres;
            }
        }
    }
}

float FuelSender::displayedLitres(float fallbackLitres) const {
    return (displayedLitres_ >= 0.0f) ? displayedLitres_ : fallbackLitres;
}
