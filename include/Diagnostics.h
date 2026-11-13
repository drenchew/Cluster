#pragma once

/**
 * @file Diagnostics.h
 * @brief Formatted output helpers for the serial console.
 *
 * The AVR core provides no Serial.printf, and its vsnprintf implementation
 * omits floating point conversions. Fractional values are therefore printed
 * through printFixed1, which formats a tenths value as a decimal by hand.
 */

#include <Arduino.h>
#include <stdarg.h>
#include <stdint.h>

namespace diag {

/// Longest formatted line. Chosen to fit comfortably in the Uno's SRAM.
constexpr uint8_t LineBufferBytes = 96;
constexpr uint8_t DecimalBase = 10;

/// Prints a value held in tenths as a single decimal place, for example
/// 305 as "30.5".
inline void printFixed1(uint16_t tenths) {
    Serial.print(tenths / DecimalBase);
    Serial.write('.');
    Serial.write(static_cast<char>('0' + (tenths % DecimalBase)));
}

/// printf-style output to the serial console.
inline void printf(const char *format, ...) {
    char buffer[LineBufferBytes];
    va_list arguments;
    va_start(arguments, format);
    vsnprintf(buffer, sizeof(buffer), format, arguments);
    va_end(arguments);
    Serial.print(buffer);
}

}  // namespace diag
