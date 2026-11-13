#pragma once

/**
 * @file CanMessage.h
 * @brief Value type representing a single classic CAN frame.
 */

#include <stdint.h>

/**
 * @brief An eleven bit identifier together with up to eight data bytes.
 *
 * The class owns its payload by value, so a message can be built on the stack,
 * returned from a builder function and handed to a bus without any dynamic
 * allocation. Element access is provided through operator[], which clamps the
 * index so that a malformed caller cannot write past the payload.
 */
class CanMessage {
public:
    /// Maximum payload of a classic CAN frame.
    static constexpr uint8_t MaxDataBytes = 8;

    CanMessage() = default;

    /**
     * @param identifier Eleven bit CAN identifier.
     * @param length     Payload length in bytes, clamped to MaxDataBytes.
     */
    CanMessage(uint16_t identifier, uint8_t length);

    uint16_t identifier() const;
    uint8_t length() const;

    /// Read only view of the payload, for handing to a driver.
    const uint8_t *data() const;

    void setIdentifier(uint16_t identifier);
    void setLength(uint8_t length);

    /// Writable access to one payload byte.
    uint8_t &operator[](uint8_t index);

    /// Read only access to one payload byte.
    const uint8_t &operator[](uint8_t index) const;

private:
    /// Shared bounds rule for both subscript operators.
    static uint8_t clampIndex(uint8_t index);

    uint16_t identifier_ = 0;
    uint8_t length_ = 0;
    uint8_t data_[MaxDataBytes] = {};
};
