#include "CanMessage.h"

CanMessage::CanMessage(uint16_t identifier, uint8_t length)
    : identifier_(identifier), length_(0) {
    setLength(length);
}

uint16_t CanMessage::identifier() const {
    return identifier_;
}

uint8_t CanMessage::length() const {
    return length_;
}

const uint8_t *CanMessage::data() const {
    return data_;
}

void CanMessage::setIdentifier(uint16_t identifier) {
    identifier_ = identifier;
}

void CanMessage::setLength(uint8_t length) {
    length_ = (length > MaxDataBytes) ? MaxDataBytes : length;
}

uint8_t CanMessage::clampIndex(uint8_t index) {
    return (index >= MaxDataBytes) ? (MaxDataBytes - 1) : index;
}

uint8_t &CanMessage::operator[](uint8_t index) {
    return data_[clampIndex(index)];
}

const uint8_t &CanMessage::operator[](uint8_t index) const {
    return data_[clampIndex(index)];
}
