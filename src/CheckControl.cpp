#include "CheckControl.h"

namespace {

/// 0x592 payload layout.
constexpr uint8_t SymbolFrameBytes = 8;
constexpr uint8_t IndexCommand = 0;
constexpr uint8_t IndexIdentifierLow = 1;
constexpr uint8_t IndexIdentifierHigh = 2;
constexpr uint8_t IndexState = 3;
constexpr uint8_t IndexPaddingFirst = 4;
constexpr uint8_t CommandDisplay = 0x40;
constexpr uint8_t StateShow = 0x31;
constexpr uint8_t StateClear = 0x30;
constexpr uint8_t PaddingByte = 0xFF;
constexpr uint8_t ByteMask = 0xFF;
constexpr uint8_t ByteBits = 8;

/// Identifier reserved to mean "this slot is unused".
constexpr uint16_t UnusedSymbol = 0;

}  // namespace

CheckControl::CheckControl() {
    const uint16_t identifiers[SlotCount] = {
        cfg::symbol::Dtc,         cfg::symbol::DscTriangle, cfg::symbol::DscOff,
        cfg::symbol::CheckEngine, cfg::symbol::OilPressure, cfg::symbol::Battery,
        cfg::symbol::Overheat,    cfg::symbol::FuelReserve, cfg::symbol::AdBlue,
        cfg::symbol::BrakeFault,  cfg::symbol::TyrePressure, UnusedSymbol};

    for (uint8_t i = 0; i < SlotCount; ++i) {
        slots_[i].identifier = identifiers[i];
        slots_[i].requested = false;
        slots_[i].transmitted = false;
        slots_[i].lastSentMs = 0;
    }
}

CanMessage CheckControl::buildMessage(uint16_t identifier, bool on) {
    CanMessage message(cfg::canid::CheckControlSymbol, SymbolFrameBytes);
    message[IndexCommand] = CommandDisplay;
    message[IndexIdentifierLow] = static_cast<uint8_t>(identifier & ByteMask);
    message[IndexIdentifierHigh] = static_cast<uint8_t>(identifier >> ByteBits);
    message[IndexState] = on ? StateShow : StateClear;
    for (uint8_t i = IndexPaddingFirst; i < SymbolFrameBytes; ++i) {
        message[i] = PaddingByte;
    }
    return message;
}

void CheckControl::setLamp(Lamp lamp, bool on) {
    const uint8_t index = static_cast<uint8_t>(lamp);
    if (index < SlotCount) {
        slots_[index].requested = on;
    }
}

void CheckControl::setCustomSymbol(uint16_t identifier, bool on, ICanBus &bus) {
    Slot &slot = slots_[static_cast<uint8_t>(Lamp::Custom)];
    if (slot.identifier != identifier) {
        if (slot.transmitted) {
            bus.send(buildMessage(slot.identifier, false));
        }
        slot.identifier = identifier;
        slot.transmitted = false;
    }
    slot.requested = on;
}

void CheckControl::applyWarningMask(uint8_t mask) {
    setLamp(Lamp::CheckEngine, (mask & cfg::warn::CheckEngine) != 0);
    setLamp(Lamp::OilPressure, (mask & cfg::warn::OilPressure) != 0);
    setLamp(Lamp::Battery, (mask & cfg::warn::Battery) != 0);
    setLamp(Lamp::Overheat, (mask & cfg::warn::Coolant) != 0);
    setLamp(Lamp::FuelReserve, (mask & cfg::warn::FuelReserve) != 0);
    setLamp(Lamp::AdBlue, (mask & cfg::warn::AdBlue) != 0);
    setLamp(Lamp::Brake, (mask & cfg::warn::Brake) != 0);
    setLamp(Lamp::TyrePressure, (mask & cfg::warn::TyrePressure) != 0);
}

void CheckControl::clearAll() {
    for (uint8_t i = 0; i < SlotCount; ++i) {
        slots_[i].requested = false;
    }
}

bool CheckControl::hasPendingChange(const Slot &slot) {
    return slot.requested != slot.transmitted;
}

bool CheckControl::isRefreshDue(const Slot &slot, uint32_t nowMs) {
    const uint16_t period =
        slot.requested ? cfg::symbol::OnRefreshMs : cfg::symbol::OffRefreshMs;
    return (nowMs - slot.lastSentMs) >= period;
}

void CheckControl::service(ICanBus &bus, uint32_t nowMs) {
    // Two passes: outstanding changes are always served before refreshes, so
    // a newly requested telltale appears without waiting behind a repeat.
    bool sent = false;
    for (uint8_t pass = 0; pass < 2 && !sent; ++pass) {
        for (uint8_t offset = 0; offset < SlotCount && !sent; ++offset) {
            const uint8_t index = static_cast<uint8_t>((cursor_ + offset) % SlotCount);
            Slot &slot = slots_[index];
            if (slot.identifier != UnusedSymbol) {
                const bool due = (pass == 0) ? hasPendingChange(slot)
                                             : isRefreshDue(slot, nowMs);
                if (due) {
                    bus.send(buildMessage(slot.identifier, slot.requested));
                    slot.transmitted = slot.requested;
                    slot.lastSentMs = nowMs;
                    cursor_ = static_cast<uint8_t>((index + 1) % SlotCount);
                    sent = true;
                }
            }
        }
    }
}
