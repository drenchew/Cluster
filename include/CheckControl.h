#pragma once

/**
 * @file CheckControl.h
 * @brief Manages the numbered warning symbols shown by the cluster.
 */

#include <stdint.h>

#include "Config.h"
#include "ICanBus.h"

/**
 * @brief Raises and clears check control symbols on 0x592.
 *
 * Most telltales on this cluster have no dedicated bus signal. They are
 * numbered symbols that a control unit asks the cluster to display, and which
 * stay displayed until explicitly cleared. Two consequences follow and are
 * handled here: a displayed symbol is refreshed periodically so that it
 * survives a lost frame, and a cleared one is re-cleared occasionally so that
 * a lost clear cannot strand a telltale on the dial.
 *
 * At most one frame is emitted per call. Changes are served before refreshes,
 * and a rotating cursor ensures that one rapidly changing symbol cannot
 * prevent the others from ever being serviced.
 */
class CheckControl {
public:
    /// Symbols this class can display, in the order they occupy slots.
    enum class Lamp : uint8_t {
        Dtc = 0,
        DscTriangle,
        DscOff,
        CheckEngine,
        OilPressure,
        Battery,
        Overheat,
        FuelReserve,
        AdBlue,
        Brake,
        TyrePressure,
        Custom,     ///< any identifier chosen at run time
        Count
    };

    CheckControl();

    /// Requests that a symbol be displayed or cleared.
    void setLamp(Lamp lamp, bool on);

    /// Points the spare slot at an arbitrary symbol identifier. Changing the
    /// identifier while the previous symbol is displayed clears it first,
    /// since nothing else would ever turn it off.
    void setCustomSymbol(uint16_t identifier, bool on, ICanBus &bus);

    /// Maps a warning bitmask from the host onto the individual lamps.
    void applyWarningMask(uint8_t mask);

    /// Clears every request. Used when the host link is lost.
    void clearAll();

    /// Emits at most one frame. Call periodically.
    void service(ICanBus &bus, uint32_t nowMs);

    /// Builds a symbol frame without sending it.
    static CanMessage buildMessage(uint16_t identifier, bool on);

private:
    /// One managed symbol: what was asked for, and what the bus was last told.
    struct Slot {
        uint16_t identifier;
        bool requested;
        bool transmitted;
        uint32_t lastSentMs;
    };

    static constexpr uint8_t SlotCount = static_cast<uint8_t>(Lamp::Count);

    /// True when the slot has an outstanding change to transmit.
    static bool hasPendingChange(const Slot &slot);

    /// True when the slot is due a periodic repeat.
    static bool isRefreshDue(const Slot &slot, uint32_t nowMs);

    Slot slots_[SlotCount];
    uint8_t cursor_ = 0;
};
