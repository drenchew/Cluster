#pragma once

/**
 * @file ClusterCan.h
 * @brief Drives a BMW E9x instrument cluster over the K-CAN bus.
 */

#include <stdint.h>

#include "CheckControl.h"
#include "ClusterFeedback.h"
#include "Config.h"
#include "FrameLayout.h"
#include "FuelSender.h"
#include "ICanBus.h"
#include "TimeOfDay.h"
#include "VehicleState.h"

/**
 * @brief Coordinates the objects that together present a vehicle to the
 *        instrument cluster.
 *
 * The class is a facade over four collaborators: the setpoints being
 * displayed (VehicleState), the clock (TimeOfDay), the fuel gauge model
 * (FuelSender) and the warning symbols (CheckControl). It owns the
 * transmission schedule and the encoding of each frame, and reaches the bus
 * through the ICanBus interface rather than a concrete controller, which is
 * what allows the same logic to be exercised without hardware.
 *
 * Timing is owned here rather than by the host. If the serial link falls
 * silent the schedule continues, so the cluster never sees the bus stop.
 */
class ClusterCan {
public:
    /**
     * @param bus Transport to transmit on. The reference must outlive this
     *            object; it is normally a Mcp2515Bus declared beside it.
     */
    explicit ClusterCan(ICanBus &bus);

    /// Brings the bus up. Returns once the controller is ready.
    bool begin();

    /// Runs the schedule. Call from the main loop as often as possible; work
    /// is performed only when a tick is due.
    void update();

    /// Setpoints shown on the dials. VehicleState validates its own values,
    /// so exposing it directly does not weaken those guarantees.
    VehicleState &state();
    const VehicleState &state() const;

    const TimeOfDay &clock() const;
    const ClusterFeedback &feedback() const;

    /// Sets the displayed time. The host is treated as the time master.
    void setClock(uint8_t hour, uint8_t minute, uint8_t second, uint8_t day,
                  uint8_t month, uint16_t year);

    /// Stability control telltales, which have no dedicated bus signal.
    void setTractionLamps(bool tractionMode, bool intervening, bool disabled);

    /// Warning telltales, as a cfg::warn bitmask.
    void setWarnings(uint8_t mask);

    /// Displays or clears any symbol by identifier.
    void setSymbol(uint16_t identifier, bool on);

    /// Cruise control set speed in km/h; zero switches the telltale off.
    void setCruise(uint8_t kmh);

    /// Enables or disables fuel gauge damping. Disabled makes the needle
    /// follow the setpoint immediately, which is required when the gauge is
    /// displaying something that changes faster than a tank can empty.
    void setFuelDamping(bool enabled);

    bool fuelDamping() const;

    /// Selects how the distance accumulator advances and restarts it.
    void setOdometerMode(OdometerMode mode);

    /// Clears every telltale this class raises. Used when the host is lost.
    void clearDriverRequests();

    uint16_t odometerCounter() const;
    ICanBus::Statistics statistics() const;

private:
    /// Dispatches the frames belonging to one slot of the 200 ms cycle.
    void serviceSlot(uint8_t slot);

    /// Handles the frames that run on their own longer intervals.
    void serviceAperiodic(uint32_t nowMs);

    /// Reads and decodes everything the cluster has sent.
    void drainReceiveQueue();

    /// Encodes road speed for the 0x1B4 word.
    uint16_t brakeSpeedCounts() const;

    /// Advances the distance accumulator for one 0x1A6 frame.
    void advanceDistance(uint32_t nowMs);

    /// Advances the injected fuel accumulator for one 0x1D0 frame.
    void advanceInjection(uint32_t nowMs);

    void sendTerminalStatus();
    void sendIgnitionStatus();
    void sendEngineSpeed();
    void sendDistance();
    void sendDynamicsSpeed();
    void sendWheelSpeeds();
    void sendBrake();
    void sendEngineTemperature();
    void sendStabilityStatus();
    void sendParkBrake();
    void sendAbsCounter();
    void sendAirbagCounter();
    void sendFuelLevel();
    void sendTimeAndDate();
    void sendLighting();
    void sendDimmer();
    void sendLcdIllumination();
    void sendTurnSignals();
    void sendCruise();
    void sendGear();
    void sendReverse();
    void sendTemperatureAndRange();

    ICanBus &bus_;
    VehicleState state_;
    TimeOfDay clock_;
    FuelSender fuel_;
    CheckControl checkControl_;
    ClusterFeedback feedback_;

    uint32_t lastTickMs_ = 0;
    uint32_t tickCount_ = 0;
    uint32_t lastClockSendMs_ = 0;

    // Sequence counters required by individual frames.
    uint8_t terminalCounter_ = 0;
    uint8_t engineSpeedCounter_ = 0;
    uint8_t engineTemperatureCounter_ = 0;
    uint8_t absCounter_ = frame::absCounter::First;
    uint8_t airbagCounter_ = 0;
    uint8_t brakeCounter_ = 0;
    uint8_t gearCounter_ = 0;
    uint8_t stabilityCounter_ = 0;
    uint8_t cruiseTimer_ = 0;
    uint8_t lastCruiseSpeed_ = frame::cruise::SpeedWhenOff;

    /// Last park brake state placed on the bus, so the frame can be sent on
    /// change as a vehicle does rather than continuously.
    bool lastParkBrakeSent_ = false;

    /// Cumulative injected fuel reported in 0x1D0, with the fractional part
    /// carried between frames so that a low demand still accumulates.
    uint16_t injectionCounter_ = 0;
    float injectionRemainder_ = 0.0f;
    uint32_t lastInjectionMs_ = 0;

    // 0x1A6 accumulator state.
    uint16_t distanceCounter_ = 0;
    float distanceRemainder_ = 0.0f;
    uint32_t lastDistanceMs_ = 0;
    uint16_t distanceTimeCounter_ = 0;
};
