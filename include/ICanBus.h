#pragma once

/**
 * @file ICanBus.h
 * @brief Abstract interface to a CAN controller.
 */

#include <stdint.h>

#include "CanMessage.h"

/**
 * @brief Transport used by the cluster driver to reach the vehicle bus.
 *
 * Declaring the transport as an interface keeps ClusterCan independent of any
 * particular controller: the driver is written against this abstraction and
 * receives a concrete bus by reference. Mcp2515Bus is the implementation used
 * on the Arduino; a recording implementation is substituted when the firmware
 * logic is exercised on a workstation, which is what allows the frame output
 * to be verified without hardware.
 */
class ICanBus {
public:
    /// Transmission and reception totals, used by the diagnostic report.
    struct Statistics {
        uint32_t transmitted = 0;
        uint32_t failed = 0;
        uint32_t received = 0;
    };

    /// Required because instances are destroyed through this interface.
    virtual ~ICanBus() = default;

    /// Brings the controller up. Returns true once it is ready to transmit.
    virtual bool begin() = 0;

    /// Queues one frame. Returns false if the controller rejected it.
    virtual bool send(const CanMessage &message) = 0;

    /// Retrieves one pending frame. Returns false when none is waiting.
    virtual bool receive(CanMessage &message) = 0;

    virtual Statistics statistics() const = 0;
};
