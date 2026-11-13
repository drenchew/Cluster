#pragma once

/**
 * @file Mcp2515Bus.h
 * @brief ICanBus implementation for the MCP2515 controller.
 */

#include <mcp2515.h>
#include <stdint.h>

#include "Config.h"
#include "ICanBus.h"

/**
 * @brief Drives an MCP2515 over hardware SPI.
 *
 * The bitrate and crystal constants are declared here rather than in Config.h
 * because they are expressed in the driver library's own enumerations, which
 * keeps the configuration header free of third party dependencies.
 *
 * The E9x instrument cluster sits on K-CAN at 100 kbit/s. The powertrain bus
 * runs at 500 kbit/s and is not connected to the cluster.
 */
class Mcp2515Bus : public ICanBus {
public:
    static constexpr CAN_SPEED Bitrate = CAN_100KBPS;
    static constexpr CAN_CLOCK CrystalFrequency = MCP_8MHZ;

    Mcp2515Bus();

    /// Configures SPI and retries initialisation until the controller
    /// responds, reporting the likely wiring fault on each attempt.
    bool begin() override;

    bool send(const CanMessage &message) override;
    bool receive(CanMessage &message) override;
    Statistics statistics() const override;

private:
    /// Reads one controller register directly over SPI, bypassing the driver
    /// library. Used before initialisation to distinguish a wiring fault from
    /// a configuration failure.
    uint8_t readRegisterRaw(uint8_t address);

    /// Resets the controller and applies the bus timing.
    bool configure();

    /// Prints the wiring fault implied by a raw status register value.
    static void reportFault(uint8_t statusRegister);

    MCP2515 controller_;
    Statistics statistics_;
};
