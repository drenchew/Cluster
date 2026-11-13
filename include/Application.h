#pragma once

/**
 * @file Application.h
 * @brief Composition root: owns every object and runs the main loop.
 */

#include <stdint.h>

#include "ClusterCan.h"
#include "DemoSweep.h"
#include "HostProtocol.h"
#include "Mcp2515Bus.h"
#include "SerialConsole.h"

/**
 * @brief Assembles the firmware and drives it.
 *
 * Every collaborator is constructed here and connected by reference, so the
 * dependencies between them are visible in one place and no object has to
 * find its own. The concrete bus is chosen here too; nothing below this class
 * depends on the controller being an MCP2515.
 */
class Application {
public:
    Application();

    /// Initialises hardware and reports the starting configuration.
    void begin();

    /// Performs one pass of the main loop. Never blocks.
    void run();

private:
    /// Switches between host control and the local sweep.
    void updateControlSource();

    /// Emits the periodic diagnostic reports.
    void updatePeriodicReports();

    /// Serial port speed, matched by the host tools.
    static constexpr uint32_t SerialBaud = 115200;
    /// Allows the serial port to settle before the banner is printed.
    static constexpr uint32_t StartupDelayMs = 300;
    static constexpr uint32_t StatusPeriodMs = 5000;
    static constexpr uint32_t HaltRetryMs = 1000;

    Mcp2515Bus bus_;
    ClusterCan cluster_;
    DemoSweep demo_;
    HostProtocol host_;
    SerialConsole console_;

    bool hostWasLive_ = false;
    uint32_t lastFeedbackMs_ = 0;
    uint32_t lastStatusMs_ = 0;
};
