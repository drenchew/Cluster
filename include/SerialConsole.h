#pragma once

/**
 * @file SerialConsole.h
 * @brief Line based text interface for operating the jig without a host.
 */

#include <stdint.h>

#include "ClusterCan.h"
#include "Config.h"
#include "DemoSweep.h"
#include "HostProtocol.h"

/**
 * @brief Parses typed commands from the serial port.
 *
 * The port is shared with the binary host link. Each received byte is offered
 * to the protocol parser first, and only bytes it declines are treated as
 * text. While the host is driving, or immediately after a binary frame was
 * cut short, the console ignores input entirely: the payload of an
 * interrupted frame can contain any byte value, and those bytes must not be
 * interpreted as commands.
 *
 * Input is accumulated a line at a time and parsed without blocking. Reading
 * a number directly from the stream would stall the main loop for the read
 * timeout whenever a command carried no argument, which would interrupt the
 * transmission schedule for long enough to reset the cluster.
 */
class SerialConsole {
public:
    SerialConsole(ClusterCan &cluster, DemoSweep &demo, HostProtocol &host);

    /// Consumes all available input.
    void poll();

    /// Prints the startup banner and current configuration.
    void printBanner() const;

    /// Prints a summary of driver and link state.
    void printStatus() const;

    /// Prints what the cluster has most recently reported about itself.
    void printClusterFeedback() const;

private:
    /// Executes one parsed command.
    void execute(char command, const char *argument);

    /// Handles the time setting command, which accepts HHMM or HHMMSS.
    void applyTimeCommand(const char *argument);

    /// Handles the symbol command, which takes an identifier and a state.
    void applySymbolCommand(const char *argument);

    /// Splits a completed line into a command letter and its argument.
    void dispatchLine();

    ClusterCan &cluster_;
    DemoSweep &demo_;
    HostProtocol &host_;

    char line_[cfg::console::MaxLineLength] = {};
    uint8_t lineLength_ = 0;
};
