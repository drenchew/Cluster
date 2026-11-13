#pragma once

/**
 * @file HostProtocol.h
 * @brief Binary setpoint link between the host simulator and the firmware.
 */

#include <stdint.h>

#include "ClusterCan.h"
#include "Config.h"

/**
 * @brief One frame of setpoints as sent by the host.
 *
 * The layout is shared byte for byte with host/cluster_sim/link.py. The
 * structure is packed because the host writes it with no padding, and the
 * static assertion below fails the build if the two definitions diverge.
 */
struct __attribute__((packed)) HostState {
    uint16_t engineSpeedRpm;
    uint16_t roadSpeedKmh;
    uint16_t fuelTenthLitres;
    int16_t coolantCelsius;
    int16_t outsideCelsius;
    uint16_t rangeKm;
    uint8_t flags;          ///< see HostFlag
    uint8_t lightMask;      ///< see cfg::lights
    uint8_t turnSignals;    ///< bit 0 left, bit 1 right
    uint8_t gearPosition;   ///< GearPosition value
    uint8_t gearNumber;     ///< displayed gear, zero blanks the field
    uint8_t gearMode;       ///< GearMode value
    uint8_t dimmer;
    uint8_t pedalPercent;
    uint8_t extraFlags;     ///< see HostExtraFlag
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t day;
    uint8_t month;
    uint16_t year;
    uint8_t warningMask;    ///< see cfg::warn
    uint8_t cruiseKmh;      ///< zero switches cruise control off
    uint8_t consumptionPercent;  ///< instantaneous fuel demand, 0 to 100
};

static_assert(sizeof(HostState) == cfg::host::PayloadBytes,
              "HostState must match cfg::host::PayloadBytes and link.py");

/// Bits in HostState::flags.
enum HostFlag : uint8_t {
    HostFlagTerminal15 = 0x01,
    HostFlagEngineRunning = 0x02,
    HostFlagHandbrake = 0x04,
    HostFlagReverse = 0x08
};

/// Bits in HostState::extraFlags. The stability telltales are check control
/// symbols rather than signals, so they are carried as plain requests.
enum HostExtraFlag : uint8_t {
    HostExtraTractionMode = 0x01,
    HostExtraIntervening = 0x02,
    HostExtraStabilityOff = 0x04,
    HostExtraClockValid = 0x08,
    /// Report the fuel level without gauge damping. Set by hosts that drive
    /// the gauge from a fast changing quantity rather than a tank level.
    HostExtraFuelDirect = 0x10
};

/**
 * @brief Framing and validation for the host link.
 *
 * Frames are delimited by a two byte preamble, a length and a trailing
 * exclusive-or checksum. The same serial port also carries the text console,
 * so bytes are only claimed once a preamble has been seen.
 */
class HostProtocol {
public:
    /**
     * @brief Offers one received byte to the parser.
     * @return true if the byte belonged to a binary frame, false if it should
     *         be passed to the text console instead.
     */
    bool feed(uint8_t value, ClusterCan &cluster);

    /// True while frames are arriving regularly.
    bool hostLive() const;

    /**
     * @brief True shortly after any byte was claimed as binary.
     *
     * A frame cut short leaves payload bytes in the stream, and those can
     * hold any value including printable characters. The console stays shut
     * briefly afterwards so that such bytes cannot be run as commands.
     */
    bool binaryRecent() const;

    uint32_t frames() const;
    uint32_t errors() const;

private:
    /// Parser position within a frame.
    enum class Stage : uint8_t {
        WaitPreambleFirst,
        WaitPreambleSecond,
        WaitLength,
        ReadPayload,
        WaitChecksum
    };

    /// At 50 Hz frames arrive 20 ms apart, so this covers an interrupted
    /// frame without muting a console that is genuinely being typed at.
    static constexpr uint32_t BinaryQuietMs = 250;

    /// Copies a validated frame into the cluster driver.
    void apply(ClusterCan &cluster);

    uint8_t buffer_[cfg::host::PayloadBytes] = {};
    Stage stage_ = Stage::WaitPreambleFirst;
    uint8_t index_ = 0;
    uint8_t checksum_ = 0;
    uint32_t lastFrameMs_ = 0;
    uint32_t lastByteMs_ = 0;
    uint32_t frames_ = 0;
    uint32_t errors_ = 0;
};
