#include "HostProtocol.h"

#include <Arduino.h>
#include <string.h>

namespace {

constexpr uint8_t TurnLeftBit = 0x01;
constexpr uint8_t TurnRightBit = 0x02;
constexpr float TenthsPerLitre = 10.0f;

}  // namespace

bool HostProtocol::feed(uint8_t value, ClusterCan &cluster) {
    // Record the time before anything else, so that binaryRecent() also
    // covers a frame that is abandoned part way through.
    if (stage_ != Stage::WaitPreambleFirst || value == cfg::host::SyncFirst) {
        lastByteMs_ = millis();
    }

    bool consumed = true;

    switch (stage_) {
        case Stage::WaitPreambleFirst:
            if (value == cfg::host::SyncFirst) {
                stage_ = Stage::WaitPreambleSecond;
            } else {
                consumed = false;
            }
            break;

        case Stage::WaitPreambleSecond:
            if (value == cfg::host::SyncSecond) {
                stage_ = Stage::WaitLength;
            } else if (value == cfg::host::SyncFirst) {
                // Two preamble bytes in a row: the first was noise and this
                // is the real start, so hold position rather than discarding
                // a frame that is about to be valid.
                stage_ = Stage::WaitPreambleSecond;
            } else {
                stage_ = Stage::WaitPreambleFirst;
                ++errors_;
            }
            break;

        case Stage::WaitLength:
            if (value == cfg::host::PayloadBytes) {
                index_ = 0;
                checksum_ = value;
                stage_ = Stage::ReadPayload;
            } else {
                ++errors_;
                stage_ = (value == cfg::host::SyncFirst) ? Stage::WaitPreambleSecond
                                                         : Stage::WaitPreambleFirst;
            }
            break;

        case Stage::ReadPayload:
            buffer_[index_] = value;
            ++index_;
            checksum_ ^= value;
            if (index_ >= cfg::host::PayloadBytes) {
                stage_ = Stage::WaitChecksum;
            }
            break;

        case Stage::WaitChecksum:
        default:
            if (value == checksum_) {
                apply(cluster);
                ++frames_;
                lastFrameMs_ = millis();
            } else {
                ++errors_;
            }
            stage_ = Stage::WaitPreambleFirst;
            break;
    }

    return consumed;
}

void HostProtocol::apply(ClusterCan &cluster) {
    HostState frame;
    // Both ends are little endian, so the payload maps directly onto the
    // structure without any field by field conversion.
    memcpy(&frame, buffer_, sizeof(frame));

    VehicleState &state = cluster.state();

    // Ignition and engine state are applied first. The idle floor is only
    // enforced while the engine is marked as running, so this ordering
    // decides whether a commanded zero is honoured as a stall or raised.
    state.setTerminal15((frame.flags & HostFlagTerminal15) != 0);
    state.setEngineRunning((frame.flags & HostFlagEngineRunning) != 0);

    state.setEngineSpeed(frame.engineSpeedRpm);
    state.setRoadSpeed(frame.roadSpeedKmh);
    state.setPedalPercent(frame.pedalPercent);
    state.setFuelLitres(static_cast<float>(frame.fuelTenthLitres) / TenthsPerLitre);
    state.setCoolantCelsius(frame.coolantCelsius);
    state.setOutsideCelsius(frame.outsideCelsius);
    state.setRangeKm(frame.rangeKm);

    state.setHandbrake((frame.flags & HostFlagHandbrake) != 0);
    state.setReverse((frame.flags & HostFlagReverse) != 0);
    state.setLightMask(frame.lightMask);
    state.setDimmer(frame.dimmer);
    state.setTurnSignal((frame.turnSignals & TurnLeftBit) != 0,
                        (frame.turnSignals & TurnRightBit) != 0);
    state.setGear(static_cast<GearPosition>(frame.gearPosition), frame.gearNumber,
                  static_cast<GearMode>(frame.gearMode));

    cluster.setTractionLamps((frame.extraFlags & HostExtraTractionMode) != 0,
                             (frame.extraFlags & HostExtraIntervening) != 0,
                             (frame.extraFlags & HostExtraStabilityOff) != 0);
    cluster.setWarnings(frame.warningMask);
    cluster.setCruise(frame.cruiseKmh);
    // Absent flag means a tank level, which is damped; set means a live
    // measurement, which must not be.
    cluster.setFuelDamping((frame.extraFlags & HostExtraFuelDirect) == 0);

    // A host with no clock of its own leaves the flag clear, and the cluster
    // keeps running the time it already holds.
    if ((frame.extraFlags & HostExtraClockValid) != 0) {
        cluster.setClock(frame.hour, frame.minute, frame.second, frame.day, frame.month,
                         frame.year);
    }
}

bool HostProtocol::hostLive() const {
    return (lastFrameMs_ != 0) && ((millis() - lastFrameMs_) < cfg::host::TimeoutMs);
}

bool HostProtocol::binaryRecent() const {
    return (lastByteMs_ != 0) && ((millis() - lastByteMs_) < BinaryQuietMs);
}

uint32_t HostProtocol::frames() const {
    return frames_;
}

uint32_t HostProtocol::errors() const {
    return errors_;
}
