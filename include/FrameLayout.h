#pragma once

/**
 * @file FrameLayout.h
 * @brief Wire format of each transmitted CAN frame.
 *
 * Byte positions, fixed payload values and counter moduli are declared here so
 * that the frame builders in ClusterCan read as a description of the protocol
 * rather than a sequence of unexplained numbers. Each namespace documents the
 * evidence for its layout.
 */

#include <stdint.h>

namespace frame {

/// Values that recur across several frames.
namespace common {
constexpr uint8_t ByteMask = 0xFF;
constexpr uint8_t ByteBits = 8;
constexpr uint8_t NibbleMask = 0x0F;
constexpr uint8_t NibbleBits = 4;
constexpr uint8_t Filler = 0xFF;
constexpr uint8_t Unused = 0x00;
/// Several frames carry a four bit sequence counter that wraps before 15.
constexpr uint8_t CounterModulus = 15;
}  // namespace common

/// 0x130 terminal status, five bytes, 100 ms.
///
/// Byte 4 is a packed field: the high nibble is a sequence number and the low
/// nibble a check value derived from it. Every payload reported as working on
/// this cluster holds the byte constant. Incrementing the sequence without
/// recomputing the check value produces a byte the cluster rejects, after
/// which it treats terminal status as implausible and repeats its power-on
/// sequence indefinitely.
namespace terminal {
constexpr uint8_t Length = 5;
constexpr uint8_t IndexKeyStatus = 0;
constexpr uint8_t IndexTransponder = 1;
constexpr uint8_t IndexTerminalStatus = 2;
constexpr uint8_t IndexSteeringLock = 3;
constexpr uint8_t IndexPacked = 4;
constexpr uint8_t KeyPresent = 0x45;
constexpr uint8_t KeyAbsent = 0x00;
constexpr uint8_t Transponder = 0x40;
constexpr uint8_t TerminalStatus = 0x21;
constexpr uint8_t SteeringLock = 0x8F;
constexpr uint8_t PackedIgnitionOn = 0xFE;
constexpr uint8_t PackedIgnitionOff = 0x50;
/// Enabling this restores the rejected rolling counter described above. It
/// exists so the behaviour can be demonstrated, and defaults to disabled.
constexpr bool UseRollingCounter = false;
}  // namespace terminal

/// 0x26E ignition status, eight bytes, 200 ms. Static payload.
namespace ignition {
constexpr uint8_t Length = 8;
constexpr uint8_t Payload[Length] = {0x40, 0x40, 0x7F, 0x50, 0xFF, 0xFF, 0xFF, 0xFF};
}  // namespace ignition

/// 0x12F engine controller status, eight bytes, 100 ms. Static payload.
///
/// Transmitted because the instant consumption reading is a function of the
/// engine controller, and the cluster does not display engine derived values
/// unless that controller is reporting. Taken from the reference driver for
/// this cluster, which drives its consumption gauge successfully.
namespace dmeStatus {
constexpr uint8_t Length = 8;
constexpr uint8_t Payload[Length] = {0x3F, 0x00, 0x00, 0x00, 0x00, 0x40, 0x01, 0x00};
}  // namespace dmeStatus

/// 0x0AA engine speed and accelerator position, eight bytes, 20 ms.
/// Drives the tachometer needle.
namespace engineSpeed {
constexpr uint8_t Length = 8;
constexpr uint8_t IndexUnknown = 0;
constexpr uint8_t IndexCounter = 1;
constexpr uint8_t IndexPedalLow = 2;
constexpr uint8_t IndexPedalHigh = 3;
constexpr uint8_t IndexSpeedLow = 4;
constexpr uint8_t IndexSpeedHigh = 5;
constexpr uint8_t IndexFlags = 6;
constexpr uint8_t IndexTrailer = 7;
constexpr uint8_t UnknownValue = 0xFE;
constexpr uint8_t CounterBase = 0xF0;
constexpr uint8_t TrailerValue = 0x99;
/// Engine speed is transmitted in quarter revolution steps.
constexpr uint8_t SpeedScale = 4;
/// Accelerator position is reported in hundredths of a percent.
constexpr uint8_t PedalScale = 100;
constexpr uint8_t FlagsAllSet = 0xFE;
/// Throttle pressed and accelerator pressed, cleared at rest.
constexpr uint8_t FlagsPressedMask = 0x14;
}  // namespace engineSpeed

/// 0x1A6 distance accumulator and time counter, eight bytes, 100 ms.
///
/// Bytes 0 to 5 hold three independent sixteen bit accumulators rather than a
/// repeated speed value; the cluster differentiates them to recover speed, so
/// a constant payload leaves the speedometer at rest. Bytes 6 and 7 hold a
/// separate free running twelve bit counter whose high nibble is always set.
/// The transmission interval forms part of the encoding: sending faster than
/// 100 ms scales the derived speed by the same factor.
namespace distance {
constexpr uint8_t Length = 8;
constexpr uint8_t IndexCopyFirst = 0;
constexpr uint8_t CopyCount = 3;
constexpr uint8_t IndexTimeLow = 6;
constexpr uint8_t IndexTimeHigh = 7;
constexpr uint8_t TimeHighBase = 0xF0;
}  // namespace distance

/// 0x1A0 vehicle dynamics, eight bytes, 100 ms. Carries the direct speed
/// value the needle reads.
///
/// Byte 1 holds the speed high bits, a motion flag and a bit set in every
/// observed frame. Byte 6 carries a liveness nibble derived from elapsed time
/// rather than a per frame increment. Byte 7 is a checksum verified against
/// every frame of a reference capture.
namespace dynamics {
constexpr uint8_t Length = 8;
constexpr uint8_t IndexSpeedLow = 0;
constexpr uint8_t IndexSpeedHigh = 1;
constexpr uint8_t IndexAliveNibble = 6;
constexpr uint8_t IndexChecksum = 7;
constexpr uint8_t ChecksumSeed = 0xA0;
constexpr uint8_t ChecksumFinalAdd = 0x01;
constexpr uint8_t ChecksumBytes = 7;
constexpr uint8_t StatusShift = 4;
constexpr uint8_t AlwaysSetBit = 0x80;
constexpr uint8_t AliveShift = 4;
/// The liveness nibble completes a cycle every 300 ms.
constexpr uint32_t AlivePeriodMs = 300;
constexpr uint32_t AliveSteps = 16;
}  // namespace dynamics

/// 0x0CE wheel speeds, eight bytes, 100 ms. Four sixteen bit little endian
/// values, transmitted equal because this firmware models no differential.
namespace wheelSpeed {
constexpr uint8_t Length = 8;
constexpr uint8_t WheelCount = 4;
}  // namespace wheelSpeed

/// 0x1B4 speed and handbrake, eight bytes, 100 ms.
///
/// Byte 1 carries two flag bits that are set in every observed frame, byte 2 a
/// constant, and bytes 5 and 6 two encodings of the same handbrake state that
/// always agree.
namespace brake {
constexpr uint8_t Length = 8;
constexpr uint8_t IndexSpeedLow = 0;
constexpr uint8_t IndexSpeedHigh = 1;
constexpr uint8_t IndexConstant = 2;
constexpr uint8_t IndexCounterHigh = 3;
constexpr uint8_t IndexSpare = 4;
constexpr uint8_t IndexHandbrake = 5;
constexpr uint8_t IndexHandbrakeMirror = 6;
constexpr uint8_t IndexCounter = 7;
constexpr uint8_t SpeedHighFlags = 0xC0;
constexpr uint8_t ConstantValue = 0xE2;
constexpr uint8_t CounterBase = 0xF0;
constexpr uint8_t HandbrakeEngaged = 0x32;
constexpr uint8_t HandbrakeReleased = 0x30;
constexpr uint8_t MirrorEngaged = 0xFE;
constexpr uint8_t MirrorReleased = 0xFC;
}  // namespace brake

/// 0x1D0 engine temperatures, eight bytes, 100 ms.
///
/// Byte 2 combines a liveness counter in the low nibble with the engine run
/// state in the high nibble. Counting the whole byte would sweep the state
/// field through "off" and "invalid", which the cluster answers with warning
/// telltales, so only the low nibble is counted.
namespace engineTemperature {
constexpr uint8_t Length = 8;
constexpr uint8_t IndexCoolant = 0;
constexpr uint8_t IndexOil = 1;
constexpr uint8_t IndexCounterAndState = 2;
constexpr uint8_t IndexFillerFirst = 3;
/// Cumulative injected fuel, little endian, differentiated by the cluster
/// into its instant consumption reading.
constexpr uint8_t IndexInjectionLow = 4;
constexpr uint8_t IndexInjectionHigh = 5;
constexpr uint8_t IndexFillerAfterState = 3;
constexpr uint8_t IndexDistinctFiller = 6;
constexpr uint8_t IndexFillerLast = 7;
/// Fixed values taken from the reference driver. An earlier capture read
/// 0xFF, 0xCC and 0xFF in these positions and the temperatures worked, but
/// the consumption reading did not; these are the values the working driver
/// sends.
constexpr uint8_t FillerAfterStateValue = 0xCD;
constexpr uint8_t FillerLastValue = 0xA8;
constexpr int16_t TemperatureOffset = 48;
/// Observed oil temperature runs below coolant by roughly this margin.
constexpr int16_t OilBelowCoolant = 20;
constexpr uint8_t StateRunning = 0x20;
constexpr uint8_t StateStopped = 0x00;
constexpr uint8_t DistinctFillerValue = 0xCD;
}  // namespace engineTemperature

/// 0x19E stability control status, eight bytes, 200 ms.
/// A status frame that never changes reads as a failed sender, so the counter
/// in byte 7 and the rotating nibble in byte 2 both advance.
namespace stability {
constexpr uint8_t Length = 8;
constexpr uint8_t IndexRotating = 2;
constexpr uint8_t IndexCounter = 7;
constexpr uint8_t Payload1 = 0xE0;
constexpr uint8_t RotatingLowNibble = 0x03;
constexpr uint8_t RotatingModulus = 3;
constexpr uint8_t Payload3 = 0xFC;
constexpr uint8_t Payload4 = 0xCC;
constexpr uint8_t IndexBrakeStatus = 5;
/// Byte 5 carries brake system status. A capture of a healthy vehicle holds
/// 0x00, 0x40, 0x41 or 0x61 here; which bits mean what is not established, so
/// the value is adjustable at run time for bring-up.
constexpr uint8_t BrakeStatusDefault = 0x00;
}  // namespace stability

/// 0x34F park brake status, two bytes. Event driven with a periodic repeat.
namespace parkBrake {
constexpr uint8_t Length = 2;
constexpr uint8_t Engaged = 0xFE;
constexpr uint8_t Released = 0xFD;
}  // namespace parkBrake

/// 0x0C0 anti-lock brake counter, two bytes. Cycles through the high nibble.
namespace absCounter {
constexpr uint8_t Length = 2;
constexpr uint8_t First = 0xF0;
constexpr uint8_t Last = 0xFF;
}  // namespace absCounter

/// 0x0D7 airbag counter, two bytes. Free running.
namespace airbagCounter {
constexpr uint8_t Length = 2;
}  // namespace airbagCounter

/// 0x349 fuel level, five bytes.
///
/// The length is part of the protocol: the frame is defined as five bytes and
/// transmitting eight causes the cluster to discard it, leaving the gauge
/// inert and the low fuel warning active.
namespace fuelLevel {
constexpr uint8_t Length = 5;
constexpr uint8_t IndexLeftLow = 0;
constexpr uint8_t IndexLeftHigh = 1;
constexpr uint8_t IndexRightLow = 2;
constexpr uint8_t IndexRightHigh = 3;
constexpr uint8_t IndexSpare = 4;
}  // namespace fuelLevel

/// 0x39E time and date, eight bytes, 1 s. Sets the cluster clock, which is
/// otherwise blank because nothing on a bench supplies a time.
namespace timeAndDate {
constexpr uint8_t Length = 8;
constexpr uint8_t IndexHour = 0;
constexpr uint8_t IndexMinute = 1;
constexpr uint8_t IndexSecond = 2;
constexpr uint8_t IndexDay = 3;
constexpr uint8_t IndexMonth = 4;
constexpr uint8_t IndexYearLow = 5;
constexpr uint8_t IndexYearHigh = 6;
constexpr uint8_t IndexTrailer = 7;
constexpr uint8_t MonthShift = 4;
constexpr uint8_t MonthLowNibble = 0x0F;
constexpr uint8_t TrailerValue = 0xF2;
}  // namespace timeAndDate

/// 0x21A lighting status, three bytes, 100 ms. Drives the backlight and the
/// lighting telltales.
namespace lighting {
constexpr uint8_t Length = 3;
constexpr uint8_t IndexMask = 0;
constexpr uint8_t IndexSecondary = 1;
constexpr uint8_t IndexSpare = 2;
constexpr uint8_t SecondaryOn = 0x10;
constexpr uint8_t SecondaryOff = 0x00;
}  // namespace lighting

/// 0x202 dimmer level, two bytes. Layout inferred, disabled by default.
namespace dimmer {
constexpr uint8_t Length = 2;
}  // namespace dimmer

/// 0x2C0 display illumination, three bytes. Layout inferred, disabled by
/// default.
namespace lcdLight {
constexpr uint8_t Length = 3;
}  // namespace lcdLight

/// 0x1F6 turn signals, eight bytes, 200 ms.
///
/// Byte 0 encodes the stalk position. Byte 1 is held constant: the cluster
/// runs its own flash timer, and a byte that changes between frames re-phases
/// that timer, which makes the indicators flash unevenly instead of at a
/// steady rate.
namespace turnSignals {
constexpr uint8_t Length = 8;
constexpr uint8_t IndexState = 0;
constexpr uint8_t IndexConstant = 1;
constexpr uint8_t Base = 0x80;
constexpr uint8_t ActiveBit = 0x01;
constexpr uint8_t LeftBit = 0x10;
constexpr uint8_t RightBit = 0x20;
constexpr uint8_t ConstantValue = 0xF0;
}  // namespace turnSignals

/// 0x193 cruise control, eight bytes, 200 ms. Drives the telltale and the set
/// speed readout.
///
/// The database attributes this identifier to the cluster, yet the cluster
/// also accepts it. A bench run showed no transmission failures, indicating no
/// contention in practice; the frame can be disabled if a particular cluster
/// disagrees.
namespace cruise {
constexpr uint8_t Length = 8;
constexpr uint8_t IndexTimer = 0;
constexpr uint8_t IndexSetSpeed = 1;
constexpr uint8_t IndexStatus = 2;
constexpr uint8_t IndexAssistFlags = 3;
constexpr uint8_t IndexAssistExtra = 4;
constexpr uint8_t IndexEnableFlag = 5;
constexpr uint8_t IndexChanged = 6;
constexpr uint8_t IndexSpare = 7;
/// The cluster ignores the frame unless this counter keeps moving.
constexpr uint8_t TimerStep = 17;
constexpr uint8_t SpeedWhenOff = 0xFE;
constexpr uint8_t StatusActiveKmh = 0xF4;
constexpr uint8_t StatusOff = 0xF1;
constexpr uint8_t EnableActive = 0x58;
constexpr uint8_t EnableOff = 0x50;
constexpr uint8_t ChangedFlag = 0x01;
constexpr uint8_t ChangedNone = 0x00;
}  // namespace cruise

/// 0x1D2 gear display, eight bytes. Layout partly inferred.
///
/// Byte 0 carries the lever position in the low nibble and its bitwise
/// complement in the high nibble.
namespace gear {
constexpr uint8_t Length = 8;
constexpr uint8_t IndexPosition = 0;
constexpr uint8_t IndexNumber = 1;
constexpr uint8_t IndexFiller = 2;
constexpr uint8_t IndexCounterAndSport = 3;
constexpr uint8_t IndexMode = 4;
constexpr uint8_t IndexFillerFirst = 5;
constexpr uint8_t NumberShift = 4;
constexpr uint8_t NumberLowNibble = 0x0F;
constexpr uint8_t CounterShift = 4;
constexpr uint8_t SportBit = 0x04;
constexpr uint8_t NoSport = 0x00;
}  // namespace gear

/// 0x3B0 reverse gear, two bytes. Layout inferred, disabled by default.
namespace reverse {
constexpr uint8_t Length = 2;
constexpr uint8_t Engaged = 0x40;
constexpr uint8_t Released = 0x00;
}  // namespace reverse

/// 0x366 outside temperature and remaining range, four bytes.
///
/// The database names the cluster as the sender, so this frame may have no
/// effect. Disabled by default for that reason.
namespace temperatureAndRange {
constexpr uint8_t Length = 4;
constexpr uint8_t IndexTemperature = 0;
constexpr uint8_t IndexRangeLow = 1;
constexpr uint8_t IndexRangeHigh = 2;
constexpr uint8_t IndexSpare = 3;
constexpr int16_t TemperatureOffset = 80;
constexpr int16_t TemperatureScale = 2;
constexpr uint32_t MetresPerKilometre = 1000UL;
constexpr uint32_t MetresPerMile = 1609UL;
constexpr uint16_t RangeScale = 16;
}  // namespace temperatureAndRange

}  // namespace frame
