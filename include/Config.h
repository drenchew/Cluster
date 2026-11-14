#pragma once

/**
 * @file Config.h
 * @brief Compile-time configuration for the BMW E9x instrument cluster driver.
 *
 * Target hardware: Arduino Uno (ATmega328P) with an MCP2515 CAN controller on
 * an 8 MHz crystal, connected to the K-CAN bus of an E9x KOMBI.
 *
 * Every tunable value used by the firmware is declared here so that no literal
 * constant appears in the logic. Values are grouped into nested namespaces by
 * the subsystem that consumes them.
 *
 * Confidence annotations on the frame groups:
 *   [verified]  corroborated by a bus capture and/or a DBC signal definition
 *   [proposed]  layout inferred from documentation only; disabled by default
 */

#include <stdint.h>

namespace cfg {

/// Arduino pin assignments. The Uno's hardware SPI pins are fixed by the MCU.
namespace pins {
constexpr uint8_t ChipSelect = 10;   ///< MCP2515 CS, also the Uno hardware SS
constexpr uint8_t Clock = 13;        ///< SCK, fixed by hardware SPI
constexpr uint8_t MasterOut = 11;    ///< MOSI, fixed by hardware SPI
constexpr uint8_t MasterIn = 12;     ///< MISO, fixed by hardware SPI
constexpr uint8_t Interrupt = 2;     ///< Receive interrupt; diagnostic only
}  // namespace pins

/// SPI link to the CAN controller. This is not the CAN bitrate.
namespace spi {
/// Breadboard wiring is unreliable above about 1 MHz. The controller itself
/// tolerates 10 MHz on a printed circuit board.
constexpr uint32_t ClockHz = 1000000UL;
constexpr uint8_t ReadInstruction = 0x03;   ///< MCP2515 READ opcode
/// Consecutive transmission failures after which the controller is
/// reinitialised. A controller that has entered bus-off stays mute until it
/// is reset, which otherwise means the link never returns after the cluster
/// is powered down and back up. Set high enough that losing arbitration
/// repeatedly on a busy bus cannot trigger it.
constexpr uint32_t FailuresBeforeReset = 64;
constexpr uint8_t StatusRegister = 0x0E;    ///< CANSTAT
/// Values returned by a status read that indicate a specific wiring fault.
constexpr uint8_t NoReply = 0x00;
constexpr uint8_t StuckHigh = 0xFF;
constexpr uint32_t RetryDelayMs = 1000;
}  // namespace spi

/// CAN identifiers transmitted or received by this firmware.
namespace canid {
constexpr uint16_t AbsCounter = 0x0C0;
constexpr uint16_t AirbagCounter = 0x0D7;
constexpr uint16_t DmeStatus = 0x12F;
constexpr uint16_t EngineSpeed = 0x0AA;
constexpr uint16_t WheelSpeeds = 0x0CE;
constexpr uint16_t TerminalStatus = 0x130;
constexpr uint16_t CruiseControl = 0x193;
constexpr uint16_t DynamicsSpeed = 0x1A0;
constexpr uint16_t DistanceAccumulator = 0x1A6;
constexpr uint16_t BrakeAndSpeed = 0x1B4;
constexpr uint16_t EngineTemperature = 0x1D0;
constexpr uint16_t GearDisplay = 0x1D2;
constexpr uint16_t StabilityStatus = 0x19E;
constexpr uint16_t TurnSignals = 0x1F6;
constexpr uint16_t DimmerLevel = 0x202;
constexpr uint16_t LightingStatus = 0x21A;
constexpr uint16_t IgnitionStatus = 0x26E;
constexpr uint16_t LcdIllumination = 0x2C0;
constexpr uint16_t FuelLevel = 0x349;
constexpr uint16_t ParkBrakeStatus = 0x34F;
constexpr uint16_t TemperatureAndRange = 0x366;
constexpr uint16_t ReverseGear = 0x3B0;
constexpr uint16_t TimeAndDate = 0x39E;
constexpr uint16_t CheckControlSymbol = 0x592;

/// Transmitted by the cluster itself, so this firmware only listens.
/// Transmitting either would place two senders behind one identifier and
/// produce bus error frames.
constexpr uint16_t ClusterTemperature = 0x2CA;
constexpr uint16_t ClusterClock = 0x2F8;
}  // namespace canid

/// Enables for each group of transmitted frames.
namespace send {
constexpr bool WarningSuppression = true;  ///< [verified] 0x19E, 0x0C0, 0x0D7
/// 0x12F engine controller status, every 100 ms. Kept enabled even though the
/// consumption reading it was added for is not driven: the cluster expects
/// the engine controller to be reporting, and disabling this frame coincided
/// with a warning appearing on the dial. It is not responsible for the
/// transmission failures this jig records, which measure the same with it on
/// and off.
constexpr bool DmeStatus = true;
constexpr bool Fuel = true;                ///< [verified] 0x349
constexpr bool Coolant = true;             ///< [verified] 0x1D0
constexpr bool Lights = true;              ///< [verified] 0x21A
constexpr bool TurnSignals = true;         ///< [verified] 0x1F6
constexpr bool Cruise = true;              ///< [verified] 0x193
/// 0x1B4 must NOT be transmitted. A receive-identifier dump shows the cluster
/// sending it about ten times a second, and the database names KOMBI as its
/// sender: the cluster owns this frame, including the handbrake bits and the
/// speed word in it. Transmitting it put two senders behind one identifier,
/// which collide bit for bit and generate error frames, and left the cluster
/// being contradicted about its own brake state. Speed reaches the needle
/// through 0x1A0 and 0x0CE, and the handbrake through 0x34F, which the
/// cluster does not transmit.
constexpr bool Brake = false;              ///< 0x1B4
constexpr bool ParkBrake = true;           ///< [verified] 0x34F
constexpr bool Clock = true;               ///< [verified] 0x39E
constexpr bool Symbols = true;             ///< [verified] 0x592
constexpr bool Dimmer = false;             ///< [proposed] 0x202
constexpr bool LcdLight = false;           ///< [proposed] 0x2C0
constexpr bool Gear = false;               ///< [proposed] 0x1D2
constexpr bool Reverse = false;            ///< [proposed] 0x3B0
constexpr bool Range = false;              ///< [proposed] 0x366
}  // namespace send

/// Transmission scheduler. One tick drives a rotating slot table.
namespace schedule {
constexpr uint32_t TickMs = 10;
constexpr uint8_t SlotsPerCycle = 20;      ///< 200 ms cycle
constexpr uint8_t TicksPer100Ms = 10;
constexpr uint8_t TicksPer1Second = 100;
constexpr uint16_t TicksPer5Seconds = 500;

/// Slot numbers within the 200 ms cycle. Each slot carries at most two
/// frames; an eight byte frame occupies the bus for about 1.1 ms at
/// 100 kbit/s, so two per 10 ms tick is roughly 22 percent bus load.
constexpr uint8_t SlotTerminalA = 0;
constexpr uint8_t SlotTerminalB = 10;
constexpr uint8_t SlotDynamicsSpeedA = 1;
constexpr uint8_t SlotDynamicsSpeedB = 11;
constexpr uint8_t SlotLightsA = 2;
constexpr uint8_t SlotLightsB = 12;
constexpr uint8_t SlotIgnition = 3;
constexpr uint8_t SlotBrakeA = 4;
constexpr uint8_t SlotBrakeB = 14;
constexpr uint8_t SlotCoolantA = 5;
constexpr uint8_t SlotCoolantB = 15;
/// The two small suppression counters share a slot; both are two byte frames.
constexpr uint8_t SlotSuppressionCounters = 6;
constexpr uint8_t SlotDmeStatusA = 2;
constexpr uint8_t SlotDmeStatusB = 12;
constexpr uint8_t SlotDistanceA = 7;
constexpr uint8_t SlotDistanceB = 17;
constexpr uint8_t SlotWheelSpeedA = 8;
constexpr uint8_t SlotWheelSpeedB = 18;
constexpr uint8_t SlotFuel = 9;
constexpr uint8_t SlotTurnSignals = 13;
constexpr uint8_t SlotGear = 16;
constexpr uint8_t SlotReverseAndLcd = 19;

/// Phases of the shorter repeating sub-cycles.
constexpr uint8_t PhaseCruise = 11;        ///< within the 200 ms cycle
constexpr uint8_t PhaseSymbols = 5;        ///< within each 100 ms group
constexpr uint8_t PhaseClock = 3;          ///< within each 100 ms group
constexpr uint8_t PhaseRange = 6;          ///< within each 1 s group
constexpr uint8_t PhaseParkBrake = 6;      ///< within each 5 s group
}  // namespace schedule

/// Engine and vehicle limits applied when setpoints are accepted.
namespace limits {
/// A cluster told that terminal 15 is on while the crankshaft reads zero
/// treats the engine as stalled and raises the charge, oil pressure and brake
/// telltales. Engine speed is therefore held at idle while the engine is
/// marked as running.
constexpr uint16_t IdleRpm = 800;
constexpr uint16_t MaxRpm = 7000;
constexpr uint16_t MaxSpeedKmh = 400;
constexpr uint8_t MaxPedalPercent = 100;
constexpr float MinFuelLitres = 0.0f;
constexpr float MaxFuelLitres = 80.0f;
constexpr int16_t MinCoolantC = -40;
constexpr int16_t MaxCoolantC = 150;
constexpr int16_t MinOutsideC = -40;
constexpr int16_t MaxOutsideC = 80;
constexpr uint16_t MaxRangeKm = 2000;
constexpr uint8_t MaxGearNumber = 8;
constexpr uint8_t MaxCruiseKmh = 250;
constexpr bool EngineRunningAtBoot = true;

/// Engaging the handbrake above this speed is treated by the cluster as a
/// brake event and latched: releasing it does not extinguish the warning,
/// which stays lit until the cluster observes a clean release at standstill.
/// Observed on an A2C53117692 at roughly 20 km/h.
constexpr uint16_t HandbrakeLatchSpeedKmh = 20;
/// Separation between the engage and release of the clearing pulse.
constexpr uint32_t HandbrakeClearPulseMs = 250;
}  // namespace limits

/// Values the cluster is driven with before anything supplies setpoints.
/// They are chosen so that a board powered up on its own shows a plausible
/// stationary vehicle rather than a dashboard of warnings.
namespace defaults {
constexpr uint16_t FuelTenthLitres = 300;   ///< 30.0 litres
constexpr int16_t CoolantCelsius = 90;
constexpr int16_t OutsideCelsius = 20;
constexpr uint16_t RangeKm = 500;
constexpr uint8_t Dimmer = 200;
constexpr bool Terminal15 = true;
}  // namespace defaults

/// Speed signal scaling. Each frame carries its own encoding.
namespace speed {
/// 0x1B4 speed word, from the community DBC.
constexpr float BrakeKmhPerCount = 0.103f;
/// 0x1A0 vehicle speed, 0.1 km/h per count. 50 km/h encodes as 0x01F4.
constexpr float DynamicsKmhPerCount = 0.1f;
/// 0x0CE wheel pulses per 100 ms frame. Measured against 0x1A0 over a
/// 1154 frame capture at 1.573 counts per km/h, consistent with 48 pulses per
/// revolution on a 2.10 m tyre circumference.
constexpr float WheelKmhPerCount = 0.636f;
constexpr uint16_t TwelveBitMax = 0x0FFF;

/// Selects which additional speed frames are transmitted alongside 0x1A6.
constexpr uint8_t SourceDynamics = 0x01;   ///< 0x1A0
constexpr uint8_t SourceWheels = 0x02;     ///< 0x0CE
constexpr uint8_t SourceMask = SourceDynamics | SourceWheels;
constexpr uint8_t DefaultSources = SourceDynamics | SourceWheels;
}  // namespace speed

/// 0x1A6 distance accumulator and free running time counter.
namespace odometer {
/// Bytes 0..5 hold three independent 16 bit accumulators, not a repeated
/// speed word. A capture of a moving vehicle shows them climbing
/// monotonically and reading zero when parked. The cluster differentiates
/// them to recover speed, so a constant value yields a stationary needle.
/// Rate measured at 12.3 to 13.7 counts per km/h per second.
constexpr float CountsPerKmhPerSecond = 13.0f;
/// Counts added per frame in Alive mode.
constexpr uint8_t AliveStep = 1;
/// Bytes 6..7 advance by this much per 100 ms frame, wrapping at 12 bits.
constexpr uint16_t TimeCounterStep = 400;
/// Clamp applied to the integration interval after a stall or first call.
constexpr uint32_t MaxIntegrationMs = 500;
}  // namespace odometer

/// 0x349 fuel level. The tank has a sender in each half and the two do not
/// share a transfer curve. Breakpoints are counts at a given fill fraction.
namespace fuel {
constexpr bool UseMeasuredCurve = true;
constexpr float TankLitres = 60.0f;
constexpr float FlatCountsPerLitre = 160.0f;  ///< fallback linear encoding
constexpr uint8_t CurvePoints = 6;
constexpr uint16_t PermilleFullScale = 1000;

/// A real gauge is heavily damped and treats a step change as refuelling.
/// The transmitted level therefore follows the setpoint at a limited rate.
///
/// This is the value used at power-up and whenever the host link is lost. The
/// host can override it per frame, and the console can toggle it, because the
/// gauge is also used to display quantities that change far faster than a
/// fuel level. See FuelSender::setDamping.
constexpr bool EnableSlew = true;
constexpr float SlewFractionPerSecond = 0.015f;
constexpr uint32_t SlewIntervalMs = 50;
}  // namespace fuel

/// 0x1D0 bytes 4 and 5 carry a cumulative count of injected fuel, which the
/// cluster differentiates to drive its instant consumption reading. On higher
/// trim variants that reading is a separate needle; on others it appears on
/// the multi function display.
///
/// Because the value is differentiated, the result depends on the rate of
/// accumulation and therefore on the transmission interval, exactly as the
/// distance accumulator does. The increment below is per 100 ms of elapsed
/// time, and the frame builder scales it by the actual interval so that the
/// reading does not change if the schedule does.
namespace consumption {
constexpr uint8_t MaxPercent = 100;
/// Counts added per 100 ms at full demand. Calibrated on hardware: raise it
/// if the needle reads low across its range, lower it if it pegs.
constexpr float CountsPerPercentPer100Ms = 0.6f;
constexpr uint32_t ReferenceIntervalMs = 100;
}  // namespace consumption

/// 0x39E time and date. A cluster that has never received this frame displays
/// no clock at all.
namespace clock {
constexpr uint32_t SendPeriodMs = 1000;
constexpr uint32_t MillisecondsPerSecond = 1000;
constexpr uint8_t SecondsPerMinute = 60;
constexpr uint8_t MinutesPerHour = 60;
constexpr uint8_t HoursPerDay = 24;
constexpr uint8_t MonthsPerYear = 12;
constexpr uint8_t FebruaryIndex = 2;
constexpr uint8_t FebruaryLeapDays = 29;
constexpr uint8_t FallbackMonthDays = 31;
/// Placeholder shown until the host supplies a time.
constexpr uint8_t BootHour = 12;
constexpr uint8_t BootMinute = 0;
constexpr uint8_t BootDay = 1;
constexpr uint8_t BootMonth = 1;
constexpr uint16_t BootYear = 2026;
}  // namespace clock

/// 0x592 check control symbols. The cluster has no dedicated signal for most
/// telltales; they are numbered symbols raised and cleared by identifier.
namespace symbol {
constexpr uint16_t Dtc = 357;            ///< traction mode indicator
constexpr uint16_t DscTriangle = 215;    ///< stability control intervention
constexpr uint16_t DscOff = 36;          ///< stability control disabled
constexpr uint16_t Service = 281;        ///< service due
constexpr uint16_t CheckEngine = 31;
constexpr uint16_t OilPressure = 212;
constexpr uint16_t Battery = 213;
constexpr uint16_t Overheat = 39;
constexpr uint16_t FuelReserve = 275;
constexpr uint16_t AdBlue = 643;
constexpr uint16_t BrakeFault = 308;
constexpr uint16_t TyrePressure = 50;

/// A displayed symbol is refreshed periodically because the originating
/// control unit repeats it; a cleared one is re-cleared more slowly so that a
/// frame lost to a bus error cannot leave a telltale lit.
constexpr uint16_t OnRefreshMs = 200;
constexpr uint16_t OffRefreshMs = 3000;
/// A cluster removed from a vehicle usually powers up with a service message.
constexpr bool SuppressServiceAtBoot = true;

/// Symbols a cluster can be holding from its life in a vehicle, or from an
/// earlier bench session, and which nothing on this bus would otherwise
/// clear. They are cleared once at startup. The periodic refresh only covers
/// the symbols this firmware manages, so a latched warning outside that set
/// stays displayed indefinitely; a red brake warning left showing after a
/// handbrake test is the case that prompted this.
constexpr uint16_t LatchedAtBoot[] = {
    BrakeFault,     // 308 red brake
    353,            // brakes hot
    41,             // red triangle
    37,             // yellow triangle
    302,            // vehicle can roll away
    244,            // foot on brake
    40,             // start, foot on brake
};
constexpr uint8_t LatchedAtBootCount =
    static_cast<uint8_t>(sizeof(LatchedAtBoot) / sizeof(LatchedAtBoot[0]));
}  // namespace symbol

/// Warning lamp bits in the mask exchanged with the host.
namespace warn {
constexpr uint8_t CheckEngine = 0x01;
constexpr uint8_t OilPressure = 0x02;
constexpr uint8_t Battery = 0x04;
constexpr uint8_t Coolant = 0x08;
constexpr uint8_t FuelReserve = 0x10;
constexpr uint8_t AdBlue = 0x20;
constexpr uint8_t Brake = 0x40;
constexpr uint8_t TyrePressure = 0x80;
}  // namespace warn

/// 0x21A lighting status bits.
namespace lights {
constexpr uint8_t SideLight = 0x01;
constexpr uint8_t HighBeam = 0x02;
constexpr uint8_t LowBeam = 0x04;
constexpr uint8_t FrontFog = 0x20;
constexpr uint8_t RearFog = 0x40;
}  // namespace lights

/// Diagnostic reporting of frames received from the cluster.
namespace feedback {
constexpr bool Enabled = true;
constexpr uint32_t ReportPeriodMs = 5000;
/// A reading is considered current only this long after it arrived.
constexpr uint32_t FreshnessMs = 10000;
/// Values the cluster sends when it has no reading to report.
constexpr uint8_t TemperatureUnavailable = 0xFF;
/// Encoding of 0x2CA byte 0: degrees Celsius in half degree steps, offset -40.
constexpr int16_t TemperatureStepDeciC = 5;
constexpr int16_t TemperatureOffsetDeciC = -400;
}  // namespace feedback

/// Binary link to the host simulator. The firmware always owns CAN timing;
/// the host supplies setpoints only, so losing the link cannot reset the
/// cluster.
namespace host {
constexpr uint32_t TimeoutMs = 1500;
constexpr uint8_t SyncFirst = 0xAA;
constexpr uint8_t SyncSecond = 0x55;
constexpr uint8_t PayloadBytes = 31;   ///< must equal sizeof(HostState)
}  // namespace host

/// Local demonstration sweep, active only while no host is connected.
namespace demo {
constexpr uint16_t MaxRpm = 4500;
constexpr uint16_t MaxSpeedKmh = 120;
constexpr uint16_t RpmPerSecond = 1200;
constexpr uint32_t StartDelayMs = 4000;   ///< cluster power-on needle sweep
constexpr uint32_t IdleDwellMs = 2000;
/// Left off here. A lit brake telltale combined with any road speed makes
/// the cluster sound its warning continuously, and the sweep would raise the
/// speed every time the host disconnects. The tachometer alone is enough to
/// show that the board is alive.
constexpr bool SweepSpeed = false;
}  // namespace demo

/// Text console shared with the binary host link.
namespace console {
constexpr uint32_t ReadTimeoutMs = 20;
constexpr uint8_t MaxLineLength = 24;
}  // namespace console

}  // namespace cfg
