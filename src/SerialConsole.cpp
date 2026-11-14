#include "SerialConsole.h"

#include <Arduino.h>
#include <stdlib.h>

#include "Diagnostics.h"

namespace {

/// Command letters accepted by the console.
constexpr char CmdEngineSpeed = 'r';
constexpr char CmdRoadSpeed = 's';
constexpr char CmdPedal = 'p';
constexpr char CmdFuel = 'f';
constexpr char CmdCoolant = 'c';
constexpr char CmdOutside = 't';
constexpr char CmdLights = 'l';
constexpr char CmdDimmer = 'b';
constexpr char CmdGear = 'g';
constexpr char CmdTurn = 'n';
constexpr char CmdHandbrake = 'h';
constexpr char CmdEngineRunning = 'e';
constexpr char CmdTerminal15 = 'k';
constexpr char CmdOdometerMode = 'o';
constexpr char CmdSpeedSources = 'x';
constexpr char CmdClock = 'w';
constexpr char CmdTractionLamps = 'u';
constexpr char CmdWarnings = 'm';
constexpr char CmdCruise = 'v';
constexpr char CmdSymbol = 'y';
constexpr char CmdDemo = 'd';
constexpr char CmdConsumption = 'i';
constexpr char CmdInjectionRate = 'j';
constexpr char CmdBrakeStatus = 'z';
constexpr char CmdFuelDamping = 'a';
constexpr char CmdStatus = '?';

/// Bit positions used by the traction lamp and turn signal commands.
constexpr uint8_t BitFirst = 0x01;
constexpr uint8_t BitSecond = 0x02;
constexpr uint8_t BitThird = 0x04;

/// Time command parsing. Values above this are read as HHMMSS.
constexpr uint32_t TimeWithSecondsThreshold = 9999;
constexpr uint32_t HoursDivisorWithSeconds = 10000;
constexpr uint32_t HoursDivisor = 100;
constexpr uint32_t MinutesDivisor = 100;

constexpr float TenthsPerUnit = 10.0f;
constexpr float RoundingOffset = 0.5f;
constexpr int NumericBaseAuto = 0;

bool isBlank(char value) {
    return (value == ' ') || (value == '\t');
}

}  // namespace

SerialConsole::SerialConsole(ClusterCan &cluster, DemoSweep &demo, HostProtocol &host)
    : cluster_(cluster), demo_(demo), host_(host) {
}

void SerialConsole::printBanner() const {
    Serial.println(F("\nE90 cluster bench driver (Uno, K-CAN 100 kbit/s, MCP2515 8 MHz)"));
    Serial.println(F("TX running: 0x130/0x1A0/0x1B4 @100ms, 0x1A6 @100ms, 0x0AA @20ms"));
    diag::printf("idle %u rpm, 0x130 B4 0x%02X, odo mode %u, 0x1A0 %s\n",
                static_cast<unsigned>(cfg::limits::IdleRpm),
                frame::terminal::PackedIgnitionOn,
                static_cast<unsigned>(cluster_.state().odometerMode()),
                cluster_.state().usesSpeedSource(cfg::speed::SourceDynamics) ? "on" : "off");
    Serial.println(F("waiting for host; type ? for state, d for demo"));
}

void SerialConsole::printStatus() const {
    const VehicleState &state = cluster_.state();
    const ICanBus::Statistics stats = cluster_.statistics();

    diag::printf("rpm %u spd %u lights 0x%02X dim %u hb %u eng %u kl15 %u\n",
                state.engineSpeed(), state.roadSpeed(), state.lightMask(), state.dimmer(),
                static_cast<unsigned>(state.handbrake()),
                static_cast<unsigned>(state.engineRunning()),
                static_cast<unsigned>(state.terminal15()));
    diag::printf("0x1A6: odo mode %u dist 0x%04X | extra speed 0x%02X\n",
                static_cast<unsigned>(state.odometerMode()),
                static_cast<unsigned>(cluster_.odometerCounter()),
                static_cast<unsigned>(state.speedSources()));
    diag::printf("clock %02u:%02u:%02u %s\n", static_cast<unsigned>(cluster_.clock().hour()),
                static_cast<unsigned>(cluster_.clock().minute()),
                static_cast<unsigned>(cluster_.clock().second()),
                cluster_.clock().isValid() ? "(sent on 0x39E)"
                                           : "(never set - no 0x39E yet)");
    diag::printf("fuel gauge %s\n",
                 cluster_.fuelDamping() ? "damped (tank)" : "direct (live)");
    diag::printf("tx %lu fail %lu rx %lu | host %s frames %lu err %lu\n",
                static_cast<unsigned long>(stats.transmitted),
                static_cast<unsigned long>(stats.failed),
                static_cast<unsigned long>(stats.received),
                host_.hostLive() ? "live" : "idle",
                static_cast<unsigned long>(host_.frames()),
                static_cast<unsigned long>(host_.errors()));
}

void SerialConsole::printClusterFeedback() const {
    const ClusterFeedback &feedback = cluster_.feedback();
    const uint32_t now = millis();
    const bool haveTemperature = feedback.temperatureFresh(now);
    const bool haveClock = feedback.clockFresh(now);

    if (haveTemperature || haveClock) {
        Serial.print(F("kombi:"));
        if (haveTemperature) {
            if (!feedback.temperatureValid()) {
                // The cluster reports no reading at all, which on a bench
                // means nothing is connected across its sensor pins.
                Serial.print(F(" outside -- (sensor open on pins 4/5)"));
            } else {
                int16_t tenths = feedback.temperatureDeciCelsius();
                Serial.print(F(" outside "));
                if (tenths < 0) {
                    Serial.write('-');
                    tenths = static_cast<int16_t>(-tenths);
                }
                diag::printFixed1(static_cast<uint16_t>(tenths));
                Serial.print(F("C"));
            }
        }
        if (haveClock) {
            if (!feedback.clockValid()) {
                Serial.print(F(" clock not set (send 0x39E: 'w 1435')"));
            } else {
                diag::printf(" clock %02u:%02u:%02u",
                            static_cast<unsigned>(feedback.clockHour()),
                            static_cast<unsigned>(feedback.clockMinute()),
                            static_cast<unsigned>(feedback.clockSecond()));
            }
        }
        Serial.println();
    }
}

void SerialConsole::applyTimeCommand(const char *argument) {
    const uint32_t digits = static_cast<uint32_t>(atol(argument));
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;

    // Parsed as HHMM or HHMMSS so that "w 0905" reads as five past nine.
    if (digits > TimeWithSecondsThreshold) {
        hour = static_cast<uint8_t>(digits / HoursDivisorWithSeconds);
        minute = static_cast<uint8_t>((digits / MinutesDivisor) % MinutesDivisor);
        second = static_cast<uint8_t>(digits % MinutesDivisor);
    } else {
        hour = static_cast<uint8_t>(digits / HoursDivisor);
        minute = static_cast<uint8_t>(digits % MinutesDivisor);
    }

    cluster_.setClock(hour, minute, second, cluster_.clock().day(),
                      cluster_.clock().month(), cluster_.clock().year());
    diag::printf("clock %02u:%02u:%02u\n", static_cast<unsigned>(cluster_.clock().hour()),
                static_cast<unsigned>(cluster_.clock().minute()),
                static_cast<unsigned>(cluster_.clock().second()));
}

void SerialConsole::applySymbolCommand(const char *argument) {
    char *rest = nullptr;
    const uint16_t identifier =
        static_cast<uint16_t>(strtoul(argument, &rest, NumericBaseAuto));
    const bool on = (rest != nullptr) && (strtol(rest, nullptr, NumericBaseAuto) != 0);
    cluster_.setSymbol(identifier, on);
    diag::printf("symbol %u %s\n", static_cast<unsigned>(identifier), on ? "on" : "off");
}

void SerialConsole::execute(char command, const char *argument) {
    VehicleState &state = cluster_.state();
    const float value = atof(argument);
    const bool truthy = (value != 0.0f);

    switch (command) {
        case CmdEngineSpeed:
            demo_.stop();
            state.setEngineSpeed(static_cast<uint16_t>(value));
            diag::printf("rpm %u\n", state.engineSpeed());
            if (state.engineRunning() && value < cfg::limits::IdleRpm) {
                diag::printf("  floored at idle %u; 'e 0' for a real stall\n",
                            static_cast<unsigned>(cfg::limits::IdleRpm));
            }
            break;

        case CmdRoadSpeed:
            demo_.stop();
            state.setRoadSpeed(static_cast<uint16_t>(value));
            diag::printf("spd %u\n", state.roadSpeed());
            break;

        case CmdPedal:
            state.setPedalPercent(static_cast<uint8_t>(value));
            diag::printf("pedal %u%%\n", static_cast<unsigned>(state.pedalPercent()));
            break;

        case CmdFuel:
            state.setFuelLitres(value);
            Serial.print(F("fuel "));
            diag::printFixed1(
                static_cast<uint16_t>(state.fuelLitres() * TenthsPerUnit + RoundingOffset));
            Serial.println(F(" l"));
            break;

        case CmdCoolant:
            state.setCoolantCelsius(static_cast<int16_t>(value));
            diag::printf("coolant %d C\n", static_cast<int>(state.coolantCelsius()));
            break;

        case CmdOutside:
            state.setOutsideCelsius(static_cast<int16_t>(value));
            diag::printf("outside %d C\n", static_cast<int>(state.outsideCelsius()));
            break;

        case CmdLights:
            state.setLightMask(static_cast<uint8_t>(value));
            diag::printf("lights 0x%02X\n", static_cast<unsigned>(state.lightMask()));
            break;

        case CmdDimmer:
            state.setDimmer(static_cast<uint8_t>(value));
            diag::printf("dimmer %u\n", static_cast<unsigned>(state.dimmer()));
            break;

        case CmdGear:
            state.setGear(static_cast<GearPosition>(static_cast<uint8_t>(value)), 0,
                          GearMode::Normal);
            diag::printf("gear %u\n", static_cast<unsigned>(state.gearPosition()));
            break;

        case CmdTurn: {
            const uint8_t bits = static_cast<uint8_t>(value);
            state.setTurnSignal((bits & BitFirst) != 0, (bits & BitSecond) != 0);
            diag::printf("turn %u\n", static_cast<unsigned>(bits));
            break;
        }

        case CmdHandbrake:
            state.setHandbrake(truthy);
            Serial.println(truthy ? F("handbrake on") : F("handbrake off"));
            break;

        case CmdEngineRunning:
            state.setEngineRunning(truthy);
            Serial.println(truthy ? F("engine running")
                                  : F("engine off (key on) - cluster will warn"));
            break;

        case CmdTerminal15:
            state.setTerminal15(truthy);
            Serial.println(truthy ? F("terminal 15 on") : F("terminal 15 off"));
            break;

        case CmdOdometerMode:
            cluster_.setOdometerMode(static_cast<OdometerMode>(static_cast<uint8_t>(value)));
            diag::printf("odo mode %u (0 frozen, 1 alive, 2 distance)\n",
                        static_cast<unsigned>(state.odometerMode()));
            break;

        case CmdSpeedSources:
            state.setSpeedSources(static_cast<uint8_t>(value));
            diag::printf("extra speed frames: %s%s%s\n",
                        state.usesSpeedSource(cfg::speed::SourceDynamics) ? "0x1A0 " : "",
                        state.usesSpeedSource(cfg::speed::SourceWheels) ? "0x0CE " : "",
                        (state.speedSources() != 0) ? "" : "none (0x1A6 only)");
            break;

        case CmdClock:
            applyTimeCommand(argument);
            break;

        case CmdTractionLamps: {
            const uint8_t bits = static_cast<uint8_t>(value);
            cluster_.setTractionLamps((bits & BitFirst) != 0, (bits & BitSecond) != 0,
                                      (bits & BitThird) != 0);
            diag::printf("traction lamps: DTC %u DSC %u DSC-off %u\n",
                        static_cast<unsigned>((bits & BitFirst) != 0),
                        static_cast<unsigned>((bits & BitSecond) != 0),
                        static_cast<unsigned>((bits & BitThird) != 0));
            break;
        }

        case CmdWarnings: {
            // The legend for these bits is in the banner rather than here,
            // because a format string is held in RAM on this target.
            const uint8_t mask = static_cast<uint8_t>(value);
            cluster_.setWarnings(mask);
            diag::printf("warn 0x%02X\n", static_cast<unsigned>(mask));
            break;
        }

        case CmdCruise:
            cluster_.setCruise(static_cast<uint8_t>(value));
            if (value > 0) {
                diag::printf("cruise %u km/h\n", static_cast<unsigned>(value));
            } else {
                Serial.println(F("cruise off"));
            }
            break;

        case CmdSymbol:
            applySymbolCommand(argument);
            break;

        case CmdDemo:
            demo_.resume();
            Serial.println(F("demo sweep resumed"));
            break;

        case CmdConsumption:
            cluster_.state().setConsumptionPercent(static_cast<uint8_t>(value));
            diag::printf("consumption demand %u%%\n",
                         static_cast<unsigned>(state.consumptionPercent()));
            break;

        case CmdInjectionRate:
            cluster_.setInjectionRateOverride(value);
            if (value >= 0.0f) {
                diag::printf("injection rate forced to %ld counts/100ms\n",
                             static_cast<long>(value));
            } else {
                Serial.println(F("injection rate back to the demand percentage"));
            }
            break;

        case CmdBrakeStatus:
            cluster_.setBrakeStatus(static_cast<uint8_t>(value));
            diag::printf("0x19E brake status byte = 0x%02X\n",
                         static_cast<unsigned>(cluster_.brakeStatus()));
            break;

        case CmdFuelDamping:
            cluster_.setFuelDamping(truthy);
            Serial.println(truthy ? F("fuel gauge damped (tank behaviour)")
                                  : F("fuel gauge direct (live measurement)"));
            break;

        case CmdStatus:
            printStatus();
            break;

        default:
            break;
    }
}

void SerialConsole::dispatchLine() {
    line_[lineLength_] = '\0';
    uint8_t index = 0;
    while (isBlank(line_[index])) {
        ++index;
    }
    if (line_[index] != '\0') {
        const char command = line_[index];
        ++index;
        while (isBlank(line_[index])) {
            ++index;
        }
        execute(command, &line_[index]);
    }
    lineLength_ = 0;
}

void SerialConsole::poll() {
    while (Serial.available() > 0) {
        const uint8_t received = static_cast<uint8_t>(Serial.read());

        if (!host_.feed(received, cluster_)) {
            if (host_.hostLive() || host_.binaryRecent()) {
                lineLength_ = 0;
            } else {
                const char character = static_cast<char>(received);
                if (character == '\n') {
                    dispatchLine();
                } else if (character != '\r') {
                    if (lineLength_ < (cfg::console::MaxLineLength - 1)) {
                        line_[lineLength_] = character;
                        ++lineLength_;
                    }
                }
            }
        }
    }
}
