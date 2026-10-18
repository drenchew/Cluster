/*
 * BMW E90/E91/E92/E93 instrument cluster bench jig
 * Arduino Uno (ATmega328P) + HW-184 (MCP2515, 8 MHz) -> K-CAN @ 100 kbit/s
 *
 * Two ways to drive it, both on the same USB serial port:
 *
 *   1. The PC simulator in host/ - binary setpoint frames at 50 Hz, with a
 *      keyboard-driven vehicle model, dashboard and engine audio.
 *   2. Typed commands, for poking at one thing without the GUI:
 *        r 3000  RPM           s 100  speed km/h      p 50   pedal %
 *        f 35    fuel litres   c 90   coolant degC    t 20   outside degC
 *        l 5     lights mask   b 200  dimmer 0..255   g 8    gear 1P 2R 4N 8D
 *        n 1     turn: 0 off 1 left 2 right 3 hazard
 *        h 1     handbrake     e 0|1  engine          k 0|1  terminal 15
 *        o 0|1|2 0x1A6 distance counter: frozen / alive / distance
 *        x 0..3  extra speed frames: +1 = 0x1A0, +2 = 0x0CE
 *        d       resume demo sweep    ?      print state
 *
 * The firmware always owns CAN timing. If the host goes quiet the heartbeat
 * keeps running and the demo sweep takes over, so closing the GUI or pulling
 * the USB cable can never reset the cluster.
 *
 * Wiring (Uno -> HW-184): CS=D10, SCK=D13, SI=D11, SO=D12, INT=D2,
 * VCC=5V, GND=GND.  Cluster: CAN H->pin6, CAN L->pin7, +12V->pin9, GND->pin18.
 */
#include <Arduino.h>
#include "Config.h"
#include "ClusterCan.h"
#include "Protocol.h"
#include "Demo.h"

static ClusterCan cluster;
static Demo       demo;
static Protocol   host;

static char    line_[cfg::SERIAL_LINE_MAX];
static uint8_t lineLen_  = 0;
static bool    hostWas_  = false;

static void printState() {
  ClusterCan::Stats s = cluster.stats();
  logf("rpm %u spd %u lights 0x%02X dim %u hb %u eng %u kl15 %u\n",
       cluster.rpm(), cluster.speed(), cluster.lights(), cluster.dimmer(),
       (unsigned)cluster.handbrake(), (unsigned)cluster.engineRunning(),
       (unsigned)cluster.terminal15());
  logf("0x1A6: odo mode %u dist 0x%04X | extra speed 0x%02X\n",
       (unsigned)cluster.odoMode(), (unsigned)cluster.odoCounter(),
       (unsigned)cluster.speedSources());
  logf("tx %lu fail %lu rx %lu | host %s frames %lu err %lu\n",
       (unsigned long)s.txOk, (unsigned long)s.txFail, (unsigned long)s.rx,
       host.hostLive() ? "live" : "idle",
       (unsigned long)host.frames(), (unsigned long)host.errors());
}

static void runCommand(char c, const char *arg) {
  float v = atof(arg);
  switch (c) {
    case 'r':
      demo.stop(); cluster.setRPM((uint16_t)v);
      logf("rpm %u\n", cluster.rpm());
      if (cluster.engineRunning() && v < cfg::ENGINE_IDLE_RPM)
        logf("  floored at idle %u; 'e 0' for a real stall\n",
             (unsigned)cfg::ENGINE_IDLE_RPM);
      break;
    case 's': demo.stop(); cluster.setSpeed((uint16_t)v); logf("spd %u\n", cluster.speed()); break;
    case 'p': cluster.setPedal((uint8_t)v);               logf("pedal %u%%\n", (unsigned)v); break;
    case 'f':
      cluster.setFuelLitres(v);
      Serial.print(F("fuel "));
      printFix1((uint16_t)(constrain(v, 0.0f, 80.0f) * 10.0f + 0.5f));
      Serial.println(F(" l"));
      break;
    case 'c': cluster.setCoolantC((int16_t)v); logf("coolant %d C\n", (int)v); break;
    case 't': cluster.setOutsideC((int16_t)v); logf("outside %d C\n", (int)v); break;
    case 'l': cluster.setLights((uint8_t)v);   logf("lights 0x%02X\n", (unsigned)(uint8_t)v); break;
    case 'b': cluster.setDimmer((uint8_t)v);   logf("dimmer %u\n", (unsigned)(uint8_t)v); break;
    case 'g': cluster.setGear((uint8_t)v, 0, MODE_NORMAL); logf("gear %u\n", (unsigned)v); break;
    case 'n': {
      uint8_t t = (uint8_t)v;
      cluster.setTurn(t & 0x01, t & 0x02);
      logf("turn %u\n", (unsigned)t);
      break;
    }
    case 'h': cluster.setHandbrake(v != 0.0f);
              Serial.println(v != 0.0f ? F("handbrake on") : F("handbrake off")); break;
    case 'e': cluster.setEngineRunning(v != 0.0f);
              Serial.println(v != 0.0f ? F("engine running")
                                       : F("engine off (key on) - cluster will warn")); break;
    case 'k': cluster.setTerminal15(v != 0.0f);
              Serial.println(v != 0.0f ? F("terminal 15 on") : F("terminal 15 off")); break;
    case 'o': cluster.setOdoMode((uint8_t)v);
              logf("odo mode %u (0 frozen, 1 alive, 2 distance)\n",
                   (unsigned)cluster.odoMode()); break;
    case 'x': cluster.setSpeedSources((uint8_t)v);
              logf("extra speed frames: %s%s%s\n",
                   (cluster.speedSources() & cfg::SPEED_SRC_1A0) ? "0x1A0 " : "",
                   (cluster.speedSources() & cfg::SPEED_SRC_0CE) ? "0x0CE " : "",
                   cluster.speedSources() ? "" : "none (0x1A6 only)"); break;
    case 'd': demo.resume(); Serial.println(F("demo sweep resumed")); break;
    case '?': printState(); break;
    default: break;
  }
}

/* Line-buffered and non-blocking, and it shares the port with the binary host
 * link: Protocol::feed() claims a byte only once it has seen 0xAA, and ASCII
 * never starts a line with 0xAA, so typed commands still get through.
 *
 * The old version called Serial.parseFloat() straight after reading the
 * command letter, which blocks for the whole stream timeout (1 s by default)
 * whenever no digits follow - on every 'd', and again on every trailing
 * newline. A 1 s stall in loop() stops the 0x130 heartbeat and 0x0AA/0x1A6
 * dead, which on its own is enough to make the cluster reset. */
static void handleSerial() {
  while (Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();
    if (host.feed(b, cluster)) continue;          // consumed by the binary link

    // While the GUI is driving - or a binary frame was cut short a moment ago
    // - ignore the text console entirely. Binary payload bytes can take any
    // value, so the leftovers of a truncated frame would otherwise spill into
    // the line parser and run whatever command those bytes happened to spell.
    if (host.hostLive() || host.binaryRecent()) { lineLen_ = 0; continue; }

    char ch = (char)b;
    if (ch == '\r') continue;
    if (ch == '\n') {
      line_[lineLen_] = '\0';
      uint8_t i = 0;
      while (line_[i] == ' ' || line_[i] == '\t') i++;
      if (line_[i]) {
        char cmd = line_[i++];
        while (line_[i] == ' ' || line_[i] == '\t') i++;
        runCommand(cmd, &line_[i]);
      }
      lineLen_ = 0;
      continue;
    }
    if (lineLen_ < cfg::SERIAL_LINE_MAX - 1) line_[lineLen_++] = ch;
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(cfg::SERIAL_TIMEOUT_MS);
  delay(300);
  Serial.println(F("\nE90 cluster bench driver (Uno, K-CAN 100 kbit/s, MCP2515 8 MHz)"));

  if (!cluster.begin()) {
    Serial.println(F("CAN init FAILED - check SPI wiring and 5 V on the HW-184. Halting."));
    while (true) delay(1000);
  }
  cluster.setEngineRunning(cfg::ENGINE_RUNNING_AT_BOOT);
  cluster.setLights(cfg::LIGHT_SIDE | cfg::LIGHT_LOW);   // backlight on at boot

  Serial.println(F("TX running: 0x130/0x1A0/0x1B4 @100ms, 0x1A6 @100ms, 0x0AA @20ms"));
  logf("idle %u rpm, 0x130 B4 0x%02X, odo mode %u, 0x1A0 %s\n",
       (unsigned)cfg::ENGINE_IDLE_RPM, cfg::T130_BYTE4_ON,
       (unsigned)cfg::ODO_MODE,
       (cfg::SPEED_SOURCES & cfg::SPEED_SRC_1A0) ? "on" : "off");
  Serial.println(F("waiting for host; type ? for state, d for demo"));
}

void loop() {
  handleSerial();

  // The host owns the setpoints while it is alive; the demo takes over the
  // moment it goes quiet, so the needles never freeze at whatever the host
  // last said and the cluster always sees a plausible running engine.
  bool live = host.hostLive();
  if (live != hostWas_) {
    hostWas_ = live;
    if (live) { demo.stop();   Serial.println(F("host connected")); }
    else      { demo.resume(); Serial.println(F("host lost - demo sweep")); }
  }

  demo.update(cluster);
  cluster.update();                       // paces the CAN frames off millis()

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 5000) {
    lastPrint = millis();
    if (!live) printState();
  }
}
