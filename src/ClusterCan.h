#pragma once
/*
 * BMW E90/E91/E92/E93 instrument cluster bench jig - Arduino Uno build
 *
 * Uno (ATmega328P) + HW-184 (MCP2515, 8 MHz) -> K-CAN @ 100 kbit/s.
 *
 * Unlike the ESP32 version there is no FreeRTOS task: update() is called from
 * loop() and paces the frames off millis(). Keep loop() short - nothing in
 * loop() may block, or the 100 ms 0x130 heartbeat slips and the cluster
 * restarts. (That is why the serial console is line-buffered rather than
 * using the blocking Serial.parseFloat().)
 *
 * This class owns the whole cluster state. The PC simulator only pushes
 * setpoints into it; if the host disappears the frames keep flowing.
 */
#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include "Config.h"

// Shift lever positions, as the cluster encodes them in 0x1D2.
enum GearPos : uint8_t { GEAR_OFF = 0, GEAR_P = 1, GEAR_R = 2, GEAR_N = 4, GEAR_D = 8 };
enum GearMode : uint8_t { MODE_NORMAL = 0, MODE_SPORT = 1, MODE_MANUAL = 2 };

class ClusterCan {
public:
  struct Stats { uint32_t txOk, txFail, rx; };

  bool begin();                         // init MCP2515 (retries until wired up)
  void update();                        // call from loop(): sends due frames

  // ---- drivetrain --------------------------------------------------------
  void setRPM(uint16_t rpm);            // engine speed, rpm
  void setSpeed(uint16_t kmh);          // vehicle speed, km/h
  void setPedal(uint8_t percent);       // accelerator 0..100 -> 0x0AA
  void setGear(uint8_t pos, uint8_t num, uint8_t mode);
  void setReverse(bool on);

  // ---- engine / ignition -------------------------------------------------
  void setEngineRunning(bool running);  // false = key on, engine off (stall)
  void setTerminal15(bool on);          // false = KL15 off, cluster shuts down

  // ---- gauges ------------------------------------------------------------
  void setFuelLitres(float litres);     // 0..80 l
  void setCoolantC(int16_t celsius);
  void setOutsideC(int16_t celsius);
  void setRangeKm(uint16_t km);

  // ---- body / lighting ---------------------------------------------------
  void setLights(uint8_t mask);         // cfg::LIGHT_* bits
  void setDimmer(uint8_t level);        // 0..255 instrument illumination
  void setTurn(bool left, bool right);  // both = hazards
  void setHandbrake(bool on);

  // ---- 0x1A6 bring-up knobs, switchable at runtime -----------------------
  // So the speedometer can be diagnosed without reflashing between tries.
  void setOdoMode(uint8_t mode);        // cfg::ODO_FROZEN / ALIVE / DISTANCE
  void setSpeedSources(uint8_t mask);   // cfg::SPEED_SRC_* extra speed frames

  // ---- getters -----------------------------------------------------------
  uint16_t rpm()   const { return rpm_; }
  uint16_t speed() const { return speedKmh_; }
  uint8_t  lights()    const { return lights_; }
  uint8_t  dimmer()    const { return dimmer_; }
  uint8_t  odoMode()   const { return odoMode_; }
  uint16_t odoCounter()const { return dist1A6_; }
  uint8_t  speedSources() const { return speedSrc_; }
  bool     handbrake() const { return handbrake_; }
  bool     engineRunning() const { return engineOn_; }
  bool     terminal15()    const { return kl15_; }
  Stats    stats() const { return {txOk_, txFail_, rx_}; }

private:
  uint8_t rawReadReg(uint8_t addr);     // diagnostic raw SPI read
  bool    tryInit();
  bool    send(uint32_t id, const uint8_t *data, uint8_t len);
  void    drainRx();
  uint16_t speedRaw() const;            // km/h -> 12-bit 0x1B4/0x0CE word

  // ---- frame builders ----------------------------------------------------
  void send130();   // 0x130 terminal status
  void send26E();   // 0x26E ignition status
  void send0AA();   // 0x0AA RPM + pedal
  void send1A6();   // 0x1A6 distance accumulator + time counter
  void send1A0();   // 0x1A0 "Speed K-CAN" (DSC speed, the needle's source)
  void send0CE();   // 0x0CE wheel speeds   (alternative speed source)
  void send1B4();   // 0x1B4 speed + handbrake
  void send1D0();   // 0x1D0 coolant
  void send19E();   // 0x19E DSC status
  void send0C0();   // 0x0C0 ABS counter
  void send0D7();   // 0x0D7 airbag counter
  void send349();   // 0x349 fuel level
  void send21A();   // 0x21A lighting status / backlight
  void send202();   // 0x202 dimmer level
  void send2C0();   // 0x2C0 LCD illumination
  void send1F6();   // 0x1F6 turn signals
  void send1D2();   // 0x1D2 gear display
  void send3B0();   // 0x3B0 reverse gear
  void send366();   // 0x366 outside temp + range

  MCP2515 mcp_{cfg::PIN_CS, cfg::SPI_HZ};

  // ---- shared values (written from loop, read by update) -----------------
  volatile uint16_t rpm_       = 0;
  volatile uint16_t speedKmh_  = 0;
  volatile uint16_t fuelDl_    = 300;   // 0.1 l units
  volatile int16_t  coolantC_  = 90;
  volatile int16_t  outsideC_  = 20;
  volatile uint16_t rangeKm_   = 500;
  volatile uint8_t  pedalPct_  = 0;
  volatile uint8_t  dimmer_    = 200;
  volatile uint8_t  lights_    = 0;     // cfg::LIGHT_* bitmask
  volatile uint8_t  turn_      = 0;     // bit0 left, bit1 right
  volatile uint8_t  gearPos_   = GEAR_P;
  volatile uint8_t  gearNum_   = 0;     // 1..6, 0 = blank
  volatile uint8_t  gearMode_  = MODE_NORMAL;
  volatile bool     handbrake_ = true;
  volatile bool     reverse_   = false;
  volatile bool     engineOn_  = cfg::ENGINE_RUNNING_AT_BOOT;
  volatile bool     kl15_      = true;
  volatile uint8_t  odoMode_    = cfg::ODO_MODE;
  volatile uint8_t  speedSrc_   = cfg::SPEED_SOURCES;
  volatile uint32_t txOk_ = 0, txFail_ = 0, rx_ = 0;

  // ---- scheduler + rolling counters --------------------------------------
  uint32_t lastTick_ = 0;
  uint32_t tick_     = 0;
  uint8_t  cnt130_ = 0, cnt0AA_ = 0, cnt1D0_ = 0, cnt0C0_ = 0xF0, cnt0D7_ = 0;
  uint8_t  cnt1B4_ = 0, cnt1D2_ = 0;
  uint8_t  lastTurn_ = 0;       // to emit the 0x1F6 "new blink" byte once
  uint16_t dist1A6_   = 0;      // 0x1A6 B0..B5 cumulative distance accumulator
  float    distAccum_ = 0.0f;   // sub-count remainder, so low speeds still add up
  uint32_t lastDist_  = 0;      // millis() at the last 0x1A6, for dt
  uint16_t tick1A6_   = 0;      // 0x1A6 B6..B7 12-bit time counter (+400/frame)
};
