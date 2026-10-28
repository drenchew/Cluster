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

  // ---- DSC / traction control -------------------------------------------
  // These are check-control symbols on 0x592, not dedicated frames: see
  // cfg::sym. tcLamp is the "DTC"/traction-mode lamp, dscLamp is the DSC
  // triangle the real car flashes while it is actually intervening (the host
  // does the flashing - it knows when an intervention started), dscOff is the
  // solid lamp for DSC switched off.
  void setTractionLamps(bool tcLamp, bool dscLamp, bool dscOff);
  void setSymbol(uint16_t id, bool on);   // any id from the E92 symbol list

  // ---- clock -------------------------------------------------------------
  // A cluster that was never told the time shows no clock at all. Seconds are
  // advanced locally off millis() between host updates, so the clock keeps
  // running (and keeps being sent) even with the USB cable out.
  void setClock(uint8_t hour, uint8_t minute, uint8_t second,
                uint8_t day, uint8_t month, uint16_t year);

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
  bool     clockValid() const { return clockValid_; }
  uint8_t  clockHour() const { return hour_; }
  uint8_t  clockMin()  const { return min_; }
  uint8_t  clockSec()  const { return sec_; }
  uint8_t  clockDay()  const { return day_; }
  uint8_t  clockMonth()const { return month_; }
  uint16_t clockYear() const { return year_; }
  // What the cluster itself last broadcast, or absent if it has not. The
  // temperature is in tenths of a degree C so AVR printf (which has no %f)
  // can print it; see cfg::REPORT_CLUSTER_RX.
  bool     kombiTempKnown() const {
    return kombiTempSeen_ && (millis() - kombiTempMs_) < KOMBI_RX_FRESH_MS;
  }
  // The cluster broadcasts 0xFF for "no reading" - which is what an open
  // sensor across pins 4 and 5 gives, and the reason the display sits at the
  // bottom of its scale. Decoded it would be a nonsensical +87.5 C.
  bool     kombiTempValid() const { return kombiTempRaw_ != 0xFF; }
  // Likewise 0xFD on 0x2F8: the clock has never been set.
  bool     kombiClockValid() const {
    return kombiH_ < 24 && kombiM_ < 60 && kombiS_ < 60;
  }
  int16_t  kombiTempDeciC() const { return kombiTempDeciC_; }
  bool     kombiClockKnown() const {
    return kombiClockSeen_ && (millis() - kombiClockMs_) < KOMBI_RX_FRESH_MS;
  }
  void     kombiClock(uint8_t &h, uint8_t &m, uint8_t &s2) const {
    h = kombiH_; m = kombiM_; s2 = kombiS_;
  }
  bool     terminal15()    const { return kl15_; }
  Stats    stats() const { return {txOk_, txFail_, rx_}; }

private:
  // A cluster that has been switched off stops broadcasting; its last reading
  // must not go on being reported as current.
  static const uint32_t KOMBI_RX_FRESH_MS = 10000;

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
  void send34F();   // 0x34F handbrake status (2 bytes, event + 5 s)
  void send39E();   // 0x39E time + date (sets the cluster clock)
  void send592(uint16_t id, bool on);   // 0x592 check-control symbol
  void pumpSymbols();                   // one due 0x592 frame per call
  void tickClock(uint32_t now);         // advance the local clock
  void slewFuel(uint32_t now);          // damp host fuel steps, like a tank
  uint16_t fuelCounts(float frac, bool right) const;

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
  // Released at boot. The cluster lights the park-brake telltale and gongs for
  // as long as this is set, and both the host GUI and the firmware used to
  // power up with it engaged - so the bench booted into "handbrake applied".
  volatile bool     handbrake_ = false;
  volatile bool     reverse_   = false;
  volatile bool     engineOn_  = cfg::ENGINE_RUNNING_AT_BOOT;
  volatile bool     kl15_      = true;
  volatile uint8_t  odoMode_    = cfg::ODO_MODE;
  volatile uint8_t  speedSrc_   = cfg::SPEED_SOURCES;
  volatile uint32_t txOk_ = 0, txFail_ = 0, rx_ = 0;

  // ---- clock -------------------------------------------------------------
  volatile uint8_t  hour_  = cfg::CLOCK_BOOT_HOUR;
  volatile uint8_t  min_   = cfg::CLOCK_BOOT_MIN;
  volatile uint8_t  sec_   = 0;
  volatile uint8_t  day_   = cfg::CLOCK_BOOT_DAY;
  volatile uint8_t  month_ = cfg::CLOCK_BOOT_MONTH;
  volatile uint16_t year_  = cfg::CLOCK_BOOT_YEAR;
  volatile bool     clockValid_ = false;   // true once anything set the time
  uint32_t lastSecMs_ = 0;                 // millis() of the last second tick
  uint32_t lastClock_ = 0;                 // millis() of the last 0x39E

  // ---- 0x592 check-control symbols ---------------------------------------
  enum : uint8_t { SYM_DTC = 0, SYM_DSC, SYM_DSC_OFF, SYM_EXTRA, SYM_COUNT };
  struct SymSlot {
    uint16_t id;
    bool     want;      // what the host asked for
    bool     sent;      // what the bus has been told
    uint32_t lastMs;    // millis() of the last frame for this slot
  };
  SymSlot sym_[SYM_COUNT] = {
    {cfg::sym::DTC,          false, false, 0},
    {cfg::sym::DSC_TRIANGLE, false, false, 0},
    {cfg::sym::DSC_OFF,      false, false, 0},
    {0,                      false, false, 0},   // setSymbol() scratch slot
  };
  uint8_t symCursor_ = 0;

  // ---- fuel slew ---------------------------------------------------------
  float    fuelShown_  = -1.0f;   // litres actually on the bus, < 0 = unset
  uint32_t lastFuelMs_ = 0;

  // ---- what the cluster told us (0x2CA / 0x2F8) --------------------------
  bool    kombiTempSeen_  = false;
  uint8_t kombiTempRaw_   = 0xFF;
  int16_t kombiTempDeciC_ = 0;
  bool    kombiClockSeen_ = false;
  uint8_t kombiH_ = 0, kombiM_ = 0, kombiS_ = 0;
  uint32_t kombiTempMs_ = 0, kombiClockMs_ = 0;

  // ---- scheduler + rolling counters --------------------------------------
  uint32_t lastTick_ = 0;
  uint32_t tick_     = 0;
  uint8_t  cnt130_ = 0, cnt0AA_ = 0, cnt1D0_ = 0, cnt0C0_ = 0xF0, cnt0D7_ = 0;
  uint8_t  cnt1B4_ = 0, cnt1D2_ = 0, cnt19E_ = 0;
  bool     hb34F_  = false;       // last 0x34F handbrake state put on the bus
  uint8_t  lastTurn_ = 0;       // to emit the 0x1F6 "new blink" byte once
  uint16_t dist1A6_   = 0;      // 0x1A6 B0..B5 cumulative distance accumulator
  float    distAccum_ = 0.0f;   // sub-count remainder, so low speeds still add up
  uint32_t lastDist_  = 0;      // millis() at the last 0x1A6, for dt
  uint16_t tick1A6_   = 0;      // 0x1A6 B6..B7 12-bit time counter (+400/frame)
};
