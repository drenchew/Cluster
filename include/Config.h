#pragma once
/*
 * Config.h - every tunable for the E9x cluster bench jig lives here.
 * Target: Arduino Uno (ATmega328P) + HW-184 / MCP2515 (8 MHz).
 *
 * CONFIDENCE KEY on the frame groups below:
 *   [OK]   corroborated by a working bench capture and/or a DBC signal def
 *   [?]    plausible but unverified - the layout is a documented guess, so it
 *          defaults to off. Enable one at a time and watch for new
 *          check-control noise.
 */
#include <Arduino.h>
#include <stdarg.h>
#include <mcp2515.h>

/* printf-style logging for AVR (Serial.printf does not exist on Uno).
 * AVR's default vfprintf has no %f, so fuel is printed as a fixed-point
 * decimal via printFix1() instead. */
inline void printFix1(uint16_t deciLitres) {
  Serial.print(deciLitres / 10);
  Serial.write('.');
  Serial.write('0' + (deciLitres % 10));
}

inline void logf(const char *fmt, ...) {
  char buf[96];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print(buf);
}

namespace cfg {

// ---- Hardware (Arduino Uno <-> HW-184 / MCP2515) ---------------------------
// Uno hardware SPI is fixed: SCK=D13, MISO=D12, MOSI=D11 (do not move these).
// CS and INT are plain GPIO - D10 is the Uno hardware SS (safe default).
constexpr uint8_t  PIN_CS   = 10;   // module "CS"
constexpr uint8_t  PIN_SCK  = 13;   // module "SCK"  (hardware SPI)
constexpr uint8_t  PIN_MOSI = 11;   // module "SI"   (hardware SPI)
constexpr uint8_t  PIN_MISO = 12;   // module "SO"   (hardware SPI)
constexpr uint8_t  PIN_INT  = 2;    // diagnostic only, TX does not need it
// SPI bus clock to the MCP2515 (NOT the CAN crystal). Keep this low (1 MHz)
// on breadboard/dupont wiring - 8 MHz over long jumpers is a common cause of
// "reset FAIL". The MCP2515 itself can do 10 MHz on a clean PCB.
constexpr uint32_t SPI_HZ   = 1000000UL;

// ---- Bus -------------------------------------------------------------------
// E9x instrument cluster is on K-CAN = 100 kbit/s.  PT-CAN (500k) is NOT
// wired to the cluster.  HW-184 crystal = 8 MHz.
constexpr CAN_SPEED BUS_BITRATE = CAN_100KBPS;
constexpr CAN_CLOCK BUS_CLOCK   = MCP_8MHZ;

// ---- Frame groups ----------------------------------------------------------
constexpr bool SEND_WARNING_SUPPRESS = true;   // [OK] 0x19E, 0x0C0, 0x0D7
constexpr bool SEND_FUEL             = true;   // [OK] 0x349
constexpr bool SEND_COOLANT          = true;   // [OK] 0x1D0 (DBC-confirmed B0)
constexpr bool SEND_LIGHTS           = true;   // [OK] 0x21A illumination/backlight
constexpr bool SEND_TURN             = true;   // [OK] 0x1F6 turn signals
constexpr bool SEND_BRAKE            = true;   // [OK] 0x1B4 handbrake + speed
constexpr bool SEND_PARK_BRAKE       = true;   // [OK] 0x34F handbrake status (2 byte)
constexpr bool SEND_DIMMER           = false;  // [?]  0x202 dimmer level
constexpr bool SEND_LCD_LIGHT        = false;  // [?]  0x2C0 LCD illumination
constexpr bool SEND_GEAR             = false;  // [?]  0x1D2 gear display P/R/N/D
constexpr bool SEND_REVERSE          = false;  // [?]  0x3B0 reverse gear
constexpr bool SEND_RANGE            = false;  // [?]  0x366 outside temp + range
constexpr bool SEND_CLOCK            = true;   // [OK] 0x39E time + date -> LCD clock
constexpr bool SEND_SYMBOLS          = true;   // [OK] 0x592 check-control symbols

// NEVER transmit 0x2CA (outside temperature) or 0x2F8 (time/date): on an E9x
// the KOMBI is the *sender* of both. Two nodes putting different data behind
// the same identifier collide bit-for-bit and generate CAN error frames, which
// is a far worse problem than a wrong reading. The jig listens to them instead
// (see REPORT_CLUSTER_RX) - they are the cluster telling you what it currently
// shows, which is exactly what you want while chasing the "-40" ambient
// reading or an unset clock. 0x366 has KOMBI as its sender in the DBC too,
// hence the default-off above.

// ---- 0x130 terminal status -------------------------------------------------
// Byte 4 of 0x130 is a *packed* field: high nibble = rolling counter,
// low nibble = check value derived from it. Every bench capture reported
// working on an E9x KOMBI keeps this byte CONSTANT (0xFE from a raw capture,
// 0xE2 from the e46canbus blog). Incrementing the counter nibble while the
// check nibble stays at 0x2 produces a byte the cluster rejects on 14 of
// every 15 frames -> it sees terminal status as absent/implausible and
// re-runs its power-on sequence (gong + telltales) in a loop.
//
// Bit 1 of this byte is "IgnitionOn" (engine running) - it is set in 0xFE,
// in 0xE2, and clear in the KL15-OFF payload 0x50. Keep it set.
constexpr uint8_t T130_BYTE4_ON  = 0xFE;  // KL15 on, engine running
constexpr uint8_t T130_BYTE4_OFF = 0x50;  // KL15 off
// Leave false. Only set true if you want to experiment with a rolling
// counter nibble; expect the reset loop to come back.
constexpr bool T130_ROLLING_COUNTER = false;

// ---- Engine state ----------------------------------------------------------
// A BMW cluster that is told "terminal 15 on" but "0 rpm" concludes the
// engine has stalled and responds exactly like a real car: gong plus the
// charge / oil-pressure / brake / DSC telltales. On a bench jig you almost
// never want that, so RPM is floored at idle while the engine is "running".
// Use the 'e 0' serial command to deliberately simulate key-on-engine-off.
constexpr uint16_t ENGINE_IDLE_RPM = 800;   // diesel KOMBI idles ~800
constexpr bool     ENGINE_RUNNING_AT_BOOT = true;

// ---- Scaling ---------------------------------------------------------------
// 0x1B4 speed word: km/h per count (community DBC value). 0x1A6 and 0x1A0 and
// 0x0CE carry their own scalings in their sections below.
constexpr float SPEED_KMH_PER_COUNT = 0.103f;
constexpr uint16_t RPM_LIMIT   = 7000;
constexpr uint16_t SPEED_LIMIT = 400;

// ---- 0x1A6 bytes 0..5: a cumulative distance accumulator, NOT speed --------
// This was the speedometer bug. Verified on a real E90 M3 K-CAN capture
// (1154 frames, 68.7 s, speeds 0..15.9 km/h):
//
//  * Bytes 0..1, 2..3, 4..5 are three INDEPENDENT 16-bit little-endian
//    accumulators, not one speed word repeated three times. They climb
//    monotonically 0 -> 8793 while the car accelerates, and they disagree by a
//    margin that grows (copy3 - copy1 runs -1 -> +658). Three separately
//    integrated values, not 309/309/308 wheel speeds as the single frame
//    quoted in older comments suggested.
//  * Parked, every copy reads 00 00. So they freeze at zero, they are not a
//    free-running counter.
//  * Rate: 12.3 .. 13.7 counts per km/h per second (copy3 runs fastest,
//    copy1/copy2 sit at 12.3-12.9 in 200-frame windows; whole-span mean
//    13.23).
//
// The jig used to send a CONSTANT speed word here. The cluster differentiates
// an accumulator to get speed, a constant differentiates to 0, and the needle
// never moves - while every other gauge, driven by direct-value frames, works
// fine. That is exactly the reported symptom.
constexpr float ODO_COUNTS_PER_KMH_PER_SEC = 13.0f;
// 13.0 is the veikkos/e90-can-cluster driver's "speedo reads true" calibration
// (its odometer-exact default is 13.6) and sits within ~2% of copies 1 and 2
// of the real capture. Use 13.6 if you want the odometer reading true rather
// than the needle: both are close enough that the cluster cannot tell.
//
// ODO_MODE selects how bytes 0..5 advance; switch at runtime with the 'o'
// console command so bring-up does not need a reflash.
enum : uint8_t {
  ODO_FROZEN   = 0,  // never advances - "is the counter alive?" diagnostic
  ODO_ALIVE    = 1,  // +ODO_ALIVE_STEP per frame regardless of speed. If the
                     // cluster only wants a message counter this keeps the
                     // frame valid and adds nothing to the odometer, but the
                     // speedometer will read 0 - it differentiates.
  ODO_DISTANCE = 2,  // integrates speed, like a real car. THE DEFAULT: this
                     // is what a real 0x1A6 does. Mileage accrues for real.
};
constexpr uint8_t ODO_MODE = ODO_DISTANCE;

// ODO_ALIVE: counts added per 0x1A6 frame. A real 0x1A6 goes out every
// 100 ms, so a real-car equivalent is +1 per 100 ms frame, not per 10 ms tick.
constexpr uint8_t ODO_ALIVE_STEP = 1;

// ---- 0x1A6 bytes 6..7: free-running 12-bit time counter --------------------
// Verified on the same capture: it advances +400 per 100 ms frame (mean tick
// delta exactly 400.0 over every frame), wraps at 12 bits, and B7 always has
// its high nibble at 0xF (0 exceptions in 1154 frames). Encoded as
// B6 = n & 0xFF, B7 = 0xF0 | ((n >> 8) & 0x0F) - the same formula the working
// veikkos/e90-can-cluster driver uses.
//
// It is time-based, not distance-based: parked, the distance copies sit at 0
// while this keeps ticking. The jig's old +1-per-20 ms was ~80x too slow.
constexpr uint16_t TICK_1A6_STEP = 400;   // counts added per 0x1A6 frame

// ---- 0x21A illumination / backlight ----------------------------------------
// 3-byte FRM "lighting status". The payload reported working on a bench is
// B0=0x05 B1=0x10 B2=0x00 for "on" and all-zero for "off". The individual
// bits below are inferred from that pair - B0 bit0 and bit2 are the two that
// are set in the working value. Note the cluster also derives brightness from
// its own ambient phototransistor, so on a bench the backlight may sit near
// full regardless of the dimmer frame until you cover that sensor.
constexpr uint8_t L21A_B1_ON  = 0x10;
constexpr uint8_t LIGHT_SIDE  = 0x01;   // standing/parking light   (in 0x05)
constexpr uint8_t LIGHT_LOW   = 0x04;   // low beam                 (in 0x05)
constexpr uint8_t LIGHT_HIGH  = 0x02;   // high beam                [?]
constexpr uint8_t LIGHT_FOGF  = 0x20;   // front fog                [?]
constexpr uint8_t LIGHT_FOGR  = 0x40;   // rear fog                 [?]

// ---- Additional speed sources ----------------------------------------------
// 0x1A6 carries a distance accumulator, not an instantaneous speed, so the
// needle's speed now comes from 0x1A0 "Speed K-CAN" (a DSC frame: a real
// capture has it every 100 ms alongside 0x1A6). Real DSC sends both, so both
// are on by default - 0x1A6 so the cluster can differentiate a distance rate
// if that is what it wants, 0x1A0 so it can read a direct speed value.
// 0x1A6 alone was never enough on this jig.
//
// 0x1A0's layout, alive nibble and checksum are verified against the capture.
//
// 0x0CE layout is now decoded from the same capture (see SPEED_0CE_PER_COUNT):
// four signed 16-bit little-endian wheel speeds, and real DSC sends it every
// 100 ms whether or not anything else is on the bus. It is on by default so
// the DSC side of the cluster has the wheel speeds it expects - a DSC that
// reports no wheel speed at all is the classic way to end up with the ABS,
// DSC and tyre-pressure telltales lit.
constexpr uint8_t SPEED_SRC_1A0 = 0x01;   // 0x1A0 "Speed K-CAN", 12-bit, 0.1 km/h
constexpr uint8_t SPEED_SRC_0CE = 0x02;   // 0x0CE four 16-bit wheel speeds
constexpr uint8_t SPEED_SOURCES = SPEED_SRC_1A0 | SPEED_SRC_0CE;   // 0x1A6 + 0x1A0 + 0x0CE

// 0x1A0 scaling: 0.1 km/h per count (E65 document). Verified against the
// capture - V_VEH peaks at 159 while the 0x1A6 rate integrates to ~15 km/h.
// 50 km/h is 500 = 0x1F4 -> B0=0xF4, B1=0x01.
constexpr float SPEED_1A0_PER_COUNT = 0.1f;
// 0x0CE: four 16-bit LE wheel speeds, SIGNED. Measured against 0x1A0's V_VEH
// over the 1154-frame capture the word reads 1.573 +/- 0.01 counts per km/h at
// every speed from 40 to 150 km/h (bucket means 1.567 .. 1.597), i.e. about
// 0.636 km/h per count. That is wheel pulses per 100 ms frame at 48 pulses per
// revolution on a 225/45R17 (2.10 m circumference -> 0.636 counts per km/h),
// which is what a real DSC counts. The sign flips when the car drives
// backwards: the 91 negative frames in the capture fall exactly inside the
// 0x1D2 = R gear window (t 668094..682657), |word| still tracks V_VEH.
// Send magnitude only - this jig never simulates reverse speed.
constexpr float SPEED_0CE_PER_COUNT = 0.636f;

// ---- 0x1A6 / scheduler cadence ---------------------------------------------
// Real cadences measured off the capture: 0x1A6 = 100.0 ms, 0x1A0 = 99.7 ms,
// 0x1B4 = 100 ms, 0x0CE = 100 ms. 0x1A6 MUST run at 100 ms: the cluster
// differentiates it, so sending it faster inflates the derived speed by the
// same factor. (0x0AA is sent at 20 ms here and RPM reads true, because the
// cluster reads it as a direct value - faster cadence is tolerated there.)
// 0x1A6 goes out on scheduler slots 7 and 17 of the 200 ms / 20 tick cycle.

// ---- 0x349 fuel level: two senders, neither of them linear -----------------
// The E9x tank is a saddle tank with a sender in each half, and 0x349 carries
// both as 16-bit counts. The flat "litres * 160" the jig used to send is only
// right for the left sender near full - the gauge is deliberately non-linear
// (slow to fall off full, quick through the bottom third), and the two senders
// do not even share a curve. These are the breakpoints from the working
// veikkos/e90-can-cluster driver, as counts at a given fill fraction:
//
//     fill    1.000  0.875  0.750  0.500  0.250  0.000
//     left     9700   8200   6250   3600   1950    625
//     right    8400   5400   4600   3350   2200    950
//
// Interpolating these makes the needle sit where a real tank's would. Set
// FUEL_CURVE_REAL = false to go back to the flat 160 counts/litre if you want
// to compare the two on the bench.
constexpr bool  FUEL_CURVE_REAL = true;
constexpr float FUEL_TANK_L     = 60.0f;   // tank the curve above is scaled to
constexpr float FUEL_PER_LITRE  = 160.0f;  // flat fallback, DBC counts/litre

// A real gauge is heavily damped: the level you see is a long average, and the
// cluster treats a step change as a refuelling event. The host can jump the
// level by 40 litres in one frame (one keypress), so the firmware slews it
// instead - 1.5 %/s is about what a real tank reads back when you fill it.
constexpr float FUEL_SLEW_FRAC_PER_S = 0.015f;
constexpr bool  FUEL_SLEW = true;

// ---- 0x39E time / date (sets the cluster clock) ----------------------------
// Verified against the veikkos/e90-can-cluster driver, which is the one
// open-source implementation known to set an E9x KOMBI clock:
//   B0 hour  B1 minute  B2 second  B3 day
//   B4 (month << 4) | 0x0F        B5..B6 year, 16-bit little-endian
//   B7 0xF2
// The cluster echoes what it ends up believing on 0x2F8, so you can read your
// own frame back (REPORT_CLUSTER_RX) rather than squinting at the LCD.
//
// A cluster that has never been given the time shows no clock at all, which is
// what "no clock on the computer" on a bench jig always is: nothing on the bus
// ever told it what time it is. One frame per second is plenty.
constexpr uint32_t CLOCK_PERIOD_MS = 1000;
// Boot time, used until the host sends its own. 12:00 on 2026-01-01 is a
// deliberately obvious placeholder - if you see it, the host link is not
// supplying a clock.
constexpr uint8_t  CLOCK_BOOT_HOUR  = 12;
constexpr uint8_t  CLOCK_BOOT_MIN   = 0;
constexpr uint8_t  CLOCK_BOOT_DAY   = 1;
constexpr uint8_t  CLOCK_BOOT_MONTH = 1;
constexpr uint16_t CLOCK_BOOT_YEAR  = 2026;

// ---- 0x592 check-control symbols ------------------------------------------
// The E9x KOMBI does not have a "light the DSC lamp" signal. Telltales that
// are not hard-wired to a frame of their own are check-control *symbols*,
// addressed by number on 0x592:
//
//   B0 0x40   B1..B2 symbol id, 16-bit little-endian
//   B3 0x31 = show, 0x30 = clear   B4..B7 0xFF
//
// Symbol ids are from the E92 check-control symbol list (see the veikkos
// driver's external/ folder). Only a handful matter here; the full list has
// several hundred, and a cluster silently ignores an id it does not know.
namespace sym {
constexpr uint16_t DTC          = 357;  // "DTC" - traction mode / TC enabled
constexpr uint16_t DSC_TRIANGLE = 215;  // DSC triangle alone - intervention
constexpr uint16_t DSC_OFF      = 36;   // DSC triangle + double warning = off
constexpr uint16_t SERVICE      = 281;  // service due - suppressed at boot
}  // namespace sym

// An ON symbol is refreshed periodically (the real DSC repeats its symbols at
// 50-200 ms); an OFF one only needs an occasional re-clear so a frame lost to
// a bus error does not leave a lamp stuck on.
constexpr uint16_t SYM_ON_REFRESH_MS  = 200;
constexpr uint16_t SYM_OFF_REFRESH_MS = 3000;
// The jig has no service-interval data, and a cluster fresh off a scrapyard
// car usually boots with the service message up. Clear it once at startup.
constexpr bool SUPPRESS_SERVICE_LIGHT = true;

// ---- Listening to the cluster ---------------------------------------------
// The KOMBI broadcasts what it currently believes: 0x2CA = the outside
// temperature it measures on its own sensor pins (B0 * 0.5 - 40 degC) and
// 0x2F8 = its clock. Printing them every few seconds turns two of the most
// annoying bench questions - "what does the thermometer actually read?" and
// "did the clock take?" - into a line in the log pane.
constexpr bool     REPORT_CLUSTER_RX = true;
constexpr uint32_t REPORT_PERIOD_MS  = 5000;

// ---- Host link (PC simulator over USB serial) ------------------------------
// The firmware always owns CAN timing. The host only supplies setpoints; if
// it goes quiet the heartbeat keeps running and the jig falls back to the
// local demo sweep, so unplugging the USB cable can never reset the cluster.
constexpr uint32_t HOST_TIMEOUT_MS = 1500;
constexpr uint8_t  HOST_SYNC0      = 0xAA;
constexpr uint8_t  HOST_SYNC1      = 0x55;
constexpr uint8_t  HOST_PAYLOAD    = 28;   // sizeof(HostState) - keep in sync

// ---- Demo sweep ------------------------------------------------------------
// Runs only while no host is connected. Bottoms out at idle, never at 0 rpm.
constexpr uint16_t DEMO_RPM_MAX         = 4500;
constexpr uint16_t DEMO_SPEED_MAX_KMH   = 120;
constexpr uint16_t DEMO_RPM_PER_SECOND  = 1200;
constexpr uint32_t DEMO_START_DELAY_MS  = 4000;  // cluster boot / needle self-test
constexpr uint32_t DEMO_IDLE_DWELL_MS   = 2000;  // sit at idle between sweeps
constexpr bool     DEMO_SWEEP_SPEED     = true;  // false = tacho only, speedo at 0

// ---- Serial console --------------------------------------------------------
// Serial.parseFloat() blocks for the stream timeout when no digits follow the
// command letter, which stalls loop() and therefore the 0x130 heartbeat. The
// console is parsed line-at-a-time instead; this is only a safety net.
constexpr uint32_t SERIAL_TIMEOUT_MS = 20;
constexpr uint8_t  SERIAL_LINE_MAX   = 24;

// ---- TX scheduler ----------------------------------------------------------
constexpr uint32_t TX_TICK_MS = 10;

}  // namespace cfg
