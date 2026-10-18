/*
 * E90_Cluster.ino  -  BMW E90/E91/E92/E93 (and E87) instrument cluster bench driver
 * Target cluster : A2C53117692 (diesel, KOMBI)
 * MCU            : ESP32 (Arduino framework)
 * CAN controller : HW-184 / MCP2515 module, 8 MHz crystal
 * Library        : autowp/arduino-mcp2515  (Library Manager: "mcp2515" by autowp)
 *
 * ---------------------------------------------------------------------------
 *  !!  READ THIS FIRST  !!
 * ---------------------------------------------------------------------------
 * 1) BITRATE. The E9x instrument cluster lives on the K-CAN bus, which runs at
 *    100 kbit/s, NOT 500 kbit/s. 500 kbit/s is PT-CAN (engine/DSC), which the
 *    cluster is not connected to. At 500 kbit/s the cluster will never answer.
 *    -> This sketch uses CAN_100KBPS with MCP_8MHZ.  (500k is one #define away.)
 *
 * 2) RPM IS NOT 0x316 ON AN E9x. 0x316 is the E46/E39-era MS4x RPM frame.
 *    On the E9x the RPM the cluster reads is in 0x0AA (DME "throttle/RPM"):
 *    bytes 4..5 = RPM * 4, little-endian.
 *
 * 3) Speed comes from TWO frames, and the one everybody assumes is not what
 *    you think. 0x1A0 (DSC "Speed K-CAN") carries the direct 12-bit value at
 *    0.1 km/h per count. 0x1A6 bytes 0..5 are NOT speed - they are three
 *    copies of a cumulative distance accumulator that the cluster
 *    differentiates to get speed. Send a constant word there and the needle
 *    stays at 0 while every other gauge works. See send1A6()/send1A0().
 *
 * 4) Terminal-15 ("ignition on") is 0x130 (5 bytes) every 100 ms, driven from a
 *    dedicated FreeRTOS task that loop() cannot starve. Byte 4 is a packed
 *    counter+check pair and is sent CONSTANT (0xFE): incrementing the counter
 *    nibble alone makes the cluster reject the frame and loop its power-on
 *    alarm sequence. What must not stall is the frame itself, not that byte.
 *
 * 5) Never command 0 rpm while terminal 15 is on. The cluster reads it as the
 *    engine stalling and answers with the gong plus the charge, oil-pressure,
 *    brake and DSC telltales. RPM is floored at ENGINE_IDLE_RPM.
 *
 * Serial commands (115200 baud), handy for calibration:
 *    r 3000   -> hold RPM at 3000 (stops the demo sweep)
 *    s 100    -> hold speed at 100 km/h
 *    f 35     -> fuel level 35 litres
 *    d        -> resume the automatic demo sweep
 */

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

// ------------------------------- Pin map -----------------------------------
static const uint8_t PIN_CS   = 5;
static const uint8_t PIN_SCK  = 18;
static const uint8_t PIN_MOSI = 23;   // module "SI"
static const uint8_t PIN_MISO = 19;   // module "SO"  (see level-shifter note!)
static const uint8_t PIN_INT  = 4;    // not required for TX, read for diagnostics

// ----------------------------- CAN settings --------------------------------
#define CAN_SPEED        CAN_100KBPS   // K-CAN. Use CAN_500KBPS only for PT-CAN.
#define CAN_XTAL         MCP_8MHZ      // HW-184 crystal is 8 MHz

// ------------------------- Optional frame groups ---------------------------
// Frames seen in working bench setups that stop warning lamps / fill the fuel
// gauge. Disable any group that misbehaves with your cluster.
#define SEND_WARNING_SUPPRESS  1   // 0x19E (DSC), 0x0C0 (ABS ctr), 0x0D7 (airbag ctr)
#define SEND_FUEL              1   // 0x349 fuel level
#define SEND_COOLANT           0   // 0x1D0 coolant temp (layout partly unverified)

// ------------------------------ Scaling ------------------------------------
// 0x1A0 V_VEH: 12-bit, 0.1 km/h per count (E65 document, and verified against
// a real E90 M3 K-CAN capture). If the needle reads wrong, adjust this.
static const float SPEED_1A0_PER_COUNT = 0.1f;

// 0x1A6 bytes 0..5: counts added per km/h per second - the integrator the
// cluster differentiates into speed. Measured off a real E90 capture: 12.3 to
// 13.7 counts/s per km/h. The veikkos/e90-can-cluster driver (same frames,
// same layout) reads true at 13.0 with 13.6 for an exact odometer.
static const float ODO_COUNTS_PER_KMH_PER_SEC = 13.0f;

// False holds the accumulator at 0 (diagnostic: proves whether the needle
// follows this field), true integrates like a real car - the odometer then
// accrues real mileage, which cannot be undone.
static const bool ODO_TRACK_DISTANCE = true;

// 0x1A6 bytes 6..7: a free-running 12-bit TIME counter, +400 per 100 ms frame
// (mean delta measured at exactly 400.0 over 1154 real frames; B7 high nibble
// always 0xF). It is time-based, not distance - it ticks while parked.
static const uint16_t TICK_1A6_STEP = 400;

// Crank speed below which the cluster decides the engine has stalled and
// answers with the gong plus the charge/oil/brake/DSC telltales. The demo
// sweep bottoms out here, never at 0 rpm.
static const uint16_t ENGINE_IDLE_RPM = 800;

// ------------------------------ Demo sweep ---------------------------------
static const uint16_t DEMO_RPM_MAX        = 4500;
static const uint16_t DEMO_SPEED_MAX_KMH  = 120;
static const uint16_t DEMO_RPM_PER_SECOND = 1200;   // ramp rate
static const uint32_t DEMO_START_DELAY_MS = 4000;   // let the cluster finish boot/self-test
static const uint32_t DEMO_IDLE_DWELL_MS  = 2000;   // sit at idle between sweeps

// ------------------------------ Shared state -------------------------------
// 32-bit aligned reads/writes are atomic on the ESP32, so volatile is enough.
static volatile uint16_t g_rpm      = 0;     // engine speed, rpm
static volatile uint16_t g_speedKmh = 0;     // vehicle speed, km/h
static volatile uint16_t g_fuel_dl  = 300;   // fuel in 0.1 l  (30.0 l)
static volatile int16_t  g_coolantC = 90;    // coolant temp, deg C

static volatile uint32_t g_txOk = 0, g_txFail = 0, g_rxCount = 0;

MCP2515 mcp(PIN_CS, 8000000UL);   // 8 MHz SPI clock is safe for the MCP2515 (max 10 MHz)

// ============================================================================
//  Public API
// ============================================================================
/* RPM is floored at idle: the cluster reads 0 rpm with terminal 15 on as a
 * stall and sets off the gong plus the charge/oil/brake telltales. */
void setRPM(uint16_t rpm)        { if (rpm > 7000) rpm = 7000;
                                   if (rpm < ENGINE_IDLE_RPM) rpm = ENGINE_IDLE_RPM;
                                   g_rpm = rpm; }
void setSpeed(uint16_t kmh)      { g_speedKmh = (kmh > 400) ? 400 : kmh; }
void setFuelLitres(float litres) { g_fuel_dl = (uint16_t)(constrain(litres, 0.0f, 80.0f) * 10.0f); }
void setCoolantC(int16_t c)      { g_coolantC = constrain(c, -40, 150); }

// ============================================================================
//  Frame builders  (all called ONLY from the TX task -> SPI is single-owner)
// ============================================================================
static bool canSend(uint32_t id, const uint8_t *d, uint8_t len) {
  can_frame f;
  f.can_id  = id;
  f.can_dlc = len;
  memset(f.data, 0, 8);
  memcpy(f.data, d, len);
  bool ok = (mcp.sendMessage(&f) == MCP2515::ERROR_OK);
  if (ok) g_txOk = g_txOk + 1; else g_txFail = g_txFail + 1;
  return ok;
}

/* 0x130  Terminal status (CAS -> all), 5 bytes, every 100 ms
 *   B0 0x45  key status          (0x40 = key out / 0x45 = key in, ignition)
 *   B1 0x40  key/transponder     (0x42 = transponder detected)
 *   B2 0x21  terminal status     (0x21 = KL15 on in captured bench traffic;
 *                                 0x69 also seen with B1=0x42)
 *   B3 0x8F  steering lock / misc
 *   B4       4-bit rolling counter (upper nibble, 0..E) | 0x2 flag nibble
 * The counter MUST advance every frame or the cluster resets. Check/CRC nibble
 * is not validated by the cluster in the bench setups that worked. */
static void send130() {
  // B4 is held CONSTANT. Its high nibble is a sequence number and its low
  // nibble a check value derived from it, so incrementing one alone yields a
  // byte the cluster rejects -> it treats terminal status as implausible and
  // re-runs its power-on alarm sequence in a loop. Both payloads reported
  // working keep B4 fixed: 0xFE (raw capture) or 0xE2 (e46canbus blog).
  uint8_t d[5] = {0x45, 0x40, 0x21, 0x8F, 0xFE};
  canSend(0x130, d, 5);
}

/* 0x26E  Ignition / "engine running" status, 8 bytes, ~200 ms
 * Static content copied from a working bench sketch. */
static void send26E() {
  const uint8_t d[8] = {0x40, 0x40, 0x7F, 0x50, 0xFF, 0xFF, 0xFF, 0xFF};
  canSend(0x26E, d, 8);
}

/* 0x0AA  DME engine speed / pedal, 8 bytes, every 20 ms  <-- THE RPM NEEDLE
 *   B0      unknown (0xFE / 0x5F in captures)
 *   B1      low nibble = rolling counter 0..14, high nibble 0xF
 *   B2..B3  accelerator pedal (0xFF 0x00 = idle in captures)
 *   B4..B5  RPM * 4, little-endian   (idle 845 rpm -> 0x0D34)
 *   B6      pedal/cruise flags       (0xFE)
 *   B7      0x99 */
static void send0AA() {
  static uint8_t cnt = 0;
  uint16_t raw = (uint16_t)g_rpm * 4;
  uint8_t d[8] = {0xFE, (uint8_t)(0xF0 | cnt), 0xFF, 0x00,
                  (uint8_t)(raw & 0xFF), (uint8_t)(raw >> 8), 0xFE, 0x99};
  cnt = (cnt + 1) % 15;
  canSend(0x0AA, d, 8);
}

/* 0x1A6  Distance accumulator + time counter, 8 bytes, every 100 ms
 *   B0..B5 three copies of a 16-bit little-endian cumulative distance counter
 *   B6..B7 12-bit time counter, +400 per frame, B7 = 0xF0 | high nibble
 *
 * This frame carries NO speed value. A real E90 M3 capture shows the three
 * words climbing 0 -> 8793 while the car does 0 -> 15.9 km/h, at 12.3..13.7
 * counts per km/h per second, reading 00 00 at a standstill. The cluster
 * differentiates the field: send a constant word and the needle stays at 0
 * while every direct-value gauge works.
 *
 * Cadence matters - the real cadence is 100 ms, and sending this faster makes
 * the derived speed proportionally too fast. */
static void send1A6() {
  static uint16_t distCnt   = 0;
  static float    distAccum = 0.0f;
  static uint16_t tickCnt   = 0;
  static uint32_t lastDist  = 0;
  uint32_t now = millis();
  uint32_t dtMs = now - lastDist;
  lastDist = now;
  if (dtMs > 500) dtMs = 500;

  if (ODO_TRACK_DISTANCE && g_speedKmh) {
    distAccum += (float)g_speedKmh * ODO_COUNTS_PER_KMH_PER_SEC
                 * ((float)dtMs / 1000.0f);
    uint16_t whole = (uint16_t)distAccum;      // 16-bit wrap, like the car
    distAccum -= (float)whole;
    distCnt = (uint16_t)(distCnt + whole);
  }

  tickCnt = (uint16_t)(tickCnt + TICK_1A6_STEP);
  uint8_t lo = distCnt & 0xFF;
  uint8_t hi = distCnt >> 8;
  uint8_t d[8] = {lo, hi, lo, hi, lo, hi,
                  (uint8_t)(tickCnt & 0xFF),
                  (uint8_t)(0xF0 | ((tickCnt >> 8) & 0x0F))};
  canSend(0x1A6, d, 8);
}

/* 0x1A0  DSC vehicle dynamics / "Speed K-CAN", 8 bytes, every 100 ms
 *   B0..B1 12-bit V_VEH at 0.1 km/h/count; B1 bits 4..6 = ST_VEH_DVCO
 *          (0 at a standstill, 1 once moving), bit 7 = 0x80 always set
 *   B2..B5 acceleration / yaw - zero here, which the cluster accepts
 *   B6 high nibble = ALIV_V, a time-based alive nibble (5-6 steps per 100 ms
 *          frame in the capture, exactly what ((millis()%300)*16/300)&0x0F
 *          produces - NOT a per-frame +1); low nibble = yaw high bits (0)
 *   B7 checksum, verified 1158/1158 on the capture:
 *          sum = 0x00A0 + B0+..+B6;  B7 = (sum + (sum>>8) + 0x01) & 0xFF
 * https://github.com/HeinrichG-V12/E65_ReverseEngineering/blob/main/docs/0x1A0.md */
static void send1A0() {
  uint16_t raw = (uint16_t)((float)g_speedKmh / SPEED_1A0_PER_COUNT);
  if (raw > 0x0FFF) raw = 0x0FFF;
  uint8_t st  = raw ? 1 : 0;
  uint8_t aliv = (uint8_t)(((millis() % 300) * 16 / 300) & 0x0F);

  uint8_t d[8];
  d[0] = (uint8_t)(raw & 0xFF);
  d[1] = (uint8_t)(((raw >> 8) & 0x0F) | (st << 4) | 0x80);
  d[2] = 0x00;
  d[3] = 0x00;
  d[4] = 0x00;
  d[5] = 0x00;
  d[6] = (uint8_t)(aliv << 4);
  uint16_t sum = 0x00A0;
  for (uint8_t i = 0; i < 7; i++) sum = (uint16_t)(sum + d[i]);
  d[7] = (uint8_t)(sum + (sum >> 8) + 0x01);
  canSend(0x1A0, d, 8);
}

#if SEND_COOLANT
/* 0x1D0  DME engine data: B0 = coolant + 48 degC, B2 = rolling counter,
 * B6 = 0xCC (clutch released). Other bytes are filler -> unverified. */
static void send1D0() {
  static uint8_t cnt = 0;
  uint8_t d[8] = {(uint8_t)(g_coolantC + 48), 0xFF, cnt, 0xFF, 0xFF, 0xFF, 0xCC, 0xFF};
  cnt++;
  canSend(0x1D0, d, 8);
}
#endif

#if SEND_WARNING_SUPPRESS
/* 0x19E  DSC/ABS status (static capture), 0x0C0 ABS counter, 0x0D7 airbag counter */
static void send19E() {
  const uint8_t d[8] = {0x00, 0xE0, 0xB3, 0xFC, 0xF0, 0x43, 0x00, 0x65};
  canSend(0x19E, d, 8);
}
static void send0C0() {
  static uint8_t c = 0xF0;
  uint8_t d[2] = {c, 0xFF};
  c = (c == 0xFF) ? 0xF0 : c + 1;
  canSend(0x0C0, d, 2);
}
static void send0D7() {
  static uint8_t c = 0;
  uint8_t d[2] = {c++, 0xFF};
  canSend(0x0D7, d, 2);
}
#endif

#if SEND_FUEL
/* 0x349  Fuel level: litres*160 (16-bit LE), sent twice (two tank halves), B4=0 */
static void send349() {
  uint16_t v = (uint16_t)((g_fuel_dl / 10.0f) * 160.0f);
  uint8_t d[5] = {(uint8_t)(v & 0xFF), (uint8_t)(v >> 8), (uint8_t)(v & 0xFF), (uint8_t)(v >> 8), 0x00};
  canSend(0x349, d, 5);
}
#endif

// ============================================================================
//  Background TX task: 10 ms heartbeat, frames staggered so they never pile up
//  (a single 8-byte frame at 100 kbit/s occupies the bus for ~1.1 ms).
// ============================================================================
static void canTxTask(void *) {
  TickType_t last = xTaskGetTickCount();
  uint32_t t = 0;                                  // 10 ms ticks
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    t++;

    if ((t & 1) == 0) send0AA();                   // every 20 ms  (RPM)
    if (t % 10 == 3)  send1A6();                   // every 100 ms (distance)
    if (t % 10 == 7)  send1A0();                   // every 100 ms (speed)

    if (t % 10 == 4)  send130();                   // every 100 ms (terminal 15)
#if SEND_COOLANT
    if (t % 10 == 8)  send1D0();                   // every 100 ms
#endif
    if (t % 20 == 6)  send26E();                   // every 200 ms
#if SEND_WARNING_SUPPRESS
    if (t % 20 == 12) send19E();
    if (t % 20 == 14) send0C0();
    if (t % 20 == 16) send0D7();
#endif
#if SEND_FUEL
    if (t % 20 == 18) send349();
#endif

    // Drain anything the cluster transmits (diagnostic only).
    can_frame rx;
    while (mcp.readMessage(&rx) == MCP2515::ERROR_OK) g_rxCount = g_rxCount + 1;
  }
}

// ============================================================================
//  Demo sweep + serial commands (non-blocking, called from loop())
// ============================================================================
static bool demoActive = true;

/* Sweeps idle -> DEMO_RPM_MAX -> idle, dwells at idle, repeats. It never
 * reaches 0 rpm: 0 rpm with terminal 15 on is a stall to the cluster, and it
 * answers with the gong and the charge/oil/brake/DSC telltales. The old sweep
 * did that at the bottom of every ramp, so the alarm fired once per cycle. */
static void runDemo() {
  static uint32_t lastMs     = 0;
  static uint32_t dwellUntil = 0;
  static float    rpm        = ENGINE_IDLE_RPM;
  static int8_t   dir        = +1;
  uint32_t now = millis();
  if (now < DEMO_START_DELAY_MS || now - lastMs < 20) return;
  float dt = (lastMs == 0) ? 0.02f : (now - lastMs) / 1000.0f;
  lastMs = now;

  const float idle = (float)ENGINE_IDLE_RPM;

  if (dwellUntil) {
    setRPM((uint16_t)idle);
    setSpeed(0);
    if (now >= dwellUntil) { dwellUntil = 0; rpm = idle; dir = +1; }
    return;
  }

  if (rpm < idle) rpm = idle;
  rpm += dir * DEMO_RPM_PER_SECOND * dt;
  if (rpm >= DEMO_RPM_MAX) { rpm = DEMO_RPM_MAX; dir = -1; }
  if (rpm <= idle)         { rpm = idle; dwellUntil = now + DEMO_IDLE_DWELL_MS; }

  setRPM((uint16_t)rpm);

  // Speed tracks only the part of the sweep above idle, so the speedo reads 0
  // at idle and the distance counter stops adding to the odometer there.
  float span = (float)DEMO_RPM_MAX - idle;
  float frac = span > 1.0f ? (rpm - idle) / span : 0.0f;
  if (frac < 0.0f) frac = 0.0f;
  setSpeed((uint16_t)(frac * DEMO_SPEED_MAX_KMH));
}

/* Line-buffered and non-blocking. Serial.parseFloat() straight after the
 * command letter blocks for the whole stream timeout (1 s by default) when no
 * digits follow - on every 'd', and again on every trailing newline. A 1 s
 * stall in loop() is harmless here because TX lives in its own task, but it
 * still swallows commands, and it is fatal in the single-threaded Uno build. */
static void handleSerial() {
  static char    line[24];
  static uint8_t len = 0;

  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch != '\n') {
      if (len < sizeof(line) - 1) line[len++] = ch;
      continue;
    }
    line[len] = '\0';
    len = 0;

    uint8_t i = 0;
    while (line[i] == ' ' || line[i] == '\t') i++;
    if (!line[i]) continue;
    char c = line[i++];
    while (line[i] == ' ' || line[i] == '\t') i++;
    float v = atof(&line[i]);

    switch (c) {
      case 'r': demoActive = false; setRPM((uint16_t)v);   Serial.printf("RPM -> %u\n", (unsigned)g_rpm); break;
      case 's': demoActive = false; setSpeed((uint16_t)v); Serial.printf("Speed -> %u km/h\n", (unsigned)g_speedKmh); break;
      case 'f': setFuelLitres(v);                          Serial.printf("Fuel -> %.1f l\n", v); break;
      case 'c': setCoolantC((int16_t)v);                   Serial.printf("Coolant -> %d C\n", (int)v); break;
      case 'd': demoActive = true;                         Serial.println("Demo sweep resumed"); break;
      default: break;
    }
  }
}

// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nE90 cluster bench driver");

  pinMode(PIN_INT, INPUT_PULLUP);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  MCP2515::ERROR e = mcp.reset();
  Serial.printf("MCP2515 reset      : %s\n", e == MCP2515::ERROR_OK ? "OK" : "FAIL");
  e = mcp.setBitrate(CAN_SPEED, CAN_XTAL);
  Serial.printf("setBitrate 100k/8MHz: %s\n", e == MCP2515::ERROR_OK ? "OK" : "FAIL (check crystal/define)");
  e = mcp.setNormalMode();
  Serial.printf("Normal mode        : %s\n", e == MCP2515::ERROR_OK ? "OK" : "FAIL");

  if (e != MCP2515::ERROR_OK) {
    Serial.println("CAN init failed - check SPI wiring and 5V supply to the HW-184. Halting.");
    while (true) delay(1000);
  }

  xTaskCreatePinnedToCore(canTxTask, "canTx", 4096, nullptr, 5, nullptr, 1);
  Serial.println("TX task running (0x130 @100ms, 0x1A6/0x1A0 @100ms, 0x0AA @20ms)");
}

void loop() {
  handleSerial();
  if (demoActive) runDemo();

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 2000) {
    lastPrint = millis();
    Serial.printf("RPM %4u | %3u km/h | TX ok %lu fail %lu | RX %lu\n",
                  g_rpm, g_speedKmh, (unsigned long)g_txOk, (unsigned long)g_txFail,
                  (unsigned long)g_rxCount);
  }
}
