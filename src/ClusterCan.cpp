#include "ClusterCan.h"

// ============================================================================
//  Raw SPI register read (bypasses the library) - diagnostic only.
//  MCP2515 READ instruction = 0x03. A healthy chip returns ~0x80 (config mode)
//  from CANSTAT right after power-up.
//    reads 0x00  -> no reply: MISO not connected, module unpowered, or GND open
//    reads 0xFF  -> MISO stuck high: miswire, or module unpowered
//    reads 0x80  -> SPI link OK
// ============================================================================
uint8_t ClusterCan::rawReadReg(uint8_t addr) {
  SPI.beginTransaction(SPISettings(cfg::SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(cfg::PIN_CS, LOW);
  SPI.transfer(0x03);                 // READ
  SPI.transfer(addr);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(cfg::PIN_CS, HIGH);
  SPI.endTransaction();
  return v;
}

bool ClusterCan::tryInit() {
  if (mcp_.reset()               != MCP2515::ERROR_OK) return false;
  if (mcp_.setBitrate(cfg::BUS_BITRATE, cfg::BUS_CLOCK) != MCP2515::ERROR_OK) return false;
  if (mcp_.setNormalMode()       != MCP2515::ERROR_OK) return false;
  return true;
}

// ============================================================================
//  Setup - retries forever so you can fix wiring and watch it connect live
//  without re-flashing. Returns true once the MCP2515 is up.
// ============================================================================
bool ClusterCan::begin() {
  pinMode(cfg::PIN_INT, INPUT_PULLUP);
  pinMode(cfg::PIN_CS, OUTPUT);
  digitalWrite(cfg::PIN_CS, HIGH);
  SPI.begin();                          // Uno hardware SPI (D13/D12/D11)

  for (uint32_t attempt = 1; ; ++attempt) {
    uint8_t canstat = rawReadReg(0x0E);   // CANSTAT
    if (tryInit()) {
      logf("MCP2515 up (CANSTAT=0x%02X). Bitrate 100k / xtal 8MHz, normal mode.\n", canstat);
      lastTick_ = millis();
      return true;
    }
    logf("MCP2515 reset FAIL (attempt %lu, CANSTAT raw=0x%02X) - ",
         (unsigned long)attempt, canstat);
    if (canstat == 0x00)      Serial.println(F("0x00: no SPI reply. Check GND, 5V power, and SO->D12."));
    else if (canstat == 0xFF) Serial.println(F("0xFF: MISO stuck high. Check wiring / module power."));
    else                      Serial.println(F("partial SPI reply. Check for loose jumpers / lower SPI clock."));
    delay(1000);
  }
}

// ============================================================================
//  Public API
// ============================================================================

/* While the engine is "running" the RPM is floored at idle. A cluster that is
 * told terminal 15 is on but the crank is at 0 rpm treats it as a stall and
 * answers with the gong plus the charge/oil/brake telltales - which is what
 * made the old demo sweep alarm once per cycle at the bottom of its ramp. */
void ClusterCan::setRPM(uint16_t rpm) {
  if (rpm > cfg::RPM_LIMIT) rpm = cfg::RPM_LIMIT;
  if (engineOn_ && rpm < cfg::ENGINE_IDLE_RPM) rpm = cfg::ENGINE_IDLE_RPM;
  rpm_ = rpm;
}
void ClusterCan::setSpeed(uint16_t kmh){ speedKmh_ = kmh > cfg::SPEED_LIMIT ? cfg::SPEED_LIMIT : kmh; }
void ClusterCan::setPedal(uint8_t pct) { pedalPct_ = pct > 100 ? 100 : pct; }
void ClusterCan::setFuelLitres(float l){ fuelDl_ = (uint16_t)(constrain(l, 0.0f, 80.0f) * 10.0f); }
void ClusterCan::setCoolantC(int16_t c){ coolantC_ = constrain(c, (int16_t)-40, (int16_t)150); }
void ClusterCan::setOutsideC(int16_t c){ outsideC_ = constrain(c, (int16_t)-40, (int16_t)80); }
void ClusterCan::setRangeKm(uint16_t k){ rangeKm_ = k > 2000 ? 2000 : k; }
void ClusterCan::setLights(uint8_t m)  { lights_ = m; }
void ClusterCan::setDimmer(uint8_t l)  { dimmer_ = l; }
void ClusterCan::setHandbrake(bool on) { handbrake_ = on; }
void ClusterCan::setReverse(bool on)   { reverse_ = on; }

void ClusterCan::setOdoMode(uint8_t m) {
  odoMode_ = m > cfg::ODO_DISTANCE ? cfg::ODO_ALIVE : m;
  distAccum_ = 0.0f;
}

void ClusterCan::setSpeedSources(uint8_t m) { speedSrc_ = m & 0x03; }

void ClusterCan::setTurn(bool left, bool right) {
  turn_ = (uint8_t)((left ? 0x01 : 0) | (right ? 0x02 : 0));
}

void ClusterCan::setGear(uint8_t pos, uint8_t num, uint8_t mode) {
  gearPos_  = pos;
  gearNum_  = num > 8 ? 8 : num;
  gearMode_ = mode > MODE_MANUAL ? MODE_NORMAL : mode;
}

void ClusterCan::setEngineRunning(bool running) {
  engineOn_ = running;
  if (running) { if (rpm_ < cfg::ENGINE_IDLE_RPM) rpm_ = cfg::ENGINE_IDLE_RPM; }
  else         { rpm_ = 0; }          // deliberate key-on-engine-off: expect the gong
}

void ClusterCan::setTerminal15(bool on) { kl15_ = on; }

// ============================================================================
//  Scheduler: 10 ms tick. 0x0AA goes out on every even tick (20 ms); 0x1A6 on
//  slots 7 and 17 (100 ms - see cfg::TICK_1A6_STEP: the cluster differentiates
//  this frame, so sending it faster than the real cadence inflates the speed).
//  Everything else sits in a numbered slot of the 200 ms (20 tick) cycle so no
//  single tick ever carries more than two frames. An 8-byte frame at
//  100 kbit/s occupies the bus for ~1.1 ms, so two per 10 ms tick is ~22% load.
// ============================================================================
void ClusterCan::update() {
  uint32_t now = millis();
  if (now - lastTick_ < cfg::TX_TICK_MS) return;
  lastTick_ = now;
  tick_++;

  if ((tick_ & 1) == 0) send0AA();               // every 20 ms, direct value

  uint8_t slot = (uint8_t)(tick_ % 20);
  switch (slot) {
    case 0:  case 10: send130(); break;                 // 100 ms terminal status
    case 1:  case 11: if (speedSrc_ & cfg::SPEED_SRC_1A0) send1A0(); break;
    case 2:  case 12: if (cfg::SEND_LIGHTS) send21A(); break;   // 100 ms illumination
    case 4:  case 14: if (cfg::SEND_BRAKE)  send1B4(); break;   // 100 ms handbrake
    case 8:  case 18: if (speedSrc_ & cfg::SPEED_SRC_0CE) send0CE(); break;
    case 7:  if (cfg::SEND_WARNING_SUPPRESS) send19E();
             send1A6(); break;                          // 100 ms distance accumulator
    case 17: if (cfg::SEND_DIMMER) send202();
             send1A6(); break;                          // 100 ms distance accumulator
    case 3:  send26E(); break;                          // 200 ms ignition
    case 5:  if (cfg::SEND_COOLANT)          send1D0(); break;
    case 6:  if (cfg::SEND_WARNING_SUPPRESS) send0C0(); break;
    case 9:  if (cfg::SEND_FUEL)             send349(); break;
    case 13: if (cfg::SEND_TURN)             send1F6(); break;
    case 15: if (cfg::SEND_WARNING_SUPPRESS) send0D7(); break;
    case 16: if (cfg::SEND_GEAR)             send1D2(); break;
    case 19:
      if (cfg::SEND_REVERSE)   send3B0();
      if (cfg::SEND_LCD_LIGHT) send2C0();
      break;
    default: break;
  }
  // 0x34F is event driven in a real car: on a state change, plus a keepalive
  // roughly every 5 s. The capture shows exactly that pattern (FD/FE bursts on
  // the gear change, then one frame per 5 s while the state holds).
  if (cfg::SEND_PARK_BRAKE && (handbrake_ != hb34F_ || (tick_ % 500) == 6)) {
    hb34F_ = handbrake_;
    send34F();
  }
  if (cfg::SEND_RANGE && (tick_ % 100) == 6) send366();  // 1 s
  drainRx();
}

void ClusterCan::drainRx() {
  can_frame f;
  while (mcp_.readMessage(&f) == MCP2515::ERROR_OK) rx_ = rx_ + 1;
}

bool ClusterCan::send(uint32_t id, const uint8_t *data, uint8_t len) {
  can_frame f;
  memset(&f, 0, sizeof(f));
  f.can_id  = id;
  f.can_dlc = len;
  memcpy(f.data, data, len);
  bool ok = (mcp_.sendMessage(&f) == MCP2515::ERROR_OK);
  if (ok) txOk_ = txOk_ + 1; else txFail_ = txFail_ + 1;
  return ok;
}

uint16_t ClusterCan::speedRaw() const {
  uint16_t raw = (uint16_t)((float)speedKmh_ / cfg::SPEED_KMH_PER_COUNT);
  return raw > 0x0FFF ? 0x0FFF : raw;
}

// ============================================================================
//  Frame builders
// ============================================================================

/* 0x130  Terminal status (CAS), 5 bytes, 100 ms
 *   B0 0x45 key status   B1 0x40 transponder   B2 0x21 terminal status (KL15 on)
 *   B3 0x8F steering lock/misc
 *   B4 packed counter + check nibble; bit 1 = IgnitionOn
 *
 * B4 is held CONSTANT. It is not a free-running counter you can drive on its
 * own: the high nibble is a sequence number and the low nibble is a check
 * value derived from it, so incrementing one without the other yields a byte
 * the cluster rejects. It then sees terminal status as implausible and re-runs
 * its power-on sequence (gong + every telltale) over and over. Both payloads
 * the community reports working on an E9x KOMBI keep B4 fixed - 0xFE from a
 * raw bus capture, 0xE2 from the e46canbus blog. */
void ClusterCan::send130() {
  uint8_t b4 = kl15_ ? cfg::T130_BYTE4_ON : cfg::T130_BYTE4_OFF;
  if (cfg::T130_ROLLING_COUNTER && kl15_) {
    b4 = (uint8_t)((cnt130_ << 4) | (b4 & 0x0F));
    cnt130_ = (cnt130_ + 1) % 15;
  }
  uint8_t d[5] = {(uint8_t)(kl15_ ? 0x45 : 0x00), 0x40, 0x21, 0x8F, b4};
  send(0x130, d, 5);
}

/* 0x26E  Ignition status, 8 bytes, 200 ms (static, from a working bench sketch) */
void ClusterCan::send26E() {
  const uint8_t d[8] = {0x40, 0x40, 0x7F, 0x50, 0xFF, 0xFF, 0xFF, 0xFF};
  send(0x26E, d, 8);
}

/* 0x0AA  DME engine speed / pedal, 8 bytes, 20 ms  -> RPM NEEDLE
 *   B0 unknown (0xFE)   B1 0xF0 | counter 0..14 (DBC: Counter1 = B1 low nibble)
 *   B2..B3 accelerator pedal, 16-bit LE, 0.01 %/count (DBC AccelPedalPercentage)
 *   B4..B5 = RPM * 4, little endian
 *   B6 pedal/cruise flag bits: bit2 ThrottlePressed, bit4 AccelPedalPressed
 *   B7 0x99 */
void ClusterCan::send0AA() {
  uint16_t raw   = (uint16_t)rpm_ * 4;
  uint16_t pedal = (uint16_t)pedalPct_ * 100;   // percent -> 0.01 % counts
  uint8_t  b6    = 0xFE;
  if (pedalPct_ == 0) b6 &= (uint8_t)~0x14;     // clear pressed bits at 0 %
  uint8_t d[8] = {0xFE, (uint8_t)(0xF0 | cnt0AA_),
                  (uint8_t)(pedal & 0xFF), (uint8_t)(pedal >> 8),
                  (uint8_t)(raw & 0xFF), (uint8_t)(raw >> 8), b6, 0x99};
  cnt0AA_ = (cnt0AA_ + 1) % 15;
  send(0x0AA, d, 8);
}

/* 0x1A6  Distance accumulator + time counter, 8 bytes, 100 ms  -> NEEDLE
 *   B0..B5 three copies of a 16-bit little-endian cumulative distance counter
 *   B6..B7 12-bit time counter, +400 per frame, B7 = 0xF0 | high nibble
 *
 * This is NOT a speed frame. Verified on a real E90 M3 capture (1154 frames):
 * the three words are independent accumulators climbing 0 -> 8793 while the
 * car does 0 -> 15.9 km/h, at 12.3..13.7 counts per km/h per second, and they
 * read 00 00 when parked. The old jig sent a CONSTANT speed word here; the
 * cluster differentiates the field, a constant differentiates to 0, and the
 * speedometer needle never moved while every direct-value gauge worked.
 *
 * B6..B7 is a separate free-running time counter: +400 per 100 ms frame
 * (mean delta 400.0 over all frames), wraps at 12 bits, B7 high nibble always
 * 0xF. The jig's old +1 per 20 ms was ~80x too slow.
 *
 * Cadence matters: sending this frame faster than 100 ms makes the derived
 * speed proportionally too fast, so it lives in scheduler slots 7 and 17. */
void ClusterCan::send1A6() {
  uint16_t kmh = speedKmh_;

  uint32_t now = millis();
  uint32_t dtMs = now - lastDist_;
  lastDist_ = now;
  if (dtMs > 500) dtMs = 500;                 // first call / after a stall

  if (odoMode_ == cfg::ODO_DISTANCE && kmh) {
    // counts = km/h * counts per km/h per second * seconds
    distAccum_ += (float)kmh * cfg::ODO_COUNTS_PER_KMH_PER_SEC
                  * ((float)dtMs / 1000.0f);
    uint16_t whole = (uint16_t)distAccum_;
    distAccum_ -= (float)whole;
    dist1A6_ = (uint16_t)(dist1A6_ + whole);   // 16-bit wrap, like the car
  } else if (odoMode_ == cfg::ODO_ALIVE) {
    dist1A6_ = (uint16_t)(dist1A6_ + cfg::ODO_ALIVE_STEP);
  }                                            // ODO_FROZEN: hold

  tick1A6_ = (uint16_t)(tick1A6_ + cfg::TICK_1A6_STEP);
  uint8_t lo = dist1A6_ & 0xFF;
  uint8_t hi = dist1A6_ >> 8;

  uint8_t d[8] = {lo, hi, lo, hi, lo, hi,
                  (uint8_t)(tick1A6_ & 0xFF),
                  (uint8_t)(0xF0 | ((tick1A6_ >> 8) & 0x0F))};
  send(0x1A6, d, 8);
}

/* 0x1A0  DSC vehicle dynamics / "Speed K-CAN", 8 bytes, 100 ms  -> NEEDLE
 *   B0..B1 12-bit V_VEH, 0.1 km/h per count, Intel order
 *          B1 low nibble = value high bits; B1 bits 4..6 = ST_VEH_DVCO
 *          (0 when V_VEH is 0, 1 once moving - verified over the capture);
 *          B1 bit 7 = 0x80 set in every real frame
 *   B2..B5 acceleration / yaw rate - sent as 0, which is what the working
 *          veikkos/e90-can-cluster driver sends and what the cluster accepts
 *   B6     low nibble = yaw high bits (0 with yaw 0); high nibble = ALIV_V,
 *          a time-based alive nibble: 5-6 steps per 100 ms frame in the
 *          capture, exactly what ((millis()%300)*16/300)&0x0F produces. NOT a
 *          per-frame +1: only 118 of 1157 real frames conform to that.
 *   B7     checksum, verified 1158/1158 on the capture:
 *            sum = 0x00A0 + B0+..+B6;  B7 = (sum + (sum>>8) + 0x01) & 0xFF
 *
 * Enabled by default (cfg::SPEED_SOURCES): real DSC sends this every 100 ms
 * alongside 0x1A6, and it carries the direct speed value the needle reads. */
void ClusterCan::send1A0() {
  uint16_t raw = (uint16_t)((float)speedKmh_ / cfg::SPEED_1A0_PER_COUNT);
  if (raw > 0x0FFF) raw = 0x0FFF;
  uint8_t st = raw ? 1 : 0;
  uint8_t aliv = (uint8_t)(((millis() % 300) * 16 / 300) & 0x0F);

  uint8_t d[8];
  d[0] = (uint8_t)(raw & 0xFF);
  d[1] = (uint8_t)(((raw >> 8) & 0x0F) | (st << 4) | 0x80);
  d[2] = 0x00;                                  // acc_long
  d[3] = 0x00;                                  // acc_long high | acc_lat low
  d[4] = 0x00;                                  // acc_lat high
  d[5] = 0x00;                                  // yaw rate
  d[6] = (uint8_t)(aliv << 4);                  // yaw high | ALIV_V
  uint16_t sum = 0x00A0;
  for (uint8_t i = 0; i < 7; i++) sum = (uint16_t)(sum + d[i]);
  d[7] = (uint8_t)(sum + (sum >> 8) + 0x01);
  send(0x1A0, d, 8);
}

/* 0x0CE  DSC wheel speeds, 8 bytes, 100 ms  -> alternative speed source
 * Four 16-bit little-endian wheel speeds (FL, FR, RL, RR), all sent equal.
 * Scaling and sign are capture-verified - see cfg::SPEED_0CE_PER_COUNT.
 * On by default now (cfg::SPEED_SOURCES). */
void ClusterCan::send0CE() {
  uint16_t raw = (uint16_t)((float)speedKmh_ / cfg::SPEED_0CE_PER_COUNT);
  uint8_t lo = raw & 0xFF, hi = raw >> 8;
  uint8_t d[8] = {lo, hi, lo, hi, lo, hi, lo, hi};
  send(0x0CE, d, 8);
}

/* 0x1B4  Brake / speed, 8 bytes, 100 ms  -> HANDBRAKE telltale
 *   B0..B1 12-bit speed (0.1-ish km/h/count) with B1 bits 14..15 = 0xC0
 *   B2 0xE2   B3 0xF0 | counter   B4 0x00
 *   B5 handbrake: 0x30 released, 0x32 engaged
 *   B6 0xFC released / 0xFE engaged   B7 counter
 *
 * Verified over 1154 real frames: B1 = 0xC0 in ALL of them (the 0xC0 pair are
 * static flag bits, not speed - they are set even with the car parked and the
 * speed field at 0), B2 = 0xE2 in ALL of them, and B6 tracks B5 exactly -
 * B5=0x30 always pairs with B6=0xFC (869 frames) and B5=0x32 with B6=0xFE
 * (245 frames). Sending B1 without 0xC0 means transmitting a speed signal the
 * car never sends.
 *
 * B5 has two independent confirmations: a bench capture uses 0x30, and the DBC
 * puts HandbrakeStatus1 at B5 with scaling (0.5, -24) - so 48 (0x30) decodes
 * to 0 "Disengaged" and 50 (0x32) to 1 "Engaged".
 *
 * B0..B1 here reads ~17% higher than 0x1A0's V_VEH at the same instant in the
 * capture (mean ratio 1.17 over 663 matched frame pairs), so they are two
 * different filtered signals - do not expect them to agree exactly. */
void ClusterCan::send1B4() {
  uint16_t raw = speedRaw();
  uint8_t d[8] = {(uint8_t)(raw & 0xFF), (uint8_t)(0xC0 | ((raw >> 8) & 0x0F)),
                  0xE2, (uint8_t)(0xF0 | cnt1B4_), 0x00,
                  (uint8_t)(handbrake_ ? 0x32 : 0x30),
                  (uint8_t)(handbrake_ ? 0xFE : 0xFC), cnt1B4_};
  cnt1B4_ = (cnt1B4_ + 1) % 15;
  send(0x1B4, d, 8);
}

/* 0x1D0  Engine data, 8 bytes, 100 ms
 *   B0 = coolant + 48 degC (DBC confirmed: EngineCoolantTemp = raw - 48)
 *   B1 = oil + 48 degC, same offset (veikkos driver + capture both)
 *   B2 = (alive & 0x0F) | engine_run_state << 4, state 0 off / 1 starting /
 *        2 running / 3 invalid
 *   B6/B7 static filler
 *
 * B2 used to be a raw byte counter, cnt1D0_++, so after 16 frames its upper
 * nibble - the engine state - started rolling through off / starting /
 * invalid for a quarter of every 256 frames while the cluster was being told
 * KL15 on and engine running. A cluster told that keeps gonging and lighting
 * the charge / oil / brake telltales (see cfg::ENGINE_IDLE_RPM), so the state
 * nibble has to be pinned and only the low nibble counted. */
void ClusterCan::send1D0() {
  int16_t oil = coolantC_ - 20;                  // capture runs ~20 C below coolant
  if (oil < -48) oil = -48;
  uint8_t state = engineOn_ ? 0x20 : 0x00;       // 0x02 << 4 = running
  uint8_t d[8] = {(uint8_t)(coolantC_ + 48), (uint8_t)(oil + 48),
                  (uint8_t)((cnt1D0_ = (cnt1D0_ + 1) & 0x0F) | state),
                  0xFF, 0xFF, 0xFF, 0xCC, 0xFF};
  send(0x1D0, d, 8);
}

/* 0x19E DSC status, 8 bytes, 200 ms
 *   B2 high nibble rotates (counter % 3), low nibble 0x03
 *   B3 0xFC   B4 0xCC   B5 0x00   B6 0x00
 *   B7 rolling counter
 *
 * This used to be a frozen payload {00 E0 B3 FC F0 43 00 65}: no counter
 * anywhere, so a DSC that never once changes its status frame reads as a dead
 * sender - and B5=0x43 appears in none of the 1154 real frames (real values
 * are 0x00, 0x41, 0x61, 0x40). The bytes here are the veikkos/e90-can-cluster
 * driver's, which is the one open-source payload proven to keep an E9x KOMBI
 * happy. The raw capture uses B2 low nibble 0x0F / B4 0xFE instead, so both
 * variants evidently pass - what neither does is stop counting. */
void ClusterCan::send19E() {
  uint8_t d[8] = {0x00, 0xE0, (uint8_t)(((cnt19E_ % 3) << 4) | 0x03),
                  0xFC, 0xCC, 0x00, 0x00, cnt19E_++};
  send(0x19E, d, 8);
}

/* 0x34F  Handbrake status, 2 bytes [OK]  -> PARK-BRAKE telltale + gong
 *   B0 0xFD released, 0xFE engaged      B1 0xFF
 * DLC 2 confirmed on the capture (22 frames: 20 x "FD FF" released while
 * driving, 2 x "FE FF" parked), and 0x1B4 B5/B6 says the same thing with a
 * different signal. Sent on change plus a 5 s keepalive - see update(). */
void ClusterCan::send34F() {
  uint8_t d[2] = {(uint8_t)(handbrake_ ? 0xFE : 0xFD), 0xFF};
  send(0x34F, d, 2);
}

/* 0x0C0 ABS counter: B0 cycles 0xF0..0xFF, B1 0xFF */
void ClusterCan::send0C0() {
  uint8_t d[2] = {cnt0C0_, 0xFF};
  cnt0C0_ = (cnt0C0_ == 0xFF) ? 0xF0 : cnt0C0_ + 1;
  send(0x0C0, d, 2);
}

/* 0x0D7 airbag counter: B0 counter, B1 0xFF */
void ClusterCan::send0D7() {
  uint8_t d[2] = {cnt0D7_++, 0xFF};
  send(0x0D7, d, 2);
}

/* 0x349 fuel level: litres*160 (16-bit LE) twice (two tank halves), B4 = 0.
 * DLC is 5, not 8 - that is the real frame length (the DBC gives 0x349 five
 * bytes: two 16-bit sensors at 0.00165108 gal/count, i.e. exactly 160
 * counts/litre). Sending DLC 8 makes the cluster discard the frame, so the
 * gauge stays dead and the low-fuel check-control warning keeps chiming. */
void ClusterCan::send349() {
  uint16_t raw = (uint16_t)(fuelDl_ / 10.0f * 160.0f);
  uint8_t d[5] = {(uint8_t)(raw & 0xFF), (uint8_t)(raw >> 8),
                  (uint8_t)(raw & 0xFF), (uint8_t)(raw >> 8), 0x00};
  send(0x349, d, 5);
}

/* 0x21A  FRM lighting status, 3 bytes, 100 ms  -> BACKLIGHT + light telltales
 * The bench-verified pair is B0=0x05 B1=0x10 B2=0x00 for lights on and
 * 00 00 00 for off. 0x05 is LIGHT_SIDE|LIGHT_LOW, so sending the mask
 * reproduces that payload exactly when both are set; the other bits are
 * inferred and may not map the way the names suggest. */
void ClusterCan::send21A() {
  bool on = lights_ != 0;
  uint8_t d[3] = {lights_, (uint8_t)(on ? cfg::L21A_B1_ON : 0x00), 0x00};
  send(0x21A, d, 3);
}

/* 0x202  FRM dimmer status, 2 bytes [?] - layout is a guess: B0 = level.
 * The cluster computes final brightness from its own ambient phototransistor
 * combined with this dimmer signal, so expect limited effect on a bench until
 * that sensor is covered. */
void ClusterCan::send202() {
  uint8_t d[2] = {dimmer_, 0x00};
  send(0x202, d, 2);
}

/* 0x2C0  FRM LCD illumination, 3 bytes [?] - same caveat as 0x202. */
void ClusterCan::send2C0() {
  uint8_t d[3] = {dimmer_, 0x00, 0x00};
  send(0x2C0, d, 3);
}

/* 0x1F6  Turn signals, 2 bytes  -> INDICATOR telltales + clicking
 *   B0 0x80 base | 0x10 left | 0x20 right | 0x01
 *   B1 0xF2 on the first frame of a new blink, 0xF1 while it continues
 * Derived from a bench capture (left 0x91/0xF2 then 0x91/0xF1, right swaps
 * 0x91 for 0xA1) and corroborated by the DBC, which puts TurnSignal_Left at
 * bit 4 and TurnSignal_Right at bit 5 - exactly the 0x10/0x20 difference. */
void ClusterCan::send1F6() {
  uint8_t t = turn_;
  uint8_t b0 = 0x80;
  if (t) b0 |= 0x01 | (uint8_t)((t & 0x01) ? 0x10 : 0) | (uint8_t)((t & 0x02) ? 0x20 : 0);
  uint8_t b1 = t ? (uint8_t)((t != lastTurn_) ? 0xF2 : 0xF1) : 0xF0;
  lastTurn_ = t;
  uint8_t d[2] = {b0, b1};
  send(0x1F6, d, 2);
}

/* 0x1D2  Gear display, 8 bytes [?]  -> P R N D and D1..D6 / S1..S6 / M1..M6
 *   B0 low nibble = ShiftLeverPosition (1=P 2=R 4=N 8=D), high nibble = its
 *      XOR complement; B1 high nibble = gear number (DBC "GearRelated_TBD");
 *   B3 high nibble = counter, bit2 = sport button; B4 low 2 bits = mode. */
void ClusterCan::send1D2() {
  uint8_t pos = gearPos_ & 0x0F;
  uint8_t d[8] = {(uint8_t)(((~pos & 0x0F) << 4) | pos),
                  (uint8_t)((gearNum_ << 4) | 0x0F),
                  0xFF,
                  (uint8_t)((cnt1D2_ << 4) | (gearMode_ == MODE_SPORT ? 0x04 : 0x00)),
                  gearMode_, 0xFF, 0xFF, 0xFF};
  cnt1D2_ = (cnt1D2_ + 1) % 15;
  send(0x1D2, d, 8);
}

/* 0x3B0  Reverse gear, 2 bytes [?] - DBC puts ReverseGear at bit 6 of B0. */
void ClusterCan::send3B0() {
  uint8_t d[2] = {(uint8_t)(reverse_ ? 0x40 : 0x00), 0xFF};
  send(0x3B0, d, 2);
}

/* 0x366  Outside temperature + remaining range, 4 bytes [?]
 *   B0 = (degC + 80) * 2        (DBC: factor 0.5, offset -80)
 *   B1..B2 = range in miles * 16 (DBC: factor 0.0625, unit miles)
 * The DBC lists KOMBI as the sender, meaning the cluster may measure outside
 * temperature itself - in which case this frame does nothing. Default off. */
void ClusterCan::send366() {
  uint16_t miles = (uint16_t)((uint32_t)rangeKm_ * 1000UL / 1609UL);
  uint16_t rraw  = miles * 16;
  uint8_t d[4] = {(uint8_t)((outsideC_ + 80) * 2),
                  (uint8_t)(rraw & 0xFF), (uint8_t)(rraw >> 8), 0x00};
  send(0x366, d, 4);
}
