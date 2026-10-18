#pragma once
/*
 * Protocol - binary setpoint link from the PC simulator.
 *
 * Wire format, little-endian, 24 bytes total:
 *     0xAA 0x55  <len=20>  <20 payload bytes>  <xor>
 * where xor covers the length byte and the payload.
 *
 * Design rule: the host NEVER drives CAN timing. It only pushes setpoints
 * into ClusterCan, which keeps building and pacing frames on its own. If the
 * host stops sending - crash, unplugged cable, closed window - hostLive()
 * goes false after cfg::HOST_TIMEOUT_MS and main() falls back to the local
 * demo sweep. The 0x130 heartbeat never stops, so the cluster never resets.
 *
 * Typed text commands still work on the same port: a line of ASCII cannot
 * start with 0xAA, so the two are told apart by the first byte.
 */
#include <Arduino.h>
#include "Config.h"
#include "ClusterCan.h"

// Keep this layout byte-for-byte in sync with host/cluster_sim/link.py.
// Packed explicitly: the host packs with struct.pack("<...") and no padding,
// and the static_assert below fails the build if the two ever drift apart.
struct __attribute__((packed)) HostState {
  uint16_t rpm;         // engine speed, rpm
  uint16_t speedKmh;    // vehicle speed, km/h
  uint16_t fuelDl;      // fuel, 0.1 l
  int16_t  coolantC;    // coolant, degC
  int16_t  outsideC;    // outside air, degC
  uint16_t rangeKm;     // remaining range, km
  uint8_t  flags;       // see HF_* below
  uint8_t  lights;      // cfg::LIGHT_* bitmask
  uint8_t  turn;        // bit0 left, bit1 right (both = hazards)
  uint8_t  gearPos;     // GearPos enum: 1=P 2=R 4=N 8=D
  uint8_t  gearNum;     // 1..6 displayed, 0 = blank
  uint8_t  gearMode;    // GearMode enum: 0 normal, 1 sport, 2 manual
  uint8_t  dimmer;      // 0..255 instrument illumination
  uint8_t  pedalPct;    // accelerator 0..100 -> 0x0AA pedal bytes
};                      // = 20 bytes, matches cfg::HOST_PAYLOAD

static_assert(sizeof(HostState) == cfg::HOST_PAYLOAD,
              "HostState size must equal cfg::HOST_PAYLOAD and host/link.py _FMT");

enum HostFlags : uint8_t {
  HF_KL15      = 0x01,  // terminal 15 on
  HF_ENGINE    = 0x02,  // engine running (vs key-on-engine-off)
  HF_HANDBRAKE = 0x04,
  HF_REVERSE   = 0x08,
};

class Protocol {
public:
  // Feeds one byte. Returns true if it was consumed as part of a binary
  // frame; false means "this is console text, hand it to the line parser".
  bool feed(uint8_t b, ClusterCan &can);

  bool hostLive() const {
    return lastRx_ && (millis() - lastRx_) < cfg::HOST_TIMEOUT_MS;
  }

  // True just after any byte was taken as part of a binary frame, whether or
  // not that frame completed. A frame truncated by a dropped byte leaves its
  // remaining payload bytes in the stream, and those can spell anything - so
  // the console must stay shut for a moment afterwards even before the first
  // good frame has arrived and hostLive() turns true.
  bool binaryRecent() const {
    return lastByte_ && (millis() - lastByte_) < BINARY_QUIET_MS;
  }
  uint32_t frames() const { return frames_; }
  uint32_t errors() const { return errors_; }

private:
  // One frame at 50 Hz is 20 ms apart, so a quarter second covers a frame
  // interrupted mid-flight without muting a console you are actually typing at.
  static const uint32_t BINARY_QUIET_MS = 250;

  void apply(ClusterCan &can);

  uint8_t  buf_[cfg::HOST_PAYLOAD];
  uint8_t  state_  = 0;     // 0 idle, 1 got sync0, 2 got sync1, 3 payload, 4 xor
  uint8_t  idx_    = 0;
  uint8_t  xor_    = 0;
  uint32_t lastRx_   = 0;   // last *valid* frame
  uint32_t lastByte_ = 0;   // last byte consumed as binary
  uint32_t frames_ = 0, errors_ = 0;
};
