#include "Protocol.h"

bool Protocol::feed(uint8_t b, ClusterCan &can) {
  // Anything below this point is a byte we are claiming as binary, so note
  // the time first; binaryRecent() uses it to keep the console shut while a
  // frame is in flight or has just been cut short.
  if (state_ != 0 || b == cfg::HOST_SYNC0) lastByte_ = millis();

  switch (state_) {
    case 0:
      if (b != cfg::HOST_SYNC0) return false;    // plain text, not ours
      state_ = 1;
      return true;

    case 1:
      if (b == cfg::HOST_SYNC1) { state_ = 2; return true; }
      // 0xAA 0xAA means the first one was noise and this is the real start,
      // so stay here rather than throwing away a good frame.
      if (b == cfg::HOST_SYNC0) return true;
      state_ = 0;
      errors_++;
      return true;

    case 2:
      if (b != cfg::HOST_PAYLOAD) {
        errors_++;
        state_ = (b == cfg::HOST_SYNC0) ? 1 : 0;   // resync on a fresh header
        return true;
      }
      idx_ = 0;
      xor_ = b;
      state_ = 3;
      return true;

    case 3:
      buf_[idx_++] = b;
      xor_ ^= b;
      if (idx_ >= cfg::HOST_PAYLOAD) state_ = 4;
      return true;

    default:
      if (b == xor_) { apply(can); frames_++; lastRx_ = millis(); }
      else           { errors_++; }
      state_ = 0;
      return true;
  }
}

void Protocol::apply(ClusterCan &can) {
  HostState s;
  memcpy(&s, buf_, sizeof(s));            // AVR is little-endian, same as the host

  // Ignition and engine state first: setRPM() floors at idle only while the
  // engine is running, so the order here decides whether a commanded 0 rpm
  // is honoured as a stall or clamped.
  can.setTerminal15((s.flags & HF_KL15) != 0);
  can.setEngineRunning((s.flags & HF_ENGINE) != 0);

  can.setRPM(s.rpm);
  can.setSpeed(s.speedKmh);
  can.setPedal(s.pedalPct);
  can.setFuelLitres((float)s.fuelDl / 10.0f);
  can.setCoolantC(s.coolantC);
  can.setOutsideC(s.outsideC);
  can.setRangeKm(s.rangeKm);

  can.setHandbrake((s.flags & HF_HANDBRAKE) != 0);
  can.setReverse((s.flags & HF_REVERSE) != 0);
  can.setLights(s.lights);
  can.setDimmer(s.dimmer);
  can.setTurn((s.turn & 0x01) != 0, (s.turn & 0x02) != 0);
  can.setGear(s.gearPos, s.gearNum, s.gearMode);

  can.setTractionLamps((s.flags2 & HF2_TC_LAMP) != 0,
                       (s.flags2 & HF2_DSC_LAMP) != 0,
                       (s.flags2 & HF2_DSC_OFF) != 0);

  // An older host, or one with no clock of its own, simply leaves the flag
  // clear and the cluster keeps running the clock it already has.
  if (s.flags2 & HF2_CLOCK)
    can.setClock(s.hour, s.minute, s.second, s.day, s.month, s.year);
}
