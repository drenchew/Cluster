#include "Demo.h"

/* Sweeps idle -> DEMO_RPM_MAX -> idle, dwells at idle, repeats.
 *
 * The sweep never reaches 0 rpm. Dropping the crank speed to 0 while 0x130
 * still reports terminal 15 on is a stall as far as the cluster is concerned,
 * and it answers the way a real car does: gong plus the charge, oil-pressure,
 * brake and DSC telltales. The old sweep did that at the bottom of every ramp,
 * which is why the whole alarm sequence fired once per cycle, forever. */
void Demo::update(ClusterCan &can) {
  if (!active_) return;
  uint32_t now = millis();
  if (now < cfg::DEMO_START_DELAY_MS) return;
  if (now - lastMs_ < 20) return;

  float dt = (lastMs_ == 0) ? 0.02f : (now - lastMs_) / 1000.0f;
  lastMs_ = now;

  const float idle = (float)cfg::ENGINE_IDLE_RPM;

  if (dwellUntil_) {                        // sitting at idle between sweeps
    can.setRPM((uint16_t)idle);
    if (cfg::DEMO_SWEEP_SPEED) can.setSpeed(0);
    if (now >= dwellUntil_) { dwellUntil_ = 0; rpm_ = idle; dir_ = +1; }
    return;
  }

  if (rpm_ < idle) rpm_ = idle;             // first step after the start delay
  rpm_ += (float)dir_ * cfg::DEMO_RPM_PER_SECOND * dt;

  if (rpm_ >= (float)cfg::DEMO_RPM_MAX) { rpm_ = (float)cfg::DEMO_RPM_MAX; dir_ = -1; }
  if (rpm_ <= idle) {
    rpm_ = idle;
    dwellUntil_ = now + cfg::DEMO_IDLE_DWELL_MS;
  }

  can.setRPM((uint16_t)rpm_);

  if (cfg::DEMO_SWEEP_SPEED) {
    // Speed tracks the part of the sweep above idle, so the speedo reads 0
    // whenever the engine is at idle - and the 0x1A6 distance counter, which
    // is integrated from this speed, stops adding to the odometer there too.
    float span = (float)cfg::DEMO_RPM_MAX - idle;
    float frac = span > 1.0f ? (rpm_ - idle) / span : 0.0f;
    if (frac < 0.0f) frac = 0.0f;
    can.setSpeed((uint16_t)(frac * (float)cfg::DEMO_SPEED_MAX_KMH));
  }
}
