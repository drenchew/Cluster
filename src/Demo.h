#pragma once
/*
 * Demo - non-blocking RPM/speed sweep: idle -> DEMO_RPM_MAX -> idle, with a
 * dwell at idle, repeating. It deliberately never passes below idle; 0 rpm
 * with terminal 15 on reads as a stall to the cluster and sets off the gong.
 * Call update() from loop(); it returns immediately when nothing is due.
 */
#include <Arduino.h>
#include "ClusterCan.h"

class Demo {
public:
  void update(ClusterCan &can);
  void stop()   { active_ = false; }
  void resume() { active_ = true; lastMs_ = 0; dwellUntil_ = 0;
                  rpm_ = (float)cfg::ENGINE_IDLE_RPM; dir_ = +1; }
  bool active() const { return active_; }

private:
  bool     active_     = true;
  uint32_t lastMs_     = 0;
  uint32_t dwellUntil_ = 0;   // non-zero while parked at idle between sweeps
  float    rpm_        = 0;
  int8_t   dir_        = +1;
};
