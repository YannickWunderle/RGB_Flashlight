#pragma once

#include "fan_config.h"

// LED power is normalised so that full white (R+G+B = 3 * 255) is 1.0; heat is assumed equal per
// PWM count on each channel. Values below are fitted to the 2026-11-07 fixture run with the fan at
// 255: about 29 C per unit of power, time constant about 122 s, effective deadtime about 10 s.
#ifndef DERATE_TARGET_C
#define DERATE_TARGET_C 60.0f
#endif
#ifndef DERATE_RELEASE_C
#define DERATE_RELEASE_C 60.0f  // a trip at FAN_TRIP_C cuts the LEDs until the mean is below this
#endif
#ifndef DERATE_FAULT_CAP
#define DERATE_FAULT_CAP 0.5f  // about 15 C above room at fan 255
#endif
// SIMC tuning, closed-loop time constant 30 s: Kp = tau / (K * (tauC + theta)), Ti = tau.
#ifndef DERATE_KP
#define DERATE_KP 0.106f  // power per C
#endif
#ifndef DERATE_KI
#define DERATE_KI 0.00087f  // power per C per s
#endif
#ifndef DERATE_UPDATE_MS
#define DERATE_UPDATE_MS 1000UL
#endif
#ifndef LED_POWER_WINDOW_US
#define LED_POWER_WINDOW_US 1000000UL
#endif
#ifndef LED_POWER_TAU_US
#define LED_POWER_TAU_US 5000000UL
#endif

// Time-weighted average of the requested LED power, so strobe modes count with their duty cycle.
struct LedPowerMeter {
  float level;
  float energy;  // power * microseconds in the current window
  float avg;
  unsigned long lastUs;
  unsigned long windowStartUs;
  bool started;

  void reset() {
    level = 0;
    energy = 0;
    avg = 0;
    started = false;
  }

  void accumulate(unsigned long nowUs) {
    if (!started) {
      started = true;
      lastUs = nowUs;
      windowStartUs = nowUs;
      return;
    }
    energy += level * (float)(nowUs - lastUs);
    lastUs = nowUs;
  }

  void set(float p, unsigned long nowUs) {
    accumulate(nowUs);
    level = p;
  }

  void tick(unsigned long nowUs) {
    accumulate(nowUs);
    unsigned long span = nowUs - windowStartUs;
    if (span >= LED_POWER_WINDOW_US) {
      float window = energy / (float)span;
      avg += (window - avg) * (float)span / (float)(LED_POWER_TAU_US + span);
      energy = 0;
      windowStartUs = nowUs;
    }
  }

  float average() const { return avg; }
};

// Caps the average LED power to hold the heatsink at DERATE_TARGET_C. While the cap does not
// limit ("open") the controller tracks the current demand, so it starts acting from the demand
// the moment the box is too hot and never has to sink through a range where it has no effect.
struct LedDerate {
  float cap;   // allowed average power; 1.0 when open
  float u;     // controller output while closed
  float ePrev;
  bool open;
  bool tripped;
  bool resync;
  bool started;
  unsigned long lastMs;

  void reset() {
    cap = 1.0f;
    u = 1.0f;
    ePrev = 0;
    open = true;
    tripped = false;
    resync = true;
    started = false;
    lastMs = 0;
  }

  float scale(float demand) const { return demand > cap ? cap / demand : 1.0f; }

  float update(float meanC, float instC, bool sensorFault, float demand, unsigned long nowMs) {
    if (!sensorFault && instC >= FAN_TRIP_C) {
      tripped = true;
    }
    if (sensorFault) {
      cap = tripped ? 0.0f : DERATE_FAULT_CAP;
      resync = true;
      return cap;
    }
    if (tripped) {
      if (meanC >= DERATE_RELEASE_C) {
        cap = 0.0f;
        resync = true;
        return cap;
      }
      tripped = false;
    }
    if (started && !resync && nowMs - lastMs < DERATE_UPDATE_MS) {
      return cap;
    }
    started = true;
    lastMs = nowMs;

    const float e = DERATE_TARGET_C - meanC;
    if (resync) {
      resync = false;
      ePrev = e;
      u = cap;
      open = false;
    }
    if (open) {
      if (e >= 0) {
        ePrev = e;
        u = demand;
        return cap;
      }
      open = false;
      u = demand;
    }
    u += DERATE_KP * (e - ePrev) + DERATE_KI * e * (DERATE_UPDATE_MS / 1000.0f);
    ePrev = e;
    if (u < 0) {
      u = 0;
    }
    if (u >= demand || u >= 1.0f) {
      open = true;
      u = demand;
      cap = 1.0f;
    } else {
      cap = u;
    }
    return cap;
  }
};
