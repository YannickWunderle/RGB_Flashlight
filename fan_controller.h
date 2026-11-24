#pragma once

#include "fan_config.h"

struct FanController {
  bool started;
  bool running;
  bool kicking;
  unsigned long lastMs;
  unsigned long kickStartMs;
  float integral;
  float dutyF;

  void reset() {
    started = false;
    running = false;
    kicking = false;
    lastMs = 0;
    kickStartMs = 0;
    integral = 0;
    dutyF = 0;
  }

  int duty() const { return (int)(dutyF + 0.5f); }

  int update(float tempC, bool sensorFault, unsigned long nowMs) {
    if (sensorFault || tempC >= FAN_TRIP_C) {
      running = true;
      kicking = false;
      integral = 0;
      dutyF = 255;
      return 255;
    }
    if (started && nowMs - lastMs < FAN_UPDATE_MS) return duty();
    started = true;
    lastMs = nowMs;

    if (!running) {
      if (tempC > FAN_SETPOINT_C + FAN_ON_MARGIN_C) {
        running = true;
        kicking = true;
        kickStartMs = nowMs;
        integral = 0;
        dutyF = FAN_KICK_DUTY;
      } else {
        dutyF = 0;
      }
      return duty();
    }

    if (tempC < FAN_SETPOINT_C - FAN_OFF_MARGIN_C) {
      running = false;
      kicking = false;
      integral = 0;
      dutyF = 0;
      return 0;
    }

    if (kicking) {
      if (nowMs - kickStartMs < FAN_KICK_MS) {
        dutyF = FAN_KICK_DUTY;
        return duty();
      }
      kicking = false;
    }

    const float dt = FAN_UPDATE_MS / 1000.0f;
    const float e = tempC - FAN_SETPOINT_C;
    integral += FAN_KI * e * dt;
    if (integral < 0) integral = 0;
    if (integral > 255 - FAN_RUN_DUTY) integral = 255 - FAN_RUN_DUTY;

    float target = FAN_RUN_DUTY + FAN_KP * e + integral;
    if (target < FAN_RUN_DUTY) target = FAN_RUN_DUTY;
    if (target > 255) target = 255;

    const float maxStep = FAN_SLEW_PER_S * dt;
    if (target > dutyF + maxStep) target = dutyF + maxStep;
    if (target < dutyF - maxStep) target = dutyF - maxStep;
    dutyF = target;
    return duty();
  }
};
