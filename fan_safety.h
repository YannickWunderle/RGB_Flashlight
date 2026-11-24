#pragma once

#include "fan_config.h"

inline bool fanSensorFault(int raw) {
  return raw < 3 || raw > 1020;
}

inline float fanRawToC(int raw) {
  return raw * (500.0f / 1023.0f);
}

inline float fanRawToC(float raw) {
  return raw * (500.0f / 1023.0f);
}

struct FanMedian5 {
  int v[5];
  int count;
  int next;

  void reset() {
    count = 0;
    next = 0;
  }

  int median() const {
    int s[5];
    for (int i = 0; i < count; i++) {
      int x = v[i];
      int j = i;
      while (j > 0 && s[j - 1] > x) {
        s[j] = s[j - 1];
        j--;
      }
      s[j] = x;
    }
    return s[count / 2];
  }

  int push(int x) {
    v[next] = x;
    next = (next + 1) % 5;
    if (count < 5) count++;
    return median();
  }
};
