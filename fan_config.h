#pragma once

// Fitted to the fan characterisation run of 2026-11-07 (full white, room 18.75 C):
//   LED heating 0.236 C/s at full white; loss (T - room) * g with g = 0.0003 + ~3.3e-5 * duty per s
//   (duty 20 has almost no effect); about 6 s deadtime, effective deadtime ~10 s incl. ramp-in.
//   Full white at fan 255 settles about 30 C above room.
#ifndef FAN_SETPOINT_C
#define FAN_SETPOINT_C 38.0f
#endif
#ifndef FAN_ON_MARGIN_C
#define FAN_ON_MARGIN_C 1.0f
#endif
#ifndef FAN_OFF_MARGIN_C
#define FAN_OFF_MARGIN_C 2.0f
#endif
#ifndef FAN_RUN_DUTY
#define FAN_RUN_DUTY 70  // lowest duty with a measurable effect
#endif
#ifndef FAN_KICK_DUTY
#define FAN_KICK_DUTY 255
#endif
#ifndef FAN_KICK_MS
#define FAN_KICK_MS 1000UL
#endif
// SIMC tuning for an integrating process: k' = 6.6e-4 C/s per duty at 20 C above room,
// effective deadtime 10 s, closed-loop time constant 30 s.
#ifndef FAN_KP
#define FAN_KP 38.0f   // duty per C
#endif
#ifndef FAN_KI
#define FAN_KI 0.24f   // duty per C per s
#endif
#ifndef FAN_SLEW_PER_S
#define FAN_SLEW_PER_S 20.0f
#endif
#ifndef FAN_UPDATE_MS
#define FAN_UPDATE_MS 1000UL
#endif
#ifndef FAN_TRIP_C
#define FAN_TRIP_C 65.0f
#endif
