#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/moonpilot/lateral_engage_declarations.h"

// moonpilot: steering is permitted while the car's cruise main switch is on and
// openpilot is engaged, without stock ACC being set — the "half-engaged" state.
// This is an extra permission stacked on top of upstream's, never a
// replacement: controls_allowed, and therefore every longitudinal check
// (get_longitudinal_allowed), keeps upstream's semantics untouched.
//
// Enabled per-brand by that brand's safety param (one bit in each of Toyota's,
// Honda's and Volkswagen's own flag spaces), so a stock config never reaches
// the rule. Brake and regen deliberately do not appear here: keeping steering
// through a brake tap is the feature. What drops it is the cruise main switch,
// a steering override, openpilot disengaging, and the lag/validity exits in
// safety_tick.
//
// The arm is the driver's own switch, and it may not wait for the heartbeat:
// that bit rides pandad's 10 Hz USB heartbeat, which openpilot necessarily sets
// after it has already begun to steer, and a frame this layer rejects resets
// desired_torque_last — after which the rate limiter blocks every later frame
// until the command comes home within MAX_RATE_UP of zero. The heartbeat is
// kept as the disengage cross-check instead, with a grace window that covers
// that refresh period, so a heartbeat which merely arrives late cannot take the
// steering with it.
//
// The steering-override term below is upstream's `steering_disengage`, which
// only Tesla's rx hook ever sets (opendbc/safety/modes/tesla.h): on every brand
// the fork enables the rule for that branch is inert by construction, and a
// driver's torque is handled the way upstream handles it — the car blends it,
// and openpilot overrides rather than disengaging.

bool controls_allowed_lateral = false;

static bool lateral_engage_enabled = false;
static bool lateral_engage_acc_main_prev = false;
static bool lateral_engage_heartbeat_prev = false;
static bool lateral_engage_steering_override_prev = false;
static uint32_t lateral_engage_heartbeat_ts = 0U;

void lateral_engage_init(void) {
  controls_allowed_lateral = false;
  lateral_engage_enabled = false;
  lateral_engage_acc_main_prev = false;
  lateral_engage_heartbeat_prev = false;
  lateral_engage_steering_override_prev = false;
  lateral_engage_heartbeat_ts = 0U;
}

void lateral_engage_set_enabled(bool enabled) {
  lateral_engage_enabled = enabled;
}

void lateral_engage_update(bool acc_main, bool heartbeat,
                           bool steering_override) {
  // 3x pandad's 10 Hz heartbeat: how stale a `heartbeat` reading may stand for openpilot being
  // engaged before the permission is dropped. Block scope, not file scope: MISRA-C:2012 8.9.
  const uint32_t HEARTBEAT_GRACE_US = 300000U;
  uint32_t ts = microsecond_timer_get();

  if (lateral_engage_enabled && acc_main) {
    // Arm on the driver switching the cruise main switch on, or on openpilot
    // engaging later than the switch did. Deliberately not on both being true:
    // the heartbeat lags the arm by up to a period of pandad's heartbeat, and a
    // rejected frame in that window costs the rest of the engagement.
    if (!lateral_engage_acc_main_prev ||
        (heartbeat && !lateral_engage_heartbeat_prev)) {
      controls_allowed_lateral = true;
      lateral_engage_heartbeat_ts = ts;
    }

    if (heartbeat) {
      // openpilot is engaged: the permission holds, and the window restarts
      lateral_engage_heartbeat_ts = ts;
    } else {
      // openpilot is gone rather than one heartbeat late: the permission goes with it.
      // Not an `else if`: MISRA-C:2012 15.7 wants every if/else-if chain terminated by an else.
      if (safety_get_ts_elapsed(ts, lateral_engage_heartbeat_ts) > HEARTBEAT_GRACE_US) {
        controls_allowed_lateral = false;
      }
    }
  } else {
    // The cruise main switch off ends it outright
    controls_allowed_lateral = false;
  }

  // A steering override always wins, whatever armed it. Tesla-only signal, so
  // on no brand the fork enables this for does the branch fire — see the note
  // at the top of this file.
  if (steering_override && !lateral_engage_steering_override_prev) {
    controls_allowed_lateral = false;
  }

  lateral_engage_acc_main_prev = acc_main;
  lateral_engage_heartbeat_prev = heartbeat;
  lateral_engage_steering_override_prev = steering_override;
}

void lateral_engage_exit(void) {
  // Lagging or invalid rx checks, called from safety_tick: the same conditions
  // that drop controls_allowed drop lateral permission. Re-arming takes a main
  // switch or ACC transition.
  controls_allowed_lateral = false;
}
