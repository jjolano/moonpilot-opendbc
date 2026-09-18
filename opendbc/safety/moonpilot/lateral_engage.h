#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/moonpilot/lateral_engage_declarations.h"

// moonpilot: steering is permitted while the car's cruise main switch is on and
// openpilot is engaged, without stock ACC being set — the "half-engaged" state.
// This is an extra permission stacked on top of upstream's, never a
// replacement: controls_allowed, and therefore every longitudinal check
// (get_longitudinal_allowed), keeps upstream's semantics untouched.
//
// Enabled per-brand by the safety param (see toyota_init), so a stock config
// never reaches the rule. Brake and regen deliberately do not appear here:
// keeping steering through a brake tap is the feature. What drops it is the
// cruise main switch, a steering override, openpilot disengaging, and the
// lag/validity exits in safety_tick.
//
// The steering-override term below is upstream's `steering_disengage`, which
// only Tesla's rx hook ever sets (opendbc/safety/modes/tesla.h): on the fork's
// one supported brand that branch is inert by construction, and a driver's
// torque is handled the way upstream handles it — the car blends it, and
// openpilot overrides rather than disengaging.

bool controls_allowed_lateral = false;

static bool lateral_engage_enabled = false;
static bool lateral_engage_acc_main_prev = false;
static bool lateral_engage_heartbeat_prev = false;
static bool lateral_engage_steering_override_prev = false;

void lateral_engage_init(void) {
  controls_allowed_lateral = false;
  lateral_engage_enabled = false;
  lateral_engage_acc_main_prev = false;
  lateral_engage_heartbeat_prev = false;
  lateral_engage_steering_override_prev = false;
}

void lateral_engage_set_enabled(bool enabled) {
  lateral_engage_enabled = enabled;
}

void lateral_engage_update(bool acc_main, bool heartbeat,
                           bool steering_override) {
  if (lateral_engage_enabled && acc_main && heartbeat) {
    // Latch on the rising edge of either: the driver switching the cruise main
    // switch on, or openpilot engaging (again, after a user disengage). Held
    // while both stay true, so nothing that only clears controls_allowed — a
    // brake or gas tap — can take the steering with it.
    if (!lateral_engage_acc_main_prev || !lateral_engage_heartbeat_prev) {
      controls_allowed_lateral = true;
    }
  } else {
    // The cruise main switch off, or openpilot not engaged, ends the permission
    // outright.
    controls_allowed_lateral = false;
  }

  // A steering override always wins, whatever armed it. Tesla-only signal, so
  // on Toyota this branch does not fire — see the note at the top of this file.
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
