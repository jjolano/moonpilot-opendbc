#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/moonpilot/lateral_engage_declarations.h"

// moonpilot: steering is permitted while openpilot is engaged and the car's own
// cruise system is on, without stock ACC being set — the "half-engaged" state.
// This is an extra permission stacked on top of upstream's, never a
// replacement: controls_allowed, and therefore every longitudinal check
// (get_longitudinal_allowed), keeps upstream's semantics untouched.
//
// Enabled per-brand by that brand's safety param (one bit in each brand's own
// flag space), so a stock config never reaches the rule. Brake and regen
// deliberately do not appear here: keeping steering through a brake tap is the
// feature. What drops it is the arm's own signal, a steering override,
// openpilot disengaging, and the lag/validity exits in safety_tick.
//
// Two arming modes, because the evidence a brand has differs, and the brand
// picks one at init:
//
//   LATERAL_ENGAGE_ARM_SWITCH — the car's own cruise main switch
//   (`acc_main_on`), where the brand
//     decodes it. The arm is the driver's switch, and it may not wait for the
//     heartbeat: that bit rides pandad's 10 Hz USB heartbeat, which openpilot
//     necessarily sets after it has already begun to steer, and a frame this
//     layer rejects resets desired_torque_last — after which the rate limiter
//     blocks every later frame until the command comes home within MAX_RATE_UP
//     of zero. The heartbeat is kept as the disengage cross-check instead, with
//     a grace window that covers that refresh period, so a heartbeat which
//     merely arrives late cannot take the steering with it. Toyota, Honda and
//     Volkswagen MQB/MEB arm this way. VW PQ and MLB do not: PQ populates
//     `acc_main_on` only under openpilot longitudinal control and MLB never
//     does, so both are ARM_HOST brands like every brand with no main-switch
//     decode.
//
//   LATERAL_ENGAGE_ARM_HOST — openpilot's own engaged heartbeat, for every
//   brand whose mode
//     decodes no main switch to require. This trusts the host where ARM_SWITCH
//     does not: the permission rises while openpilot claims it is engaged, and
//     the car corroborates nothing. It is the same trust sunnypilot's MADS
//     places in `heartbeat_engaged_mads`, taken deliberately: the alternative
//     is a CAN decode per brand, each one a signal hunt validated on that car,
//     and this fork would rather grant steering to an openpilot that says it is
//     engaged than only ever work on four brands. It is level-armed rather than
//     edge-armed, so a drop — a stale heartbeat, the lag exit below — comes
//     back by itself on the next fresh heartbeat instead of waiting for a car
//     signal to transition. What keeps it honest is everything else in this
//     layer: upstream drops `controls_allowed` after three mismatched
//     heartbeats, and the openpilot side stops commanding the moment
//     `PandaState.controlsAllowedLateral` is not set (`controlsd`'s actuator
//     gate), so a grant that disappears takes the command with it instead of
//     producing frames the car's own lane-keeping ECU would read as a dropout.
//
// The steering-override term below is upstream's `steering_disengage`, which
// only Tesla's rx hook ever sets (opendbc/safety/modes/tesla.h): on every other
// brand the fork enables the rule for, that branch is inert by construction,
// and a driver's torque is handled the way upstream handles it — the car blends
// it, and openpilot overrides rather than disengaging.

bool controls_allowed_lateral = false;

static bool lateral_engage_enabled = false;
static bool lateral_engage_host_armed = false;
static bool lateral_engage_acc_main_prev = false;
static bool lateral_engage_heartbeat_prev = false;
static uint32_t lateral_engage_heartbeat_ts = 0U;

void lateral_engage_init(void) {
  controls_allowed_lateral = false;
  lateral_engage_enabled = false;
  lateral_engage_host_armed = false;
  lateral_engage_acc_main_prev = false;
  lateral_engage_heartbeat_prev = false;
  lateral_engage_heartbeat_ts = 0U;
}

void lateral_engage_set_enabled(bool enabled, LateralEngageArm arm) {
  lateral_engage_enabled = enabled;
  lateral_engage_host_armed = arm == LATERAL_ENGAGE_ARM_HOST;
}

void lateral_engage_update(bool acc_main, bool heartbeat,
                           bool steering_override) {
  // 3x pandad's 10 Hz heartbeat: how stale a `heartbeat` reading may stand for
  // openpilot being engaged before the permission is dropped. Block scope, not
  // file scope: MISRA-C:2012 8.9.
  const uint32_t HEARTBEAT_GRACE_US = 300000U;
  uint32_t ts = microsecond_timer_get();

  if (lateral_engage_enabled) {
    if (lateral_engage_host_armed) {
      // Host-armed: there is no car signal to wait for, so the freshest
      // heartbeat is the whole condition and a grant can never be older than
      // the grace window.
      if (heartbeat) {
        controls_allowed_lateral = true;
        lateral_engage_heartbeat_ts = ts;
      } else {
        // openpilot is gone rather than one heartbeat late: the permission goes
        // with it. Not an `else if`: MISRA-C:2012 15.7 wants every if/else-if
        // chain terminated by an else.
        if (safety_get_ts_elapsed(ts, lateral_engage_heartbeat_ts) >
            HEARTBEAT_GRACE_US) {
          controls_allowed_lateral = false;
        }
      }
    } else {
      // Switch-armed: `acc_main` is required, and either its own rising edge or
      // the heartbeat's with the switch already on is what arms.
      if (acc_main) {
        // Deliberately not on both being true: the heartbeat lags the arm by up
        // to a period of pandad's heartbeat, and a rejected frame in that
        // window costs the rest of the engagement — see the note at the top of
        // this file.
        if (!lateral_engage_acc_main_prev ||
            (heartbeat && !lateral_engage_heartbeat_prev)) {
          controls_allowed_lateral = true;
          lateral_engage_heartbeat_ts = ts;
        }

        if (heartbeat) {
          // openpilot is engaged: the permission holds, and the window restarts
          lateral_engage_heartbeat_ts = ts;
        } else {
          // Not an `else if`, for the same MISRA-C:2012 15.7 reason as above
          if (safety_get_ts_elapsed(ts, lateral_engage_heartbeat_ts) >
              HEARTBEAT_GRACE_US) {
            controls_allowed_lateral = false;
          }
        }
      } else {
        // The cruise main switch off ends it outright
        controls_allowed_lateral = false;
      }
    }
  } else {
    // A stock config, or a safety mode whose brand never opted in
    controls_allowed_lateral = false;
  }

  // A steering override always wins, whatever armed it, and it holds while it
  // is asserted: the signal is upstream's `steering_disengage`, which Tesla's
  // rx hook raises while the driver's torque is over the threshold, so a
  // host-armed car would otherwise take the permission back on the next
  // heartbeat with the driver still holding the wheel.
  if (steering_override) {
    controls_allowed_lateral = false;
  }

  lateral_engage_acc_main_prev = acc_main;
  lateral_engage_heartbeat_prev = heartbeat;
}

void lateral_engage_exit(void) {
  // Lagging or invalid rx checks, called from safety_tick: the same conditions
  // that drop controls_allowed drop lateral permission. Under ARM_SWITCH
  // re-arming takes a main switch or ACC transition; under ARM_HOST the next
  // fresh heartbeat is enough, which is what level-arming buys.
  controls_allowed_lateral = false;
}
