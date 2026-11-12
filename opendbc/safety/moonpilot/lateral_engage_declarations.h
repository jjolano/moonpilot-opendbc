#pragma once

#include <stdbool.h>

// The fork's lateral-only rule function declarations. Its permission and
// arming history are fields of the redundant SafetyState in declarations.h;
// tests access the permission through the safety.c shims.
//
// controls_allowed keeps upstream's exact meaning — stock ACC engagement — and
// is what every longitudinal check reads. controls_allowed_lateral is additive
// and gates steering only.

// How a brand's rule arms. A brand passes one of these to
// `lateral_engage_set_enabled` from its own init, beside its param bit.
typedef enum {
  // The car's own cruise main switch (`acc_main_on`): the driver's evidence,
  // and the only mode
  // that can require it. Toyota, Honda and Volkswagen MQB/MEB.
  LATERAL_ENGAGE_ARM_SWITCH = 0,
  // openpilot's own engaged heartbeat, for a brand whose mode decodes no main
  // switch to require.
  LATERAL_ENGAGE_ARM_HOST,
} LateralEngageArm;


void lateral_engage_init(void);
void lateral_engage_set_enabled(bool enabled, LateralEngageArm arm);
void lateral_engage_update(bool acc_main, bool heartbeat,
                           bool steering_override);
void lateral_engage_exit(void);
