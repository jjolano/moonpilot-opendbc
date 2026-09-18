#pragma once

#include <stdbool.h>

// moonpilot: the fork's lateral-only permission, declared here so the
// definitions in opendbc/safety/moonpilot/lateral_engage.h have a compatible
// declaration in scope (MISRA-C:2012 rule 8.4) and so panda and the test
// harness can reach them.
//
// controls_allowed keeps upstream's exact meaning — stock ACC engagement — and
// is what every longitudinal check reads. controls_allowed_lateral is additive
// and gates steering only.

extern bool controls_allowed_lateral;

void lateral_engage_init(void);
void lateral_engage_set_enabled(bool enabled);
void lateral_engage_update(bool acc_main, bool heartbeat,
                           bool steering_override);
void lateral_engage_exit(void);
