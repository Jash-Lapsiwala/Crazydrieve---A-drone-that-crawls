#pragma once
#include <stdbool.h>
#include "stabilizer_types.h"

bool controllerLqrEnabled(void);
void controllerLqrReset(const state_t *state);
void controllerLqr(control_t *control, const setpoint_t *setpoint,
                   const sensorData_t *sensors, const state_t *state,
                   const stabilizerStep_t stabilizerStep);
