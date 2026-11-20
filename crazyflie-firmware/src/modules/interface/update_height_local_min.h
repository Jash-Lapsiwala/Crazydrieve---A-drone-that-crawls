#pragma once
#include "stabilizer_types.h"

void localMinUpdateSetpoint(setpoint_t *setpoint, sensorData_t const *sensorData,
                            state_t const *state, stabilizerStep_t stabilizerStep);