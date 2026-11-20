#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "update_height_local_min.h"
#include "range.h"
#include "param.h"
#include "log.h"

#define MAX_VALID_RANGE_M  3.0f
#define SETTLE_SAMPLES     3       // distinct samples after an edge used to refine the target

// Params
static uint8_t lmEnable = 0;
static float stepThreshM = 0.15f;  // minimum jump that counts as a table edge

// Detector state
static bool wasEnabled = false;
static bool havePrev = false;
static float prevZ = 0.0f;
static int8_t settleLeft = 0;

// Override state
static bool adjustActive = false;
static float targetHeightM = 0.0f;

// Logging
static uint8_t overTable = 0;
static uint8_t stepCount = 0;

void localMinUpdateSetpoint(setpoint_t *setpoint, sensorData_t const *sensorData,
                            state_t const *state, stabilizerStep_t stabilizerStep)
{
  if (!lmEnable) {
    if (wasEnabled) {                  // fresh detector on every re-enable
      havePrev = false;
      adjustActive = false;
      overTable = 0;
      settleLeft = 0;
      wasEnabled = false;
    }
    return;
  }
  wasEnabled = true;

  // Runs every stabilizer tick. rangeGet() holds its value between sensor
  // updates, so only distinct readings are processed.
  float z = rangeGet(rangeDown) / 1000.0f;       // mm -> m
  if (z > 0.0f && z <= MAX_VALID_RANGE_M && (!havePrev || z != prevZ)) {
    if (havePrev) {
      float dz = z - prevZ;
      if (dz < -stepThreshM) {                   // range dropped: table edge entered
        targetHeightM = z;
        adjustActive = true;
        overTable = 1;
        settleLeft = SETTLE_SAMPLES;
        stepCount++;
      } else if (dz > stepThreshM) {             // range jumped up: table edge left
        adjustActive = false;                    // host setpoint takes over again
        overTable = 0;
        settleLeft = 0;
        stepCount++;
      } else if (settleLeft > 0) {               // refine target past the blended edge sample
        targetHeightM = fminf(targetHeightM, z);
        settleLeft--;
      }
    }
    prevZ = z;
    havePrev = true;
  }

  // Re-applied every tick, since the setpoint is fresh each loop.
  // Only touch z in absolute mode, so the stop setpoint (kill switch)
  // and other modes are left alone.
  if (adjustActive && setpoint->mode.z == modeAbs) {
    setpoint->position.z = targetHeightM;
  }
}

LOG_GROUP_START(localMin)
  LOG_ADD(LOG_FLOAT, target, &targetHeightM)
  LOG_ADD(LOG_UINT8, overTable, &overTable)
  LOG_ADD(LOG_UINT8, count, &stepCount)
LOG_GROUP_STOP(localMin)

PARAM_GROUP_START(localMin)
  PARAM_ADD(PARAM_UINT8, enable, &lmEnable)
  PARAM_ADD(PARAM_FLOAT, stepThresh, &stepThreshM)
PARAM_GROUP_STOP(localMin)