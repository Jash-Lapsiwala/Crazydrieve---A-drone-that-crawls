/*
 * controller_lqr.c - LQR controller, laid out like controller_pid.c.
 *
 * The PID does:  position PID (100 Hz) -> attitude PID -> rate PID (500 Hz) -> legacy output
 * This does:     error vector e (500 Hz) -> V = -K*e -> same legacy output
 *
 * Everything else (setpoint handling, yaw handling, zero-thrust reset, output format)
 * is copied from controller_pid.c. To retune: change Q, R in MATLAB, paste the new K.
 */
#include <string.h>
#include <math.h>

#include "stabilizer_types.h"
#include "attitude_controller.h"     // ATTITUDE_RATE
#include "controller_lqr.h"

#include "log.h"
#include "param.h"
#include "math3d.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

// ---------------- the only part you change when retuning ----------------
// K = lqr(A, B_v, Q, R) from Crazyflie_Param.m
// Rows (inputs):  [dT  tau_x  tau_y  tau_z]          N, N*m
// Cols (states):  [x y z  psi theta phi  u v w  r q p]   m, rad, m/s, rad/s
static const float K[4][12] = {
  {0, 0, 9.4868f, 0, 0, 0, 0, 0, 1.0350f, 0, 0, 0},
  {0, -0.26833f, 0, 0, 0, 0.45339f, 0, -0.15978f, 0, 0, 0, 0.019695f},
  {0.26833f, 0, 0, 0, 0.45501f, 0, 0.16006f, 0, 0, 0, 0.020165f, 0},
  {0, 0, 0, 0.075895f, 0, 0, 0, 0, 0, 0.0051423f, 0, 0}
};
// --------------------------------------------------------------------------

#define ARM    (0.70710678f * 0.04475f)   // m, motor offset from the x and y axes
#define KQ     0.006f                     // m, yaw torque per N of thrust

static uint8_t useLqr = 0;   // param lqr.use: 0 = PID flies, 1 = LQR flies
static float fHover   = 0.428f;           // hover command per motor, fraction of 65535 (from logs)
static float S        = 0.258f;           // N of thrust per 65535 counts, per motor

static attitude_t attitudeDesired;        // only .yaw is used, same as the PID
static float cmd_thrust, cmd_roll, cmd_pitch, cmd_yaw;
static float V[4];                        // LQR output [dT tau_x tau_y tau_z], logged

bool controllerLqrEnabled(void) { return useLqr != 0; }

// ---- copied from controller_pid.c ----
static float capAngle(float angle) {
  float result = angle;
  while (result > 180.0f)  { result -= 360.0f; }
  while (result < -180.0f) { result += 360.0f; }
  return result;
}

static bool setpointModeChanged(const setpoint_t *setpoint)
{
  static setpoint_mode_t previous_mode;
  bool is_mode_changed = (memcmp(&setpoint->mode, &previous_mode, sizeof(previous_mode)) != 0);
  previous_mode = setpoint->mode;
  return is_mode_changed;
}

void controllerLqrReset(const state_t *state)
{
  attitudeDesired.yaw = state->attitude.yaw;
}
// --------------------------------------

void controllerLqr(control_t *control, const setpoint_t *setpoint,
                   const sensorData_t *sensors, const state_t *state,
                   const stabilizerStep_t stabilizerStep)
{
  control->controlMode = controlModeLegacy;

  if (setpointModeChanged(setpoint)) {
    controllerLqrReset(state);                       // same as controllerPidReinitialize
  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {

    // ---- yaw setpoint: copied from controller_pid.c ----
    if (setpoint->mode.yaw == modeVelocity) {
      attitudeDesired.yaw = capAngle(attitudeDesired.yaw + setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT);
    } else if (setpoint->mode.yaw == modeAbs) {
      attitudeDesired.yaw = setpoint->attitude.yaw;
    } else if (setpoint->mode.quat == modeAbs) {
      struct quat setpoint_quat = mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y,
                                         setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w);
      struct vec rpy = quat2rpy(setpoint_quat);
      attitudeDesired.yaw = degrees(rpy.z);
    }
    attitudeDesired.yaw = capAngle(attitudeDesired.yaw);

    // ---- position/velocity errors in the yaw-rotated (body) frame, like position_controller_pid.c ----
    const float cosyaw = cosf(radians(state->attitude.yaw));
    const float sinyaw = sinf(radians(state->attitude.yaw));
    const float x_b  =  state->position.x * cosyaw + state->position.y * sinyaw;
    const float y_b  = -state->position.x * sinyaw + state->position.y * cosyaw;
    const float vx_b =  state->velocity.x * cosyaw + state->velocity.y * sinyaw;
    const float vy_b = -state->velocity.x * sinyaw + state->velocity.y * cosyaw;

    // Desired velocity: given directly by velocity setpoints (MotionCommander),
    // used as feedforward by position setpoints (high-level commander)
    float vxd = setpoint->velocity.x, vyd = setpoint->velocity.y;
    if (!setpoint->velocity_body) {                  // world -> body, same as the PID
      vxd =  setpoint->velocity.x * cosyaw + setpoint->velocity.y * sinyaw;
      vyd = -setpoint->velocity.x * sinyaw + setpoint->velocity.y * cosyaw;
    }

    // Position error only when a position is commanded (PID: runPid only if modeAbs)
    float ex = 0.0f, ey = 0.0f, ez = 0.0f;
    if (setpoint->mode.x == modeAbs) {
      ex = x_b - ( setpoint->position.x * cosyaw + setpoint->position.y * sinyaw);
    }
    if (setpoint->mode.y == modeAbs) {
      ey = y_b - (-setpoint->position.x * sinyaw + setpoint->position.y * cosyaw);
    }
    if (setpoint->mode.z == modeAbs) {
      ez = state->position.z - setpoint->position.z;
    }

    // ---- error vector e = X - X_ref, order [x y z psi theta phi u v w r q p] ----
    // state angles are in deg and firmware pitch is inverted, gyro is deg/s
    const float e[12] = {
      ex, ey, ez,
      radians(capAngle(state->attitude.yaw - attitudeDesired.yaw)),
      radians(-state->attitude.pitch),
      radians(state->attitude.roll),
      vx_b - vxd,
      vy_b - vyd,
      state->velocity.z - setpoint->velocity.z,
      radians(sensors->gyro.z),
      radians(sensors->gyro.y),
      radians(sensors->gyro.x)
    };

    // ---- this replaces positionController + attitude PID + rate PID ----
    for (int i = 0; i < 4; i++) {
      float sum = 0.0f;
      for (int j = 0; j < 12; j++) {
        sum += K[i][j] * e[j];
      }
      V[i] = -sum;
    }

    // ---- N, N*m -> legacy counts (65535 counts = S newtons per motor) ----
    // Chosen so powerDistributionLegacy() gives each motor the force the LQR wants.
    const float c = 65535.0f / S;
    float thrust = fHover * 65535.0f + 0.25f * V[0] * c;
    float roll   =  2.0f * 0.25f * V[1] / ARM * c;
    float pitch  = -2.0f * 0.25f * V[2] / ARM * c;
    float yaw    = -0.25f * V[3] / KQ * c;

    if (setpoint->mode.z == modeDisable) {           // same as the PID: no height command
      thrust = setpoint->thrust;                     // = 0 after a stop setpoint
    }

    control->thrust = fminf(fmaxf(thrust, 0.0f), 65535.0f);
    control->roll   = (int16_t)fminf(fmaxf(roll,  -32767.0f), 32767.0f);
    control->pitch  = (int16_t)fminf(fmaxf(pitch, -32767.0f), 32767.0f);
    control->yaw    = (int16_t)fminf(fmaxf(yaw,   -32767.0f), 32767.0f);

    cmd_thrust = control->thrust;
    cmd_roll   = control->roll;
    cmd_pitch  = control->pitch;
    cmd_yaw    = control->yaw;
  }

  // ---- copied from controller_pid.c ----
  if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    cmd_thrust = cmd_roll = cmd_pitch = cmd_yaw = 0;

    controllerLqrReset(state);
  }
}

PARAM_GROUP_START(lqr)
PARAM_ADD(PARAM_UINT8, use, &useLqr)
PARAM_ADD(PARAM_FLOAT, fHover, &fHover)
PARAM_ADD(PARAM_FLOAT, S, &S)
PARAM_GROUP_STOP(lqr)

LOG_GROUP_START(lqr)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, dT, &V[0])
LOG_ADD(LOG_FLOAT, tauX, &V[1])
LOG_ADD(LOG_FLOAT, tauY, &V[2])
LOG_ADD(LOG_FLOAT, tauZ, &V[3])
LOG_ADD(LOG_FLOAT, yaw, &attitudeDesired.yaw)
LOG_GROUP_STOP(lqr)
