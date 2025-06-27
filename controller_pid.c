
#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "position_controller.h"
#include "controller_pid.h"

#include "log.h"
#include "param.h"
#include "math3d.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;
static float e3;
static float dp_hat;
static float e3z;
static float dpz_hat;


void controllerPidInit(void)
{
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  positionControllerInit();
  
  e3 = 0;
  e3z = 0;  
}

bool controllerPidTest(void)
{
  bool pass = true;

  pass &= attitudeControllerTest();

  return pass;
}

static float capAngle(float angle) {
  float result = angle;

  while (result > 180.0f) {
    result -= 360.0f;
  }

  while (result < -180.0f) {
    result += 360.0f;
  }

  return result;
}

void controllerPid(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep)
{
  control->controlMode = controlModeLegacy;

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
    // Rate-controled YAW is moving YAW angle setpoint
    if (setpoint->mode.yaw == modeVelocity) {
      attitudeDesired.yaw = capAngle(attitudeDesired.yaw + setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT);

      float yawMaxDelta = attitudeControllerGetYawMaxDelta();
      if (yawMaxDelta != 0.0f)
      {
      float delta = capAngle(attitudeDesired.yaw-state->attitude.yaw);
      // keep the yaw setpoint within +/- yawMaxDelta from the current yaw
        if (delta > yawMaxDelta)
        {
          attitudeDesired.yaw = state->attitude.yaw + yawMaxDelta;
        }
        else if (delta < -yawMaxDelta)
        {
          attitudeDesired.yaw = state->attitude.yaw - yawMaxDelta;
        }
      }
    } else if (setpoint->mode.yaw == modeAbs) {
      attitudeDesired.yaw = setpoint->attitude.yaw;
    } else if (setpoint->mode.quat == modeAbs) {
      struct quat setpoint_quat = mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w);
      struct vec rpy = quat2rpy(setpoint_quat);
      attitudeDesired.yaw = degrees(rpy.z);
    }

    attitudeDesired.yaw = capAngle(attitudeDesired.yaw);
  }

  if (RATE_DO_EXECUTE(POSITION_RATE, stabilizerStep)) {
    positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
    // Switch between manual and automatic position control
    if (setpoint->mode.z == modeDisable) {
      actuatorThrust = setpoint->thrust;
    }
    if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
      attitudeDesired.roll = setpoint->attitude.roll;
      attitudeDesired.pitch = setpoint->attitude.pitch;
    }

    attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

    // For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
    // value. Also reset the PID to avoid error buildup, which can lead to unstable
    // behavior if level mode is engaged later
    if (setpoint->mode.roll == modeVelocity) {
      rateDesired.roll = setpoint->attitudeRate.roll;
      attitudeControllerResetRollAttitudePID(state->attitude.roll);
    }
    if (setpoint->mode.pitch == modeVelocity) {
      rateDesired.pitch = setpoint->attitudeRate.pitch;
      attitudeControllerResetPitchAttitudePID(state->attitude.pitch);
    }

    // TODO: Investigate possibility to subtract gyro drift.
    attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                             rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

    attitudeControllerGetActuatorOutput(&control->roll,
                                        &control->pitch,
                                        &control->yaw);

    control->yaw = -control->yaw;
    
    float dt = 1.0f / ATTITUDE_RATE;
    // DOB for Z pos
    // Precompute differences for clarity and efficiency
    //float dz_vel = setpoint->velocity.z - state->velocity.z;
    float dz_pos = setpoint->position.z - state->position.z;
    //float dz_acc = setpoint->acceleration.z - sensors->acc.z;
    
    // Compute pz and lz using precomputed differences
    //float pz = 10.0f * dz_vel + 800.0f * dz_pos;
    //float lz = 10.0f * dz_acc + 800.0f * dz_vel;
    float pz =  0.1f * dz_pos + 0.005f * dz_pos* dz_pos* dz_pos;
    float lz =  0.1f + 0.015f* dz_pos* dz_pos);
    
    // Compute z_est from the thrust command
    float z_est = (control->thrust - 40000.0f) / 1000.0f;
    
    // Simplify fx calculation by combining constants
    // Note: -0.103*(-gyro.y)*velocity.x becomes 0.103*gyro.y*velocity.x
    float fx = 0.103f * (sensors->gyro.y * state->velocity.x - 9.81f * cosf(state->attitude.pitch));
    
    // Update e3z using the computed derivatives
    float e3z_dot = lz * (-e3z - pz * (fx + z_est));
    e3z += e3z_dot * dt;
    e3z = clamp(e3z, -1.0f, 1.0f);
    
    // Compute dpz_hat from the updated e3z and pz
    dpz_hat = (e3z + pz) * 1000.0f + 40000.0f;  

    cmd_thrust = control->thrust - dpz_hat;
    cmd_roll = control->roll;
    
    // DOB for pitch
    // Precompute differences for clarity and efficiency
    //float dx_vel = rateDesired.pitch - (-sensors->gyro.y);
    float dx_pos = setpoint->attitude.pitch- state->attitude.pitch;
    //float dz_acc = setpoint->acceleration.z - sensors->acc.z;
    // Compute intermediate variables
    //float px = 2.0f * (-sensors->gyro.y + 80.0f * state->attitude.pitch);
    //float lx = 2.0f * (state->attitude.pitch + 80.0f);
    float px = 1.0f * dx_pos + 0.0025f * dx_pos* dx_pos* dx_pos);
    float lx = 1.0f + 0.0075f * dx_pos* dx_pos);

    
    // Compute ld using the sine of the control pitch
    float ld = 0.08f * sinf(control->pitch);
    
    // Compute flap frequency from the thrust command
    float flap_freq = (0.5f * z_est + 0.1903f) / 0.0379f;
    
    // Simplify inner expressions: 
    // state->velocity.x - 0.075f*(-gyro_y) becomes (vel_x + CONST_0_075*gyro_y)
    float Dx = -0.0237f * flap_freq * (state->velocity.x + 0.075f * -sensors->gyro.y);
    
    // Similarly, state->velocity.z - ld*(-gyro_y) becomes (vel_z + ld*gyro_y)
    float Dz = 0.0084f * flap_freq * (state->velocity.z + ld * sensors->gyro.y) + z_est;
    
    // Note: lz should be defined elsewhere. If it's not, define it accordingly.
    // Compute the derivative of e3 using the simplified equation
    float e3_dot = lx * (e3 - (px + lz * Dx + ld * Dz)) * 8300.0f;
    
    // Update e3 and clamp it to the range [-20, 20]
    e3 += e3_dot * dt;
    e3 = clamp(e3, -20.0f, 20.0f);
    
    // Finally, compute dp_hat from e3 and px
    dp_hat = e3 + px;     
    
    cmd_pitch = control->pitch - dp_hat;
    cmd_yaw = control->yaw;
    r_roll = radians(sensors->gyro.x);
    r_pitch = -radians(sensors->gyro.y);
    r_yaw = radians(sensors->gyro.z);
    accelz = sensors->acc.z;
  }

  control->thrust = actuatorThrust;

  if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;
    
    dp_hat = 0;
    dpz_hat = 0;
    
    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    attitudeControllerResetAllPID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw);
    positionControllerResetAllPID(state->position.x, state->position.y, state->position.z);

    // Reset the calculated YAW angle for rate control
    attitudeDesired.yaw = state->attitude.yaw;
  }
}

/**
 * Logging variables for the command and reference signals for the
 * altitude PID controller
 */
LOG_GROUP_START(controller)
/**
 * @brief Thrust command
 */
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
/**
 * @brief Roll command
 */
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
/**
 * @brief Pitch command
 */
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
/**
 * @brief yaw command
 */
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
/**
 * @brief Gyro roll measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
/**
 * @brief Gyro pitch measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
/**
 * @brief Yaw  measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
/**
 * @brief Acceleration in the zaxis in G-force
 */
LOG_ADD(LOG_FLOAT, accelz, &accelz)
/**
 * @brief Thrust command without (tilt)compensation
 */
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
/**
 * @brief Desired roll setpoint
 */
LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
/**
 * @brief Desired pitch setpoint
 */
LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
/**
 * @brief Desired yaw setpoint
 */
LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
/**
 * @brief Desired roll rate setpoint
 */
LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll)
/**
 * @brief Desired pitch rate setpoint
 */
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
/**
 * @brief Desired yaw rate setpoint
 */
LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw)
LOG_ADD(LOG_FLOAT, e3,   &e3)
LOG_ADD(LOG_FLOAT, dp_hat,   &dp_hat) 
LOG_ADD(LOG_FLOAT, e3z,   &e3z)
LOG_ADD(LOG_FLOAT, dpz_hat,   &dpz_hat) 
LOG_GROUP_STOP(controller)
