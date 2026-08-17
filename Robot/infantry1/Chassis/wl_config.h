#ifndef __WL_CONFIG_H__
#define __WL_CONFIG_H__
#include "pyro_algo_common.h"
#include "lqr_coef.h"

namespace pyro
{
constexpr float loop_fp32_PI(float val)
{
    while (val > PI)
    {
        val -= 2 * PI;
    }
    while (val < -PI)
    {
        val += 2 * PI;
    }
    return val;
}
constexpr float OJ5                     = 0.0945f;
constexpr float J4J5                    = 0.1125f;
constexpr float OJ8                     = 0.2100f;
// constexpr float MIN_LEG_LENGTH = 0.18f;
// constexpr float MAX_LEG_LENGTH = 2.5f;
constexpr float HIP_CALIBRATION_OFFSET  = PI;
constexpr float KNEE_CALIBRATION_OFFSET = 0.33231068958f;


constexpr float LEFT_HIP_OFFSET =
    -loop_fp32_PI(-2.51256f + HIP_CALIBRATION_OFFSET);
constexpr float LEFT_KNEE_OFFSET =
    -loop_fp32_PI(2.87429f + KNEE_CALIBRATION_OFFSET);
constexpr float RIGHT_HIP_OFFSET =
    -loop_fp32_PI(0.14089f + HIP_CALIBRATION_OFFSET);
constexpr float RIGHT_KNEE_OFFSET =
    -loop_fp32_PI(-2.98023f + KNEE_CALIBRATION_OFFSET);

// constexpr float LEFT_HIP_OFFSET   =0;
// constexpr float LEFT_KNEE_OFFSET  =0;
// constexpr float RIGHT_HIP_OFFSET  =0;
// constexpr float RIGHT_KNEE_OFFSET =0;

constexpr float MAX_LEG_LENGTH        = 0.38f;
constexpr float MIN_LEG_LENGTH        = 0.18f;

constexpr float LEFT_LEG_DIRECTION    = -1.0f;
constexpr float RIGHT_LEG_DIRECTION   = 1.0f;

constexpr float LEFT_WHEEL_DIRECTION  = -1.0f;
constexpr float RIGHT_WHEEL_DIRECTION = 1.0f;

// Single-leg gravity compensation parameters.
constexpr float SINGLE_LEG_BODY_MASS  = 6.0f;
constexpr float LEG_MASS              = 1.232f;
constexpr float GRAVITY_ACCELERATION  = 9.81f;
constexpr float K_t                                  = 0.21777611f;
constexpr float reduction_ratio                      = 13.94f;
constexpr float rec_reduction_ratio                  = 1 / reduction_ratio;
constexpr float MAX_F_L                              = 300.0f;
constexpr float MAX_T_P                              = 60.0f;
constexpr float MAX_CURRENT                          = 15.0f;
constexpr float MAX_T_W                              = K_t * MAX_CURRENT;


constexpr uint32_t L_WP_POLY_DEGREE                  = 3;

// L_wp(L) = 0.0581 + 0.3760*L + 0.7972*L^2 - 0.7381*L^3.
// Coefficients are in ascending-power order: c0, c1, c2, c3.
constexpr float L_WP_POLY_COEF[L_WP_POLY_DEGREE + 1] = {0.0581f, 0.3760f,
                                                        0.7972f, -0.7381f};

constexpr float WHEEL_RADIUS                         = 0.06f;

namespace pose_kf_cfg
{
// Timing and geometry.
constexpr float NOMINAL_DT_S                         = 0.001f;
constexpr float WHEEL_HALF_TRACK_M                   = 0.20025f;
constexpr float WHEEL_TRACK_M                        = 2.0f * WHEEL_HALF_TRACK_M;
constexpr float WHEEL_YAW_RATE_SIGN                  = 1.0f;

// Fixed IMU-frame orientation in the chassis body frame. The implementation
// builds R_I^B = Rz(yaw) * Ry(pitch) * Rx(roll). DIRECT_3 handles only the
// coarse axis mapping; use these angles for the remaining mounting error.
constexpr float IMU_MOUNT_ROLL_RAD                   = 0.0f;
constexpr float IMU_MOUNT_PITCH_RAD                  = 0.0f;
constexpr float IMU_MOUNT_YAW_RAD                    = 0.0f;

// Positive when the IMU is in front of the wheel-center midpoint.
constexpr float IMU_FORWARD_OFFSET_M                 = 0.0f;
constexpr float MIN_ABS_COS_PITCH                    = 0.10f;

// Continuous-time process noise amplitude densities. The implementation
// squares these values to obtain q_a and q_alpha when constructing Q_d(dt).
constexpr float ACCEL_PROCESS_NOISE_DENSITY          = 3.16227766f;
constexpr float YAW_ACCEL_PROCESS_NOISE_DENSITY      = 3.16227766f;

// Measurement standard deviations. Wheel values are linear speeds in m/s;
// gyro yaw-rate is in rad/s. Correlation must stay in [-1, 1].
constexpr float WHEEL_LEFT_VELOCITY_STD_MPS          = 0.31622777f;
constexpr float WHEEL_RIGHT_VELOCITY_STD_MPS         = 0.31622777f;
constexpr float WHEEL_VELOCITY_CORRELATION           = 0.0f;
constexpr float GYRO_YAW_RATE_STD_RADPS               = 0.07071068f;

// Initial-state standard deviations used to construct P0.
constexpr float INITIAL_X_STD_M                       = 0.02f;
constexpr float INITIAL_V_STD_MPS                     = 0.25f;
constexpr float INITIAL_YAW_STD_RAD                   = 0.08726646f;
constexpr float INITIAL_YAW_RATE_STD_RADPS            = 0.10f;
} // namespace pose_kf_cfg


namespace leg_def
{
enum : uint8_t
{
    L = 0, // LEFT
    R = 1  // RIGHT
};
}
namespace joint_def
{
enum : uint8_t
{
    HIP  = 0,
    KNEE = 1,
};
}
} // namespace pyro
#endif
