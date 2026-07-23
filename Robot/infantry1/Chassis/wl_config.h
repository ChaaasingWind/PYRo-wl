#ifndef __WL_CONFIG_H__
#define __WL_CONFIG_H__
#include "pyro_algo_common.h"


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
constexpr float OJ5                   = 0.0945f;
constexpr float J4J5                  = 0.1125f;
constexpr float OJ8                   = 0.2100f;
// constexpr float MIN_LEG_LENGTH = 0.18f;
// constexpr float MAX_LEG_LENGTH = 2.5f;
constexpr float HIP_CALIBRATION_OFFSET = PI;
constexpr float KNEE_CALIBRATION_OFFSET = 0.33231068958f;

// constexpr float LEFT_HIP_OFFSET       = 0.6f;
// constexpr float LEFT_KNEE_OFFSET      = 3.11f;
constexpr float LEFT_HIP_OFFSET       = -loop_fp32_PI(-2.51256f + HIP_CALIBRATION_OFFSET);
constexpr float LEFT_KNEE_OFFSET      = -loop_fp32_PI(2.87429f+KNEE_CALIBRATION_OFFSET);
constexpr float RIGHT_HIP_OFFSET      = -loop_fp32_PI(0.15709f+HIP_CALIBRATION_OFFSET);
constexpr float RIGHT_KNEE_OFFSET     = -loop_fp32_PI(-3.02165f+KNEE_CALIBRATION_OFFSET);

constexpr float MAX_LEG_LENGTH        = 0.38f;
constexpr float MIN_LEG_LENGTH        = 0.18f;

constexpr float LEFT_LEG_DIRECTION    = -1.0f;
constexpr float RIGHT_LEG_DIRECTION   = 1.0f;

constexpr float LEFT_WHEEL_DIRECTION  = -1.0f;
constexpr float RIGHT_WHEEL_DIRECTION = 1.0f;
constexpr float STOP_VELOCITY_DEADBAND = 0.05f;

constexpr float K_t                   = 0.21777611f;
constexpr float reduction_ratio       = 13.94f;
constexpr float rec_reduction_ratio   = 1 / reduction_ratio;
constexpr uint32_t L_WP_POLY_DEGREE = 3;
constexpr uint32_t K_POLY_DEGREE = 3;
constexpr float L_WP_POLY_COEF[L_WP_POLY_DEGREE + 1] = {};
constexpr float K_POLY_COEF[2][6][K_POLY_DEGREE + 1] = {};

// Incremental wheel-odometry parameter. Fill in the real wheel radius [m]
// before using odom_x/odom_dx for walking experiments.
constexpr float WHEEL_RADIUS           = 0.06f;


namespace leg_def
{
enum : uint8_t
{
    L = 0, //LEFT
    R = 1  //RIGHT
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
