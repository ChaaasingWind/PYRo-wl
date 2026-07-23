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
constexpr float OJ5                     = 0.0945f;
constexpr float J4J5                    = 0.1125f;
constexpr float OJ8                     = 0.2100f;
// constexpr float MIN_LEG_LENGTH = 0.18f;
// constexpr float MAX_LEG_LENGTH = 2.5f;
constexpr float HIP_CALIBRATION_OFFSET  = PI;
constexpr float KNEE_CALIBRATION_OFFSET = 0.33231068958f;

// constexpr float LEFT_HIP_OFFSET       = 0.6f;
// constexpr float LEFT_KNEE_OFFSET      = 3.11f;
constexpr float LEFT_HIP_OFFSET =
    -loop_fp32_PI(-2.51256f + HIP_CALIBRATION_OFFSET);
constexpr float LEFT_KNEE_OFFSET =
    -loop_fp32_PI(2.87429f + KNEE_CALIBRATION_OFFSET);
constexpr float RIGHT_HIP_OFFSET =
    -loop_fp32_PI(0.15709f + HIP_CALIBRATION_OFFSET);
constexpr float RIGHT_KNEE_OFFSET =
    -loop_fp32_PI(-3.02165f + KNEE_CALIBRATION_OFFSET);

constexpr float MAX_LEG_LENGTH                       = 0.38f;
constexpr float MIN_LEG_LENGTH                       = 0.18f;

constexpr float LEFT_LEG_DIRECTION                   = -1.0f;
constexpr float RIGHT_LEG_DIRECTION                  = 1.0f;

constexpr float LEFT_WHEEL_DIRECTION                 = -1.0f;
constexpr float RIGHT_WHEEL_DIRECTION                = 1.0f;
constexpr float STOP_VELOCITY_DEADBAND               = 0.05f;

constexpr float K_t                                  = 0.21777611f;
constexpr float reduction_ratio                      = 13.94f;
constexpr float rec_reduction_ratio                  = 1 / reduction_ratio;
constexpr uint32_t L_WP_POLY_DEGREE                  = 3;
constexpr uint32_t K_POLY_DEGREE                     = 3;
// L_wp(L) = 0.0581 + 0.3760*L + 0.7972*L^2 - 0.7381*L^3.
// Coefficients are in ascending-power order: c0, c1, c2, c3.
constexpr float L_WP_POLY_COEF[L_WP_POLY_DEGREE + 1] = {
    0.0581f, 0.3760f, 0.7972f, -0.7381f};
constexpr float K_POLY_COEF[2][6][K_POLY_DEGREE + 1] = {
    {{-2.2511405690e+00f, -1.3787627377e+01f, 1.8199995743e+01f,
      -3.9921055309e+00f},
     {-4.0374942379e+00f, -8.6005350490e+00f, -8.1370577156e-01f,
      1.5764201307e+01f},
     {8.7032690090e-01f, 1.3461569166e+02f, -2.3572317345e+02f,
      1.8000387217e+02f},
     {3.4704696690e-03f, 1.2758931780e+01f, -1.9465925895e+00f,
      -4.1155181304e-01f},
     {-1.3711006700e+01f, 2.9832498720e+01f, -1.7194364391e+01f,
      -1.3692258979e+01f},
     {-2.5637696237e+00f, 5.6068136602e+00f, -5.3973128262e+00f,
      7.5147572243e-01f}},
    {{3.8908861144e+00f, 9.6739506975e+01f, -4.1565251565e+02f,
      4.8693148314e+02f},
     {5.9985177075e+00f, 1.0607369732e+02f, -4.7383355572e+02f,
      5.6498290536e+02f},
     {-2.7852598841e+01f, -3.3775101587e+02f, 1.4420728438e+03f,
      -1.6682849344e+03f},
     {-1.6669111181e+00f, -5.0640026102e+01f, 1.7039142010e+02f,
      -1.8094685384e+02f},
     {-5.6118770865e+00f, -1.8557286679e+02f, 3.3226343334e+02f,
      -2.0407868896e+02f},
     {-6.1344936527e-01f, -2.7287267129e+01f, 3.8817417453e+01f,
      -1.3951639600e+01f}}};

// Incremental wheel-odometry parameter. Fill in the real wheel radius [m]
// before using odom_x/odom_dx for walking experiments.
constexpr float WHEEL_RADIUS = 0.06f;


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
