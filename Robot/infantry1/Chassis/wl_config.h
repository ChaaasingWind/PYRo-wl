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

constexpr float MAX_LEG_LENGTH         = 0.38f;
constexpr float MIN_LEG_LENGTH         = 0.18f;

constexpr float LEFT_LEG_DIRECTION     = -1.0f;
constexpr float RIGHT_LEG_DIRECTION    = 1.0f;

constexpr float LEFT_WHEEL_DIRECTION   = -1.0f;
constexpr float RIGHT_WHEEL_DIRECTION  = 1.0f;
constexpr float STOP_VELOCITY_DEADBAND = 0.05f;

// Single-leg gravity compensation parameters.
constexpr float SINGLE_LEG_BODY_MASS   = 6.0f;
constexpr float LEG_MASS               = 1.232f;
constexpr float GRAVITY_ACCELERATION   = 9.81f;
constexpr float LEG_GRAVITY_FORCE =
    (SINGLE_LEG_BODY_MASS + LEG_MASS) * GRAVITY_ACCELERATION;
constexpr float MAX_TOTAL_LEG_FORCE                  = 100.0f;

constexpr float K_t                                  = 0.21777611f;
constexpr float reduction_ratio                      = 13.94f;
constexpr float rec_reduction_ratio                  = 1 / reduction_ratio;
constexpr uint32_t L_WP_POLY_DEGREE                  = 3;
constexpr uint32_t K_POLY_DEGREE                     = 3;
// L_wp(L) = 0.0581 + 0.3760*L + 0.7972*L^2 - 0.7381*L^3.
// Coefficients are in ascending-power order: c0, c1, c2, c3.
constexpr float L_WP_POLY_COEF[L_WP_POLY_DEGREE + 1] = {0.0581f, 0.3760f,
                                                        0.7972f, -0.7381f};
constexpr float K_POLY_COEF[2][6][K_POLY_DEGREE + 1] = {
    {{-2.5760390761e+00f, -1.0918047588e+01f, 8.2455759447e+00f,
      7.2067514987e+00f},
     {-4.3700672681e+00f, -4.6576385923e+00f, -1.1685683548e+01f,
      2.6896963847e+01f},
     {4.5946835683e-01f, 1.3228538470e+02f, -2.2870950919e+02f,
      1.6421748343e+02f},
     {1.6201939145e-02f, 1.1226057862e+01f, 1.1531708402e+00f,
      -4.3325614993e+00f},
     {-1.2241623602e+01f, 2.4539702212e+01f, -7.0584338453e+00f,
      -2.0894766066e+01f},
     {-2.3597192210e+00f, 4.9348966694e+00f, -4.1132356269e+00f,
      -1.1580700881e-01f}},
    {{4.7167183070e+00f, 1.1834986918e+02f, -5.1286174196e+02f,
      6.0247753752e+02f},
     {6.6252695206e+00f, 1.2681924101e+02f, -5.7041325676e+02f,
      6.8238059344e+02f},
     {-3.8359934395e+01f, -4.0542310260e+02f, 1.8115317477e+03f,
      -2.1218156300e+03f},
     {-1.5218468500e+00f, -5.9563538882e+01f, 2.0109992317e+02f,
      -2.1220384563e+02f},
     {-2.2493423432e+00f, -1.9214712587e+02f, 3.1371028533e+02f,
      -1.6005001403e+02f},
     {1.5749098093e-02f, -2.7932451730e+01f, 3.2487248357e+01f,
      -2.1614638873e+00f}}};

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
