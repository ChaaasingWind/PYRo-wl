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

constexpr float MAX_LEG_LENGTH         = 0.38f;
constexpr float MIN_LEG_LENGTH         = 0.18f;

constexpr float LEFT_LEG_DIRECTION     = -1.0f;
constexpr float RIGHT_LEG_DIRECTION    = 1.0f;

constexpr float LEFT_WHEEL_DIRECTION   = -1.0f;
constexpr float RIGHT_WHEEL_DIRECTION  = 1.0f;

// Single-leg gravity compensation parameters.
constexpr float SINGLE_LEG_BODY_MASS   = 6.0f;
constexpr float LEG_MASS               = 1.232f;
constexpr float GRAVITY_ACCELERATION   = 9.81f;
constexpr float LEG_GRAVITY_FORCE =
    (SINGLE_LEG_BODY_MASS + LEG_MASS) * GRAVITY_ACCELERATION * 0.9f;
constexpr float MAX_F_L                              = 300.0f;
constexpr float MAX_T_P                              = 60.0f;

constexpr float K_t                                  = 0.21777611f;
constexpr float reduction_ratio                      = 13.94f;
constexpr float rec_reduction_ratio                  = 1 / reduction_ratio;
constexpr uint32_t L_WP_POLY_DEGREE                  = 3;
constexpr uint32_t K_POLY_DEGREE                     = 3;
// L_wp(L) = 0.0581 + 0.3760*L + 0.7972*L^2 - 0.7381*L^3.
// Coefficients are in ascending-power order: c0, c1, c2, c3.
constexpr float L_WP_POLY_COEF[L_WP_POLY_DEGREE + 1] = {0.0581f, 0.3760f,
                                                        0.7972f, -0.7381f};

// Q , R 矩阵
// Q_diag = [80.0, 20.0, 500.0, 5.0, 100.0, 5.0]
// R_diag = [2.4, 0.1]


constexpr float K_POLY_COEF[2][6][K_POLY_DEGREE + 1] = {
    {
        {-2.4663969430e+00f, -1.1221246755e+01f, 8.2441383874e+00f, 7.7491808382e+00f},
        {-4.0800048863e+00f, -4.2162117186e+00f, -1.3067217413e+01f, 2.7811855784e+01f},
        {1.0199790603e+00f, 1.1105800554e+02f, -1.7896297152e+02f, 1.2388780332e+02f},
        {1.7376472929e-02f, 1.0854695756e+01f, 8.4590182091e-01f, -3.5491427925e+00f},
        {-9.7972102301e+00f, 2.1962639916e+01f, -1.7319281907e+01f, -2.1049385000e+00f},
        {-2.2620303317e+00f, 4.6166263075e+00f, -3.5155168256e+00f, -6.1514272087e-01f}
    },
    {
        {-5.0242044524e+00f, -1.1967726249e+02f, 5.1828332182e+02f, -6.0840074206e+02f},
        {-5.8894723376e+00f, -1.2382312965e+02f, 5.4711432440e+02f, -6.5059566124e+02f},
        {2.9331702013e+01f, 3.7144255222e+02f, -1.5917377417e+03f, 1.8416359281e+03f},
        {1.6006021765e+00f, 5.7295834353e+01f, -1.9353820750e+02f, 2.0497514478e+02f},
        {1.3098854040e-01f, 1.3652621658e+02f, -2.0067016188e+02f, 7.7438914050e+01f},
        {-3.8384769954e-02f, 2.7607774817e+01f, -3.2680929502e+01f, 3.0500754499e+00f}
    }
};

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
