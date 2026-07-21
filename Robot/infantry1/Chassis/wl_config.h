#ifndef __WL_CONFIG_H__
#define __WL_CONFIG_H__


namespace pyro
{

constexpr float OJ5                   = 0.0945f;
constexpr float J4J5                  = 0.1125f;
constexpr float OJ8                   = 0.2100f;
// constexpr float MIN_LEG_LENGTH = 0.18f;
// constexpr float MAX_LEG_LENGTH = 2.5f;
constexpr float LEFT_HIP_OFFSET       = 0.6f;
constexpr float LEFT_KNEE_OFFSET      = 3.11f;

constexpr float RIGHT_HIP_OFFSET      = 0.0f;
constexpr float RIGHT_KNEE_OFFSET     = 0.0f;

constexpr float MAX_LEG_LENGTH        = 0.38f;
constexpr float MIN_LEG_LENGTH        = 0.18f;

constexpr float LEFT_LEG_DIRECTION    = -1.0f;
constexpr float RIGHT_LEG_DIRECTION   = 1.0f;

constexpr float LEFT_WHEEL_DIRECTION  = -1.0f;
constexpr float RIGHT_WHEEL_DIRECTION = 0.0f;

constexpr float K_t                   = 0.21777611f;
constexpr float reduction_ratio       = 13.94f;
constexpr float rec_reduction_ratio   = 1 / reduction_ratio;


namespace leg_def
{
enum : uint8_t
{
    L = 0, //LEFT
    R = 1  //RIGHT
};
}
namespace motor_def
{
enum : uint8_t
{
    HIP  = 0,
    KNEE = 1,
};
}
} // namespace pyro
#endif
