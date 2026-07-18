#ifndef __WL_CONFIG_H__
#define __WL_CONFIG_H__


namespace pyro
{

constexpr float OJ5               = 0.0945f;
constexpr float J4J5              = 0.1125f;
constexpr float OJ8               = 0.2100f;
// constexpr float MIN_LEG_LENGTH = 0.18f;
// constexpr float MAX_LEG_LENGTH = 2.5f;
constexpr float LEFT_HIP_OFFSET   = 0.6f;
constexpr float LEFT_KNEE_OFFSET  = 3.11f;

constexpr float RIGHT_HIP_OFFSET  = 0.0f;
constexpr float RIGHT_KNEE_OFFSET = 0.0f;

constexpr float LEFT_DIRECTION    = -1.0f;
constexpr float RIGHT_DIRECTION   = 1.0f;

namespace leg_def
{
enum : uint8_t
{
    LEFT  = 0,
    RIGHT = 1
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
}
#endif
