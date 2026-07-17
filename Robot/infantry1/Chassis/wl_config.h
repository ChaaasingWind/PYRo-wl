#ifndef __WL_CONFIG_H__
#define __WL_CONFIG_H__

constexpr float OJ5 = 0.0945f;
constexpr float J4J5 = 0.1125f;
constexpr float OJ8 = 0.2100f;

namespace leg_def
{
enum : uint8_t
{
    LEFT = 0,
    RIGHT = 1
};
}
namespace motor_def
{
enum : uint8_t
{
    HIP = 0,
    KNEE = 1,
};
}
#endif
