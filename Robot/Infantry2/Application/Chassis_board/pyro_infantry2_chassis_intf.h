/*
 * @Author: vod-x vod_x@outlook.com
 * @Date: 2026-05-29 02:30:45
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-05-29 04:33:49
 * @FilePath: \Wheel-Legged-Robot\embedded_system\PYRo\Application\Mission\Infantry_2\pyro_infantry2_chassis_intf.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include <stdint.h>
#include <array>

namespace pyro
{
union GimbalToChassisComm {

    __attribute__((packed)) struct {
        int32_t vx    : 6; //  正方向： 向前
        int32_t vy    : 6; // 正方向： 向左
        uint32_t mode : 4;
        uint32_t shootEn  : 1;
        uint32_t resetUI  : 1;
        uint32_t fn1Switch: 1;
        uint32_t turboMode    : 1; // [R] 飞坡
        uint32_t stepClimb    : 1; // [E] 上台阶
        uint32_t legLength    : 2; // [Z] 腿长 (0/1/2)
        uint32_t selfRescue   : 1; // [G] 自救
        uint32_t manualRescue : 1; // [Ctrl] 手动自救
        uint32_t gimbalReverse: 1; // [X] 调头
        uint32_t jump         : 1; // [V] 跳跃
        uint32_t capSwitch    : 1; // [C] 超级电容开关
        uint32_t fireState    : 4; // 发射机构 FSM 状态 (FireState)
        uint32_t aimMode      : 2; // [B] 自瞄模式 (0~3)
        int16_t yawvel     ;
    } msg;

    std::array<uint8_t, 8> buffer;
};

union ChassisToGimbalComm {

    __attribute__((packed)) struct {
        // 将 float (4字节) 压缩为 uint16_t (2字节) 传初速度，乘以 100 发送，云台除以 100
        uint32_t initialSpeedX100      : 15; // 弹丸初速度 * 100 (2 Bytes)
        uint32_t shooter17mmBarrelHeat : 16; // 17mm 枪口当前热量 (2 Bytes)
        uint32_t heatLimit             : 9; // 热量上限 (如 150, 240, 360)
        uint32_t coolingRate           : 7; // 冷却速率 (如 40, 60, 80)
         uint8_t chassisReady           : 1;
        uint8_t robotId              : 8;     // 机器人 ID (1 Byte)
        int8_t chassisYawSpeed        : 8;
      
    } msg;

    std::array<uint8_t, 8> buffer;
};

struct cmd_t
{
    float vx;
    float vy;
    float turn_angle;
    float v;
    float yaw_vel;
    enum
    {
        PASSIVE = 0x00,
        ACTIVE = 0x01,
        SPIN = 0x02,
        STEP_CLIMB= 0x03,
        REVERSE = 0x04,
    }mode;
    uint8_t leg_length_mode;
};
}