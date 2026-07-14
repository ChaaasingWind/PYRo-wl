/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-28 15:55:50
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-03-10 13:50:42
 * @Description: Zero torque test state implementation / 电机零力矩使能测试状态实现
 *
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved.
 */
#include "pyro_wl_chassis.h"
#include "pyro_algo_common.h"
namespace pyro
{
void wl_chassis_t::fsm_active_t::state_test_t::enter(wl_chassis_t *owner)
{
}

/**
 * @brief Execute test state loop. Sends zero torque to all motors.
 *        执行零力矩测试逻辑。所有的轮毂电机和直驱关节电机全部写入 0.0f 扭矩。
 *        用以测试在带电使能状态下，各关节在受到外力偏转时的编码器零位与传感器状态反馈。
 */
void wl_chassis_t::fsm_active_t::state_test_t::execute(wl_chassis_t *owner)
{
    /* Output zero torque command to 2 wheel motors / 车轮输出零力矩 */
    owner->_ctx.motor.wheel[wl_chassis_t::R]->send_torque(0.0f);
    owner->_ctx.motor.wheel[wl_chassis_t::L]->send_torque(0.0f);

    /* Output zero torque command to 4 joint motors / 关节连杆输出零力矩 */
    owner->_ctx.motor.joint[wl_chassis_t::RF]->send_torque(0.0f);
    owner->_ctx.motor.joint[wl_chassis_t::RB]->send_torque(0.0f);
    owner->_ctx.motor.joint[wl_chassis_t::LF]->send_torque(0.0f);
    owner->_ctx.motor.joint[wl_chassis_t::LB]->send_torque(0.0f);
}

void wl_chassis_t::fsm_active_t::state_test_t::exit(wl_chassis_t *owner)
{
}
}
