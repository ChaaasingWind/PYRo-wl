/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-26 19:51:12
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-05-25 13:12:07
 * @Description: Wheel-legged chassis passive state implementation / 轮腿底盘被动失能状态实现
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#include "pyro_wl_chassis.h"
#include "pyro_algo_pid.h"

namespace pyro
{
    
static uint8_t wheel_disable_flag[2] = {0, 0}; /* Wheel brake state flag / 车轮刹车制动状态标志 */

/* Brake PID to bring wheel speed to zero smoothly
   刹车比例阻尼控制器，用于在失能时平缓地把轮毂转速刹停到 0，防止由于反电动势或惯性导致暴冲 */
pid_t wheel_disable_pid[2] = {
    pid_t(0.1f, 0.0f, 0.0f, 0.0f, 10.0f), 
    pid_t(0.1f, 0.0f, 0.0f, 0.0f, 10.0f)};

/**
 * @brief Enter passive state. Disables joint motors.
 *        进入 Passive（失能被动）状态。首先直接失能 4 个关节达妙电机，并开启车轮制动标志。
 * @param owner Pointer to the wheel-legged chassis instance.
 */
void wl_chassis_t::state_passive_t::enter(wl_chassis_t *owner)
{
    /* Disable all 4 joint motors immediately / 立即停止 4 个关节电机的驱动使能 */
    for(uint8_t i = 0; i < 4; i++)
    {
        owner->_motor_drv[i]->disable();
    }
    
    /* Set wheel braking flags / 激活车轮制动状态 */
    for(uint8_t i = 0; i < 2; i++)
    {
        wheel_disable_flag[i] = 1;
    }
}

/**
 * @brief Execute passive state logic. Damps wheel speeds to zero.
 *        Passive 状态周期执行逻辑。对左右车轮应用比例阻尼刹车，直到速度低于 0.1 rad/s 时完全释放使能。
 *        同时将所有估算的控制偏差（里程计、倾角、俯仰角偏差）逐步置零/复位。
 */
void wl_chassis_t::state_passive_t::execute(wl_chassis_t *owner)
{
    for(uint8_t i = 0; i < 2; i++)
    {
        /* Reset telemetry biases to prevent integrator windup on next standup
           复位偏差量，防止下一次起动时控制器发生积分饱和 */
        owner->_leg_data[i].x_bias = 0.0f;
        owner->_leg_data[i].d_x_bias = 0.0f;
        owner->_leg_data[i].beta_bias = 0.0f;
        owner->_leg_data[i].d_beta_bias = 0.0f;
        owner->_leg_data[i].gamma_bias = 0.0f - owner->_leg_data[i].gamma;
        owner->_leg_data[i].d_gamma_bias = 0.0f - owner->_leg_data[i].d_gamma;
        
        /* Apply damping torque to wheels / 如果刹车标志激活，计算阻尼力矩施加在轮子上 */
        if(1 == wheel_disable_flag[i])
        {
            float t = wheel_disable_pid[i].calculate(0.0f,
                 owner->_wheel_drv[i]->get_current_rotate());
            owner->_wheel_drv[i]->send_torque(t);
        }
        else 
        {
            owner->_wheel_drv[i]->send_torque(0.0f);
        }
        
        /* Disable wheel motor completely when speed is near zero
           转速低于阈值 0.1 rad/s，彻底失能大疆轮毂电机，完全断电 */
        if(0.1f > abs(owner->_wheel_drv[i]->get_current_rotate()))
        {
            wheel_disable_flag[i] = 0;
            owner->_wheel_drv[i]->disable();
        }
    }
    
    /* Ensure joint motors receive zero torque safety command
       保证已失能的达妙关节电机不产生任何寄生输出 */
    for(uint8_t i = 0; i < 4; i++)
    {
        owner->_motor_drv[i]->send_torque(0.0f);
    }
}

void wl_chassis_t::state_passive_t::exit(wl_chassis_t *owner)
{
}

}