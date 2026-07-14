/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-26 20:03:11
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-05-28 04:02:51
 * @Description: Active state machine transition logic / 主动状态机分发与转换逻辑
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#include "pyro_wl_chassis.h"
#include "pyro_algo_common.h"

namespace pyro
{

/**
 * @brief Enter active state container. Enables all motors and resets telemetry.
 *        进入 Active 主动模式总入口。清除电机报错，使能所有直驱关节电机与轮毂电机，
 *        并将所有的里程计累计和滤波器状态重置清零。
 */
void wl_chassis_t::fsm_active_t::on_enter(wl_chassis_t *owner)
{ 
    /* Enable all 4 joint motors and clear errors if any
       使能 4 个关节电机。如果检测到电机驱动存在报错，进行错误清除。 */
    for(uint8_t i = 0; i < 4; i++)
    {
        if(dm_motor_drv_t::ok != owner->_motor_drv[i]->get_error_code())
        {
            owner->_motor_drv[i]->clear_error();
        }
        owner->_motor_drv[i]->enable();
    }
    
    /* Enable 2 wheel motors / 使能左右轮毂电机 */
    for(uint8_t i = 0; i < 2; i++)
    {
        owner->_wheel_drv[i]->enable();
    }
    
    /* Reset local displacement and Kalman filter states
       重置绝对位移累计，保证站立定位初始点归零。重置轮速状态滤波器。 */
    owner->_leg_data[wl_chassis_t::R].x = 0.0f;
    owner->_leg_data[wl_chassis_t::L].x = 0.0f;
    owner->_leg_data[wl_chassis_t::R].x_gain = 0.0f;
    owner->_leg_data[wl_chassis_t::L].x_gain = 0.0f;
    owner->_leg_data[wl_chassis_t::R].kf_x = 0.0f;
    owner->_leg_data[wl_chassis_t::L].kf_x = 0.0f;
    
    owner->_wheel_kf[wl_chassis_t::R].reset();
    owner->_wheel_kf[wl_chassis_t::L].reset();
}

/**
 * @brief Execute active state dispatching. Switches sub-states based on command.
 *        主动模式周期执行逻辑。根据接收到的底盘指令模式 `active_mode`，分发并切换至对应的自平衡或动作子状态。
 */
void wl_chassis_t::fsm_active_t::on_execute(wl_chassis_t *owner)
{
    /* Transition to Reverse / 切换至反转/倒地倒起摆腿 */
    if(owner->_cmd->active_mode == wl_cmd_t::REVERSE)
    {
        this->change_state(&_state_reverse);
    }
    
    /* Transition to Ready (Stand-up stance) / 切换至起身准备状态 */
    if(owner->_cmd->active_mode == wl_cmd_t::READY)
    {
        /* Stance transition bypass: if coming from over-step, set ready flag directly
           动作保护：如果是从跃障准备状态返回，直接标记就绪，跳过起立过程 */
        if(owner->_cmd->last_active_mode == wl_cmd_t::OVER_STEP_READY)
        {
            owner->_active_mode_flag.ready = 1;
            return;
        }
        this->change_state(&_state_ready);
    }
    
    /* Transition to Normal (Balance driving) / 切换至正常行驶与 LQR 平衡 */
    if(owner->_cmd->active_mode == wl_cmd_t::NORMAL)
    {
        this->change_state(&_state_normal);
    }
    
    /* Transition to Test (Zero torque) / 切换至电机零力矩使能测试 */
    if(owner->_cmd->active_mode == wl_cmd_t::TEST)
    {
        this->change_state(&_state_test);
    }
    
    /* Transition to Over-Step (Sweeping obstacles) / 切换至跃障跃起动作 */
    if(owner->_cmd->active_mode == wl_cmd_t::OVER_STEP)
    {
        this->change_state(&_state_over_step);
    }
    
    /* Transition to Over-Step Ready (Balanced posture above obstacles) / 切换至越障后平衡姿态 */
    if(owner->_cmd->active_mode == wl_cmd_t::OVER_STEP_READY)
    {
        this->change_state(&_state_over_step_ready);
    }
    
    /* Transition to Over-Step Reset (Stance recovery) / 切换至越障复位状态 */
    if(owner->_cmd->active_mode == wl_cmd_t::OVER_STEP_RESET)
    {
        this->change_state(&_state_over_step_reset);
    }
    
    /* Transition to Control (Manual debug target tracking) / 切换至关节角度与摆长直控调试 */
    if(owner->_cmd->active_mode == wl_cmd_t::CONTROL)
    {
        this->change_state(&_state_control);
    }
    
    /* Transition to Spin (Rapid yaw rotation) / 切换至底盘自旋（小陀螺）状态 */
    if(owner->_cmd->active_mode == wl_cmd_t::SPIN)
    {
        this->change_state(&_state_spin);
    }
}

void wl_chassis_t::fsm_active_t::on_exit(wl_chassis_t *owner)
{
}
}
