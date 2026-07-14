/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-28 15:55:50
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-05-28 06:20:05
 * @Description: Step-climbing/over-step action state implementation / 跃障扫摆及大扭矩起跳状态实现
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#include "pyro_wl_chassis.h"
#include "pyro_algo_common.h"

#define LENGTH_SPEED (0.1f/200.0f)       /* Leg extension rate during over-step sweep / 跃障时的伸缩速度 */
#define ANGLE_SPEED (PI/800.0f)         /* Linkage rotation speed / 跃障连杆角度变化量 */
#define TARGET_ANGLE (2.47f)             /* Target polar angle for landing posture after jump / 跃障摆动目标终点极坐标摆角 */
#define TEMP_ANGLE (-2.85f)              /* Temporary polar angle for initial sweep / 跃障动作预摆扫极坐标起始角 */
#define TEST_ANGLE (PI/2.0f + 0.5f)
#define TARGET_LENGTH 0.17f              /* Target leg length during sweep jump / 跃障动作高度 (m) */

namespace pyro
{
extern pid_t wheel_disable_pid[2];
static float target_length[2] = {0.0f, 0.0f};
static float target_angle[2] = {0.0f, 0.0f};
static float cur_length[2] = {0.0f, 0.0f};
static float cur_angle[2] = {0.0f, 0.0f};
static uint8_t state_flag[2] = {0, 0}; /* Trajectory sweep sub-phase flags / 扫腿跃障动作子阶段 */

/* Enter over-step state / 进入越障状态 */
void wl_chassis_t::fsm_active_t::state_over_step_t::enter(wl_chassis_t *owner)
{
    /* Reset LQR errors during joint sweep / 跃障扫腿动作前，清零 LQR 自平衡偏差以切断自平衡 */
    for(uint8_t i = 0; i < 2; i++)
    {
        owner->_leg_data[i].x_bias = 0.0f;
        owner->_leg_data[i].d_x_bias = 0.0f;
        owner->_leg_data[i].beta_bias = 0.0f;
        owner->_leg_data[i].d_beta_bias = 0.0f;
        owner->_leg_data[i].gamma_bias = 0.0f;
        owner->_leg_data[i].d_gamma_bias = 0.0f;
    }
    
    /* Record initial values / 记录起步姿态几何量 */
    owner->get_cur_angle(&cur_angle[wl_chassis_t::R], 
                        &cur_angle[wl_chassis_t::L]);
    owner->get_cur_length(&cur_length[wl_chassis_t::R], 
                    &cur_length[wl_chassis_t::L]);
                    
    target_length[wl_chassis_t::R] = cur_length[wl_chassis_t::R];
    target_length[wl_chassis_t::L] = cur_length[wl_chassis_t::L];
    target_angle[wl_chassis_t::R] = cur_angle[wl_chassis_t::R];
    target_angle[wl_chassis_t::L] = cur_angle[wl_chassis_t::L];
    
    owner->_active_mode_flag.over_step = 0;
    
    for(uint8_t i = 0; i < 2; i++)
    {
        state_flag[i] = 0;
    }
}

/**
 * @brief Execute over-step state loop. Forces high-speed wheel counter-rotation and sweep leg coordinates.
 *        执行扫腿越障逻辑。
 *        车轮控制：双轮向相反方向输入高转速（+100 rad/s / -100 rad/s），产生猛烈的前后撕扯与向前跨步动量。
 *        连杆扫摆：通过嵌套 PID，双腿先逆向偏摆至 TEMP_ANGLE (-2.85 rad)，随后迅速调整长度至 TARGET_LENGTH (0.17m)，
 *        最后偏摆扫至 TARGET_ANGLE (2.47 rad)，完成强行跃障。全部完成并防抖后，置 ready 标志退出。
 */
void wl_chassis_t::fsm_active_t::state_over_step_t::execute(wl_chassis_t *owner)
{
    /* Force high-speed counter-rotation command to wheels / 轮电机制动速度环控制到正负 100 rad/s，形成越障爬墙力矩 */
    float t = wheel_disable_pid[wl_chassis_t::R].calculate(100.0f,
                 owner->_wheel_drv[wl_chassis_t::R]->get_current_rotate());
    owner->_wheel_drv[wl_chassis_t::R]->send_torque(t);
    
    t = wheel_disable_pid[wl_chassis_t::L].calculate(-100.0f,
                 owner->_wheel_drv[wl_chassis_t::L]->get_current_rotate());
    owner->_wheel_drv[wl_chassis_t::L]->send_torque(t);

    /* Generate planned trajectory coordinates / 轨迹规划插值 */
    calc_target_value(owner);

    /* Calculate expansion force F_leg (F[0]) using nested PID / 伸缩力 F[0] */
    /* Right leg / 右腿 */
    owner->_leg_data[wl_chassis_t::R].ref_d_l =
        owner->_F_pid[wl_chassis_t::R]->calculate(target_length[wl_chassis_t::R], owner->_leg_data[wl_chassis_t::R].l);
    owner->_leg_data[wl_chassis_t::R].F[0] =
        owner->_d_F_pid[wl_chassis_t::R]->calculate(owner->_leg_data[wl_chassis_t::R].ref_d_l, owner->_leg_data[wl_chassis_t::R].d_l);
    /* Left leg / 左腿 */
    owner->_leg_data[wl_chassis_t::L].ref_d_l =
        owner->_F_pid[wl_chassis_t::L]->calculate(target_length[wl_chassis_t::L], owner->_leg_data[wl_chassis_t::L].l);
    owner->_leg_data[wl_chassis_t::L].F[0] =
        owner->_d_F_pid[wl_chassis_t::L]->calculate(owner->_leg_data[wl_chassis_t::L].ref_d_l, owner->_leg_data[wl_chassis_t::L].d_l);

    /* Calculate hip virtual torque T_hip (F[1]) using nested PID / 扫摆关节扭矩 F[1] */
    /* Right leg / 右腿 */
    float diff;
    if(target_angle[wl_chassis_t::R] - owner->_leg_data[wl_chassis_t::R].alpha > PI)
    {
        diff = -2 * PI + (target_angle[wl_chassis_t::R] - owner->_leg_data[wl_chassis_t::R].alpha);
    }
    else if(target_angle[wl_chassis_t::R] - owner->_leg_data[wl_chassis_t::R].alpha < -PI)
    {
        diff = 2 * PI + (target_angle[wl_chassis_t::R] - owner->_leg_data[wl_chassis_t::R].alpha);
    }
    else
    {
        diff = target_angle[wl_chassis_t::R] - owner->_leg_data[wl_chassis_t::R].alpha;
    }
    owner->_leg_data[wl_chassis_t::R].ref_d_alpha =
        owner->_T_pid[wl_chassis_t::R]->calculate(owner->_leg_data[wl_chassis_t::R].alpha + diff, owner->_leg_data[wl_chassis_t::R].alpha);
    owner->_leg_data[wl_chassis_t::R].F[1] =
        owner->_d_T_pid[wl_chassis_t::R]->calculate(owner->_leg_data[wl_chassis_t::R].ref_d_alpha, owner->_leg_data[wl_chassis_t::R].d_alpha);
        
    /* Left leg / 左腿 */
    if(target_angle[wl_chassis_t::L] - owner->_leg_data[wl_chassis_t::L].alpha > PI)
    {
        diff = -2 * PI + (target_angle[wl_chassis_t::L] - owner->_leg_data[wl_chassis_t::L].alpha);
    }
    else if(target_angle[wl_chassis_t::L] - owner->_leg_data[wl_chassis_t::L].alpha < -PI)
    {
        diff = 2 * PI + (target_angle[wl_chassis_t::L] - owner->_leg_data[wl_chassis_t::L].alpha);
    }
    else
    {
        diff = target_angle[wl_chassis_t::L] - owner->_leg_data[wl_chassis_t::L].alpha;
    }
    owner->_leg_data[wl_chassis_t::L].ref_d_alpha =
        owner->_T_pid[wl_chassis_t::L]->calculate(owner->_leg_data[wl_chassis_t::L].alpha + diff, owner->_leg_data[wl_chassis_t::L].alpha);
    owner->_leg_data[wl_chassis_t::L].F[1] =
        owner->_d_T_pid[wl_chassis_t::L]->calculate(owner->_leg_data[wl_chassis_t::L].ref_d_alpha, owner->_leg_data[wl_chassis_t::L].d_alpha);
    
    /* VMC Jacobian transpose mapping to convert forces / VMC 转换映射 */
    for(uint8_t i = 0; i < 2; i++)
    {
        arm_mat_vec_mult_f32(&owner->_leg_data[i].T_mat, 
                            owner->_leg_data[i].F, 
                            owner->_leg_data[i].T);
    }

    /* Send commands to joint motors / 下发关节驱动扭矩，右侧取反 */
    owner->_motor_drv[wl_chassis_t::RF]->send_torque(
                        -owner->_leg_data[wl_chassis_t::R].T[0]);
    owner->_motor_drv[wl_chassis_t::RB]->send_torque(
                        -owner->_leg_data[wl_chassis_t::R].T[1]);
    owner->_motor_drv[wl_chassis_t::LF]->send_torque(
                        owner->_leg_data[wl_chassis_t::L].T[0]);
    owner->_motor_drv[wl_chassis_t::LB]->send_torque(
                        owner->_leg_data[wl_chassis_t::L].T[1]);
                        
    /* Verify geometric target stance achieved / 检测越障目标几何高度和偏摆角度是否到位 */
    if(
       (0.05f > abs(owner->_leg_data[wl_chassis_t::R].alpha - TARGET_ANGLE)) &&
       (0.05f > abs(owner->_leg_data[wl_chassis_t::L].alpha - TARGET_ANGLE)) &&
       (0.01f > abs(owner->_leg_data[wl_chassis_t::R].l - TARGET_LENGTH)) &&
       (0.01f > abs(owner->_leg_data[wl_chassis_t::L].l - TARGET_LENGTH)))
    {
        static uint32_t cnt = 0;
        cnt++;
        if(cnt > 100)
        {
            cnt = 0;
            owner->_active_mode_flag.over_step = 1; /* Mark action complete / 标记跃障扫摆完成，通知底盘主控 */
        }
    }
}

void wl_chassis_t::fsm_active_t::state_over_step_t::exit(wl_chassis_t *owner)
{
}

/**
 * @brief Over-step sweep trajectory calculation.
 *        跃障姿态规划。
 *        第0阶段：将极轴偏角偏扫向 TEMP_ANGLE (-2.85 rad) 起点。
 *        第1阶段：将极向腿长收缩拉紧至 TARGET_LENGTH (0.17m)。
 *        第2阶段：将极轴偏角摆扫至 TARGET_ANGLE (2.47 rad) 终点。
 */
void wl_chassis_t::fsm_active_t::state_over_step_t::calc_target_value(wl_chassis_t *owner)
{
    for(uint8_t i = 0; i < 2; i++)
    {
        /* Phase 0: Target initial sweep angle / 第0阶段：逆向偏摆摆角 */
        if(0 == state_flag[i])
        {
            if(0.05f > abs(target_angle[i] - TEMP_ANGLE))
            {
                target_angle[i] = TEMP_ANGLE;
            }
            else 
            {
                target_angle[i] += ANGLE_SPEED;
                target_angle[i] = wrap2pi_f32(target_angle[i]);
            }
            if(0.05f > abs(owner->_leg_data[i].alpha - TEMP_ANGLE))
            {
                state_flag[i] = 1;
            }
        }
        
        /* Phase 1: Adjust sweep target length / 第1阶段：调节高度摆长 */
        if(1 == state_flag[i])
        {
            if(0.01f > abs(target_length[i] - TARGET_LENGTH))
            {
                target_length[i] = TARGET_LENGTH;
            }
            else if(target_length[i] < TARGET_LENGTH)
            {
                target_length[i] += LENGTH_SPEED;
            }
            else if(target_length[i] > TARGET_LENGTH)
            {
                target_length[i] -= LENGTH_SPEED;
            }

            if(0.01f > abs(owner->_leg_data[i].l - TARGET_LENGTH))
            {
                state_flag[i] = 2;
            }
        }
        
        /* Phase 2: Target sweep final angle / 第2阶段：顺向大扫角偏转，获得向上动力 */
        if(2 == state_flag[i])
        {
            if(0.05f > abs(target_angle[i] - TARGET_ANGLE))
            {
                target_angle[i] = TARGET_ANGLE;
            }
            else 
            {
                target_angle[i] -= ANGLE_SPEED;
                target_angle[i] = wrap2pi_f32(target_angle[i]);
            }
            if(0.05f > abs(owner->_leg_data[i].alpha - TARGET_ANGLE))
            {
                state_flag[i] = 3;
            }
        }
    }
}
}