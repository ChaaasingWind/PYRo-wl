/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-28 13:11:52
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-05-29 06:24:46
 * @Description: Reverse sweep state implementation / 倒地/关节反摆复原状态实现
 *
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved.
 */
#include "pyro_wl_chassis.h"

#define ANGLE_SPEED -(PI/3.0f) /* Fixed speed for active leg rotation / 倒地恢复摆角的定转速 (rad/s) */

namespace pyro
{
extern pid_t wheel_disable_pid[2];
static uint8_t wheel_disable_flag[2] = {0, 0};

/* Enter reverse sweep state / 进入反向摆腿状态 */
void wl_chassis_t::fsm_active_t::state_reverse_t::enter(wl_chassis_t *owner)
{
    /* Set wheels to brake state / 设定轮毂制动刹车状态 */
    for(uint8_t i = 0; i < 2; i++)
    {
        wheel_disable_flag[i] = 1;
    }
}

/**
 * @brief Execute reverse sweep state loop. Rotates legs backward at a fixed velocity.
 *        执行反向摆腿恢复逻辑。使用 PID 锁死轮毂电机转速为 0。
 *        双腿摆长仍维持设定的参考指令高度。摆角以固定的角速度 -(PI/3) (约-60 deg/s) 逆向摆动，
 *        以实现摔倒后依靠关节电机扫腿翻身或姿态调头。
 */
void wl_chassis_t::fsm_active_t::state_reverse_t::execute(wl_chassis_t *owner)
{
    /* Use damping PID to stop wheels / 比例微分阻尼力矩刹停左右轮毂电机 */
    for(uint8_t i = 0; i < 2; i++)
    {
        float t = wheel_disable_pid[i].calculate(0.0f,
             owner->_ctx.motor.wheel[i]->get_current_rotate());
        owner->_ctx.motor.wheel[i]->send_torque(t);
    }

    /* Keep leg expansion force F_leg (F[0]) closed-loop servo tracking the reference target length
       双腿极向伸长高度闭环控制跟踪参考指令 */
    /* Right leg / 右腿 */
    owner->_ctx.data.leg[wl_chassis_t::R].ref_d_l=
            owner->_ctx.pid.F[wl_chassis_t::R]->
        calculate(owner->_ctx.cmd->r_leg,
        owner->_ctx.data.leg[wl_chassis_t::R].l);
    owner->_ctx.data.leg[wl_chassis_t::R].F[0]=
            owner->_ctx.pid.F[wl_chassis_t::R]->
              calculate(owner->_ctx.data.leg[wl_chassis_t::R].ref_d_l,
        owner->_ctx.data.leg[wl_chassis_t::R].d_l);
    /* Left leg / 左腿 */
    owner->_ctx.data.leg[wl_chassis_t::L].ref_d_l=
        owner->_ctx.pid.F[wl_chassis_t::L]->
        calculate(owner->_ctx.cmd->l_leg,
        owner->_ctx.data.leg[wl_chassis_t::L].l);
    owner->_ctx.data.leg[wl_chassis_t::L].F[0]=
        owner->_ctx.pid.F[wl_chassis_t::L]->
        calculate(owner->_ctx.data.leg[wl_chassis_t::L].ref_d_l,
        owner->_ctx.data.leg[wl_chassis_t::L].d_l);

    /* Set target polar angular velocity to ANGLE_SPEED (-PI/3)
       关节摆轴转矩计算：设定目标摆角速度为 ANGLE_SPEED (-PI/3 rad/s) 闭环跟踪 */
    /* Right leg / 右腿 */
    owner->_ctx.data.leg[wl_chassis_t::R].ref_d_alpha = ANGLE_SPEED;
    owner->_ctx.data.leg[wl_chassis_t::R].F[1]=
        owner->_ctx.pid.d_T[wl_chassis_t::R]->
        calculate(owner->_ctx.data.leg[wl_chassis_t::R].ref_d_alpha,
        owner->_ctx.data.leg[wl_chassis_t::R].d_alpha);
    /* Left leg / 左腿 */
    owner->_ctx.data.leg[wl_chassis_t::L].ref_d_alpha = ANGLE_SPEED;
    owner->_ctx.data.leg[wl_chassis_t::L].F[1]=
        owner->_ctx.pid.d_T[wl_chassis_t::L]->
        calculate(owner->_ctx.data.leg[wl_chassis_t::L].ref_d_alpha,
        owner->_ctx.data.leg[wl_chassis_t::L].d_alpha);

    /* VMC Jacobian transpose mapping / 运动学极坐标虚拟力转换映射 */
    for(uint8_t i = 0; i < 2; i++)
    {
        arm_mat_vec_mult_f32(&owner->_ctx.data.leg[i].T_mat,
                            owner->_ctx.data.leg[i].F,
                            owner->_ctx.data.leg[i].T);
    }

    /* Send commands to joint motors / 下发关节电机力矩，右侧取反 */
    owner->_ctx.motor.joint[wl_chassis_t::RF]->send_torque(
                        -owner->_ctx.data.leg[wl_chassis_t::R].T[0]);
    owner->_ctx.motor.joint[wl_chassis_t::RB]->send_torque(
                        -owner->_ctx.data.leg[wl_chassis_t::R].T[1]);
    owner->_ctx.motor.joint[wl_chassis_t::LF]->send_torque(
                        owner->_ctx.data.leg[wl_chassis_t::L].T[0]);
    owner->_ctx.motor.joint[wl_chassis_t::LB]->send_torque(
                        owner->_ctx.data.leg[wl_chassis_t::L].T[1]);
}

void wl_chassis_t::fsm_active_t::state_reverse_t::exit(wl_chassis_t *owner)
{
}
}