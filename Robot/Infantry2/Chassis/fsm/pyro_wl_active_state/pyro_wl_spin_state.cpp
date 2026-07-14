/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-03-01 13:11:52
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-05-30 01:54:34
 * @Description: Chassis spin (top-spin) control state / 底盘自旋（小陀螺）状态控制实现
 *
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved.
 */
#include "pyro_wl_chassis.h"
#include "pyro_algo_common.h"
#include "pyro_referee.h"

namespace pyro
{
extern pid_t wheel_disable_pid[2];
extern referee_drv_t *referee_drv;

/* Yaw turn PID for small spinning correction / 小陀螺旋转偏航角速度 PID */
pid_t wheel_turn_pid[2] = {
    pid_t(5.0, 0.0f, 0.0f, 2.0f, 20.0f),
    pid_t(5.0, 0.0f, 0.0f, 2.0f, 20.0f)};

/* Enter spin-top state / 进入小陀螺自旋状态 */
void wl_chassis_t::fsm_active_t::state_spin_t::enter(wl_chassis_t *owner)
{
    /* Align target displacement to current estimated position to freeze relative motion
       锁死位置指令期望 x_gain 到当前估计点，防止由于旋转导致里程计漂移从而把车拉走 */
    owner->_ctx.data.leg[wl_chassis_t::R].x_gain = owner->_ctx.data.leg[wl_chassis_t::R].kf_x;
    owner->_ctx.data.leg[wl_chassis_t::L].x_gain = owner->_ctx.data.leg[wl_chassis_t::L].kf_x;

    owner->_ctx.data.leg[wl_chassis_t::R].d_x_gain = 0.0f;
    owner->_ctx.data.leg[wl_chassis_t::L].d_x_gain = 0.0f;
    owner->_ctx.data.active_mode_flag.ready = 1;
}

/* Execute spin-top state logic / 执行自旋控制 */
void wl_chassis_t::fsm_active_t::state_spin_t::execute(wl_chassis_t *owner)
{
    float cap_volt = owner->_ctx.power.voltage;
    if(cap_volt < 5.0f) {
        cap_volt = 18.0f;
    }

    /* Adaptively scale spinning speed based on capacitor voltage (between 3.0 and 10.0 rad/s)
       根据超级电容的剩余电压，自适应调整陀螺自旋角速度（20V以下为3 rad/s，27V以上拉满至10 rad/s） */
    float dynamic_spin_speed = 3.0f + (cap_volt - 18.0f) * (10.0f - 3.0f) / (27.0f - 20.0f);
    dynamic_spin_speed = fp32_constrain(dynamic_spin_speed, 3.0f, 10.0f);

    /* Output yaw spinning wheel differential torque T_w_turn
       利用转速 PID 让车轮跟踪自旋角速度指令计算 T_w_turn */
    owner->_ctx.data.leg[wl_chassis_t::R].T_w_turn = wheel_turn_pid[wl_chassis_t::R].calculate(dynamic_spin_speed, owner->_ctx.data.g_yaw);
    owner->_ctx.data.leg[wl_chassis_t::L].T_w_turn = wheel_turn_pid[wl_chassis_t::L].calculate(dynamic_spin_speed, owner->_ctx.data.g_yaw);

    /* Calculate anti-split joint torque correction delta_mea
       计算双腿对齐防劈叉校正力矩 T_l_gain */
    owner->_ctx.data.delta_mea = owner->_ctx.data.leg[wl_chassis_t::R].alpha
                            - owner->_ctx.data.leg[wl_chassis_t::L].alpha;
    owner->_ctx.data.d_delta_mea = owner->_ctx.data.leg[wl_chassis_t::R].d_alpha
                            - owner->_ctx.data.leg[wl_chassis_t::L].d_alpha;
    owner->_ctx.data.d_delta_ref = owner->_ctx.pid.delta->calculate(
                                0.0f, owner->_ctx.data.delta_mea);

    owner->_ctx.data.T_l_gain = owner->_ctx.pid.d_delta->calculate(owner->_ctx.data.d_delta_ref,
                                                 owner->_ctx.data.d_delta_mea);

    /* Disable roll active height compensation during spin to save energy and enhance balance
       自旋过程中，将高度主动抗侧倾补偿 roll_gain 归零以保证动能一致 */
    owner->_ctx.data.roll_gain = 0.0f;

    owner->_ctx.data.leg[wl_chassis_t::R].d_x_gain = 0.0f;
    owner->_ctx.data.leg[wl_chassis_t::L].d_x_gain = 0.0f;

    owner->_ctx.data.leg[wl_chassis_t::R].x_gain = owner->_ctx.data.leg[wl_chassis_t::R].kf_x;
    owner->_ctx.data.leg[wl_chassis_t::L].x_gain = owner->_ctx.data.leg[wl_chassis_t::L].kf_x;

    /* Average linear speed of two legs / 计算底盘整体纵向平移线速度 */
    float chassis_v = (owner->_ctx.data.leg[wl_chassis_t::L].dx + owner->_ctx.data.leg[wl_chassis_t::R].dx) / 2.0f;

    /* Calculate the target torque of VMC for each leg / 增益调度 LQR 反馈 */
    for(uint8_t i = 0; i < 2; i++)
    {
        /* Interpolate LQR gains schedule based on current leg length l */
        float l = owner->_ctx.data.leg[i].l;
        for(uint8_t j = 0; j < 2; j++)
        {
            for(uint8_t k = 0; k < 6; k++)
            {
                owner->_ctx.data.leg[i].lqr_gain[j * 6 + k] = owner->_lqr_cof[(j * 6 + k) * 4]  +
                                                          owner->_lqr_cof[(j * 6 + k) * 4 + 1] * l +
                                                          owner->_lqr_cof[(j * 6 + k) * 4 + 2] * l * l +
                                                          owner->_lqr_cof[(j * 6 + k) * 4 + 3] * l * l * l ;
            }
        }

        /* Interpolate target reference height / 跟踪极向高度目标 */
        owner->_ctx.cmd->r_leg -= owner->_ctx.data.roll_gain;
        owner->_ctx.cmd->l_leg += owner->_ctx.data.roll_gain;

        if(0.005f < abs(owner->_ctx.cmd->r_leg - owner->_ctx.data.leg[wl_chassis_t::R].ref_l))
        {
            if(owner->_ctx.cmd->r_leg < owner->_ctx.data.leg[wl_chassis_t::R].ref_l)
            {
                owner->_ctx.data.leg[wl_chassis_t::R].ref_l -= 0.0001f;
            }
            else
            {
                owner->_ctx.data.leg[wl_chassis_t::R].ref_l += 0.0001f;
            }
        }
        else
        {
            owner->_ctx.data.leg[wl_chassis_t::R].ref_l = owner->_ctx.cmd->r_leg;
        }
        if(0.005f < abs(owner->_ctx.cmd->l_leg - owner->_ctx.data.leg[wl_chassis_t::L].ref_l))
        {
            if(owner->_ctx.cmd->l_leg < owner->_ctx.data.leg[wl_chassis_t::L].ref_l)
            {
                owner->_ctx.data.leg[wl_chassis_t::L].ref_l -= 0.0001f;
            }
            else
            {
                owner->_ctx.data.leg[wl_chassis_t::L].ref_l += 0.0001f;
            }
        }
        else
        {
            owner->_ctx.data.leg[wl_chassis_t::L].ref_l = owner->_ctx.cmd->l_leg;
        }

        /* Contraction closed-loop virtual force F_leg / 高度极向拉伸力 PID */
        owner->_ctx.data.leg[wl_chassis_t::R].ref_d_l =
            owner->_ctx.pid.F[wl_chassis_t::R]->calculate(owner->_ctx.data.leg[wl_chassis_t::R].ref_l, owner->_ctx.data.leg[wl_chassis_t::R].l);
        owner->_ctx.data.leg[wl_chassis_t::R].F[0] =
                owner->_ctx.pid.d_F[wl_chassis_t::R]->calculate(owner->_ctx.data.leg[wl_chassis_t::R].ref_d_l, owner->_ctx.data.leg[wl_chassis_t::R].d_l);

        owner->_ctx.data.leg[wl_chassis_t::L].ref_d_l =
            owner->_ctx.pid.F[wl_chassis_t::L]->calculate(owner->_ctx.data.leg[wl_chassis_t::L].ref_l, owner->_ctx.data.leg[wl_chassis_t::L].l);
        owner->_ctx.data.leg[wl_chassis_t::L].F[0] =
            owner->_ctx.pid.d_F[wl_chassis_t::L]->calculate(owner->_ctx.data.leg[wl_chassis_t::L].ref_d_l, owner->_ctx.data.leg[wl_chassis_t::L].d_l);

        /* Wheel balance LQR feedback / 自平衡反馈计算 */
        owner->_ctx.data.leg[i].T_w_balance = (
                                  owner->_ctx.data.leg[i].lqr_gain[2] * (0 - owner->_ctx.data.leg[i].gamma) +
                                  owner->_ctx.data.leg[i].lqr_gain[3] * (0 - owner->_ctx.data.leg[i].d_gamma) +
                                  owner->_ctx.data.leg[i].lqr_gain[4] * (0 - owner->_ctx.data.leg[i].beta) +
                                  owner->_ctx.data.leg[i].lqr_gain[5] * (0 - owner->_ctx.data.leg[i].d_beta));

        /* Wheel motion LQR feedback: Decouple position loop, only damp translation speed chassis_v
           运动项反馈：小陀螺模式切断位置位移，只保留速度反馈来抑制漂动 */
        owner->_ctx.data.leg[i].T_w_move = owner->_ctx.data.leg[i].lqr_gain[1] * (0.0f - chassis_v);

        /* Hip balance LQR feedback: Set leg tilt angle beta reference to -0.04 to tilt inwards
           关节虚拟扭矩反馈：倾斜角目标偏置调为 -0.04 以抵消离心效应 */
        owner->_ctx.data.leg[i].F[1] = -(
                                  owner->_ctx.data.leg[i].lqr_gain[7] * (0.0f - chassis_v) +
                                  owner->_ctx.data.leg[i].lqr_gain[8] * (0 - owner->_ctx.data.leg[i].gamma) +
                                  owner->_ctx.data.leg[i].lqr_gain[9] * (0 - owner->_ctx.data.leg[i].d_gamma) +
                                  owner->_ctx.data.leg[i].lqr_gain[10] * (-0.04f - owner->_ctx.data.leg[i].beta) +
                                  owner->_ctx.data.leg[i].lqr_gain[11] * (0 - owner->_ctx.data.leg[i].d_beta));
    }

    /* Clamp contract force / 极轴伸缩力限制 */
    owner->_ctx.data.leg[wl_chassis_t::R].F[0] = fp32_constrain(owner->_ctx.data.leg[wl_chassis_t::R].F[0], -200.0f, 200.0f);
    owner->_ctx.data.leg[wl_chassis_t::L].F[0] = fp32_constrain(owner->_ctx.data.leg[wl_chassis_t::L].F[0], -200.0f, 200.0f);

    /* Apply delta joint compensation torque to hip torque F[1] / 左右关节劈叉补偿 */
    owner->_ctx.data.leg[wl_chassis_t::R].F[1] += owner->_ctx.data.T_l_gain;
    owner->_ctx.data.leg[wl_chassis_t::L].F[1] -= owner->_ctx.data.T_l_gain;

    /* VMC Jacobian mapping / 运动学逆映射雅可比转换 */
    for(uint8_t i = 0; i < 2; i++)
    {
        arm_mat_vec_mult_f32(&owner->_ctx.data.leg[i].T_mat,
                            owner->_ctx.data.leg[i].F,
                            owner->_ctx.data.leg[i].T);
    }

    /* Composite wheel commands with differential spin terms / 合成差分旋转扭矩 */
    owner->_ctx.data.leg[wl_chassis_t::R].T_w = owner->_ctx.data.leg[wl_chassis_t::R].T_w_balance + owner->_ctx.data.leg[wl_chassis_t::R].T_w_move - owner->_ctx.data.leg[wl_chassis_t::R].T_w_turn;
    owner->_ctx.data.leg[wl_chassis_t::L].T_w = owner->_ctx.data.leg[wl_chassis_t::L].T_w_balance + owner->_ctx.data.leg[wl_chassis_t::L].T_w_move + owner->_ctx.data.leg[wl_chassis_t::L].T_w_turn;

    /* Run power limit update / 功率限幅算法更新 */
    float T[2];
    const float tau_uncontrolled[2] = {
        -owner->_ctx.data.leg[wl_chassis_t::R].T_w_balance,
        owner->_ctx.data.leg[wl_chassis_t::L].T_w_balance,
    };
    const float tau_motion[2] = {
        -owner->_ctx.data.leg[wl_chassis_t::R].T_w_move +
            owner->_ctx.data.leg[wl_chassis_t::R].T_w_turn,
        owner->_ctx.data.leg[wl_chassis_t::L].T_w_move +
            owner->_ctx.data.leg[wl_chassis_t::L].T_w_turn,
    };
    const float omega[2] = {
        -owner->_ctx.data.leg[wl_chassis_t::R].w,
        -owner->_ctx.data.leg[wl_chassis_t::L].w,
    };
    owner->_solve_wheel_power_limit(tau_motion, tau_uncontrolled, omega, T);

    for(uint8_t i = 0; i < 2; i++)
    {
        owner->_ctx.data.leg[i].T_w_out = T[i];
    }

    /* Transmit motor outputs / 发送最终扭矩 */
    owner->_ctx.motor.wheel[wl_chassis_t::R]->send_torque(fp32_constrain(owner->_ctx.data.leg[wl_chassis_t::R].T_w_out / owner->_ctx.data.reduction_ratio /0.3f * (3591.0f/187.0f), -20.0f, 20.0f));
    owner->_ctx.motor.wheel[wl_chassis_t::L]->send_torque(fp32_constrain(owner->_ctx.data.leg[wl_chassis_t::L].T_w_out / owner->_ctx.data.reduction_ratio /0.3f * (3591.0f/187.0f), -20.0f, 20.0f));

    owner->_ctx.motor.joint[wl_chassis_t::RF]->send_torque(
                        -owner->_ctx.data.leg[wl_chassis_t::R].T[0]);
    owner->_ctx.motor.joint[wl_chassis_t::RB]->send_torque(
                        -owner->_ctx.data.leg[wl_chassis_t::R].T[1]);
    owner->_ctx.motor.joint[wl_chassis_t::LF]->send_torque(
                        owner->_ctx.data.leg[wl_chassis_t::L].T[0]);
    owner->_ctx.motor.joint[wl_chassis_t::LB]->send_torque(
                        owner->_ctx.data.leg[wl_chassis_t::L].T[1]);
}

void wl_chassis_t::fsm_active_t::state_spin_t::exit(wl_chassis_t *owner)
{
    owner->_ctx.data.leg[wl_chassis_t::R].x_gain = owner->_ctx.data.leg[wl_chassis_t::R].kf_x;
    owner->_ctx.data.leg[wl_chassis_t::L].x_gain = owner->_ctx.data.leg[wl_chassis_t::L].kf_x;
    owner->_ctx.data.active_mode_flag.ready = 1;
}
}
