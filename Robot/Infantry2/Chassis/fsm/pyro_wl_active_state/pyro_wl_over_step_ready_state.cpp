/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-03-01 15:55:50
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-05-28 06:30:05
 * @Description: Balanced stance LQR control after climb step / 跃障后的障上特殊自平衡姿态控制实现
 *
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved.
 */
#include "pyro_wl_chassis.h"
#include "pyro_algo_common.h"

namespace pyro
{
void wl_chassis_t::fsm_active_t::state_over_step_ready_t::enter(wl_chassis_t *owner)
{
    owner->_ctx.data.active_mode_flag.ready = 1;
}

/**
 * @brief Execute over-step balanced stance logic. Calculates torque output using over-step LQR gains.
 *        执行障上平衡控制。
 *        与 Normal 行驶平衡类似，但由于障上动力学特性不同，该状态插值使用另一套专有的状态增益表 `_lqr_cof_over_step`。
 *        双腿高度伸缩极向力 F[0] 由指令 `r_leg`/`l_leg` 作为目标极径闭环计算，
 *        轮毂自平衡力矩和关节虚拟髋力矩 F[1] 由障上增益调度 LQR 直接解算，并叠加航向转向差分 term 和横滚劈叉补偿。
 */
void wl_chassis_t::fsm_active_t::state_over_step_ready_t::execute(wl_chassis_t *owner)
{
    /* Calculate Yaw turn differential wheel torque / 偏航角云台对齐差分项 */
    float yaw_ref, g_yaw_ref, diff;
    yaw_ref = owner->_ctx.cmd->yaw;
    if(yaw_ref - owner->_ctx.data.yaw > PI)
    {
        diff = -2 * PI + (yaw_ref - owner->_ctx.data.yaw);
    }
    else if(yaw_ref - owner->_ctx.data.yaw < -PI)
    {
        diff = 2 * PI + (yaw_ref - owner->_ctx.data.yaw);
    }
    else
    {
        diff = yaw_ref - owner->_ctx.data.yaw;
    }
    g_yaw_ref = owner->_ctx.pid.yaw->calculate(owner->_ctx.data.yaw + diff, owner->_ctx.data.yaw);
    owner->_ctx.data.yaw_ref = yaw_ref;
    owner->_ctx.data.g_yaw_ref = g_yaw_ref;
    const float T_w_gain = owner->_ctx.pid.g_yaw->calculate(g_yaw_ref, owner->_ctx.data.g_yaw);

    /* Calculate anti-split joint correction torque delta_mea / 左右双腿防劈叉对齐力矩 */
    owner->_ctx.data.delta_mea = owner->_ctx.data.leg[wl_chassis_t::R].alpha
                            - owner->_ctx.data.leg[wl_chassis_t::L].alpha;
    owner->_ctx.data.d_delta_mea = owner->_ctx.data.leg[wl_chassis_t::R].d_alpha
                            - owner->_ctx.data.leg[wl_chassis_t::L].d_alpha;
    owner->_ctx.data.d_delta_ref = owner->_ctx.pid.delta->calculate(
                                0.0f, owner->_ctx.data.delta_mea);
    owner->_ctx.data.T_l_gain = owner->_ctx.pid.d_delta->calculate(owner->_ctx.data.d_delta_ref,
                                                 owner->_ctx.data.d_delta_mea);

    /* Calculate target path displacement x_gain from target vx / 积分目标指令位置 */
    static float last_d_x_gain[2] = {0.0f, 0.0f};

    owner->_ctx.data.leg[wl_chassis_t::R].d_x_gain = owner->_ctx.cmd->vx;
    owner->_ctx.data.leg[wl_chassis_t::L].d_x_gain = owner->_ctx.cmd->vx;
    owner->_ctx.data.leg[wl_chassis_t::R].x_gain += (
        owner->_ctx.data.leg[wl_chassis_t::R].d_x_gain
        + last_d_x_gain[wl_chassis_t::R]) / 2.0f /1000.0f;
    owner->_ctx.data.leg[wl_chassis_t::L].x_gain += (
        owner->_ctx.data.leg[wl_chassis_t::L].d_x_gain
        + last_d_x_gain[wl_chassis_t::L]) / 2.0f /1000.0f;

    for(uint8_t i = 0; i < 2; i++)
    {
        if(0.01f < abs(owner->_ctx.data.leg[i].d_x_gain) && 0.01f > abs(last_d_x_gain[i]))
        {
            owner->_ctx.data.leg[i].x_gain = owner->_ctx.data.leg[i].kf_x;
        }
    }

    last_d_x_gain[wl_chassis_t::R] = owner->_ctx.data.leg[wl_chassis_t::R].d_x_gain;
    last_d_x_gain[wl_chassis_t::L] = owner->_ctx.data.leg[wl_chassis_t::L].d_x_gain;

    /* Calculate feedback forces and torques using over-step LQR coefficients
       使用跃障平衡专属多项式参数，通过腿长 l 计算障上 LQR 控制增益 */
    for(uint8_t i = 0; i < 2; i++)
    {
        float l = owner->_ctx.data.leg[i].l;
        for(uint8_t j = 0; j < 2; j++)
        {
            for(uint8_t k = 0; k < 6; k++)
            {
                owner->_ctx.data.leg[i].lqr_gain[j * 6 + k] = owner->_lqr_cof_over_step[(j * 6 + k) * 4]  +
                                                          owner->_lqr_cof_over_step[(j * 6 + k) * 4 + 1] * l +
                                                          owner->_lqr_cof_over_step[(j * 6 + k) * 4 + 2] * l * l +
                                                          owner->_lqr_cof_over_step[(j * 6 + k) * 4 + 3] * l * l * l ;
            }
        }

        /* Calculate LQR state feedback error vector (X_bias)
           求解底盘障上 LQR 控制的状态误差偏差 */
        owner->_ctx.data.leg[i].x_bias = owner->_ctx.data.leg[i].x_gain - owner->_ctx.data.leg[i].kf_x;
        owner->_ctx.data.leg[i].d_x_bias = 0.5f + owner->_ctx.data.leg[i].d_x_gain - owner->_ctx.data.leg[i].kf_v;
        owner->_ctx.data.leg[i].beta_bias = 0.0f- owner->_ctx.data.leg[i].beta;
        owner->_ctx.data.leg[i].d_beta_bias = 0.0f - owner->_ctx.data.leg[i].d_beta;
        owner->_ctx.data.leg[i].gamma_bias = 0.0f - owner->_ctx.data.leg[i].gamma;
        owner->_ctx.data.leg[i].d_gamma_bias = 0.0f - owner->_ctx.data.leg[i].d_gamma;

        /* Wheel balance control torque from over-step LQR / 障上轮毂自平衡力矩 */
        owner->_ctx.data.leg[i].T_w = (
                                  owner->_ctx.data.leg[i].lqr_gain[0] * (owner->_ctx.data.leg[i].x_bias) +
                                  owner->_ctx.data.leg[i].lqr_gain[1] * (owner->_ctx.data.leg[i].d_x_bias) +
                                  owner->_ctx.data.leg[i].lqr_gain[2] * (owner->_ctx.data.leg[i].gamma_bias) +
                                  owner->_ctx.data.leg[i].lqr_gain[3] * (owner->_ctx.data.leg[i].d_gamma_bias) +
                                  owner->_ctx.data.leg[i].lqr_gain[4] * (owner->_ctx.data.leg[i].beta_bias) +
                                  owner->_ctx.data.leg[i].lqr_gain[5] * (owner->_ctx.data.leg[i].d_beta_bias))
                                  / owner->_ctx.data.reduction_ratio /0.3f * (3591.0f/187.0f);

        /* Virtual hip control torque F[1] from over-step LQR / 障上关节自平衡虚拟扭矩 */
        owner->_ctx.data.leg[i].F[1] = -(
                                  owner->_ctx.data.leg[i].lqr_gain[6] * (owner->_ctx.data.leg[i].x_bias) +
                                  owner->_ctx.data.leg[i].lqr_gain[7] * (owner->_ctx.data.leg[i].d_x_bias) +
                                  owner->_ctx.data.leg[i].lqr_gain[8] * (owner->_ctx.data.leg[i].gamma_bias) +
                                  owner->_ctx.data.leg[i].lqr_gain[9] * (owner->_ctx.data.leg[i].d_gamma_bias) +
                                  owner->_ctx.data.leg[i].lqr_gain[10] * (owner->_ctx.data.leg[i].beta_bias) +
                                  owner->_ctx.data.leg[i].lqr_gain[11] * (owner->_ctx.data.leg[i].d_beta_bias));
    }

    /* Target reference leg length interpolation / 跟踪高度指令并插值防变 */
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

    /* Expansion closed-loop force F_leg (F[0]) / 极轴伸缩高度闭环计算 */
    /* Right leg */
    owner->_ctx.data.leg[wl_chassis_t::R].ref_d_l =
        owner->_ctx.pid.F[wl_chassis_t::R]->calculate(owner->_ctx.data.leg[wl_chassis_t::R].ref_l, owner->_ctx.data.leg[wl_chassis_t::R].l);
    owner->_ctx.data.leg[wl_chassis_t::R].F[0] =
            owner->_ctx.pid.d_F[wl_chassis_t::R]->calculate(owner->_ctx.data.leg[wl_chassis_t::R].ref_d_l, owner->_ctx.data.leg[wl_chassis_t::R].d_l);
    /* Left leg */
    owner->_ctx.data.leg[wl_chassis_t::L].ref_d_l =
        owner->_ctx.pid.F[wl_chassis_t::L]->calculate(owner->_ctx.data.leg[wl_chassis_t::L].ref_l, owner->_ctx.data.leg[wl_chassis_t::L].l);
    owner->_ctx.data.leg[wl_chassis_t::L].F[0] =
        owner->_ctx.pid.d_F[wl_chassis_t::L]->calculate(owner->_ctx.data.leg[wl_chassis_t::L].ref_d_l, owner->_ctx.data.leg[wl_chassis_t::L].d_l);

    /* Clamp VMC contraction force and apply delta split compensation
       极径力限幅与左右劈叉纠正力矩叠加 */
    owner->_ctx.data.leg[wl_chassis_t::R].F[0] = fp32_constrain(owner->_ctx.data.leg[wl_chassis_t::R].F[0], -200.0f, 200.0f);
    owner->_ctx.data.leg[wl_chassis_t::L].F[0] = fp32_constrain(owner->_ctx.data.leg[wl_chassis_t::L].F[0], -200.0f, 200.0f);
    owner->_ctx.data.leg[wl_chassis_t::R].F[1] += owner->_ctx.data.T_l_gain;
    owner->_ctx.data.leg[wl_chassis_t::L].F[1] -= owner->_ctx.data.T_l_gain;

    /* VMC Jacobian transpose mapping / VMC 力矩矩阵逆投影 */
    for(uint8_t i = 0; i < 2; i++)
    {
        arm_mat_vec_mult_f32(&owner->_ctx.data.leg[i].T_mat,
                            owner->_ctx.data.leg[i].F,
                            owner->_ctx.data.leg[i].T);
    }

    /* Output commands to wheel and joint motors / 输出控制扭矩给车轮及关节，右侧取反 */
    owner->_ctx.motor.wheel[wl_chassis_t::R]->send_torque(fp32_constrain(-owner->_ctx.data.leg[wl_chassis_t::R].T_w + T_w_gain, -20.0f, 20.0f));
    owner->_ctx.motor.wheel[wl_chassis_t::L]->send_torque(fp32_constrain(owner->_ctx.data.leg[wl_chassis_t::L].T_w + T_w_gain, -20.0f, 20.0f));

    owner->_ctx.motor.joint[wl_chassis_t::RF]->send_torque(
                        -owner->_ctx.data.leg[wl_chassis_t::R].T[0]);
    owner->_ctx.motor.joint[wl_chassis_t::RB]->send_torque(
                        -owner->_ctx.data.leg[wl_chassis_t::R].T[1]);
    owner->_ctx.motor.joint[wl_chassis_t::LF]->send_torque(
                        owner->_ctx.data.leg[wl_chassis_t::L].T[0]);
    owner->_ctx.motor.joint[wl_chassis_t::LB]->send_torque(
                        owner->_ctx.data.leg[wl_chassis_t::L].T[1]);
}

void wl_chassis_t::fsm_active_t::state_over_step_ready_t::exit(wl_chassis_t *owner)
{
}
}
