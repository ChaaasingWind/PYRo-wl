/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-28 15:55:50
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-05-28 06:35:06
 * @Description: Stance recovery reset after jump / 跃障后收回复位及重新站立起立控制实现
 *
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved.
 */
#include "pyro_wl_chassis.h"
#include "pyro_algo_common.h"

#define LENGTH_SPEED (0.1f/800.0f)       /* Leg extension rate during recovery reset / 复位阶段高度拉伸角速度 */
#define ANGLE_SPEED (PI/1800.0f)        /* Linkage rotation speed / 复位角度插值渐变速度 */
#define TARGET_LENGTH 0.18f              /* Target leg length / 复位后正常平衡高度 (m) */
#define TARGET_ANGLE ((PI/2.0f) + 0.6f)  /* Target angle / 复位后正常起立摆轴倾角 */

namespace pyro
{
extern pid_t wheel_disable_pid[2];
static uint8_t wheel_disable_flag[2] = {0, 0};
static uint8_t state_flag[2] = {0, 0};

static float target_length[2] = {0.0f, 0.0f};
static float target_angle[2] = {0.0f, 0.0f};
static float cur_length[2] = {0.0f, 0.0f};
static float cur_angle[2] = {0.0f, 0.0f};

static uint8_t ready_flag = 0;
const float beta_bias = 0.3f;
const float gamma_bias = 0.3f;
const uint32_t ready_time = 1;

/* Enter over-step reset state / 进入越障后恢复复位状态 */
void wl_chassis_t::fsm_active_t::state_over_step_reset_t::enter(wl_chassis_t *owner)
{
    /* Record current linkage stance coordinates / 记录进入瞬间几何参数 */
    owner->get_cur_angle(&cur_angle[wl_chassis_t::R],
                        &cur_angle[wl_chassis_t::L]);
    owner->get_cur_length(&cur_length[wl_chassis_t::R],
                    &cur_length[wl_chassis_t::L]);

    /* Lock targets / 设定插值起点 */
    target_length[wl_chassis_t::R] = cur_length[wl_chassis_t::R];
    target_length[wl_chassis_t::L] = cur_length[wl_chassis_t::L];
    target_angle[wl_chassis_t::R] = cur_angle[wl_chassis_t::R];
    target_angle[wl_chassis_t::L] = cur_angle[wl_chassis_t::L];

    for(uint8_t i = 0; i < 2; i++)
    {
        wheel_disable_flag[i] = 1; /* Apply brake to wheels / 刹车锁死轮毂 */
        state_flag[i] = 0;
    }
    ready_flag = 0;
    owner->_ctx.data.active_mode_flag.ready = 0;
}

/**
 * @brief Execute over-step reset logic. Plans smooth sweep path back to normal upright standing.
 *        执行越障后复原恢复逻辑。
 *        第一阶段：若起立完成判定 ready_flag 未达成，对车轮实施阻尼锁定，对摆角与腿长进行平滑插值插补。
 *        通过嵌套 PID 控制摆长 F[0] 与摆角 F[1]，使其渐变回到 TARGET_ANGLE / TARGET_LENGTH。
 *        到位后，重置卡尔曼滤波观测器和里程计，切换至第二阶段。
 *        第二阶段：切入正常 LQR 自平衡自整定，防抖检测倾角在 beta_bias 和 gamma_bias 内满一定周期后，
 *        设置 over_step_reset 标志完成，恢复为正常就绪行驶状态。
 */
void wl_chassis_t::fsm_active_t::state_over_step_reset_t::execute(wl_chassis_t *owner)
{
    static uint32_t ready_cnt = 0;

    /* Check if geometry targets achieved / 判定几何角度和高度是否到位 */
    if((0.05f > abs(owner->_ctx.data.leg[wl_chassis_t::R].l - TARGET_LENGTH)) &&
       (0.05f > abs(owner->_ctx.data.leg[wl_chassis_t::L].l - TARGET_LENGTH)) &&
       (0.1f > abs(owner->_ctx.data.leg[wl_chassis_t::R].alpha - TARGET_ANGLE)) &&
       (0.1f > abs(owner->_ctx.data.leg[wl_chassis_t::L].alpha - TARGET_ANGLE)))
    {
        ready_cnt++;
        if(ready_cnt > 200)
        {
            /* Stance aligned. Reset odometry and filters / 位移和姿态已恢复到位，重置状态量与卡尔曼滤波器 */
            owner->_ctx.data.leg[wl_chassis_t::R].x = 0.0f;
            owner->_ctx.data.leg[wl_chassis_t::L].x = 0.0f;
            owner->_ctx.data.leg[wl_chassis_t::R].x_gain = 0.0f;
            owner->_ctx.data.leg[wl_chassis_t::L].x_gain = 0.0f;
            owner->_ctx.data.leg[wl_chassis_t::R].kf_x = 0.0f;
            owner->_ctx.data.leg[wl_chassis_t::R].kf_v = 0.0f;
            owner->_ctx.data.leg[wl_chassis_t::L].kf_v = 0.0f;
            owner->_ctx.data.leg[wl_chassis_t::L].kf_x = 0.0f;
            owner->_wheel_kf[wl_chassis_t::R].reset();
            owner->_wheel_kf[wl_chassis_t::L].reset();
            ready_cnt = 0;
            ready_flag = 1;
        }
    }
    else {
        ready_cnt = 0;
    }

    if(0 == ready_flag)
    {
        /* Phase 1: Planning recovery trajectory (PID control, no LQR) / 第一阶段：关节闭环插值规划恢复 */
        for(uint8_t i = 0; i < 2; i++)
        {
            owner->_ctx.data.leg[i].x_bias = 0.0f;
            owner->_ctx.data.leg[i].d_x_bias = 0.0f;
            owner->_ctx.data.leg[i].beta_bias = 0.0f;
            owner->_ctx.data.leg[i].d_beta_bias = 0.0f;
            owner->_ctx.data.leg[i].gamma_bias = 0.0f - owner->_ctx.data.leg[i].gamma;
            owner->_ctx.data.leg[i].d_gamma_bias = 0.0f - owner->_ctx.data.leg[i].d_gamma;
        }

        calc_target_value(owner);

        /* Contraction closed-loop force F_leg (F[0]) / 极轴高度力计算 */
        /* Right leg */
        owner->_ctx.data.leg[wl_chassis_t::R].ref_d_l=
                owner->_ctx.pid.F[wl_chassis_t::R]->
            calculate(target_length[wl_chassis_t::R],
            owner->_ctx.data.leg[wl_chassis_t::R].l);
        owner->_ctx.data.leg[wl_chassis_t::R].F[0]=
                owner->_ctx.pid.d_F[wl_chassis_t::R]->
                 calculate(owner->_ctx.data.leg[wl_chassis_t::R].ref_d_l,
            owner->_ctx.data.leg[wl_chassis_t::R].d_l);
        /* Left leg */
        owner->_ctx.data.leg[wl_chassis_t::L].ref_d_l=
            owner->_ctx.pid.F[wl_chassis_t::L]->
            calculate(target_length[wl_chassis_t::L],
            owner->_ctx.data.leg[wl_chassis_t::L].l);
        owner->_ctx.data.leg[wl_chassis_t::L].F[0]=
            owner->_ctx.pid.d_F[wl_chassis_t::L]->
            calculate(owner->_ctx.data.leg[wl_chassis_t::L].ref_d_l,
            owner->_ctx.data.leg[wl_chassis_t::L].d_l);

        /* Hip swing closed-loop virtual torque (F[1]) / 摆轴倾斜力矩计算 */
        /* Right leg */
        float diff;
        if(target_angle[wl_chassis_t::R] - owner->_ctx.data.leg[wl_chassis_t::R].alpha > PI)
        {
            diff = -2 * PI + (target_angle[wl_chassis_t::R] - owner->_ctx.data.leg[wl_chassis_t::R].alpha);
        }
        else if(target_angle[wl_chassis_t::R] - owner->_ctx.data.leg[wl_chassis_t::R].alpha < -PI)
        {
            diff = 2 * PI + (target_angle[wl_chassis_t::R] - owner->_ctx.data.leg[wl_chassis_t::R].alpha);
        }
        else
        {
            diff = target_angle[wl_chassis_t::R] - owner->_ctx.data.leg[wl_chassis_t::R].alpha;
        }
        owner->_ctx.data.leg[wl_chassis_t::R].ref_d_alpha=
            owner->_ctx.pid.T[wl_chassis_t::R]->
            calculate(owner->_ctx.data.leg[wl_chassis_t::R].alpha + diff,
            owner->_ctx.data.leg[wl_chassis_t::R].alpha);
        owner->_ctx.data.leg[wl_chassis_t::R].F[1]=
            owner->_ctx.pid.d_T[wl_chassis_t::R]->
            calculate(owner->_ctx.data.leg[wl_chassis_t::R].ref_d_alpha,
            owner->_ctx.data.leg[wl_chassis_t::R].d_alpha);

        /* Left leg */
        if(target_angle[wl_chassis_t::L] - owner->_ctx.data.leg[wl_chassis_t::L].alpha > PI)
        {
            diff = -2 * PI + (target_angle[wl_chassis_t::L] - owner->_ctx.data.leg[wl_chassis_t::L].alpha);
        }
        else if(target_angle[wl_chassis_t::L] - owner->_ctx.data.leg[wl_chassis_t::L].alpha < -PI)
        {
            diff = 2 * PI + (target_angle[wl_chassis_t::L] - owner->_ctx.data.leg[wl_chassis_t::L].alpha);
        }
        else
        {
            diff = target_angle[wl_chassis_t::L] - owner->_ctx.data.leg[wl_chassis_t::L].alpha;
        }
        owner->_ctx.data.leg[wl_chassis_t::L].ref_d_alpha=
            owner->_ctx.pid.T[wl_chassis_t::L]->
            calculate(owner->_ctx.data.leg[wl_chassis_t::L].alpha+diff,
            owner->_ctx.data.leg[wl_chassis_t::L].alpha);
        owner->_ctx.data.leg[wl_chassis_t::L].F[1]=
            owner->_ctx.pid.d_T[wl_chassis_t::L]->
            calculate(owner->_ctx.data.leg[wl_chassis_t::L].ref_d_alpha,
            owner->_ctx.data.leg[wl_chassis_t::L].d_alpha);

        /* Hold wheels still during recovery sweep / 阻尼制动车轮防止倾倒 */
        for(uint8_t i = 0; i < 2; i++)
        {
            if(1 == wheel_disable_flag[i])
            {
                float t = wheel_disable_pid[i].calculate(0.0f,
                     owner->_ctx.motor.wheel[i]->get_current_rotate());
                owner->_ctx.motor.wheel[i]->send_torque(t);
            }
            else
            {
                owner->_ctx.motor.wheel[i]->send_torque(0.0f);
            }
            if(0.1f > abs(owner->_ctx.motor.wheel[i]->get_current_rotate()))
            {
                wheel_disable_flag[i] = 0;
                owner->_ctx.motor.wheel[i]->disable();
            }
        }
    }
    else
    {
        /* Phase 2: Accessing LQR self-balancing loops / 第二阶段：恢复完成后进入 LQR 站立平衡环 */
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

            owner->_ctx.data.leg[i].ref_l = TARGET_LENGTH;
            /* Height contraction force F_leg / 高度闭环计算 */
            /* Right leg */
            owner->_ctx.data.leg[wl_chassis_t::R].ref_d_l=
                owner->_ctx.pid.F[wl_chassis_t::R]->
                calculate(owner->_ctx.data.leg[wl_chassis_t::R].ref_l,
                owner->_ctx.data.leg[wl_chassis_t::R].l);
            owner->_ctx.data.leg[wl_chassis_t::R].F[0]=
                    owner->_ctx.pid.d_F[wl_chassis_t::R]->
                     calculate(owner->_ctx.data.leg[wl_chassis_t::R].ref_d_l,
                owner->_ctx.data.leg[wl_chassis_t::R].d_l);
            /* Left leg */
            owner->_ctx.data.leg[wl_chassis_t::L].ref_d_l=
                owner->_ctx.pid.F[wl_chassis_t::L]->
                calculate(owner->_ctx.data.leg[wl_chassis_t::L].ref_l,
                owner->_ctx.data.leg[wl_chassis_t::L].l);
            owner->_ctx.data.leg[wl_chassis_t::L].F[0]=
                owner->_ctx.pid.d_F[wl_chassis_t::L]->
                calculate(owner->_ctx.data.leg[wl_chassis_t::L].ref_d_l,
                owner->_ctx.data.leg[wl_chassis_t::L].d_l);

            /* LQR errors (no position feedback during reset transition) / 偏置解算，舍弃位移环 */
            owner->_ctx.data.leg[i].x_bias = 0.0f;
            owner->_ctx.data.leg[i].d_x_bias = 0.0f;
            owner->_ctx.data.leg[i].beta_bias = 0.0f- owner->_ctx.data.leg[i].beta;
            owner->_ctx.data.leg[i].d_beta_bias = 0.0f - owner->_ctx.data.leg[i].d_beta;
            owner->_ctx.data.leg[i].gamma_bias = 0.0f - owner->_ctx.data.leg[i].gamma;
            owner->_ctx.data.leg[i].d_gamma_bias = 0.0f - owner->_ctx.data.leg[i].d_gamma;

            /* Wheel LQR balance torque / 自平衡车轮驱动 */
            owner->_ctx.data.leg[i].T_w_balance = (
                                      owner->_ctx.data.leg[i].lqr_gain[2] * owner->_ctx.data.leg[i].gamma_bias +
                                      owner->_ctx.data.leg[i].lqr_gain[3] * owner->_ctx.data.leg[i].d_gamma_bias +
                                      owner->_ctx.data.leg[i].lqr_gain[4] * owner->_ctx.data.leg[i].beta_bias +
                                      owner->_ctx.data.leg[i].lqr_gain[5] * owner->_ctx.data.leg[i].d_beta_bias);

            /* Joint LQR balance torque / 自平衡关节扭矩 */
            owner->_ctx.data.leg[i].F[1] = -(
                                      owner->_ctx.data.leg[i].lqr_gain[8] * owner->_ctx.data.leg[i].gamma_bias +
                                      owner->_ctx.data.leg[i].lqr_gain[9] * owner->_ctx.data.leg[i].d_gamma_bias +
                                      owner->_ctx.data.leg[i].lqr_gain[10] * owner->_ctx.data.leg[i].beta_bias +
                                      owner->_ctx.data.leg[i].lqr_gain[11] * owner->_ctx.data.leg[i].d_beta_bias);
        }

        /* Check upright balance stabilization criteria / 防抖对齐判断 */
        static uint32_t time_count = 0;
        if((beta_bias > abs(owner->_ctx.data.leg[wl_chassis_t::R].beta)) &&
           (gamma_bias > abs(owner->_ctx.data.leg[wl_chassis_t::R].gamma)) &&
           (beta_bias > abs(owner->_ctx.data.leg[wl_chassis_t::L].beta)) &&
           (gamma_bias > abs(owner->_ctx.data.leg[wl_chassis_t::L].gamma)))
        {
            time_count += 1;
        }
        else
        {
            time_count = 0;
        }

        if(time_count > ready_time)
        {
            owner->_ctx.data.active_mode_flag.over_step_reset = 1; /* Mark reset completed / 复位流程执行完成 */
        }

        /* Clamp VMC forces / 限幅关节拉伸力 */
        owner->_ctx.data.leg[wl_chassis_t::R].F[0] = fp32_constrain(owner->_ctx.data.leg[wl_chassis_t::R].F[0], -200.0f, 200.0f);
        owner->_ctx.data.leg[wl_chassis_t::L].F[0] = fp32_constrain(owner->_ctx.data.leg[wl_chassis_t::L].F[0], -200.0f, 200.0f);

        owner->_ctx.data.leg[wl_chassis_t::R].T_w = owner->_ctx.data.leg[wl_chassis_t::R].T_w_balance;
        owner->_ctx.data.leg[wl_chassis_t::L].T_w = owner->_ctx.data.leg[wl_chassis_t::L].T_w_balance;

        /* Send torques to wheels / 下发车轮平衡扭矩 */
        owner->_ctx.motor.wheel[wl_chassis_t::R]->send_torque(fp32_constrain(-owner->_ctx.data.leg[wl_chassis_t::R].T_w / owner->_ctx.data.reduction_ratio /0.3f * (3591.0f/187.0f), -20.0f, 20.0f));
        owner->_ctx.motor.wheel[wl_chassis_t::L]->send_torque(fp32_constrain(owner->_ctx.data.leg[wl_chassis_t::L].T_w / owner->_ctx.data.reduction_ratio /0.3f * (3591.0f/187.0f), -20.0f, 20.0f));
    }

    /* VMC JacobianTranspose Mapping / 极轴虚拟力转换雅可比矩阵逆映射 */
    for(uint8_t i = 0; i < 2; i++)
    {
        arm_mat_vec_mult_f32(&owner->_ctx.data.leg[i].T_mat,
                            owner->_ctx.data.leg[i].F,
                            owner->_ctx.data.leg[i].T);
    }

    /* Send torques to joint motors / 下发关节电机指令，右侧取反 */
    owner->_ctx.motor.joint[wl_chassis_t::RF]->send_torque(
                        -owner->_ctx.data.leg[wl_chassis_t::R].T[0]);
    owner->_ctx.motor.joint[wl_chassis_t::RB]->send_torque(
                        -owner->_ctx.data.leg[wl_chassis_t::R].T[1]);
    owner->_ctx.motor.joint[wl_chassis_t::LF]->send_torque(
                        owner->_ctx.data.leg[wl_chassis_t::L].T[0]);
    owner->_ctx.motor.joint[wl_chassis_t::LB]->send_torque(
                        owner->_ctx.data.leg[wl_chassis_t::L].T[1]);
}

void wl_chassis_t::fsm_active_t::state_over_step_reset_t::exit(wl_chassis_t *owner)
{
}

/**
 * @brief Interpolation planning for over-step recovery reset.
 *        越障复位轨迹渐变规划。
 *        将双腿的关节极轴摆角逐渐旋转靠拢到 TARGET_ANGLE，并将腿高极向距离拉伸/收缩至 TARGET_LENGTH。
 */
void wl_chassis_t::fsm_active_t::state_over_step_reset_t::calc_target_value(wl_chassis_t *owner)
{
    for(uint8_t i = 0; i < 2; i++)
    {
        if(0.05f < fabsf(target_angle[i] - TARGET_ANGLE))
        {
            target_angle[i] -= ANGLE_SPEED;
            target_angle[i] = wrap2pi_f32(target_angle[i]);
        }
        else
        {
            target_angle[i] = TARGET_ANGLE;

            if(target_length[i] < TARGET_LENGTH)
            {
                target_length[i] += LENGTH_SPEED;
            }
            else if(target_length[i] > TARGET_LENGTH)
            {
                target_length[i] -= LENGTH_SPEED;
            }

            if(0.01f > abs(target_length[i] - TARGET_LENGTH))
            {
                target_length[i] = TARGET_LENGTH;
            }
        }
    }
}
}