/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-28 13:11:52
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-05-30 01:54:34
 * @Description: Normal balance driving control state / 核心自平衡与运动状态控制源文件
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#include "pyro_wl_chassis.h"
#include "pyro_algo_common.h"
#include "pyro_referee.h"

static float speed_offset_ramp = 0.0f; /* Ramp generator for speed command / 速度指令渐入斜坡值 */
static float spin_decay_speed = 0.0f;  /* Dynamic spin yaw velocity decay / 退出小陀螺时的偏航速度渐变衰减 */

namespace pyro
{
extern pid_t wheel_disable_pid[2];
extern referee_drv_t *referee_drv;

/* Turn PID controllers for Yaw direction tracking
   转向偏航角速度 PID 控制器（外环跟踪 gimbal_yaw 反偏，内环作用于底盘轮机） */
pid_t turn_pid[2] = {
    pid_t(5.0f, 0.0f, 0.0f, 2.0f, 20.0f), 
    pid_t(5.0f, 0.0f, 0.0f, 2.0f, 20.0f)};

/* Soft yaw turn PID for fine tuning near center
   在偏航偏差角极小时使用的软控制 PID，防止原地高频小幅震荡震颤 */
pid_t wheel_turn_pid_soft[2] = {
    pid_t(0.001f, 0.0f, 0.0f, 0.5f, 10.0f), 
    pid_t(0.001f, 0.0f, 0.0f, 0.5f, 10.0f)};

#if ROBOT_ID == INFANTRY1_ID
/* PIDs for active leg extension while in the air (Infantry 1)
   腾空状态下，双腿极向伸长拉伸位置/速度环双闭环 PID（步兵1号） */
pid_t aerial_pid[2] = {
    pid_t(40.0f, 0.0f, 0.0f, 0.0f, 100.0f), 
    pid_t(40.0f, 0.0f, 0.0f, 0.0f, 100.0f)};
pid_t aerial_d_pid[2] = {
    pid_t(300.0f, 0.0f, 0.0f, 0.0f, 100.0f), 
    pid_t(300.0f, 0.0f, 0.0f, 0.0f, 100.0f)};
#elif ROBOT_ID == INFANTRY2_ID
/* PIDs for active leg extension while in the air (Infantry 2)
   腾空状态下，双腿极向伸长位置/速度双闭环 PID（步兵2号） */
pid_t aerial_pid[2] = {
    pid_t(1.0f, 0.0f, 0.0f, 0.0f, 100.0f), 
    pid_t(1.0f, 0.0f, 0.0f, 0.0f, 100.0f)};
pid_t aerial_d_pid[2] = {
    pid_t(200.0f, 0.0f, 0.0f, 0.0f, 100.0f), 
    pid_t(200.0f, 0.0f, 0.0f, 0.0f, 100.0f)};
#endif

float test_length = 0.20f;

/* Enter normal balance state / 进入自平衡行驶模式 */
void wl_chassis_t::fsm_active_t::state_normal_t::enter(wl_chassis_t *owner)
{
    owner->_flag.aerial_cnt = 0;
    owner->_flag.is_aerial = 0;
    owner->_flag.test = 0;
    speed_offset_ramp = 0.0f; 
    spin_decay_speed = owner->g_yaw; /* Initialize spin decay velocity with current yaw rate / 记录进入时的偏航转速 */
}

uint32_t clear_cnt;

/* Execute normal balance logic / 核心自平衡与行驶逻辑更新 */
void wl_chassis_t::fsm_active_t::state_normal_t::execute(wl_chassis_t *owner)
{
    /* 1. Ramp speed command / 速度指令渐入斜坡生成，避免起步过猛 */
    if (speed_offset_ramp < 1.0f)
    {
        speed_offset_ramp += 0.002f; 
    }
    clear_cnt++;
    if((clear_cnt > 5000)&&(owner->_cmd->vx == 0.0f))
    {
        clear_cnt = 0;
    }
    
    /* 2. Estimate Ground Support Force / 计算双腿当前对地面的垂直支持力 */
    calc_support_force(owner);
    
    /* 3. Takeoff / Landing detection
       离地与着陆判定逻辑。
       在地上时：若单侧支持力 P 低于起飞阈值（INFANTRY1为65N），启动离地计时器，计数满 AERIAL_DEBOUNCE(100ms) 切换为腾空状态。
       在空中时：若双腿杆长被撞击压缩（实际腿长比设定参考值小超过 0.1m）且惯导垂直轴向上加速度冲击 `a_upward_lpf > 3.0` 时，判定着陆。 */
    constexpr uint8_t AERIAL_DEBOUNCE = 100;
    constexpr uint8_t LANDING_DEBOUNCE = 10;
#if ROBOT_ID == INFANTRY1_ID
    constexpr float TAKEOFF_FORCE_THRESHOLD = 65.0f;
#elif ROBOT_ID == INFANTRY2_ID
    constexpr float TAKEOFF_FORCE_THRESHOLD = -100.0f;
#endif
    constexpr float LANDING_COMPRESSION_THRESHOLD = 0.1f;
    constexpr float LANDING_UPWARD_ACC_THRESHOLD = 3.0f;
    
    if(!owner->_flag.is_aerial)
    {
        /* On ground check takeoff / 地面状态，判定是否飞起 */
        if(owner->_leg_data[wl_chassis_t::R].P < TAKEOFF_FORCE_THRESHOLD
            || owner->_leg_data[wl_chassis_t::L].P < TAKEOFF_FORCE_THRESHOLD)
        {
            owner->_flag.aerial_cnt++;
            if(owner->_flag.aerial_cnt >= AERIAL_DEBOUNCE)
            {
                owner->_flag.is_aerial = 1;
                owner->_flag.aerial_cnt = 0;
            }
        }
        else
        {
            owner->_flag.aerial_cnt = 0;
        }
    }
    else
    {
        /* In air check landing / 腾空状态，判定是否着陆 */
        const bool leg_compressed =
            owner->_leg_data[wl_chassis_t::R].l
                < (owner->_leg_data[wl_chassis_t::R].ref_l - LANDING_COMPRESSION_THRESHOLD)
            || owner->_leg_data[wl_chassis_t::L].l
                < (owner->_leg_data[wl_chassis_t::L].ref_l - LANDING_COMPRESSION_THRESHOLD);
        const bool upward_impact =
            owner->a_upward_lpf > LANDING_UPWARD_ACC_THRESHOLD;

        if(leg_compressed && upward_impact)
        {
            owner->_flag.aerial_cnt++;
            if(owner->_flag.aerial_cnt >= LANDING_DEBOUNCE)
            {
                owner->_flag.is_aerial = 0;
                owner->_flag.aerial_cnt = 0;
                owner->_flag.test = 1;
                /* Lock current length as reference immediately upon landing
                   落地瞬间，立刻获取当前实际伸缩长作为闭环基准高度，防止反弹摔倒 */
                owner->get_cur_length(&owner->_leg_data[wl_chassis_t::R].ref_l, &owner->_leg_data[wl_chassis_t::L].ref_l);
            }
        }
        else
        {
            owner->_flag.aerial_cnt = 0;
        }
    }

    /* 4. Calculate Yaw rotation wheel torque (T_w_turn)
       计算差分转向扭矩。底盘通常需要对齐并跟踪云台的偏航反向，退出小陀螺时转速做平滑渐变衰减。 */
    float yaw_ref, g_yaw_ref, diff;
    yaw_ref = owner->_cmd->yaw;
    if(yaw_ref - owner->yaw > PI)
    {
        diff = -2 * PI + (yaw_ref - owner->yaw);
    }
    else if(yaw_ref - owner->yaw < -PI)
    {
        diff = 2 * PI + (yaw_ref - owner->yaw);
    }
    else
    {
        diff = yaw_ref - owner->yaw;
    }

    /* Process yaw rate decay during spin transition / 旋转速度衰减 */
    float decay_step = 0.15f;
    if (fabsf(spin_decay_speed) > decay_step) {
        if (spin_decay_speed > 0) {
            spin_decay_speed -= decay_step;
        } else {
            spin_decay_speed += decay_step;
        }
        g_yaw_ref = spin_decay_speed; 
    } else {
        spin_decay_speed = 0.0f;
        g_yaw_ref = owner->_yaw_pid->calculate(0.0f, -owner->gimbal_yaw);
    }
    owner->_yaw_ref = yaw_ref;
    owner->_g_yaw_ref = g_yaw_ref;

    /* Apply Turn PID outputs to R/L wheel motor differential components
       利用转向角速度 PID 计算转向差分力矩 T_w_turn */
    if (fabsf(owner->gimbal_yaw) < 0.05f) {
       owner->_leg_data[wl_chassis_t::R].T_w_turn  = wheel_turn_pid_soft[wl_chassis_t::R].calculate(g_yaw_ref, -owner->gimbal_g_yaw); 
       owner->_leg_data[wl_chassis_t::L].T_w_turn  = wheel_turn_pid_soft[wl_chassis_t::L].calculate(g_yaw_ref, -owner->gimbal_g_yaw);
    } else {
       owner->_leg_data[wl_chassis_t::R].T_w_turn = turn_pid[wl_chassis_t::R].calculate(g_yaw_ref, owner->g_yaw); 
       owner->_leg_data[wl_chassis_t::L].T_w_turn = turn_pid[wl_chassis_t::L].calculate(g_yaw_ref, owner->g_yaw); 
    }
    
    /* 5. Calculate Roll Compensation and Leg Bias Correction
       高度与偏角对齐。
       计算左右极坐标摆角的差值 `_delta_mea`，用以纠正双腿“劈叉”，计算纠正扭矩增益 `T_l_gain`。
       根据横滚角偏差计算 `roll_gain`，通过差分增减左右目标腿长，实现侧倾主动补偿。 */
    owner->_delta_mea = owner->_leg_data[wl_chassis_t::R].alpha 
                            - owner->_leg_data[wl_chassis_t::L].alpha;
    owner->_d_delta_mea = owner->_leg_data[wl_chassis_t::R].d_alpha 
                            - owner->_leg_data[wl_chassis_t::L].d_alpha;
    owner->_d_delta_ref = owner->_delta_pid->calculate(
                                0.0f, owner->_delta_mea);
    owner->T_l_gain = owner->_d_delta_pid->calculate(owner->_d_delta_ref,
                                                 owner->_d_delta_mea);
    owner->roll_gain = owner->_roll_pid->calculate(0.0f, owner->roll);

    /* 6. Calculate displacement trajectory (x_gain) from target vx
       计算目标平动路程期望。积分累加得到指令位置 `x_gain`。 */
    static float last_d_x_gain[2] = {0.0f, 0.0f};

    owner->_leg_data[wl_chassis_t::R].d_x_gain = owner->_cmd->vx;
    owner->_leg_data[wl_chassis_t::L].d_x_gain = owner->_cmd->vx;
    owner->_leg_data[wl_chassis_t::R].x_gain += (
        owner->_leg_data[wl_chassis_t::R].d_x_gain 
        + last_d_x_gain[wl_chassis_t::R]) / 2.0f /1000.0f;
    owner->_leg_data[wl_chassis_t::L].x_gain += (
        owner->_leg_data[wl_chassis_t::L].d_x_gain 
        + last_d_x_gain[wl_chassis_t::L]) / 2.0f /1000.0f;

    last_d_x_gain[wl_chassis_t::R] = owner->_leg_data[wl_chassis_t::R].d_x_gain;
    last_d_x_gain[wl_chassis_t::L] = owner->_leg_data[wl_chassis_t::L].d_x_gain;

    /* 7. Compute feedback forces and torques / 计算轮腿控制力 */
    for(uint8_t i = 0; i < 2; i++)
    {
        /* Interpolate LQR gains schedule based on current leg length l
           根据实际腿长，插值解算 12 个 LQR 控制律状态增益 */
        float l = owner->_leg_data[i].l;
        for(uint8_t j = 0; j < 2; j++)
        {
            for(uint8_t k = 0; k < 6; k++)
            {
                owner->_leg_data[i].lqr_gain[j * 6 + k] = owner->_lqr_cof[(j * 6 + k) * 4]  +
                                                          owner->_lqr_cof[(j * 6 + k) * 4 + 1] * l +
                                                          owner->_lqr_cof[(j * 6 + k) * 4 + 2] * l * l +
                                                          owner->_lqr_cof[(j * 6 + k) * 4 + 3] * l * l * l ;
            }
        }

        if(owner->_flag.is_aerial)
        {
            /* ── AERIAL SUB-STATE: Fly-wheel protection and leg recovery ──
               腾空状态：关闭 LQR 自平衡平动与车轮力矩，强制令车轮扭矩为 0。
               双腿极径向外拉伸至 0.32m 准备迎地碰，切断位移，极轴只做角度倾斜跟踪。 */
            owner->_leg_data[i].x_bias = 0.0f;
            owner->_leg_data[i].d_x_bias = 0.0f;
            owner->_leg_data[i].beta_bias = 0.0f- owner->_leg_data[i].beta;
            owner->_leg_data[i].d_beta_bias = 0.0f - owner->_leg_data[i].d_beta;
            owner->_leg_data[i].gamma_bias = 0.0f;
            owner->_leg_data[i].d_gamma_bias = 0.0f;
            
            owner->_leg_data[i].F[1] = -( 
                                      owner->_leg_data[i].lqr_gain[10] * owner->_leg_data[i].beta_bias + 
                                      owner->_leg_data[i].lqr_gain[11] * owner->_leg_data[i].d_beta_bias);
            owner->_leg_data[i].T_w_balance = 0.0f;
            owner->_leg_data[i].T_w_move = 0.0f;
            owner->_leg_data[i].T_w_turn = 0.0f;
            
            /* Set length reference dynamically to stretch / 腾空拉伸腿长 */
            if(0.005f < abs(0.37f - owner->_leg_data[wl_chassis_t::R].ref_l))
            {
                owner->_leg_data[wl_chassis_t::R].ref_l += 0.001f;
            }
            else
            {
                owner->_leg_data[wl_chassis_t::R].ref_l = 0.32f;
            }
            if(0.005f < abs(0.37f - owner->_leg_data[wl_chassis_t::L].ref_l))
            {
                owner->_leg_data[wl_chassis_t::L].ref_l += 0.001f;
            }
            else
            {
                owner->_leg_data[wl_chassis_t::L].ref_l = 0.32f;
            }

            /* Contraction/extension control under fly state / 腾空极径闭环计算 */
            owner->_leg_data[wl_chassis_t::R].ref_d_l =
                aerial_pid[wl_chassis_t::R].calculate(owner->_leg_data[wl_chassis_t::R].ref_l, owner->_leg_data[wl_chassis_t::R].l);
            owner->_leg_data[wl_chassis_t::R].F[0] =
                aerial_d_pid[wl_chassis_t::R].calculate(owner->_leg_data[wl_chassis_t::R].ref_d_l, owner->_leg_data[wl_chassis_t::R].d_l)-30.0f;
            
            owner->_leg_data[wl_chassis_t::L].ref_d_l =
                aerial_pid[wl_chassis_t::L].calculate(owner->_leg_data[wl_chassis_t::L].ref_l, owner->_leg_data[wl_chassis_t::L].l);
            owner->_leg_data[wl_chassis_t::L].F[0] =
                aerial_d_pid[wl_chassis_t::L].calculate(owner->_leg_data[wl_chassis_t::L].ref_d_l, owner->_leg_data[wl_chassis_t::L].d_l)-30.0f;
        }
        else 
        {
            /* ── GROUND BALANCE SUB-STATE: Gain-scheduled LQR active ──
               地面稳定行驶自平衡状态：增益调度 LQR 系统启动。
               融合横滚角补偿的目标参考高度规划。 */
            if(!owner->_flag.test)
            {
                /* Differentially adjust target lengths to compensate body roll
                   根据横滚误差 roll_gain 差分减增左右目标高度，抵抗侧倾力矩 */
                owner->_cmd->r_leg -= owner->roll_gain;
                owner->_cmd->l_leg += owner->roll_gain;

                if(0.005f < abs(owner->_cmd->r_leg - owner->_leg_data[wl_chassis_t::R].ref_l))
                {
                    if(owner->_cmd->r_leg < owner->_leg_data[wl_chassis_t::R].ref_l)
                    {
                        owner->_leg_data[wl_chassis_t::R].ref_l -= 0.0001f;
                    }
                    else
                    {
                        owner->_leg_data[wl_chassis_t::R].ref_l += 0.0001f;
                    }
                }
                else
                {
                    owner->_leg_data[wl_chassis_t::R].ref_l = owner->_cmd->r_leg;
                }
                if(0.005f < abs(owner->_cmd->l_leg - owner->_leg_data[wl_chassis_t::L].ref_l))
                {
                    if(owner->_cmd->l_leg < owner->_leg_data[wl_chassis_t::L].ref_l)
                    {
                        owner->_leg_data[wl_chassis_t::L].ref_l -= 0.0001f;
                    }
                    else
                    {
                        owner->_leg_data[wl_chassis_t::L].ref_l += 0.0001f;
                    }
                }
                else
                {
                    owner->_leg_data[wl_chassis_t::L].ref_l = owner->_cmd->l_leg;
                }
            }
            else 
            {
                /* Stance test geometry track / 跌落地面缓冲就绪 */
                if(0.005f < abs(test_length - owner->_leg_data[wl_chassis_t::R].ref_l))
                {
                    if(test_length < owner->_leg_data[wl_chassis_t::R].ref_l)
                    {
                        owner->_leg_data[wl_chassis_t::R].ref_l -= 0.001f;
                    }
                    else
                    {
                        owner->_leg_data[wl_chassis_t::R].ref_l += 0.001f;
                    }
                }
                else
                {
                    owner->_leg_data[wl_chassis_t::R].ref_l = test_length;
                }
                if(0.005f < abs(test_length - owner->_leg_data[wl_chassis_t::L].ref_l))
                {
                    if(test_length < owner->_leg_data[wl_chassis_t::L].ref_l)
                    {
                        owner->_leg_data[wl_chassis_t::L].ref_l -= 0.001f;
                    }
                    else
                    {
                        owner->_leg_data[wl_chassis_t::L].ref_l += 0.001f;
                    }
                }
                else
                {
                    owner->_leg_data[wl_chassis_t::L].ref_l = test_length;
                }
                if((owner->_leg_data[wl_chassis_t::R].ref_l == test_length) && (owner->_leg_data[wl_chassis_t::L].ref_l == test_length))
                {
                    owner->_flag.test = 0;
                }
            }
            
            /* Contract force calculation / 高度闭环力 F[0] 计算 */
            owner->_leg_data[wl_chassis_t::R].ref_d_l =
                owner->_F_pid[wl_chassis_t::R]->calculate(owner->_leg_data[wl_chassis_t::R].ref_l, owner->_leg_data[wl_chassis_t::R].l);
            owner->_leg_data[wl_chassis_t::R].F[0] =
                owner->_d_F_pid[wl_chassis_t::R]->calculate(owner->_leg_data[wl_chassis_t::R].ref_d_l, owner->_leg_data[wl_chassis_t::R].d_l);
            
            owner->_leg_data[wl_chassis_t::L].ref_d_l =
                owner->_F_pid[wl_chassis_t::L]->calculate(owner->_leg_data[wl_chassis_t::L].ref_l, owner->_leg_data[wl_chassis_t::L].l);
            owner->_leg_data[wl_chassis_t::L].F[0] =
                owner->_d_F_pid[wl_chassis_t::L]->calculate(owner->_leg_data[wl_chassis_t::L].ref_d_l, owner->_leg_data[wl_chassis_t::L].d_l);
            
            /* Calculate LQR state feedback error vector (X_bias)
               求解 LQR 的 6 个状态误差量 */
            owner->_leg_data[i].x_bias = owner->_leg_data[i].x_gain - owner->_leg_data[i].kf_x;
            owner->_leg_data[i].d_x_bias = (0.5f * speed_offset_ramp) + owner->_leg_data[i].d_x_gain - owner->_leg_data[i].kf_v;
            owner->_leg_data[i].beta_bias = 0.05f - owner->_leg_data[i].beta;
            owner->_leg_data[i].d_beta_bias = 0.0f - owner->_leg_data[i].d_beta;
            owner->_leg_data[i].gamma_bias = 0.0f - owner->_leg_data[i].gamma;
            owner->_leg_data[i].d_gamma_bias = 0.0f - owner->_leg_data[i].d_gamma;
            
            /* Calculate wheel balance torque from LQR (gamma + beta states)
               轮毂自平衡反馈：由俯仰和摆角状态误差确定 */
            owner->_leg_data[i].T_w_balance = (
                                      owner->_leg_data[i].lqr_gain[2] * owner->_leg_data[i].gamma_bias +
                                      owner->_leg_data[i].lqr_gain[3] * owner->_leg_data[i].d_gamma_bias +
                                      owner->_leg_data[i].lqr_gain[4] * owner->_leg_data[i].beta_bias +
                                      owner->_leg_data[i].lqr_gain[5] * owner->_leg_data[i].d_beta_bias
                                      );

            /* Calculate wheel motion torque from LQR (x + dx states)
               轮毂行驶反馈：由位移和速度状态误差确定 */
            owner->_leg_data[i].T_w_move = (
                                      owner->_leg_data[i].lqr_gain[0] * owner->_leg_data[i].x_bias +
                                      owner->_leg_data[i].lqr_gain[1] * owner->_leg_data[i].d_x_bias);

            /* Calculate virtual hip torque (F[1]) using all 6 LQR states
               虚拟髋关节平衡力矩计算：全状态反馈乘 LQR 增益行 1 */
            owner->_leg_data[i].F[1] = -(
                                      owner->_leg_data[i].lqr_gain[6] * owner->_leg_data[i].x_bias + 
                                      owner->_leg_data[i].lqr_gain[7] * owner->_leg_data[i].d_x_bias + 
                                      owner->_leg_data[i].lqr_gain[8] * owner->_leg_data[i].gamma_bias + 
                                      owner->_leg_data[i].lqr_gain[9] * owner->_leg_data[i].d_gamma_bias + 
                                      owner->_leg_data[i].lqr_gain[10] * owner->_leg_data[i].beta_bias + 
                                      owner->_leg_data[i].lqr_gain[11] * owner->_leg_data[i].d_beta_bias);
        }
    }
    
    /* Apply anti-split delta torque to hip torques F[1] / 叠加防劈叉力矩到关节控制中，并限幅 */
    owner->_leg_data[wl_chassis_t::R].F[1] += owner->T_l_gain;
    owner->_leg_data[wl_chassis_t::L].F[1] -= owner->T_l_gain;
    owner->_leg_data[wl_chassis_t::R].F[1] = fp32_constrain(owner->_leg_data[wl_chassis_t::R].F[1], -200.0f, 200.0f);
    owner->_leg_data[wl_chassis_t::L].F[1] = fp32_constrain(owner->_leg_data[wl_chassis_t::L].F[1], -200.0f, 200.0f);

    /* VMC Jacobian Transpose Mapping / 极坐标虚拟力转换雅可比计算映射 */
    for(uint8_t i = 0; i < 2; i++)
    {
        arm_mat_vec_mult_f32(&owner->_leg_data[i].T_mat, 
                            owner->_leg_data[i].F, 
                            owner->_leg_data[i].T);
    }
    
    /* 8. Send Command outputs / 最终驱动力矩输出 */
    if(owner->_flag.is_aerial)
    {
        /* Zero wheel torques when airborne / 腾空安全保护：直接输出 0，防止飞轮效应造成降落暴冲 */
        for(uint8_t i = 0; i < 2; i++)
        {
            owner->_wheel_drv[i]->send_torque(0.0f);
        }
    }
    else
    {
        /* Composite demanded total wheel torques
           地面状态：合成轮机总需求力矩。
           右轮: Balance + Move - Turn,  左轮: Balance + Move + Turn */
        owner->_leg_data[wl_chassis_t::R].T_w = owner->_leg_data[wl_chassis_t::R].T_w_balance + owner->_leg_data[wl_chassis_t::R].T_w_move - owner->_leg_data[wl_chassis_t::R].T_w_turn;
        owner->_leg_data[wl_chassis_t::L].T_w = owner->_leg_data[wl_chassis_t::L].T_w_balance + owner->_leg_data[wl_chassis_t::L].T_w_move + owner->_leg_data[wl_chassis_t::L].T_w_turn;
      
        /* Pack commands for power capping
           打包分量参数送入功率限制控制器。平衡力矩 tau_balance 永不限制，只限制 tau_motion。 */
        float T[2];
        const float tau_uncontrolled[2] = {
            -owner->_leg_data[wl_chassis_t::R].T_w_balance,
            owner->_leg_data[wl_chassis_t::L].T_w_balance,
        };
        const float tau_motion[2] = {
            -owner->_leg_data[wl_chassis_t::R].T_w_move +
                owner->_leg_data[wl_chassis_t::R].T_w_turn,
            owner->_leg_data[wl_chassis_t::L].T_w_move +
                owner->_leg_data[wl_chassis_t::L].T_w_turn,
        };
        const float omega[2] = {
            -owner->_leg_data[wl_chassis_t::R].w,
            -owner->_leg_data[wl_chassis_t::L].w,
        };
        /* Update PD energy control on supercapacitor voltage
           根据超电反馈电压闭环调节，并运行双轮功率预测限幅分配算法 */
        owner->_solve_wheel_power_limit(tau_motion, tau_uncontrolled, omega, T);
        
        for(uint8_t i = 0; i < 2; i++)
        {
            owner->_leg_data[i].T_w_out = T[i];
        }

        /* Transmit limited torques to wheel motors / 发送最终限幅转矩到大疆轮毂电机，注意物理减速比换算 */
        owner->_wheel_drv[wl_chassis_t::R]->send_torque(fp32_constrain(owner->_leg_data[wl_chassis_t::R].T_w_out / owner->_reduction_ratio /0.3f * (3591.0f/187.0f), -20.0f, 20.0f));
        owner->_wheel_drv[wl_chassis_t::L]->send_torque(fp32_constrain(owner->_leg_data[wl_chassis_t::L].T_w_out / owner->_reduction_ratio /0.3f * (3591.0f/187.0f), -20.0f, 20.0f));
    }
    
    /* Transmit joint torques / 发送极坐标 VMC 逆变换转矩到关节达妙电机，右侧取反 */
    owner->_motor_drv[wl_chassis_t::RF]->send_torque(
                        -owner->_leg_data[wl_chassis_t::R].T[0]);
    owner->_motor_drv[wl_chassis_t::RB]->send_torque(
                        -owner->_leg_data[wl_chassis_t::R].T[1]);
    owner->_motor_drv[wl_chassis_t::LF]->send_torque(
                        owner->_leg_data[wl_chassis_t::L].T[0]);
    owner->_motor_drv[wl_chassis_t::LB]->send_torque(
                        owner->_leg_data[wl_chassis_t::L].T[1]);
}

void wl_chassis_t::fsm_active_t::state_normal_t::exit(wl_chassis_t *owner)
{
    owner->_active_mode_flag.ready = 0;
}

/**
 * @brief Dynamic support force calculation equation.
 *        底盘垂直地面对地支持力估计物理方程。
 *        利用虚拟极轴拉伸力 F[0] 与倾摆扭矩 F[1] 在垂直重力轴向的几何投影，并叠加机体惯导运动加速度补偿项、
 *        腿部二阶缩短加速度项及倾摆向心差分力，估计轮下垂直地面真实反作用力。
 */
void wl_chassis_t::fsm_active_t::state_normal_t::calc_support_force(wl_chassis_t *owner)
{
    for(uint8_t i = 0; i < 2; i++)
    {
        owner->_leg_data[i].P 
         = owner->_leg_data[i].F[0] * arm_cos_f32(owner->_leg_data[i].beta)
         + owner->_leg_data[i].F[1] * arm_sin_f32(owner->_leg_data[i].beta) / owner->_leg_data[i].l
         + owner->a_upward_lpf 
         - owner->_leg_data[i].d2_l * arm_cos_f32(owner->_leg_data[i].beta)
         + 2.0f * owner->_leg_data[i].d_l * owner->_leg_data[i].d_beta * arm_sin_f32(owner->_leg_data[i].beta)
         + owner->_leg_data[i].l * owner->_leg_data[i].d2_beta * arm_sin_f32(owner->_leg_data[i].beta)
         + owner->_leg_data[i].l * owner->_leg_data[i].d_beta * owner->_leg_data[i].d_beta * arm_cos_f32(owner->_leg_data[i].beta);
    }
}
}
