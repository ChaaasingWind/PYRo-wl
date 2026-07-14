/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-28 15:55:50
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-05-25 14:08:56
 * @Description: Stance transition and stand-up implementation / 底盘起立准备与自平衡切入过程实现
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#include "pyro_wl_chassis.h"
#include "pyro_algo_common.h"

#define LENGTH_SPEED (0.1f/300.0f)        /* Speed of leg extension during stand-up / 起立过程中腿部的拉伸速度 (m/tick) */
#define ANGLE_SPEED (PI/800.0f)          /* Angular speed of linkage rotation during stand-up / 起立过程中关节摆动的角速度 (rad/tick) */
#define TARGET_LENGTH 0.18f               /* Target leg length for stand-up posture / 起立准备就绪的目标高度 (m) */
#define TARGET_ANGLE ((PI/2.0f) + 0.6f)   /* Target polar angle for upright stand-up / 起立就绪的目标极轴角度 */

namespace pyro
{
extern pid_t wheel_disable_pid[2];
static uint8_t wheel_disable_flag[2] = {0, 0};
static float tmp_angle = 0.0f;
static uint8_t state_flag[2] = {0, 0}; /* Interpolation state flags for legs / 双腿起身插值阶段标志 */

static float target_length[2] = {0.0f, 0.0f};
static float target_angle[2] = {0.0f, 0.0f};
static float cur_length[2] = {0.0f, 0.0f};
static float cur_angle[2] = {0.0f, 0.0f};

static uint8_t ready_flag = 0;   /* Flag indicating the legs have reached geometric targets / 几何位置到达标志 */
const float beta_bias = 0.3f;    /* Tilt angle threshold for balance activation / 自平衡控制切入的倾角偏差阈值 */
const float gamma_bias = 0.3f;   /* Pitch angle threshold for balance activation / 自平衡控制切入的俯仰角偏差阈值 */
const uint32_t ready_time = 1;   /* Stabilization time debounce (ticks) / 自平衡切入后的防抖稳定时间 (ticks) */

/* Enter Ready state / 进入 Ready（起立）状态 */
void wl_chassis_t::fsm_active_t::state_ready_t::enter(wl_chassis_t *owner)
{
    /* Record the current angle and length of the legs / 记录进入瞬间双腿真实的实际角度和长度 */
    owner->get_cur_angle(&cur_angle[wl_chassis_t::R], 
                        &cur_angle[wl_chassis_t::L]);
    owner->get_cur_length(&cur_length[wl_chassis_t::R], 
                    &cur_length[wl_chassis_t::L]);
                    
    /* Set the target angle and length of the legs as the current values
       将插值目标值初始化为当前的实际位置，防止发生突变 */
    target_length[wl_chassis_t::R] = cur_length[wl_chassis_t::R];
    target_length[wl_chassis_t::L] = cur_length[wl_chassis_t::L];
    target_angle[wl_chassis_t::R] = cur_angle[wl_chassis_t::R];
    target_angle[wl_chassis_t::L] = cur_angle[wl_chassis_t::L];
    
    for(uint8_t i = 0; i < 2; i++)
    {
        wheel_disable_flag[i] = 1; /* Apply brake to wheels during transition / 起立摆腿时制动车轮 */
        state_flag[i] = 0;         /* Reset interpolation state / 重置起立插值阶段 */
    }
    ready_flag = 0;
    owner->_active_mode_flag.ready = 0;
}

/* Execute Ready state logic / 执行起立与平衡接入控制 */
void wl_chassis_t::fsm_active_t::state_ready_t::execute(wl_chassis_t *owner)
{
    /* Check if the legs have geometrically reached Target Length and Target Angle
       检测双腿是否均已运动到目标起身几何高度（0.18m）与摆角 */
    if((0.01f > abs(owner->_leg_data[wl_chassis_t::R].l - TARGET_LENGTH)) &&
       (0.01f > abs(owner->_leg_data[wl_chassis_t::L].l - TARGET_LENGTH)) &&
       (0.05f > abs(owner->_leg_data[wl_chassis_t::R].alpha - TARGET_ANGLE)) &&
       (0.05f > abs(owner->_leg_data[wl_chassis_t::L].alpha - TARGET_ANGLE)))
    {
        ready_flag = 1;
        /* Reset odometry and filters upon standing upright
           起立成功，重置所有的绝对位移里程计、卡尔曼滤波器 */
        owner->_leg_data[wl_chassis_t::R].x = 0.0f;
        owner->_leg_data[wl_chassis_t::L].x = 0.0f;
        owner->_leg_data[wl_chassis_t::R].x_gain = 0.0f;
        owner->_leg_data[wl_chassis_t::L].x_gain = 0.0f;
        owner->_leg_data[wl_chassis_t::R].kf_x = 0.0f;
        owner->_leg_data[wl_chassis_t::R].kf_v = 0.0f;
        owner->_leg_data[wl_chassis_t::L].kf_v = 0.0f;
        owner->_leg_data[wl_chassis_t::L].kf_x = 0.0f;
        owner->_wheel_kf[wl_chassis_t::R].reset();
        owner->_wheel_kf[wl_chassis_t::L].reset();
    }
   
    if(0 == ready_flag)
    {
        /* Phase 1: Planning stand-up trajectory (PID joint control, no LQR)
           第一阶段：规划起立几何轨迹。采用极坐标 PID 闭环跟踪规划目标，不引入 LQR 平衡律。 */
        for(uint8_t i = 0; i < 2; i++)
        {
            owner->_leg_data[i].x_bias = 0.0f;
            owner->_leg_data[i].d_x_bias = 0.0f;
            owner->_leg_data[i].beta_bias = 0.0f;
            owner->_leg_data[i].d_beta_bias = 0.0f;
            owner->_leg_data[i].gamma_bias = 0.0f - owner->_leg_data[i].gamma;
            owner->_leg_data[i].d_gamma_bias = 0.0f - owner->_leg_data[i].d_gamma;
        }
        
        /* Smoothly interpolate target values / 周期性更新插值规划点 */
        calc_target_value(owner);

        /* Calculate expansion force F_leg (F[0]) using nested PID
           计算双腿目标摆长力 F[0]（外环位置 PID + 内环速度 PID） */
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
            
        /* Calculate hip virtual torque T_hip (F[1]) using nested PID
           计算左右腿目标髋扭矩 F[1]，处理短路径角度环绕偏差 */
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
            
        /* Apply braking torque to hold wheels still / 锁死轮子，防止摆腿起身时前后滑移 */
        for(uint8_t i = 0; i < 2; i++)
        {
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
            if(0.1f > abs(owner->_wheel_drv[i]->get_current_rotate()))
            {
                wheel_disable_flag[i] = 0;
                owner->_wheel_drv[i]->disable();
            }
        }
    }
    else 
    {
        /* Phase 2: Stance achieved. Accessing LQR自平衡回路
           第二阶段：几何站立位置已到。开始接入 LQR 状态反馈自平衡算法。 */
        for(uint8_t i = 0; i < 2; i++)
        {
            /* Calculate LQR gains schedule based on current leg length l
               根据腿长 l 插值计算当前的 LQR 状态反馈控制增益矩阵 */
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

            owner->_leg_data[i].ref_l = TARGET_LENGTH;
            
            /* Contraction/length closed-loop force / 摆长高度 PID 伺服 */
            /* Right leg */
            owner->_leg_data[wl_chassis_t::R].ref_d_l =
                owner->_F_pid[wl_chassis_t::R]->calculate(owner->_leg_data[wl_chassis_t::R].ref_l, owner->_leg_data[wl_chassis_t::R].l);
            owner->_leg_data[wl_chassis_t::R].F[0] =
                owner->_d_F_pid[wl_chassis_t::R]->calculate(owner->_leg_data[wl_chassis_t::R].ref_d_l, owner->_leg_data[wl_chassis_t::R].d_l);
            /* Left leg */
            owner->_leg_data[wl_chassis_t::L].ref_d_l =
                owner->_F_pid[wl_chassis_t::L]->calculate(owner->_leg_data[wl_chassis_t::L].ref_l, owner->_leg_data[wl_chassis_t::L].l);
            owner->_leg_data[wl_chassis_t::L].F[0] =
                owner->_d_F_pid[wl_chassis_t::L]->calculate(owner->_leg_data[wl_chassis_t::L].ref_d_l, owner->_leg_data[wl_chassis_t::L].d_l);

            /* Calculate balance feedback biases (position loop is disabled initially)
               计算 LQR 状态偏差。由于是起立接入阶段，舍弃位移偏差，只利用俯仰角和倾角平衡环 */
            owner->_leg_data[i].x_bias = 0.0f;
            owner->_leg_data[i].d_x_bias = 0.0f;
            owner->_leg_data[i].beta_bias = 0.0f - owner->_leg_data[i].beta;
            owner->_leg_data[i].d_beta_bias = 0.0f - owner->_leg_data[i].d_beta;
            owner->_leg_data[i].gamma_bias = 0.0f - owner->_leg_data[i].gamma;
            owner->_leg_data[i].d_gamma_bias = 0.0f - owner->_leg_data[i].d_gamma;
            
            /* Wheel balance torque / 轮机自平衡力矩项 */
            owner->_leg_data[i].T_w_balance = (
                                      owner->_leg_data[i].lqr_gain[2] * owner->_leg_data[i].gamma_bias + 
                                      owner->_leg_data[i].lqr_gain[3] * owner->_leg_data[i].d_gamma_bias + 
                                      owner->_leg_data[i].lqr_gain[4] * owner->_leg_data[i].beta_bias + 
                                      owner->_leg_data[i].lqr_gain[5] * owner->_leg_data[i].d_beta_bias);

            /* Hip torque output / 关节虚拟平衡力矩项 */
            owner->_leg_data[i].F[1] = -(
                                      owner->_leg_data[i].lqr_gain[8] * owner->_leg_data[i].gamma_bias + 
                                      owner->_leg_data[i].lqr_gain[9] * owner->_leg_data[i].d_gamma_bias + 
                                      owner->_leg_data[i].lqr_gain[10] * owner->_leg_data[i].beta_bias + 
                                      owner->_leg_data[i].lqr_gain[11] * owner->_leg_data[i].d_beta_bias);
        }

        /* Check stabilization conditions to switch FSM status
           防抖判断：当双腿偏角及底盘俯仰角均回到垂直平衡范围内，且维持 ready_time 周期，标记起立准备完成 */
        static uint32_t time_count = 0;
        if((beta_bias > abs(owner->_leg_data[wl_chassis_t::R].beta)) &&
           (gamma_bias > abs(owner->_leg_data[wl_chassis_t::R].gamma)) &&
           (beta_bias > abs(owner->_leg_data[wl_chassis_t::L].beta)) &&
           (gamma_bias > abs(owner->_leg_data[wl_chassis_t::L].gamma)))
        {
            time_count += 1;
        }
        else 
        {
            time_count = 0;
        }
        
        if(time_count > ready_time)
        {
            owner->_active_mode_flag.ready = 1;
        }
        
        /* Clamp VMC forces to prevent motor saturation / 关节极向拉伸力限幅 */
        owner->_leg_data[wl_chassis_t::R].F[0] = fp32_constrain(owner->_leg_data[wl_chassis_t::R].F[0], -200.0f, 200.0f);
        owner->_leg_data[wl_chassis_t::L].F[0] = fp32_constrain(owner->_leg_data[wl_chassis_t::L].F[0], -200.0f, 200.0f);

        owner->_leg_data[wl_chassis_t::R].T_w = owner->_leg_data[wl_chassis_t::R].T_w_balance;
        owner->_leg_data[wl_chassis_t::L].T_w = owner->_leg_data[wl_chassis_t::L].T_w_balance;

        /* Send balance torques to wheel motors / 下发轮毂自平衡制动力矩，注意减速比、转矩常数以及左右安装符号 */
        owner->_wheel_drv[wl_chassis_t::R]->send_torque(fp32_constrain(-owner->_leg_data[wl_chassis_t::R].T_w / owner->_reduction_ratio /0.3f * (3591.0f/187.0f), -20.0f, 20.0f));
        owner->_wheel_drv[wl_chassis_t::L]->send_torque(fp32_constrain(owner->_leg_data[wl_chassis_t::L].T_w / owner->_reduction_ratio /0.3f * (3591.0f/187.0f), -20.0f, 20.0f));
    }

    /* Transform virtual forces to motor torques by VMC Jacobian transpose
       使用 VMC 雅可比矩阵将极坐标虚拟伸缩力 F[0]、扭矩 F[1] 映射到 4 个连杆关节电机 RF/RB/LF/LB */
    for(uint8_t i = 0; i < 2; i++)
    {
        arm_mat_vec_mult_f32(&owner->_leg_data[i].T_mat, 
                            owner->_leg_data[i].F, 
                            owner->_leg_data[i].T);
    }

    /* Transmit commands to joint motors / 下发关节电机扭矩，右侧取反 */
    owner->_motor_drv[wl_chassis_t::RF]->send_torque(
                        -owner->_leg_data[wl_chassis_t::R].T[0]);
    owner->_motor_drv[wl_chassis_t::RB]->send_torque(
                        -owner->_leg_data[wl_chassis_t::R].T[1]);
    owner->_motor_drv[wl_chassis_t::LF]->send_torque(
                        owner->_leg_data[wl_chassis_t::L].T[0]);
    owner->_motor_drv[wl_chassis_t::LB]->send_torque(
                        owner->_leg_data[wl_chassis_t::L].T[1]);
}

void wl_chassis_t::fsm_active_t::state_ready_t::exit(wl_chassis_t *owner)
{
}

/**
 * @brief Interpolation planning for leg standing trajectories.
 *        起身过程轨迹插值规划算法。
 *        第一阶段：如果腿朝前，减小 target_angle 将其先转动收缩到后方区域 -PI/2。
 *        第二阶段：调整左右腿极轴角度差，使其运动学对称同步。
 *        第三阶段：将极轴角度转动到 upright 起身目标角，随后以恒定速度 LENGTH_SPEED 顶起腿长至 TARGET_LENGTH。
 */
void wl_chassis_t::fsm_active_t::state_ready_t::calc_target_value(wl_chassis_t *owner)
{
    float diff;

    /* 1. Phase 1: If the legs are in the front quadrant, move them back to -PI/2.
       第1阶段：如果双腿处于前半区域，先倒退偏转到 -PI/2 位置以防冲撞 */
    if((0 == state_flag[wl_chassis_t::R]) || (0 == state_flag[wl_chassis_t::L]))
    {
        for(uint8_t i = 0; i < 2; i++)
        {
            if(0 == state_flag[i])
            {
                if(((0.0f < cur_angle[i]) && (cur_angle[i] < PI/2.0f)) ||
                   ((-PI/2.0f < cur_angle[i]) && (cur_angle[i] < 0.0f)))
                {
                    float next_angle = wrap2pi_f32(target_angle[i] - ANGLE_SPEED);
                    float other_angle = target_angle[1 - i];
                    diff = wrap2pi_f32(next_angle - other_angle);
                    float current_diff = wrap2pi_f32(target_angle[i] - other_angle);
                    
                    /* A move is permitted only if moving this step will not result in a collision gap > 1.5 rad
                       防劈叉限制：限制单步角位移，防止左右摆角差异过大（> 1.5 rad）造成连杆机械撞击 */
                    if (fabsf(diff) < 1.5f || fabsf(diff) < fabsf(current_diff))
                    {
                        if(0.05f < fabsf(target_angle[i] - (-PI/2.0f)))
                        {
                            target_angle[i] -= ANGLE_SPEED; 
                            target_angle[i] = wrap2pi_f32(target_angle[i]);
                        }
                        else 
                        {
                            target_angle[i] = -PI/2.0f;
                            state_flag[i] = 1;
                        }
                    }
                }
                else 
                {
                    state_flag[i] = 2; 
                }
            }
        }
    }

    /* 2. Phase 2: Synchronize leg angles alpha
       第2阶段：差分同步。控制左右腿偏角 alpha 互相对齐，防止两腿动作不同步跌倒 */
    if(((2 == state_flag[wl_chassis_t::R]) || (2 == state_flag[wl_chassis_t::L]))
        || ((0 != state_flag[wl_chassis_t::R]) && (0 != state_flag[wl_chassis_t::L])))
    {
        if (state_flag[wl_chassis_t::R] != 3 || state_flag[wl_chassis_t::L] != 3)
        {
            diff = wrap2pi_f32(target_angle[wl_chassis_t::L] - target_angle[wl_chassis_t::R]);
            
            if (fabsf(diff) > 0.05f)
            {
                /* Catch up using shortest angular path / 领先的腿原地等待或减速，落后的腿加速赶上对齐 */
                if (diff > 0.0f)
                {
                    target_angle[wl_chassis_t::L] -= ANGLE_SPEED;
                    target_angle[wl_chassis_t::L] = wrap2pi_f32(target_angle[wl_chassis_t::L]);
                }
                else
                {
                    target_angle[wl_chassis_t::R] -= ANGLE_SPEED;
                    target_angle[wl_chassis_t::R] = wrap2pi_f32(target_angle[wl_chassis_t::R]);
                }
            }
            else
            {
                target_angle[wl_chassis_t::R] = target_angle[wl_chassis_t::L];
                state_flag[wl_chassis_t::R] = 3;
                state_flag[wl_chassis_t::L] = 3;
            }
        }
    }
    
    /* 3. Phase 3: Transition to Target Angle and extend Target Length
       第3阶段：同步到位后，首先将左右极坐标角摆动到起立目标角度 TARGET_ANGLE，然后向上拉伸摆长到目标高度 */
    if(((3 == state_flag[wl_chassis_t::R]) && (3 == state_flag[wl_chassis_t::L]))
        ||((1 == state_flag[wl_chassis_t::R]) && (1 == state_flag[wl_chassis_t::L])))
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
        
                /* Linear interpolation of leg lengths / 摆长高度恒定速度上升规划 */
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
}