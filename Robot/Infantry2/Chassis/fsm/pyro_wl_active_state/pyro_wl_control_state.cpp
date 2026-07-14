/*
 * @Author: vod-x vod_x@outlook.com
 * @Date: 2026-04-18 15:06:20
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-05-14 23:00:41
 * @FilePath: \Wheel-Legged-Robot\embedded_system\PYRo\Module\Chassis\Wheel_Legged\fsm\pyro_wl_active_state\pyro_wl_control_state.cpp
 * @Description: Manual debug state (Direct joint target tracking) / 手动关节直控与摆长测试状态实现
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#include "pyro_wl_chassis.h"
namespace pyro
{
extern pid_t wheel_disable_pid[2];

void wl_chassis_t::fsm_active_t::state_control_t::enter(wl_chassis_t *owner)
{
}

/**
 * @brief Execute manual control loop. Disables self-balancing and tracks coordinates directly.
 *        执行直控调试逻辑。忽略 LQR 平衡反馈（各项设为 0），利用阻尼力矩锁死车轮不转。
 *        双腿的摆长高度和极轴角度直接通过嵌套 PID 闭环跟踪指令中的绝对极值（`l_leg`, `r_leg`, `l_angle`, `r_angle`）。
 */
void wl_chassis_t::fsm_active_t::state_control_t::execute(wl_chassis_t *owner)
{
    /* Disable wheel driving and keep them still using damping PID
       阻尼锁死车轮，防止由于连杆伸缩摆动造成底盘滑移摔倒 */
    for(uint8_t i = 0; i < 2; i++)
    {
            owner->_leg_data[i].T_w_move = wheel_disable_pid[i].calculate(0.0f,
                 owner->_leg_data[i].w);
            owner->_leg_data[i].T_w_turn = 0.0f;
            owner->_leg_data[i].T_w_balance = 0.0f;
    }
    
    /* Calculate expansion force F_leg (F[0]) to track reference target length directly
       双腿极向伸长闭环控制，直控高度目标 */
    /* Right leg / 右腿 */
    owner->_leg_data[wl_chassis_t::R].ref_d_l=
            owner->_F_pid[wl_chassis_t::R]->
        calculate(owner->_cmd->r_leg, 
        owner->_leg_data[wl_chassis_t::R].l);
    owner->_leg_data[wl_chassis_t::R].F[0]=
            owner->_d_F_pid[wl_chassis_t::R]->
             calculate(owner->_leg_data[wl_chassis_t::R].ref_d_l, 
        owner->_leg_data[wl_chassis_t::R].d_l);
    /* Left leg / 左腿 */
    owner->_leg_data[wl_chassis_t::L].ref_d_l=
        owner->_F_pid[wl_chassis_t::L]->
        calculate(owner->_cmd->l_leg,
        owner->_leg_data[wl_chassis_t::L].l);
    owner->_leg_data[wl_chassis_t::L].F[0]=
        owner->_d_F_pid[wl_chassis_t::L]->
        calculate(owner->_leg_data[wl_chassis_t::L].ref_d_l,
        owner->_leg_data[wl_chassis_t::L].d_l);

    /* Calculate hip virtual torque T_hip (F[1]) to track reference target angle directly
       双腿极轴摆动角闭环控制，直控角度目标，并处理环绕误差 */
    /* Right leg / 右腿 */
    float diff;
    if(owner->_cmd->r_angle - owner->_leg_data[wl_chassis_t::R].alpha > PI)
    {
        diff = -2 * PI + (owner->_cmd->r_angle - owner->_leg_data[wl_chassis_t::R].alpha);
    }
    else if(owner->_cmd->r_angle - owner->_leg_data[wl_chassis_t::R].alpha < -PI)
    {
        diff = 2 * PI + (owner->_cmd->r_angle - owner->_leg_data[wl_chassis_t::R].alpha);
    }
    else
    {
        diff = owner->_cmd->r_angle - owner->_leg_data[wl_chassis_t::R].alpha;
    }
    owner->_leg_data[wl_chassis_t::R].ref_d_alpha=
        owner->_T_pid[wl_chassis_t::R]->
        calculate(owner->_leg_data[wl_chassis_t::R].alpha + diff,
        owner->_leg_data[wl_chassis_t::R].alpha);
    owner->_leg_data[wl_chassis_t::R].F[1]=
        owner->_d_T_pid[wl_chassis_t::R]->
        calculate(owner->_leg_data[wl_chassis_t::R].ref_d_alpha,
        owner->_leg_data[wl_chassis_t::R].d_alpha);
        
    /* Left leg / 左腿 */
    if(owner->_cmd->l_angle - owner->_leg_data[wl_chassis_t::L].alpha > PI)
    {
        diff = -2 * PI + (owner->_cmd->l_angle - owner->_leg_data[wl_chassis_t::L].alpha);
    }
    else if(owner->_cmd->l_angle - owner->_leg_data[wl_chassis_t::L].alpha < -PI)
    {
        diff = 2 * PI + (owner->_cmd->l_angle - owner->_leg_data[wl_chassis_t::L].alpha);
    }
    else
    {
        diff = owner->_cmd->l_angle - owner->_leg_data[wl_chassis_t::L].alpha;
    }
    owner->_leg_data[wl_chassis_t::L].ref_d_alpha=
        owner->_T_pid[wl_chassis_t::L]->
        calculate(owner->_leg_data[wl_chassis_t::L].alpha + diff,
        owner->_leg_data[wl_chassis_t::L].alpha);
    owner->_leg_data[wl_chassis_t::L].F[1]=
        owner->_d_T_pid[wl_chassis_t::L]->
        calculate(owner->_leg_data[wl_chassis_t::L].ref_d_alpha,
        owner->_leg_data[wl_chassis_t::L].d_alpha);
    
    /* VMC Jacobian mapping to convert virtual forces to linkage torques
       根据 VMC 映射，解算出实际直驱电机需要的扭矩向量 T */
    for(uint8_t i = 0; i < 2; i++)
    {
        arm_mat_vec_mult_f32(&owner->_leg_data[i].T_mat, 
                            owner->_leg_data[i].F, 
                            owner->_leg_data[i].T);
    }

    /* Send commands to joint motors / 下发力矩到 4 个关节达妙电机，右侧取反 */
    owner->_motor_drv[wl_chassis_t::RF]->send_torque(
                        -owner->_leg_data[wl_chassis_t::R].T[0]);
    owner->_motor_drv[wl_chassis_t::RB]->send_torque(
                        -owner->_leg_data[wl_chassis_t::R].T[1]);
    owner->_motor_drv[wl_chassis_t::LF]->send_torque(
                        owner->_leg_data[wl_chassis_t::L].T[0]);
    owner->_motor_drv[wl_chassis_t::LB]->send_torque(
                        owner->_leg_data[wl_chassis_t::L].T[1]);
}

void wl_chassis_t::fsm_active_t::state_control_t::exit(wl_chassis_t *owner)
{
}
}