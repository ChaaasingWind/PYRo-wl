/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-26 20:18:33
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-05-29 20:03:09
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#include "pyro_wl_chassis.h"
#include "pyro_ins.h"
#include "pyro_algo_common.h"
#include "pyro_com_canrx.h"
#include "pyro_com_cantx.h"
#include "pyro_referee.h"
#include "pyro_infantry2_chassis_intf.h"
namespace pyro
{

const float control_acc = 0.003f;
const float control_max_velocity = 1.5f;
const float control_leg_length[3] = {0.22f,0.28f,   0.36f};
float test_buffer;
float test_limit;
extern referee_drv_t *referee_drv;
extern can_drv_t *can3_drv;
wl_chassis_t *infantry2_chassis_ptr = nullptr;
wl_cmd_t     *infantry2_chassis_cmd_ptr = nullptr;
wl_cmd_t     *last_infantry2_chassis_cmd_ptr = nullptr;
void const *infantry2_rc_ctrl_ptr = nullptr;
GimbalToChassisComm gimbal_rx;
ChassisToGimbalComm gimbal_tx;
cmd_t cmd, last_cmd;
extern status_t ui_tread_init(void* argument);
}

using namespace pyro;
#define USE_GIMBAL_COM
// #define USE_DR16
#if defined(USE_GIMBAL_COM) && defined(USE_DR16)
#error "Gimbal COM and DR16 cannot be used at the same time"   
#endif

// #define USE_LEG_CTRL
#define USE_DIVERGENCY_DETECT
#if defined (USE_DIVERGENCY_DETECT)
#define DIVERGENCY_RESET_CNT 1000
#define MAX_GAMMA_BIAS PI/2
#define MAX_BETA_BIAS PI/2
#define MAX_X_BIAS 103.0f
#endif

extern wl_chassis_cfg_t infantry2_chassis_cfg;



extern "C"
{

void passive_mode(void const *rc_ctrl);
void test_mode(void const *rc_ctrl);
void ready_mode(void const *rc_ctrl);
void normal_mode(void const *rc_ctrl);
void reverse_mode(void const *rc_ctrl);
void over_step_mode(void const *rc_ctrl);
void over_step_ready_mode(void const *rc_ctrl);
void over_step_reset_mode(void const *rc_ctrl);
void control_mode(void const *rc_ctrl);
void spin_mode(void const *rc_ctrl);
#if defined(USE_DIVERGENCY_DETECT)
bool divergency_detect(float gamma_bias, float r_beta_bias, float l_beta_bias, 
                        float r_x_bias, float l_x_bias);
#endif
void infantry2_chassis_rc2cmd(void const *rc_ctrl)
{
#if defined(USE_GIMBAL_COM)
    can_rx_drv_t::get_data(pyro::bsp_can::can3, 0x100, gimbal_rx.buffer);
    memcpy(&last_cmd, &cmd, sizeof(cmd));
    if(0 != gimbal_rx.msg.vx)
    {
        if(gimbal_rx.msg.vx > 0)
        {
            cmd.vx += control_acc;
        }
        else
        {
            cmd.vx -= control_acc;
        }
    }
    else 
    {
        if(cmd.vx > control_acc)
        {
            cmd.vx -= control_acc;
        }
        else if(cmd.vx < -control_acc)
        {
            cmd.vx += control_acc;
        }

        if(fabsf(cmd.vx) <= control_acc)
        {
            cmd.vx = 0.0f;
        }
    }
    cmd.vy = gimbal_rx.msg.vy;
    cmd.vx = fp32_constrain(cmd.vx, -control_max_velocity, control_max_velocity);
    cmd.vy = fp32_constrain(cmd.vy, -control_max_velocity, control_max_velocity);
    cmd.v = sqrtf(cmd.vx * cmd.vx + cmd.vy * cmd.vy);
    cmd.turn_angle = atan2f(cmd.vy, cmd.vx);
    
    cmd.yaw_vel = (float)gimbal_rx.msg.yawvel / 100000.0f;
    if(gimbal_rx.msg.mode == cmd_t::PASSIVE)
    {
        cmd.mode = cmd_t::PASSIVE;
    }
    else if(gimbal_rx.msg.mode == cmd_t::ACTIVE)
    {
        cmd.mode = cmd_t::ACTIVE;
        if(1 == gimbal_rx.msg.stepClimb)
        {
            cmd.mode = cmd_t::STEP_CLIMB;
        }
    }
    else if(gimbal_rx.msg.mode == cmd_t::SPIN) 
    {
        cmd.mode = cmd_t::SPIN;
    }

        if(1 == gimbal_rx.msg.selfRescue)
        {
            cmd.mode = cmd_t::REVERSE;
        }
    cmd.leg_length_mode = gimbal_rx.msg.legLength;
#endif
#if defined(USE_DR16)
    pyro::read_scope_lock lock(
            pyro::rc_hub_t::get_instance(
         pyro::rc_hub_t::DR16)->get_lock());
   static auto *p_ctrl =
            static_cast<dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);  
#endif
#if defined (USE_GIMBAL_COM)
    // if(last_mode == pyro::cmd_base_t::mode_t::PASSIVE && 
    //     cmd.mode == cmd::ACTIVE)
    // {
    //     infantry2_chassis_ptr->get_cur_angle(&infantry2_chassis_cmd_ptr->r_angle,
    //                           &infantry2_chassis_cmd_ptr->l_angle);
    //     infantry2_chassis_ptr->get_cur_length(&infantry2_chassis_cmd_ptr->r_leg,
    //                         &infantry2_chassis_cmd_ptr->l_leg);
    // }
    if(cmd.mode == cmd_t::PASSIVE)
    {
        infantry2_chassis_cmd_ptr->l_leg = 0.18;
        infantry2_chassis_cmd_ptr->r_leg = 0.18;
        passive_mode(rc_ctrl);
    }
    else if(cmd.mode == cmd_t::ACTIVE)
    {
#if !defined (USE_LEG_CTRL)
        infantry2_chassis_cmd_ptr->l_leg = control_leg_length[cmd.leg_length_mode];
        infantry2_chassis_cmd_ptr->r_leg = control_leg_length[cmd.leg_length_mode];
#endif

        infantry2_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
        if(0 == infantry2_chassis_ptr->get_status_flag(wl_cmd_t::READY))
        {
            ready_mode(rc_ctrl);
        }
      
        else
        {
            normal_mode(rc_ctrl);
#if defined(USE_DIVERGENCY_DETECT)
            float r_beta_bias, l_beta_bias, gamma_bias, r_x_bias, l_x_bias;
            infantry2_chassis_ptr->get_cur_x_bias(&r_x_bias, &l_x_bias);
            infantry2_chassis_ptr->get_cur_beta_bias(&r_beta_bias, &l_beta_bias);
            infantry2_chassis_ptr->get_cur_gamma_bias(&gamma_bias);
            if(divergency_detect(gamma_bias, r_beta_bias, l_beta_bias, r_x_bias, l_x_bias))
            {
                passive_mode(rc_ctrl);
            }
#endif
        }
    }
    else if(cmd.mode == cmd_t::SPIN) 
    { 
#if !defined (USE_LEG_CTRL)
        infantry2_chassis_cmd_ptr->l_leg = 0.18f;
        infantry2_chassis_cmd_ptr->r_leg = 0.18f;
#endif
        infantry2_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
        
            spin_mode(rc_ctrl);
    }
    else if(cmd.mode == cmd_t::STEP_CLIMB)
    {
        static uint32_t over_step_cnt = 0;
        static uint32_t delay_cnt = 0;
        static uint8_t over_step_flag = 0;
        if(last_cmd.mode == cmd_t::ACTIVE)
        {
            static float temp_yaw;
            infantry2_chassis_ptr->get_cur_ins_yaw(&temp_yaw); 
            infantry2_chassis_cmd_ptr->yaw = temp_yaw;
                over_step_flag = 0;
                delay_cnt = 0;
                over_step_cnt = 500;
                infantry2_chassis_ptr->clear_status_flag(wl_cmd_t::OVER_STEP_RESET);
                infantry2_chassis_ptr->clear_status_flag(wl_cmd_t::OVER_STEP);
        }
        infantry2_chassis_cmd_ptr->yaw += cmd.yaw_vel;
        if(delay_cnt < 500)
        {
            delay_cnt++;
        }
        else
        {
            infantry2_chassis_cmd_ptr->l_leg = 0.35f;
            infantry2_chassis_cmd_ptr->r_leg = 0.35f;
        }
        constexpr float TEST_FORCE = -20.0f;
        constexpr float TEST_ANGLE = 2.0f;
        constexpr float TEST_d_ANGLE = 0.5f;
        static float temp_torque[2] = {0.0f, 0.0f};
        static float temp_angle[2] = {0.0f, 0.0f};
        static float temp_d_angle[2] = {0.0f, 0.0f};
        infantry2_chassis_ptr->get_cur_angle(&temp_angle[0], &temp_angle[1]);
        infantry2_chassis_ptr->get_cur_d_angle(&temp_d_angle[0], &temp_d_angle[1]);
        infantry2_chassis_ptr->get_cur_p_torque(&temp_torque[0],
                                    &temp_torque[1]);
        // if((0 == infantry2_chassis_ptr->get_status_flag(wl_cmd_t::OVER_STEP)) &&
        //     ((temp_torque[0] < TEST_FORCE) && (abs(temp_d_angle[0]) < TEST_d_ANGLE) && (temp_torque[1] < TEST_FORCE) && (abs(temp_d_angle[1]) < TEST_d_ANGLE)))
        // if((0 == infantry2_chassis_ptr->get_status_flag(wl_cmd_t::OVER_STEP)) &&
        //     ((temp_angle[0] > TEST_ANGLE) || (temp_angle[1] > TEST_ANGLE)))
        if(over_step_cnt == 0)
        {
            if((0 == infantry2_chassis_ptr->get_status_flag(wl_cmd_t::OVER_STEP)) &&
                ((temp_torque[0] < TEST_FORCE) || (temp_torque[1] < TEST_FORCE)))
                // ((temp_torque[0] < TEST_FORCE) || (temp_torque[1] < TEST_FORCE)) &&
                // ((abs(temp_d_angle[0]) < TEST_d_ANGLE) || (abs(temp_d_angle[1]) < TEST_d_ANGLE)))
            {

                over_step_flag = 1;
            }
        }
        else {
            over_step_cnt--;
        }
        if(0 == over_step_flag)
        {
            over_step_ready_mode(rc_ctrl);
        }
        else 
        {
            if(0 == infantry2_chassis_ptr->get_status_flag(wl_cmd_t::OVER_STEP))
            {
                over_step_mode(rc_ctrl);
            }
            else 
            {
                over_step_reset_mode(rc_ctrl);
            }
            
            if(1 == infantry2_chassis_ptr->get_status_flag(wl_cmd_t::OVER_STEP_RESET))
            {
                // over_step_flag = 0;
                // delay_cnt = 0;
                // over_step_cnt = 500;
                // infantry2_chassis_ptr->clear_status_flag(wl_cmd_t::OVER_STEP_RESET);
                // infantry2_chassis_ptr->clear_status_flag(wl_cmd_t::OVER_STEP);
                infantry2_chassis_cmd_ptr->l_leg = 0.18f;
                infantry2_chassis_cmd_ptr->r_leg = 0.18f;
                over_step_ready_mode(rc_ctrl);
            }
        }
    }
    else if(cmd.mode == cmd_t::REVERSE)
    {
        infantry2_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
        infantry2_chassis_cmd_ptr->l_leg = 0.35f;
        infantry2_chassis_cmd_ptr->r_leg = 0.35f;
        reverse_mode(rc_ctrl);
    }
#endif

#if defined(USE_DR16) 
    if( infantry2_chassis_cmd_ptr->last_active_mode != wl_cmd_t::CONTROL && 
        p_ctrl->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_DOWN)
    {
        infantry2_chassis_ptr->get_cur_angle(&infantry2_chassis_cmd_ptr->r_angle,
                              &infantry2_chassis_cmd_ptr->l_angle);
        infantry2_chassis_ptr->get_cur_length(&infantry2_chassis_cmd_ptr->r_leg,
                            &infantry2_chassis_cmd_ptr->l_leg);
        
    }
    switch (p_ctrl->rc.s_r.state) 
    {
        case dr16_drv_t::sw_state_t::SW_UP:
            passive_mode(rc_ctrl);
            break;
        case dr16_drv_t::sw_state_t::SW_DOWN:
            infantry2_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
            if(p_ctrl->rc.s_l.state == dr16_drv_t::sw_state_t::SW_UP)
            {
                test_mode(rc_ctrl);
            }
            else
            {
                control_mode(rc_ctrl);
            }
            break;
        case dr16_drv_t::sw_state_t::SW_MID:
            infantry2_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
            if(p_ctrl->rc.s_l.state == dr16_drv_t::sw_state_t::SW_UP)
            {
                ready_mode(rc_ctrl);
            }
            else if(p_ctrl->rc.s_l.state == dr16_drv_t::sw_state_t::SW_MID)
            {

                if(0 == infantry2_chassis_ptr->get_status_flag(wl_cmd_t::READY))
                {
                    ready_mode(rc_ctrl);
                }
                else
                {
                    normal_mode(rc_ctrl);
                }
            }
            else
            {
                constexpr float TEST_FORCE = -15.0f;
                static uint8_t over_step_flag = 0;
                static float temp_torque[2] = {0.0f, 0.0f};
                infantry2_chassis_ptr->get_cur_p_torque(&temp_torque[0],
                                            &temp_torque[1]);
                if((0 == infantry2_chassis_ptr->get_status_flag(wl_cmd_t::OVER_STEP)) &&
                    ((temp_torque[0] < TEST_FORCE) || (temp_torque[1] < TEST_FORCE)))
                {
                    over_step_flag = 1;

                }

                if(0 == over_step_flag)
                {
                    over_step_ready_mode(rc_ctrl);
                }
                else 
                {
                    if(0 == infantry2_chassis_ptr->get_status_flag(wl_cmd_t::OVER_STEP))
                    {
                        over_step_mode(rc_ctrl);
                    }
                    else 
                    {
                        ready_mode(rc_ctrl);
                    }
                    
                    if(1 == infantry2_chassis_ptr->get_status_flag(wl_cmd_t::READY))
                    {
                        over_step_flag = 0;
                        infantry2_chassis_ptr->clear_status_flag(wl_cmd_t::OVER_STEP);
                    }
                }
            }
            break;
        default:
            break;
    
    }
#endif
#if defined(USE_LEG_CTRL)
    infantry2_chassis_cmd_ptr->r_leg = fp32_constrain(
       infantry2_chassis_cmd_ptr->r_leg, 0.18f, 0.37f);
    infantry2_chassis_cmd_ptr->l_leg = fp32_constrain(
       infantry2_chassis_cmd_ptr->l_leg, 0.18f, 0.37f);
#endif
    infantry2_chassis_cmd_ptr->last_active_mode = infantry2_chassis_cmd_ptr->active_mode;
}
void infantry2_chassis_main_tread(void *argument)
{
    status_t ret = infantry2_chassis_ptr->start();
    
    while(1)
    { test_buffer =referee_drv->get_data().power_heat.buffer_energy;
       test_limit = referee_drv->get_data().robot_status.chassis_power_limit;

        gimbal_tx.msg.initialSpeedX100 = (uint16_t)(referee_drv->get_data().shoot.initial_speed* 100.0f);
        gimbal_tx.msg.shooter17mmBarrelHeat = referee_drv->get_data().power_heat.shooter_17mm_barrel_heat;
        gimbal_tx.msg.heatLimit = referee_drv->get_data().robot_status.shooter_barrel_heat_limit;
        gimbal_tx.msg.coolingRate = referee_drv->get_data().robot_status.shooter_barrel_cooling_value;
        gimbal_tx.msg.robotId = referee_drv->get_data().robot_status.robot_id;
        gimbal_tx.msg.chassisReady = infantry2_chassis_ptr->get_status_flag(wl_cmd_t::READY);
        gimbal_tx.msg.chassisYawSpeed = (int8_t)(infantry2_chassis_cmd_ptr->yaw * 100.0f);
       
        can_tx_drv_t::instance()->clear(0x101);
        can_tx_drv_t::instance()->add_data_raw(0x101, 64, &gimbal_tx);
        can_tx_drv_t::instance()->send(0x101, can3_drv);
        infantry2_chassis_rc2cmd(infantry2_rc_ctrl_ptr);
        infantry2_chassis_ptr->set_command(*infantry2_chassis_cmd_ptr);
        vTaskDelay(1);
    }
}




status_t infantry2_chassis_init(void *argument)
{

    can_rx_drv_t::subscribe(pyro::bsp_can::can3, 0x100);
    infantry2_chassis_cmd_ptr = new wl_cmd_t();
    infantry2_chassis_ptr = wl_chassis_t::instance();
    infantry2_rc_ctrl_ptr = nullptr;
    BaseType_t ret = 
        xTaskCreate(infantry2_chassis_main_tread, 
            "Infantry2 Chassis", 512, 
            nullptr, 1, nullptr);
    CHECK_OS_RET(ret);
    
   
    infantry2_chassis_ptr->configure(infantry2_chassis_cfg);
    ui_tread_init(nullptr);
    return status_t::PYRO_OK;
}

void passive_mode(void const *rc_ctrl)
{
    infantry2_chassis_cmd_ptr->l_angle = PI/2.0f;
    infantry2_chassis_cmd_ptr->r_angle = PI/2.0f;
    infantry2_chassis_cmd_ptr->l_leg = 0;
    infantry2_chassis_cmd_ptr->r_leg = 0;
    infantry2_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
    float yaw, pitch, roll;
    ins_drv_t::get_instance()->get_rads_b(&yaw, &pitch, &roll);
    infantry2_chassis_cmd_ptr->yaw = yaw;

}

void test_mode(void const *rc_ctrl)
{
    (void)rc_ctrl;
    infantry2_chassis_cmd_ptr->active_mode = wl_cmd_t::TEST;
}
void spin_mode(void const *rc_ctrl)
{
    (void)rc_ctrl;
    
    infantry2_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
    
  
    infantry2_chassis_cmd_ptr->active_mode = wl_cmd_t::SPIN;
}
void ready_mode(void const *rc_ctrl)
{
    (void)rc_ctrl;
    infantry2_chassis_cmd_ptr->l_angle = PI/2.0f;
    infantry2_chassis_cmd_ptr->r_angle = PI/2.0f;
    infantry2_chassis_cmd_ptr->active_mode = wl_cmd_t::READY;
#if defined(USE_LEG_CTRL)
    infantry2_chassis_cmd_ptr->r_leg = 0.18f;
    infantry2_chassis_cmd_ptr->l_leg = 0.18f;
#endif
}
void normal_mode(void const *rc_ctrl)
{
    infantry2_chassis_cmd_ptr->active_mode = wl_cmd_t::NORMAL;

#if defined (USE_GIMBAL_COM)
    infantry2_chassis_cmd_ptr->vx = cmd.vx;
#if defined (USE_LEG_CTRL)
    infantry2_chassis_cmd_ptr->r_leg += (cmd.vy / 20000.0f);
    infantry2_chassis_cmd_ptr->l_leg += (cmd.vy / 20000.0f);
#endif
#endif
#if defined (USE_DR16)
    static auto *p_ctrl =
            static_cast<dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);
#if defined (USE_LEG_CTRL)
    infantry2_chassis_cmd_ptr->r_leg += (p_ctrl->rc.ch_ry / 2000.0f);
    infantry2_chassis_cmd_ptr->l_leg += (p_ctrl->rc.ch_ry / 2000.0f);
#endif

    infantry2_chassis_cmd_ptr->yaw -= (p_ctrl->rc.ch_lx * PI / 500.0f);
    infantry2_chassis_cmd_ptr->vx = (p_ctrl->rc.ch_ly * 2.0f);
    infantry2_chassis_cmd_ptr->yaw = loop_fp32_constrain(
        infantry2_chassis_cmd_ptr->yaw, -PI, PI);
#endif
}
void reverse_mode(void const *rc_ctrl)
{
    (void)rc_ctrl;
    infantry2_chassis_cmd_ptr->active_mode = wl_cmd_t::REVERSE;
}
void control_mode(void const *rc_ctrl)
{
#if defined (USE_DR16)
    static auto *p_ctrl =
            static_cast<dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);
    infantry2_chassis_cmd_ptr->r_angle += (p_ctrl->rc.ch_rx * PI / 2000.0f);
    infantry2_chassis_cmd_ptr->l_angle += (p_ctrl->rc.ch_lx * PI / 2000.0f);
    infantry2_chassis_cmd_ptr->r_angle = loop_fp32_constrain(
        infantry2_chassis_cmd_ptr->r_angle, -PI, PI);
    infantry2_chassis_cmd_ptr->l_angle = loop_fp32_constrain(
        infantry2_chassis_cmd_ptr->l_angle, -PI, PI);

    infantry2_chassis_cmd_ptr->r_leg += (p_ctrl->rc.ch_ry / 2000.0f);
    infantry2_chassis_cmd_ptr->l_leg += (p_ctrl->rc.ch_ly / 2000.0f);
#endif
#if defined (USE_GIMBAL_COM)
    infantry2_chassis_cmd_ptr->r_angle += (cmd.vx * PI / 2000.0f);
    infantry2_chassis_cmd_ptr->l_angle += (cmd.vx * PI / 2000.0f);
    infantry2_chassis_cmd_ptr->r_leg += (cmd.vy * PI / 10000.0f);
    infantry2_chassis_cmd_ptr->l_leg += (cmd.vy * PI / 10000.0f);

#endif

    infantry2_chassis_cmd_ptr->active_mode = wl_cmd_t::CONTROL;
}
void over_step_mode(void const *rc_ctrl)
{
    (void)rc_ctrl;
    infantry2_chassis_cmd_ptr->active_mode = wl_cmd_t::OVER_STEP;
}
void over_step_reset_mode(void const *rc_ctrl)
{
    (void)rc_ctrl;
    infantry2_chassis_cmd_ptr->active_mode = wl_cmd_t::OVER_STEP_RESET;
}
void over_step_ready_mode(void const *rc_ctrl)
{
    infantry2_chassis_cmd_ptr->active_mode = wl_cmd_t::OVER_STEP_READY;

#if defined (USE_GIMBAL_COM)
    infantry2_chassis_cmd_ptr->vx = cmd.vx/3;
#endif
#if defined (USE_DR16)
    static auto *p_ctrl =
            static_cast<dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);
    infantry2_chassis_cmd_ptr->r_leg += (p_ctrl->rc.ch_ry / 2000.0f);
    infantry2_chassis_cmd_ptr->l_leg += (p_ctrl->rc.ch_ry / 2000.0f);

    infantry2_chassis_cmd_ptr->yaw -= (p_ctrl->rc.ch_lx * PI / 500.0f);
    infantry2_chassis_cmd_ptr->vx = (p_ctrl->rc.ch_ly * 2.0f);
    infantry2_chassis_cmd_ptr->yaw = loop_fp32_constrain(
        infantry2_chassis_cmd_ptr->yaw, -PI, PI);
#endif
}

#if defined(USE_DIVERGENCY_DETECT)
bool divergency_detect(float gamma_bias, float r_beta_bias, float l_beta_bias, 
                        float r_x_bias, float l_x_bias)
{
    static bool divergency_flag = 0;
    static uint16_t cnt = 0;
    if( 0 == divergency_flag)
    {
        if((fabsf(gamma_bias) > MAX_GAMMA_BIAS) || (fabsf(r_beta_bias) > MAX_BETA_BIAS) || (fabsf(l_beta_bias) > MAX_BETA_BIAS) ||
            (fabsf(r_x_bias) > MAX_X_BIAS) || (fabsf(l_x_bias) > MAX_X_BIAS))
        {
            divergency_flag = 1;
        }
    }
    else 
    {
        cnt++;
        if(cnt >= DIVERGENCY_RESET_CNT)
        {
            divergency_flag = 0;
            cnt = 0;
        }
    }
    return divergency_flag;
}
#endif


}
