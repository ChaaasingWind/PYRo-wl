#include "pyro_algo_common.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_wl_chassis.h"

#include <algorithm>

namespace pyro
{
    constexpr float align_time_ms       = 2000.0f;

    constexpr float align_max_rad       = 1.2f;
    constexpr float align_min_rad       = 0.7f;
    constexpr float align_target_rad    = 0.9f;

    constexpr float align_target_length = 0.20f;

    static float right_leg_delta_rad;
    static float left_leg_delta_rad;
    static float right_leg_delta_length;
    static float left_leg_delta_length;
    static int count;

    void wl_chassis_t::fsm_active_t::state_align_t::enter(wl_chassis_t *owner)
    {
        for (auto &leg : owner->_ctx.data.leg)
        {
            leg.target_leg_length                 = leg.current_leg_length;
            leg.target_leg_rad                    = leg.current_leg_rad;
            leg.target_leg_speed                  = leg.current_leg_speed;
            leg.target_leg_radps                  = leg.current_leg_radps;
            leg.out_F_L                           = 0;
            leg.out_T_p                           = 0;
            leg.out_joint_torque[joint_def::HIP]  = 0;
            leg.out_joint_torque[joint_def::KNEE] = 0;
        }

        //复位的角度小量计算
        right_leg_delta_rad    = (loop_fp32_constrain(align_target_rad-owner->_ctx.data.leg[leg_def::R].target_leg_rad,0,2*PI))/align_time_ms;
        left_leg_delta_rad     = (loop_fp32_constrain(align_target_rad-owner->_ctx.data.leg[leg_def::L].target_leg_rad,0,2*PI))/align_time_ms;
        right_leg_delta_length = (align_target_length - owner->_ctx.data.leg[leg_def::R].target_leg_length)/align_time_ms;
        left_leg_delta_length  = (align_target_length - owner->_ctx.data.leg[leg_def::L].target_leg_length)/align_time_ms;
        count                  = (int)align_time_ms;

        owner->_ctx.motor.wheel[leg_def::L]->disable();
        owner->_ctx.motor.wheel[leg_def::R]->disable();
    }

    void wl_chassis_t::fsm_active_t::state_align_t::execute(wl_chassis_t *owner)
    {
        //判断双腿是否在可以起身的位置
        static uint8_t keep_tick =0;
        if(owner->_ctx.data.leg[leg_def::L].current_leg_rad <= align_max_rad &&
           owner->_ctx.data.leg[leg_def::L].current_leg_rad >= align_min_rad &&
           owner->_ctx.data.leg[leg_def::R].current_leg_rad <= align_max_rad &&
           owner->_ctx.data.leg[leg_def::R].current_leg_rad >= align_min_rad &&
           owner->_ctx.data.leg[leg_def::L].current_leg_length <=MIN_LEG_LENGTH+0.04f&&
           owner->_ctx.data.leg[leg_def::R].current_leg_length <=MIN_LEG_LENGTH+0.04f&&
           
           abs(owner->_ctx.data.leg[leg_def::L].current_leg_rad-owner->_ctx.data.leg[leg_def::R].current_leg_rad)<=0.1f)
        {
            if(keep_tick >=20)
            {
                owner->_ctx.data.leg_is_ready = true;
            }
            keep_tick++;
        }
        else 
        {
            keep_tick = 0;
        }


        if(count > 0)
        {
            owner->_ctx.data.leg[leg_def::L].target_leg_rad    += left_leg_delta_rad;
            owner->_ctx.data.leg[leg_def::R].target_leg_rad    += right_leg_delta_rad;
            owner->_ctx.data.leg[leg_def::L].target_leg_length += left_leg_delta_length;
            owner->_ctx.data.leg[leg_def::R].target_leg_length += right_leg_delta_length;
            count--;
        }
        


        //限幅
        owner->_ctx.data.leg[leg_def::L].target_leg_rad =
                loop_fp32_constrain(owner->_ctx.data.leg[leg_def::L].target_leg_rad +
                owner->_current_cmd.delta_leg_rad[leg_def::L],
                       -PI, PI);
                    
        //限幅
        owner->_ctx.data.leg[leg_def::R].target_leg_rad =
                loop_fp32_constrain(owner->_ctx.data.leg[leg_def::R].target_leg_rad +
                owner->_current_cmd.delta_leg_rad[leg_def::R],
                       -PI, PI);
        
        //限幅
        owner->_ctx.data.leg[leg_def::L].target_leg_length =
            std::clamp(owner->_ctx.data.leg[leg_def::L].target_leg_length,
                       MIN_LEG_LENGTH, MAX_LEG_LENGTH);

        //限幅
            owner->_ctx.data.leg[leg_def::R].target_leg_length =
            std::clamp(owner->_ctx.data.leg[leg_def::R].target_leg_length,
                       MIN_LEG_LENGTH, MAX_LEG_LENGTH);

        owner->_ctx.data.leg[leg_def::L].error_leg_rad =
            loop_fp32_constrain(owner->_ctx.data.leg[leg_def::L].current_leg_rad -
                                owner->_ctx.data.leg[leg_def::L].target_leg_rad,
                            -PI, PI);


        owner->_ctx.data.leg[leg_def::R].error_leg_rad =
            loop_fp32_constrain(owner->_ctx.data.leg[leg_def::R].current_leg_rad -
                                owner->_ctx.data.leg[leg_def::R].target_leg_rad,
                            -PI, PI);


        owner->_manual_control();
        owner->_vmc_trans_v2j();
        owner->_send_joint_torque();
        owner->_ctx.motor.wheel[leg_def::L]->send_torque(0);
        owner->_ctx.motor.wheel[leg_def::R]->send_torque(0);
    }

    void wl_chassis_t::fsm_active_t::state_align_t::exit(wl_chassis_t *owner)
    {
        (void)owner;
    }

}