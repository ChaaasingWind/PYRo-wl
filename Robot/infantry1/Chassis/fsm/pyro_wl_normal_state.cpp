#include "pyro_wl_chassis.h"

#include <algorithm>

namespace pyro
{

void wl_chassis_t::fsm_active_t::state_normal_t::enter(wl_chassis_t *owner)
{
    owner->_ctx.data.target_yaw = owner->_ctx.data.ins.euler_rad[0];

    owner->_ctx.data.odom.real_x            = 0;
    owner->_ctx.data.odom.target_x          = 0;
    owner->_ctx.data.odom.target_dot_x[0]   = 0;

    owner->_ctx.data.target_state.x         = 0;
    owner->_ctx.data.target_state.dot_x     = 0.0f;
    owner->_ctx.data.target_state.beta      = 0.1f;
    owner->_ctx.data.target_state.dot_beta  = 0.0f;
    owner->_ctx.data.target_state.gamma     = 0.0f;
    owner->_ctx.data.target_state.dot_gamma = 0.0f;

    float avg_length =
    (owner->_ctx.data.leg[leg_def::L].current_leg_length + owner->_ctx.data.leg[leg_def::R].current_leg_length) * 0.5f;
    for (auto &leg : owner->_ctx.data.leg)
    {
        leg.target_leg_rad    = leg.current_leg_rad;
        leg.target_leg_speed  = leg.current_leg_speed;
        leg.target_leg_radps  = leg.current_leg_radps;
        leg.target_leg_length = avg_length;
        leg.out_F_L                           = 0;
        leg.out_T_p                           = 0;
        leg.out_joint_torque[joint_def::HIP]  = 0;
        leg.out_joint_torque[joint_def::KNEE] = 0;
    }
    // owner->_ctx.pid.leg_length[leg_def::L]->clear();
    // owner->_ctx.pid.leg_length[leg_def::R]->clear();
    owner->_ctx.pid.leg_length_diff->clear();
    owner->_ctx.pid.leg_rad[leg_def::L]->clear();
    owner->_ctx.pid.leg_rad[leg_def::R]->clear();
    owner->_ctx.pid.leg_rad_diff->clear();
    owner->_ctx.pid.yaw->clear();


    owner->_ctx.motor.wheel[leg_def::L]->enable();
    owner->_ctx.motor.wheel[leg_def::R]->enable();
}

void wl_chassis_t::fsm_active_t::state_normal_t::execute(wl_chassis_t *owner)
{
    owner->_ctx.data.target_yaw += owner->_current_cmd.delta_yaw;

    owner->_ctx.data.leg[leg_def::L].target_leg_length +=
        owner->_current_cmd.delta_leg_length[leg_def::L];
    owner->_ctx.data.leg[leg_def::R].target_leg_length +=
        owner->_current_cmd.delta_leg_length[leg_def::R];

    owner->_ctx.data.leg[leg_def::L].target_leg_length =
        std::clamp(owner->_ctx.data.leg[leg_def::L].target_leg_length,
                   MIN_LEG_LENGTH, MAX_LEG_LENGTH);
    owner->_ctx.data.leg[leg_def::R].target_leg_length =
        std::clamp(owner->_ctx.data.leg[leg_def::R].target_leg_length,
                   MIN_LEG_LENGTH, MAX_LEG_LENGTH);

    const float target_vx = owner->_current_cmd.v;
    owner->_ctx.data.odom.target_dot_x[0] = target_vx;




    owner->_ctx.data.odom.target_x += target_vx * owner->_ctx.data._dt;

    // 写入 LQR 目标状态。
    owner->_ctx.data.target_state.x     = owner->_ctx.data.odom.target_x;
    owner->_ctx.data.target_state.dot_x = owner->_ctx.data.odom.target_dot_x[0];

    owner->_balance_calculate();
    owner->_vmc_trans_v2j();
    owner->_send_joint_torque();
    owner->_send_wheel_torque();
}

void wl_chassis_t::fsm_active_t::state_normal_t::exit(wl_chassis_t *owner)
{
    (void)owner;
}

} // namespace pyro
