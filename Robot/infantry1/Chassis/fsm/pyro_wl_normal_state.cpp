#include "pyro_wl_chassis.h"

#include <cmath>

namespace pyro
{

void wl_chassis_t::fsm_active_t::state_normal_t::enter(wl_chassis_t *owner)
{
    owner->_ctx.data.odom.real_x            = 0;
    owner->_ctx.data.odom.target_x          = 0;
    owner->_ctx.data.odom.target_dot_x[0]   = 0;
    owner->_ctx.data.odom.target_dot_x[1]   = 0;

    owner->_ctx.data.target_state.x         = 0;
    owner->_ctx.data.target_state.dot_x     = 0.0f;
    owner->_ctx.data.target_state.beta      = 0.0f;
    owner->_ctx.data.target_state.dot_beta  = 0.0f;
    owner->_ctx.data.target_state.gamma     = 0.0f;
    owner->_ctx.data.target_state.dot_gamma = 0.0f;

    owner->_ctx.motor.wheel[leg_def::L]->enable();
    owner->_ctx.motor.wheel[leg_def::R]->enable();
}

void wl_chassis_t::fsm_active_t::state_normal_t::execute(wl_chassis_t *owner)
{
    owner->_ctx.data.odom.target_dot_x[1] =
        owner->_ctx.data.odom.target_dot_x[0];
    owner->_ctx.data.odom.target_dot_x[0] =
        fabs(owner->_current_cmd.v) > STOP_VELOCITY_DEADBAND
            ? owner->_current_cmd.v
            : 0;

    const bool breaking = (std::fabs(owner->_ctx.data.odom.target_dot_x[0]) <
                           STOP_VELOCITY_DEADBAND) &&
                          (std::fabs(owner->_ctx.data.odom.target_dot_x[1]) >=
                           STOP_VELOCITY_DEADBAND);


    // 判断停止时重置里程计
    if (breaking)
    {
        owner->_ctx.data.odom.target_x = owner->_ctx.data.odom.real_x;
    }
    else
    {
        owner->_ctx.data.odom.target_x +=
            0.5f * owner->_ctx.data._dt *
            (owner->_ctx.data.odom.target_dot_x[0] +
             owner->_ctx.data.odom.target_dot_x[1]);
    }

    // 写入 LQR 目标状态。
    owner->_ctx.data.target_state.x     = owner->_ctx.data.odom.target_x;
    owner->_ctx.data.target_state.dot_x = owner->_ctx.data.odom.target_dot_x[0];

    owner->_balance_calculate();
    owner->_vmc_trans_v2j();
    owner->_send_joint_torque();
}

void wl_chassis_t::fsm_active_t::state_normal_t::exit(wl_chassis_t *owner)
{
    (void)owner;
}

} // namespace pyro
