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

    for (auto &leg : owner->_ctx.data.leg)
    {
        leg.target_leg_length = leg.current_leg_length;
        leg.target_leg_rad    = leg.current_leg_rad;
        leg.target_leg_speed  = leg.current_leg_speed;
        leg.target_leg_radps  = leg.current_leg_radps;
        leg.out_F_L                           = 0;
        leg.out_T_p                           = 0;
        leg.out_joint_torque[joint_def::HIP]  = 0;
        leg.out_joint_torque[joint_def::KNEE] = 0;
    }
    owner->_ctx.pid.leg_length[leg_def::L]->clear();
    owner->_ctx.pid.leg_length[leg_def::R]->clear();
    owner->_ctx.pid.leg_rad[leg_def::L]->clear();
    owner->_ctx.pid.leg_rad[leg_def::R]->clear();


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

    // calculate current states and schedule LQR gains.
    for (uint8_t leg = 0; leg < 2; ++leg)
    {
        leg_ctx_t &leg_ctx = owner->_ctx.data.leg[leg];
        state_vec_t &state = owner->_ctx.data.current_state[leg];

        state.x            = owner->_ctx.data.odom.real_x;
        state.dot_x        = owner->_ctx.data.odom.real_dot_x[0];
        state.beta =
            PI / 2 - leg_ctx.current_leg_rad - owner->_ctx.data.ins.euler_rad[1];
        state.dot_beta  = -leg_ctx.current_leg_radps - owner->_ctx.data.ins.gyro[1];
        state.gamma     = owner->_ctx.data.ins.euler_rad[1];
        state.dot_gamma = owner->_ctx.data.ins.gyro[1];

        auto &K         = owner->_ctx.data.K[leg];
        for (uint8_t input = 0; input < 2; ++input)
        {
            for (uint8_t state_index = 0; state_index < 6; ++state_index)
            {
                K[input][state_index] = evaluate_polynomial_ascending(
                    leg_ctx.current_leg_length, K_POLY_COEF[input][state_index],
                    K_POLY_DEGREE);
            }
        }
    }


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
