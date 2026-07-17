#include "pyro_wl_chassis.h"

#include "dsp/fast_math_functions.h"

namespace pyro
{

wl_chassis_t::wl_chassis_t() : module_base_t("wl_chassis")
{
}

status_t wl_chassis_t::_init()
{
    return PYRO_OK;
}

void wl_chassis_t::_update_feedback()
{
}

void wl_chassis_t::_fsm_execute()
{
}

void wl_chassis_t::_vmc_trans()
{
    for (auto leg : _ctx.data.leg)
    {
        const float theta = (leg.current_motor_rad[motor_def::KNEE] -
                             leg.current_motor_rad[motor_def::HIP]) /
                            2;
        const float sin_theta  = arm_sin_f32(theta);
        const float cos_theta  = arm_cos_f32(theta);
        float OH               = OJ5 * cos_theta;
        float HJ5              = OJ5 * sin_theta;
        float HJ4              = sqrt(J4J5 * J4J5 - HJ5 * HJ5);
        float OJ4              = OH + HJ4;
        leg.current_leg_length = OJ4 * OJ8 / OJ5;
        leg.current_leg_rad    = (leg.current_motor_rad[motor_def::KNEE] +
                               leg.current_motor_rad[motor_def::HIP]) /
                              2;

        leg.J_L         = -OJ8 * sin_theta * (OJ4 / HJ4);
        leg.current_F_L = (leg.out_motor_torque[motor_def::KNEE] -
                           leg.out_motor_torque[motor_def::HIP]) /
                          leg.J_L;
        leg.current_T_p = leg.out_motor_torque[motor_def::KNEE] +
                          leg.out_motor_torque[motor_def::HIP];
    }
}

void wl_chassis_t::_calculate()
{
    _ctx.data.leg[leg_def::LEFT].out_F_L =
        _ctx.pid.leg_length[leg_def::LEFT]->calculate(
            _ctx.data.leg[leg_def::LEFT].target_leg_length,
            _ctx.data.leg[leg_def::LEFT].current_leg_length);
    _ctx.data.leg[leg_def::LEFT].out_T_p =
        _ctx.pid.leg_rad[leg_def::LEFT]->calculate(
            _ctx.data.leg[leg_def::LEFT].target_leg_rad,
            _ctx.data.leg[leg_def::LEFT].current_leg_rad);
    _ctx.data.leg[leg_def::RIGHT].out_F_L =
        _ctx.pid.leg_length[leg_def::RIGHT]->calculate(
            _ctx.data.leg[leg_def::RIGHT].target_leg_length,
            _ctx.data.leg[leg_def::RIGHT].current_leg_length);
    _ctx.data.leg[leg_def::RIGHT].out_T_p =
        _ctx.pid.leg_rad[leg_def::RIGHT]->calculate(
            _ctx.data.leg[leg_def::RIGHT].target_leg_rad,
            _ctx.data.leg[leg_def::RIGHT].current_leg_rad);
}

void wl_chassis_t::_vmc_control()
{
    for (auto leg : _ctx.data.leg)
    {
        float tau_sum = leg.out_T_p;
        float tau_diff = leg.out_F_L * leg.J_L;
        leg.out_motor_torque[motor_def::HIP] = (tau_sum - tau_diff) / 2;
        leg.out_motor_torque[motor_def::KNEE] = (tau_sum + tau_diff) / 2;
    }
}


/* state_passive_t */

void wl_chassis_t::state_passive_t::enter(wl_chassis_t *owner)
{
}

void wl_chassis_t::state_passive_t::execute(wl_chassis_t *owner)
{
}

void wl_chassis_t::state_passive_t::exit(wl_chassis_t *owner)
{
}

/* fsm_active_t */

void wl_chassis_t::fsm_active_t::on_enter(wl_chassis_t *ctx)
{
}

void wl_chassis_t::fsm_active_t::on_execute(wl_chassis_t *ctx)
{
}

void wl_chassis_t::fsm_active_t::on_exit(wl_chassis_t *ctx)
{
}

/* fsm_active_t::state_manual_t */

void wl_chassis_t::fsm_active_t::state_manual_t::enter(wl_chassis_t *owner)
{
}

void wl_chassis_t::fsm_active_t::state_manual_t::execute(wl_chassis_t *owner)
{
}

void wl_chassis_t::fsm_active_t::state_manual_t::exit(wl_chassis_t *owner)
{
}

} // namespace pyro