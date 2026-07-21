#include "pyro_wl_chassis.h"

#include "pyro_algo_common.h"
#include "dsp/fast_math_functions.h"

#include <algorithm>

namespace pyro
{

wl_chassis_t::wl_chassis_t() : module_base_t("wl_chassis")
{
}

status_t wl_chassis_t::_init()
{
    _ctx                                          = {};
    _ctx.motor                                    = _module_deps.motor;
    _ctx.pid                                      = _module_deps.pid;

    _current_cmd.delta_leg_length[leg_def::L]  = 0.0f;
    _current_cmd.delta_leg_length[leg_def::R] = 0.0f;
    _current_cmd.delta_leg_rad[leg_def::L]     = 0.0f;
    _current_cmd.delta_leg_rad[leg_def::R]    = 0.0f;

    _ctx.data.leg[leg_def::L].direction        = LEFT_LEG_DIRECTION;
    _ctx.data.leg[leg_def::R].direction       = RIGHT_LEG_DIRECTION;

    return PYRO_OK;
}

void wl_chassis_t::_update_feedback()
{
    _ctx.motor.joint[leg_def::L][motor_def::HIP]->update_feedback();
    _ctx.motor.joint[leg_def::L][motor_def::KNEE]->update_feedback();
    _ctx.motor.joint[leg_def::R][motor_def::HIP]->update_feedback();
    _ctx.motor.joint[leg_def::R][motor_def::KNEE]->update_feedback();
    _ctx.motor.wheel[leg_def::L]->update_feedback();
    _ctx.motor.wheel[leg_def::R]->update_feedback();


    float raw_rad_lh = _ctx.motor.joint[leg_def::L][motor_def::HIP]
                           ->get_current_position() +
                       LEFT_HIP_OFFSET;
    float raw_rad_lk = _ctx.motor.joint[leg_def::L][motor_def::KNEE]
                           ->get_current_position() +
                       LEFT_KNEE_OFFSET;
    float raw_rad_rh = _ctx.motor.joint[leg_def::R][motor_def::HIP]
                           ->get_current_position() +
                       RIGHT_HIP_OFFSET;
    float raw_rad_rk = _ctx.motor.joint[leg_def::R][motor_def::KNEE]
                           ->get_current_position() +
                       RIGHT_KNEE_OFFSET;
    _ctx.data.leg[leg_def::L].current_joint_rad[motor_def::HIP] =
        loop_fp32_constrain(raw_rad_lh, -PI, PI) *
        _ctx.data.leg[leg_def::L].direction;
    _ctx.data.leg[leg_def::L].current_joint_rad[motor_def::KNEE] =
        loop_fp32_constrain(raw_rad_lk, -PI, PI) *
        _ctx.data.leg[leg_def::L].direction;
    _ctx.data.leg[leg_def::R].current_joint_rad[motor_def::HIP] =
        loop_fp32_constrain(raw_rad_rh, -PI, PI) *
        _ctx.data.leg[leg_def::R].direction;
    _ctx.data.leg[leg_def::R].current_joint_rad[motor_def::KNEE] =
        loop_fp32_constrain(raw_rad_rk, -PI, PI) *
        _ctx.data.leg[leg_def::R].direction;

    _ctx.data.leg[leg_def::L].current_joint_torque[motor_def::HIP] =
        _ctx.data.leg[leg_def::L].direction *
        _ctx.motor.joint[leg_def::L][motor_def::HIP]->get_current_torque();
    _ctx.data.leg[leg_def::L].current_joint_torque[motor_def::KNEE] =
        _ctx.data.leg[leg_def::L].direction *
        _ctx.motor.joint[leg_def::L][motor_def::KNEE]->get_current_torque();
    _ctx.data.leg[leg_def::R].current_joint_torque[motor_def::HIP] =
        _ctx.data.leg[leg_def::R].direction *
        _ctx.motor.joint[leg_def::R][motor_def::HIP]->get_current_torque();
    _ctx.data.leg[leg_def::R].current_joint_torque[motor_def::KNEE] =
        _ctx.data.leg[leg_def::R].direction *
        _ctx.motor.joint[leg_def::R][motor_def::KNEE]->get_current_torque();

    _ctx.data.leg[leg_def::L].current_joint_radps[motor_def::HIP] =
        _ctx.data.leg[leg_def::L].direction *
        _ctx.motor.joint[leg_def::L][motor_def::HIP]->get_current_rotate();
    _ctx.data.leg[leg_def::L].current_joint_radps[motor_def::KNEE] =
        _ctx.data.leg[leg_def::L].direction *
        _ctx.motor.joint[leg_def::L][motor_def::KNEE]->get_current_rotate();
    _ctx.data.leg[leg_def::R].current_joint_radps[motor_def::HIP] =
        _ctx.data.leg[leg_def::R].direction *
        _ctx.motor.joint[leg_def::R][motor_def::HIP]->get_current_rotate();
    _ctx.data.leg[leg_def::R].current_joint_radps[motor_def::KNEE] =
        _ctx.data.leg[leg_def::R].direction *
        _ctx.motor.joint[leg_def::R][motor_def::KNEE]->get_current_rotate();
    // The existing VMC transform is intentionally left unchanged.
    _vmc_trans_j2v();

    _ctx.data.wheel[leg_def::L].current_radps =
        _ctx.motor.wheel[leg_def::L]->get_current_rotate() * _ctx.data.wheel[leg_def::L].direction * rec_reduction_ratio;
    _ctx.data.wheel[leg_def::R].current_radps =
        _ctx.motor.wheel[leg_def::R]->get_current_rotate() * _ctx.data.wheel[leg_def::R].direction * rec_reduction_ratio;

    _ctx.data.wheel[leg_def::L].current_T_w =
        _ctx.motor.wheel[leg_def::L]->get_current_torque() * _ctx.data.wheel[leg_def::L].direction * rec_reduction_ratio;
    _ctx.data.wheel[leg_def::R].current_T_w =
        _ctx.motor.wheel[leg_def::R]->get_current_torque() * _ctx.data.wheel[leg_def::R].direction * rec_reduction_ratio;


}

void wl_chassis_t::_fsm_execute()
{
    if (_current_cmd.mode == cmd_base_t::mode_t::ACTIVE)
    {
        _main_fsm.change_state(&_state_active);
    }
    else
    {
        _main_fsm.change_state(&_state_passive);
    }

    _main_fsm.execute(this);
}

void wl_chassis_t::_vmc_trans_j2v()
{
    for (auto &leg : _ctx.data.leg)
    {
        const float raw_2_theta = leg.current_joint_rad[motor_def::KNEE] -
                                  leg.current_joint_rad[motor_def::HIP];
        const float theta     = loop_fp32_constrain(raw_2_theta, -PI, PI) / 2;
        const float dot_theta = (leg.current_joint_radps[motor_def::KNEE] -
                                 leg.current_joint_radps[motor_def::HIP]) /
                                2;
        const float sin_theta  = arm_sin_f32(theta);
        const float cos_theta  = arm_cos_f32(theta);
        float OH               = OJ5 * cos_theta;
        float HJ5              = OJ5 * sin_theta;
        float HJ4              = sqrt(J4J5 * J4J5 - HJ5 * HJ5);
        float OJ4              = OH + HJ4;
        leg.J_L                = -OJ8 * sin_theta * (OJ4 / HJ4);
        leg.current_leg_length = OJ4 * OJ8 / OJ5;
        leg.current_leg_speed  = dot_theta * leg.J_L;

        const float raw_beta   = leg.current_joint_rad[motor_def::HIP] + theta;
        const float beta       = loop_fp32_constrain(raw_beta, -PI, PI);
        leg.current_leg_rad    = beta;
        leg.current_leg_radps  = (leg.current_joint_radps[motor_def::KNEE] +
                                 leg.current_joint_radps[motor_def::HIP]) /
                                2;
        leg.current_F_L = (leg.out_joint_torque[motor_def::KNEE] -
                           leg.out_joint_torque[motor_def::HIP]) /
                          leg.J_L;
        leg.current_T_p = leg.out_joint_torque[motor_def::KNEE] +
                          leg.out_joint_torque[motor_def::HIP];
    }
}

void wl_chassis_t::_calculate()
{
    _ctx.data.leg[leg_def::L].out_F_L =
        _ctx.pid.leg_length[leg_def::L]->calculate(
            _ctx.data.leg[leg_def::L].target_leg_length,
            _ctx.data.leg[leg_def::L].current_leg_length,
            _ctx.data.leg[leg_def::L].current_leg_speed);
    _ctx.data.leg[leg_def::L].out_T_p =
        _ctx.pid.leg_rad[leg_def::L]->calculate(
            0.0f, _ctx.data.leg[leg_def::L].error_leg_rad,
            _ctx.data.leg[leg_def::L].current_leg_radps);
    _ctx.data.leg[leg_def::R].out_F_L =
        _ctx.pid.leg_length[leg_def::R]->calculate(
            _ctx.data.leg[leg_def::R].target_leg_length,
            _ctx.data.leg[leg_def::R].current_leg_length,
            _ctx.data.leg[leg_def::R].current_leg_speed);
    _ctx.data.leg[leg_def::R].out_T_p =
        _ctx.pid.leg_rad[leg_def::R]->calculate(
            0.0f, _ctx.data.leg[leg_def::R].error_leg_rad,
            _ctx.data.leg[leg_def::R].current_leg_radps);
}

void wl_chassis_t::_vmc_trans_v2j()
{
    constexpr float MAX_MOTOR_TORQUE = 20.0f;

    for (auto &leg : _ctx.data.leg)
    {
        float tau_sum                        = leg.out_T_p;
        float tau_diff                       = leg.out_F_L * leg.J_L;
        leg.out_joint_torque[motor_def::HIP] = std::clamp(
            (tau_sum - tau_diff) / 2, -MAX_MOTOR_TORQUE, MAX_MOTOR_TORQUE);
        leg.out_joint_torque[motor_def::KNEE] = std::clamp(
            (tau_sum + tau_diff) / 2, -MAX_MOTOR_TORQUE, MAX_MOTOR_TORQUE);
    }
}

void wl_chassis_t::_send_joint_torque()
{
    _ctx.motor.joint[leg_def::L][motor_def::HIP]->send_torque(
        _ctx.data.leg[leg_def::L].direction *
        _ctx.data.leg[leg_def::L].out_joint_torque[motor_def::HIP]);
    _ctx.motor.joint[leg_def::L][motor_def::KNEE]->send_torque(
        _ctx.data.leg[leg_def::L].direction *
        _ctx.data.leg[leg_def::L].out_joint_torque[motor_def::KNEE]);
    _ctx.motor.joint[leg_def::R][motor_def::HIP]->send_torque(
        _ctx.data.leg[leg_def::R].direction *
        _ctx.data.leg[leg_def::R].out_joint_torque[motor_def::HIP]);
    _ctx.motor.joint[leg_def::R][motor_def::KNEE]->send_torque(
        _ctx.data.leg[leg_def::R].direction *
        _ctx.data.leg[leg_def::R].out_joint_torque[motor_def::KNEE]);

    // _ctx.motor.joint[leg_def::LEFT][motor_def::HIP]->send_torque(0);
    // _ctx.motor.joint[leg_def::LEFT][motor_def::KNEE]->send_torque(0);
    // _ctx.motor.joint[leg_def::RIGHT][motor_def::HIP]->send_torque(0);
    // _ctx.motor.joint[leg_def::RIGHT][motor_def::KNEE]->send_torque(0);
}

void wl_chassis_t::_send_wheel_torque()
{
    // _ctx.motor.wheel[leg_def::LEFT]->send_torque(
    //     _ctx.data.wheel[leg_def::LEFT].direction *
    //     _ctx.data.wheel[leg_def::LEFT].out_current);
    // _ctx.motor.wheel[leg_def::RIGHT]->send_torque(
    //     _ctx.data.wheel[leg_def::RIGHT].direction *
    //     _ctx.data.wheel[leg_def::RIGHT].out_current);
    _ctx.motor.wheel[leg_def::L]->send_torque(0);
    _ctx.motor.wheel[leg_def::R]->send_torque(0);
}

} // namespace pyro