#include "pyro_wl_chassis.h"

#include "pyro_algo_common.h"
#include "pyro_dwt_drv.h"
#include "pyro_ins.h"
#include "dsp/fast_math_functions.h"

#include <algorithm>

namespace pyro
{

wl_chassis_t::wl_chassis_t() : module_base_t("wl_chassis")
{
}

status_t wl_chassis_t::_init()
{
    _ctx                                      = {};
    _ctx.motor                                = _module_deps.motor;
    _ctx.pid                                  = _module_deps.pid;

    _current_cmd.delta_leg_length[leg_def::L] = 0.0f;
    _current_cmd.delta_leg_length[leg_def::R] = 0.0f;
    _current_cmd.delta_leg_rad[leg_def::L]    = 0.0f;
    _current_cmd.delta_leg_rad[leg_def::R]    = 0.0f;

    _ctx.data.leg[leg_def::L].direction       = LEFT_LEG_DIRECTION;
    _ctx.data.leg[leg_def::R].direction       = RIGHT_LEG_DIRECTION;

    _ctx.data.wheel[leg_def::L].direction     = LEFT_WHEEL_DIRECTION;
    _ctx.data.wheel[leg_def::R].direction     = RIGHT_WHEEL_DIRECTION;

    return PYRO_OK;
}

void wl_chassis_t::_update_feedback()
{
    // 1. Update timing.
    static uint32_t dwt_cnt = 0;
    const float measured_dt = dwt_drv_t::get_delta_t(&dwt_cnt);
    _ctx.data._dt =
        (measured_dt > 1.0e-5f && measured_dt < 0.1f) ? measured_dt : 0.001f;

    // 2. Update all motor feedback before reading cached values.
    for (uint8_t leg = 0; leg < 2; ++leg)
    {
        for (uint8_t joint = 0; joint < 2; ++joint)
        {
            _ctx.motor.joint[leg][joint]->update_feedback();
        }
        _ctx.motor.wheel[leg]->update_feedback();
    }

    // 3. Read and organize all leg-related feedback.
    constexpr float JOINT_POSITION_OFFSET[2][2] = {
        {LEFT_HIP_OFFSET, LEFT_KNEE_OFFSET},
        {RIGHT_HIP_OFFSET, RIGHT_KNEE_OFFSET},
    };

    for (uint8_t leg = 0; leg < 2; ++leg)
    {
        leg_ctx_t &leg_ctx     = _ctx.data.leg[leg];
        wheel_ctx_t &wheel_ctx = _ctx.data.wheel[leg];

        for (uint8_t joint = 0; joint < 2; ++joint)
        {
            motor_base_t *motor = _ctx.motor.joint[leg][joint];
            const float raw_rad = motor->get_current_position() +
                                  leg_ctx.direction * JOINT_POSITION_OFFSET[leg][joint];

            leg_ctx.current_joint_rad[joint] =
                leg_ctx.direction * loop_fp32_constrain(raw_rad, -PI, PI);
            leg_ctx.current_joint_torque[joint] =
                leg_ctx.direction * motor->get_current_torque();
            leg_ctx.current_joint_radps[joint] =
                leg_ctx.direction * motor->get_current_rotate();
        }

        motor_base_t *wheel_motor = _ctx.motor.wheel[leg];
        wheel_ctx.current_radps   = wheel_motor->get_current_rotate() *
                                  wheel_ctx.direction * rec_reduction_ratio;
        wheel_ctx.current_T_w = wheel_motor->get_current_torque() *
                                wheel_ctx.direction * rec_reduction_ratio;
    }

    // 4. Convert joint-space feedback to virtual-mechanism feedback.
    _vmc_trans_j2v();

    // 5. Update wheel odometry.
    _ctx.data.odom.real_dot_x[1] = _ctx.data.odom.real_dot_x[0];
    _ctx.data.odom.real_dot_x[0] = 0.5f * WHEEL_RADIUS *
                                   (_ctx.data.wheel[leg_def::L].current_radps +
                                    _ctx.data.wheel[leg_def::R].current_radps);
    _ctx.data.odom.real_x +=
        0.5f * (_ctx.data.odom.real_dot_x[0] + _ctx.data.odom.real_dot_x[1]) *
        _ctx.data._dt;

    // 6. Read IMU feedback.
    auto ins = ins_drv_t::get_instance();
    ins->get_rads_n(&_ctx.data.ins.euler_rad[0], &_ctx.data.ins.euler_rad[1],
                    &_ctx.data.ins.euler_rad[2]);
    ins->get_gyro_b(&_ctx.data.ins.gyro[0], &_ctx.data.ins.gyro[1],
                    &_ctx.data.ins.gyro[2]);


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
        const float raw_2_theta = leg.current_joint_rad[joint_def::KNEE] -
                                  leg.current_joint_rad[joint_def::HIP];
        const float theta     = loop_fp32_constrain(raw_2_theta, -PI, PI) / 2;
        const float dot_theta = (leg.current_joint_radps[joint_def::KNEE] -
                                 leg.current_joint_radps[joint_def::HIP]) /
                                2;
        const float sin_theta  = arm_sin_f32(theta);
        const float cos_theta  = arm_cos_f32(theta);
        float OH               = OJ5 * cos_theta;
        float HJ5              = OJ5 * sin_theta;
        float HJ4              = sqrt(J4J5 * J4J5 - HJ5 * HJ5);
        float OJ4              = OH + HJ4;
        leg.J_L                = -OJ8 * sin_theta * (OJ4 / HJ4);
        leg.current_leg_length = OJ4 * OJ8 / OJ5;
        leg.L_wp = evaluate_polynomial_ascending(leg.current_leg_length, L_WP_POLY_COEF,
                                       L_WP_POLY_DEGREE);
        leg.current_leg_speed = dot_theta * leg.J_L;

        const float raw_beta  = leg.current_joint_rad[joint_def::HIP] + theta;
        const float beta      = loop_fp32_constrain(raw_beta, -PI, PI);
        leg.current_leg_rad   = beta;
        leg.current_leg_radps = (leg.current_joint_radps[joint_def::KNEE] +
                                 leg.current_joint_radps[joint_def::HIP]) /
                                2;
        leg.current_F_L = (leg.current_joint_torque[joint_def::KNEE] -
                           leg.current_joint_torque[joint_def::HIP]) /
                          leg.J_L;
        leg.current_T_p = leg.current_joint_torque[joint_def::KNEE] +
                          leg.current_joint_torque[joint_def::HIP];
    }
}

void wl_chassis_t::_manual_calculate()
{
    _ctx.data.leg[leg_def::L].out_F_L =
        _ctx.pid.leg_length[leg_def::L]->calculate(
            _ctx.data.leg[leg_def::L].target_leg_length,
            _ctx.data.leg[leg_def::L].current_leg_length,
            _ctx.data.leg[leg_def::L].current_leg_speed);
    _ctx.data.leg[leg_def::L].out_T_p = _ctx.pid.leg_rad[leg_def::L]->calculate(
        0.0f, _ctx.data.leg[leg_def::L].error_leg_rad,
        _ctx.data.leg[leg_def::L].current_leg_radps);
    _ctx.data.leg[leg_def::R].out_F_L =
        _ctx.pid.leg_length[leg_def::R]->calculate(
            _ctx.data.leg[leg_def::R].target_leg_length,
            _ctx.data.leg[leg_def::R].current_leg_length,
            _ctx.data.leg[leg_def::R].current_leg_speed);
    _ctx.data.leg[leg_def::R].out_T_p = _ctx.pid.leg_rad[leg_def::R]->calculate(
        0.0f, _ctx.data.leg[leg_def::R].error_leg_rad,
        _ctx.data.leg[leg_def::R].current_leg_radps);
}

void wl_chassis_t::_balance_calculate()
{
    for (uint8_t leg = 0; leg < 2; ++leg)
    {
        leg_ctx_t &leg_ctx = _ctx.data.leg[leg];

        // K[leg][0] -> T_s, K[leg][1] -> T_p.
        float error[6];
        _ctx.data.control[leg].T_w = 0.0f;
        _ctx.data.control[leg].T_p = 0.0f;
        for (uint8_t state = 0; state < 6; ++state)
        {
            error[state] = _ctx.data.target_state.data[state] -
                           _ctx.data.current_state[leg].data[state];
            _ctx.data.control[leg].T_w +=
                _ctx.data.K[leg][0][state] * error[state];
            _ctx.data.control[leg].T_p +=
                _ctx.data.K[leg][1][state] * error[state];
        }

        // Keep leg-length
        leg_ctx.out_F_L = _ctx.pid.leg_length[leg]->calculate(
            leg_ctx.target_leg_length, leg_ctx.current_leg_length,
            leg_ctx.current_leg_speed);

        leg_ctx.out_T_p              = _ctx.data.control[leg].T_p;
        _ctx.data.wheel[leg].out_T_w = _ctx.data.control[leg].T_w;
        _ctx.data.wheel[leg].out_current = std::clamp(_ctx.data.wheel[leg].out_T_w * K_t,-20.0f,20.0f);
    }
}

void wl_chassis_t::_vmc_trans_v2j()
{
    constexpr float MAX_MOTOR_TORQUE = 20.0f;

    for (auto &leg : _ctx.data.leg)
    {
        float tau_sum                        = leg.out_T_p;
        float tau_diff                       = leg.out_F_L * leg.J_L;
        leg.out_joint_torque[joint_def::HIP] = std::clamp(
            (tau_sum - tau_diff) / 2, -MAX_MOTOR_TORQUE, MAX_MOTOR_TORQUE);
        leg.out_joint_torque[joint_def::KNEE] = std::clamp(
            (tau_sum + tau_diff) / 2, -MAX_MOTOR_TORQUE, MAX_MOTOR_TORQUE);
    }
}

void wl_chassis_t::_send_joint_torque()
{
    _ctx.motor.joint[leg_def::L][joint_def::HIP]->send_torque(
        _ctx.data.leg[leg_def::L].direction *
        _ctx.data.leg[leg_def::L].out_joint_torque[joint_def::HIP]);
    _ctx.motor.joint[leg_def::L][joint_def::KNEE]->send_torque(
        _ctx.data.leg[leg_def::L].direction *
        _ctx.data.leg[leg_def::L].out_joint_torque[joint_def::KNEE]);
    _ctx.motor.joint[leg_def::R][joint_def::HIP]->send_torque(
        _ctx.data.leg[leg_def::R].direction *
        _ctx.data.leg[leg_def::R].out_joint_torque[joint_def::HIP]);
    _ctx.motor.joint[leg_def::R][joint_def::KNEE]->send_torque(
        _ctx.data.leg[leg_def::R].direction *
        _ctx.data.leg[leg_def::R].out_joint_torque[joint_def::KNEE]);

    // _ctx.motor.joint[leg_def::L][joint_def::HIP]->send_torque(0);
    // _ctx.motor.joint[leg_def::L][joint_def::KNEE]->send_torque(0);
    // _ctx.motor.joint[leg_def::R][joint_def::HIP]->send_torque(0);
    // _ctx.motor.joint[leg_def::R][joint_def::KNEE]->send_torque(0);
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