#include "pyro_wl_chassis.h"

#include "pyro_algo_common.h"
#include "pyro_dwt_drv.h"
#include "pyro_ins.h"
#include "dsp/fast_math_functions.h"

#include <algorithm>
#include <cmath>

namespace pyro
{

namespace
{
constexpr uint8_t POSE_KF_STATE_DIM       = 4;
constexpr uint8_t POSE_KF_MEASUREMENT_DIM = 3;

static_assert(pose_kf_cfg::WHEEL_TRACK_M > 0.0f,
              "Pose KF wheel track must be positive.");
static_assert(pose_kf_cfg::WHEEL_YAW_RATE_SIGN == 1.0f ||
                  pose_kf_cfg::WHEEL_YAW_RATE_SIGN == -1.0f,
              "Pose KF wheel yaw-rate sign must be +1 or -1.");
static_assert(pose_kf_cfg::WHEEL_VELOCITY_CORRELATION >= -1.0f &&
                  pose_kf_cfg::WHEEL_VELOCITY_CORRELATION <= 1.0f,
              "Pose KF wheel velocity correlation must be in [-1, 1].");
static_assert(pose_kf_cfg::WHEEL_LEFT_VELOCITY_STD_MPS > 0.0f &&
                  pose_kf_cfg::WHEEL_RIGHT_VELOCITY_STD_MPS > 0.0f &&
                  pose_kf_cfg::GYRO_YAW_RATE_STD_RADPS > 0.0f,
              "Pose KF measurement standard deviations must be positive.");

void build_pose_kf_predict_model(float dt, float *A, float *B, float *G,
                                 float *Q)
{
    const float dt_2 = dt * dt;
    const float dt_3 = dt_2 * dt;
    const float q_a = pose_kf_cfg::ACCEL_PROCESS_NOISE_DENSITY *
                      pose_kf_cfg::ACCEL_PROCESS_NOISE_DENSITY;
    const float q_alpha = pose_kf_cfg::YAW_ACCEL_PROCESS_NOISE_DENSITY *
                          pose_kf_cfg::YAW_ACCEL_PROCESS_NOISE_DENSITY;

    const float q_x = q_a * dt_3 / 3.0f;
    const float q_xv = q_a * dt_2 / 2.0f;
    const float q_v = q_a * dt;
    const float q_yaw = q_alpha * dt_3 / 3.0f;
    const float q_yaw_rate = q_alpha * dt_2 / 2.0f;
    const float q_rate = q_alpha * dt;

    A[0] = 1.0f;
    A[1] = dt;
    A[2] = 0.0f;
    A[3] = 0.0f;
    A[4] = 0.0f;
    A[5] = 1.0f;
    A[6] = 0.0f;
    A[7] = 0.0f;
    A[8] = 0.0f;
    A[9] = 0.0f;
    A[10] = 1.0f;
    A[11] = dt;
    A[12] = 0.0f;
    A[13] = 0.0f;
    A[14] = 0.0f;
    A[15] = 1.0f;

    B[0] = dt_2 / 2.0f;
    B[1] = dt;
    B[2] = 0.0f;
    B[3] = 0.0f;

    G[0] = 1.0f;
    G[1] = 0.0f;
    G[2] = 0.0f;
    G[3] = 0.0f;
    G[4] = 0.0f;
    G[5] = 1.0f;
    G[6] = 0.0f;
    G[7] = 0.0f;
    G[8] = 0.0f;
    G[9] = 0.0f;
    G[10] = 1.0f;
    G[11] = 0.0f;
    G[12] = 0.0f;
    G[13] = 0.0f;
    G[14] = 0.0f;
    G[15] = 1.0f;

    Q[0] = q_x;
    Q[1] = q_xv;
    Q[2] = 0.0f;
    Q[3] = 0.0f;
    Q[4] = q_xv;
    Q[5] = q_v;
    Q[6] = 0.0f;
    Q[7] = 0.0f;
    Q[8] = 0.0f;
    Q[9] = 0.0f;
    Q[10] = q_yaw;
    Q[11] = q_yaw_rate;
    Q[12] = 0.0f;
    Q[13] = 0.0f;
    Q[14] = q_yaw_rate;
    Q[15] = q_rate;
}

void build_pose_kf_measurement_covariance(float *R)
{
    const float left_variance = pose_kf_cfg::WHEEL_LEFT_VELOCITY_STD_MPS *
                                pose_kf_cfg::WHEEL_LEFT_VELOCITY_STD_MPS;
    const float right_variance = pose_kf_cfg::WHEEL_RIGHT_VELOCITY_STD_MPS *
                                 pose_kf_cfg::WHEEL_RIGHT_VELOCITY_STD_MPS;
    const float left_right_covariance =
        pose_kf_cfg::WHEEL_VELOCITY_CORRELATION *
        pose_kf_cfg::WHEEL_LEFT_VELOCITY_STD_MPS *
        pose_kf_cfg::WHEEL_RIGHT_VELOCITY_STD_MPS;
    const float wheel_velocity_variance =
        0.25f * (left_variance + right_variance +
                  2.0f * left_right_covariance);
    const float wheel_yaw_rate_variance =
        (left_variance + right_variance - 2.0f * left_right_covariance) /
        (pose_kf_cfg::WHEEL_TRACK_M * pose_kf_cfg::WHEEL_TRACK_M);
    const float velocity_yaw_rate_covariance =
        pose_kf_cfg::WHEEL_YAW_RATE_SIGN *
        (right_variance - left_variance) /
        (2.0f * pose_kf_cfg::WHEEL_TRACK_M);
    const float gyro_yaw_rate_variance =
        pose_kf_cfg::GYRO_YAW_RATE_STD_RADPS *
        pose_kf_cfg::GYRO_YAW_RATE_STD_RADPS;

    const float r_data[POSE_KF_MEASUREMENT_DIM * POSE_KF_MEASUREMENT_DIM] = {
        wheel_velocity_variance, velocity_yaw_rate_covariance, 0.0f,
        velocity_yaw_rate_covariance, wheel_yaw_rate_variance, 0.0f,
        0.0f, 0.0f, gyro_yaw_rate_variance,
    };
    for (uint8_t i = 0;
         i < POSE_KF_MEASUREMENT_DIM * POSE_KF_MEASUREMENT_DIM; ++i)
    {
        R[i] = r_data[i];
    }
}

void build_pose_kf_initial_covariance(float *P0)
{
    for (uint8_t i = 0; i < POSE_KF_STATE_DIM * POSE_KF_STATE_DIM; ++i)
    {
        P0[i] = 0.0f;
    }

    P0[0] = pose_kf_cfg::INITIAL_X_STD_M * pose_kf_cfg::INITIAL_X_STD_M;
    P0[5] = pose_kf_cfg::INITIAL_V_STD_MPS * pose_kf_cfg::INITIAL_V_STD_MPS;
    P0[10] =
        pose_kf_cfg::INITIAL_YAW_STD_RAD * pose_kf_cfg::INITIAL_YAW_STD_RAD;
    P0[15] = pose_kf_cfg::INITIAL_YAW_RATE_STD_RADPS *
             pose_kf_cfg::INITIAL_YAW_RATE_STD_RADPS;
}
} // namespace

wl_chassis_t::wl_chassis_t()
    : module_base_t("wl_chassis"), _pose_kf(POSE_KF_STATE_DIM, 1,
                                             POSE_KF_MEASUREMENT_DIM,
                                             POSE_KF_STATE_DIM)
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
    _ctx.data._dt                             = pose_kf_cfg::NOMINAL_DT_S;

    _init_imu_to_body_rotation();
    return _init_pose_kf();
}

void wl_chassis_t::_init_imu_to_body_rotation()
{
    const float sin_roll = arm_sin_f32(pose_kf_cfg::IMU_MOUNT_ROLL_RAD);
    const float cos_roll = arm_cos_f32(pose_kf_cfg::IMU_MOUNT_ROLL_RAD);
    const float sin_pitch = arm_sin_f32(pose_kf_cfg::IMU_MOUNT_PITCH_RAD);
    const float cos_pitch = arm_cos_f32(pose_kf_cfg::IMU_MOUNT_PITCH_RAD);
    const float sin_yaw = arm_sin_f32(pose_kf_cfg::IMU_MOUNT_YAW_RAD);
    const float cos_yaw = arm_cos_f32(pose_kf_cfg::IMU_MOUNT_YAW_RAD);

    _imu_to_body_rotation[0] = cos_yaw * cos_pitch;
    _imu_to_body_rotation[1] =
        cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
    _imu_to_body_rotation[2] =
        cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;
    _imu_to_body_rotation[3] = sin_yaw * cos_pitch;
    _imu_to_body_rotation[4] =
        sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
    _imu_to_body_rotation[5] =
        sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;
    _imu_to_body_rotation[6] = -sin_pitch;
    _imu_to_body_rotation[7] = cos_pitch * sin_roll;
    _imu_to_body_rotation[8] = cos_pitch * cos_roll;
}

void wl_chassis_t::_transform_imu_feedback(const float *imu_euler_rad,
                                           const float *imu_gyro,
                                           const float *imu_accel)
{
    const float sin_roll = arm_sin_f32(imu_euler_rad[2]);
    const float cos_roll = arm_cos_f32(imu_euler_rad[2]);
    const float sin_pitch = arm_sin_f32(imu_euler_rad[1]);
    const float cos_pitch = arm_cos_f32(imu_euler_rad[1]);
    const float sin_yaw = arm_sin_f32(imu_euler_rad[0]);
    const float cos_yaw = arm_cos_f32(imu_euler_rad[0]);

    const float imu_to_navigation[9] = {
        cos_yaw * cos_pitch,
        cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll,
        cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll,
        sin_yaw * cos_pitch,
        sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll,
        sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll,
        -sin_pitch,
        cos_pitch * sin_roll,
        cos_pitch * cos_roll,
    };

    // R_B^N = R_I^N * (R_I^B)^T.
    float body_to_navigation[9];
    for (uint8_t row = 0; row < 3; ++row)
    {
        for (uint8_t col = 0; col < 3; ++col)
        {
            body_to_navigation[row * 3 + col] =
                imu_to_navigation[row * 3] *
                    _imu_to_body_rotation[col * 3] +
                imu_to_navigation[row * 3 + 1] *
                    _imu_to_body_rotation[col * 3 + 1] +
                imu_to_navigation[row * 3 + 2] *
                    _imu_to_body_rotation[col * 3 + 2];
        }
    }

    const float horizontal_heading_norm =
        sqrtf(body_to_navigation[0] * body_to_navigation[0] +
              body_to_navigation[3] * body_to_navigation[3]);
    _ctx.data.ins.euler_rad[0] =
        atan2f(body_to_navigation[3], body_to_navigation[0]);
    _ctx.data.ins.euler_rad[1] =
        atan2f(-body_to_navigation[6], horizontal_heading_norm);
    _ctx.data.ins.euler_rad[2] =
        atan2f(body_to_navigation[7], body_to_navigation[8]);

    const float gyro_imu_xyz[3] = {
        imu_gyro[2],
        imu_gyro[1],
        imu_gyro[0],
    };
    float gyro_body_xyz[3];
    for (uint8_t row = 0; row < 3; ++row)
    {
        _ctx.data.ins.accel[row] =
            _imu_to_body_rotation[row * 3] * imu_accel[0] +
            _imu_to_body_rotation[row * 3 + 1] * imu_accel[1] +
            _imu_to_body_rotation[row * 3 + 2] * imu_accel[2];
        gyro_body_xyz[row] =
            _imu_to_body_rotation[row * 3] * gyro_imu_xyz[0] +
            _imu_to_body_rotation[row * 3 + 1] * gyro_imu_xyz[1] +
            _imu_to_body_rotation[row * 3 + 2] * gyro_imu_xyz[2];
    }

    // Preserve the existing [yaw(Z), pitch(Y), roll(X)] storage order.
    _ctx.data.ins.gyro[0] = gyro_body_xyz[2];
    _ctx.data.ins.gyro[1] = gyro_body_xyz[1];
    _ctx.data.ins.gyro[2] = gyro_body_xyz[0];
}

status_t wl_chassis_t::_init_pose_kf()
{
    build_pose_kf_predict_model(pose_kf_cfg::NOMINAL_DT_S, _pose_kf_model.A,
                                _pose_kf_model.B, _pose_kf_model.G,
                                _pose_kf_model.Q);

    float H[POSE_KF_MEASUREMENT_DIM * POSE_KF_STATE_DIM] = {
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 0.0f, 1.0f,
    };
    float R[POSE_KF_MEASUREMENT_DIM * POSE_KF_MEASUREMENT_DIM];
    float x0[POSE_KF_STATE_DIM] = {};
    float P0[POSE_KF_STATE_DIM * POSE_KF_STATE_DIM];
    build_pose_kf_measurement_covariance(R);
    build_pose_kf_initial_covariance(P0);

    return _pose_kf.init(_pose_kf_model.A, _pose_kf_model.B, H,
                         _pose_kf_model.G, _pose_kf_model.Q, R, x0, P0);
}

void wl_chassis_t::_update_pose_kf(float dt)
{
    pose_kf_data_t &pose = _ctx.data.pose_kf;

    const float left_velocity =
        WHEEL_RADIUS * _ctx.data.wheel[leg_def::L].current_radps;
    const float right_velocity =
        WHEEL_RADIUS * _ctx.data.wheel[leg_def::R].current_radps;
    const float wheel_velocity = 0.5f * (left_velocity + right_velocity);
    const float wheel_yaw_rate = pose_kf_cfg::WHEEL_YAW_RATE_SIGN *
                                 (right_velocity - left_velocity) /
                                 pose_kf_cfg::WHEEL_TRACK_M;

    const float pitch = _ctx.data.ins.euler_rad[1];
    const float roll = _ctx.data.ins.euler_rad[2];
    const float sin_pitch = arm_sin_f32(pitch);
    const float cos_pitch = arm_cos_f32(pitch);
    const float sin_roll = arm_sin_f32(roll);
    const float cos_roll = arm_cos_f32(roll);
    const float gyro_yaw_rate =
        (cos_pitch > pose_kf_cfg::MIN_ABS_COS_PITCH ||
         cos_pitch < -pose_kf_cfg::MIN_ABS_COS_PITCH)
            ? (_ctx.data.ins.gyro[1] * sin_roll +
               _ctx.data.ins.gyro[0] * cos_roll) /
                  cos_pitch
            : _ctx.data.ins.gyro[0];

    const float accel_forward_y =
        cos_pitch * _ctx.data.ins.accel[0] +
        sin_pitch * sin_roll * _ctx.data.ins.accel[1] +
        sin_pitch * cos_roll * _ctx.data.ins.accel[2] +
        gyro_yaw_rate * gyro_yaw_rate * pose_kf_cfg::IMU_FORWARD_OFFSET_M;

    pose.wheel_velocity = wheel_velocity;
    pose.wheel_yaw_rate = wheel_yaw_rate;
    pose.gyro_yaw_rate = gyro_yaw_rate;
    pose.accel_forward_y = accel_forward_y;

    if (!pose.initialized)
    {
        _reset_pose_kf();
        return;
    }

    build_pose_kf_predict_model(dt, _pose_kf_model.A, _pose_kf_model.B,
                                _pose_kf_model.G, _pose_kf_model.Q);

    const status_t model_ret = _pose_kf.set_predict_model(
        _pose_kf_model.A, _pose_kf_model.B, _pose_kf_model.G,
        _pose_kf_model.Q);
    if (model_ret != PYRO_OK)
    {
        ++pose.update_errors;
        return;
    }

    float measure[POSE_KF_MEASUREMENT_DIM] = {
        wheel_velocity,
        wheel_yaw_rate,
        gyro_yaw_rate,
    };
    float control[1] = {accel_forward_y};
    float estimate[POSE_KF_STATE_DIM];
    const status_t update_ret = _pose_kf.update(measure, control, estimate);
    if (update_ret != PYRO_OK)
    {
        ++pose.update_errors;
        return;
    }

    pose.x = estimate[0];
    pose.v = estimate[1];
    pose.yaw = estimate[2];
    pose.yaw_rate = estimate[3];
}

void wl_chassis_t::_reset_pose_kf()
{
    pose_kf_data_t &pose = _ctx.data.pose_kf;
    float x0[POSE_KF_STATE_DIM] = {
        0.0f,
        pose.wheel_velocity,
        _ctx.data.ins.euler_rad[0],
        pose.gyro_yaw_rate,
    };
    float P0[POSE_KF_STATE_DIM * POSE_KF_STATE_DIM];
    build_pose_kf_initial_covariance(P0);

    if (_pose_kf.reset(x0, P0) != PYRO_OK)
    {
        ++pose.update_errors;
        return;
    }

    pose.x = x0[0];
    pose.v = x0[1];
    pose.yaw = x0[2];
    pose.yaw_rate = x0[3];
    pose.initialized = true;

    state_vec_t &state = _ctx.data.current_state;
    state.x = pose.x;
    state.dot_x = pose.v;
    state.psi = pose.yaw;
    state.dot_psi = pose.yaw_rate;
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
            const float raw_rad =
                motor->get_current_position() +
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
    float imu_euler_rad[3];
    float imu_gyro[3];
    float imu_accel[3];
    ins->get_rads_n(&imu_euler_rad[0], &imu_euler_rad[1],
                    &imu_euler_rad[2]);
    ins->get_gyro_b(&imu_gyro[0], &imu_gyro[1], &imu_gyro[2]);
    ins->get_accel_without_g_b(&imu_accel[0], &imu_accel[1], &imu_accel[2]);
    _transform_imu_feedback(imu_euler_rad, imu_gyro, imu_accel);

    // 7. Fuse wheel and IMU feedback in the heading frame.
    _update_pose_kf(_ctx.data._dt);

    // 8. State vector feedback.
    state_vec_t &state = _ctx.data.current_state;

    state.x            = _ctx.data.pose_kf.x;
    state.dot_x        = _ctx.data.pose_kf.v;
    state.psi          = _ctx.data.pose_kf.yaw;
    state.dot_psi      = _ctx.data.pose_kf.yaw_rate;
    state.theta        = _ctx.data.ins.euler_rad[1];
    state.dot_theta    = _ctx.data.ins.gyro[1];
    state.phi          = _ctx.data.ins.euler_rad[2];
    state.dot_phi      = _ctx.data.ins.gyro[2];
    state.beta1        = _ctx.data.leg[leg_def::L].current_leg_rad - PI / 2 -
                  _ctx.data.ins.euler_rad[1];
    state.dot_beta1 =
        _ctx.data.leg[leg_def::L].current_leg_radps - _ctx.data.ins.gyro[1];
    state.beta2 = _ctx.data.leg[leg_def::R].current_leg_rad - PI / 2 -
                  _ctx.data.ins.euler_rad[1];
    state.dot_beta2 =
        _ctx.data.leg[leg_def::R].current_leg_radps - _ctx.data.ins.gyro[1];

    const float cos_beta_1 = arm_cos_f32(state.beta1);
    const float sin_beta_1 = arm_sin_f32(state.beta1);
    const float cos_beta_2 = arm_cos_f32(state.beta2);
    const float sin_beta_2 = arm_sin_f32(state.beta2);

    state.h =
        0.5f * (_ctx.data.leg[leg_def::L].current_leg_length * cos_beta_1 +
                _ctx.data.leg[leg_def::R].current_leg_length * cos_beta_2);
    state.dot_h =
        0.5f * (_ctx.data.leg[leg_def::L].current_leg_speed * cos_beta_1 -
                _ctx.data.leg[leg_def::L].current_leg_length * sin_beta_1 *
                    state.dot_beta1 +
                _ctx.data.leg[leg_def::R].current_leg_speed * cos_beta_2 -
                _ctx.data.leg[leg_def::R].current_leg_length * sin_beta_2 *
                    state.dot_beta2);
}


void wl_chassis_t::_fsm_execute()
{
    static int last_chassis_reset_times = 0;
    if(_current_cmd.reset_chassis_times != last_chassis_reset_times)
    {
        _ctx.data.flag.leg_is_should_restart = false;
    }
    last_chassis_reset_times = _current_cmd.reset_chassis_times;
    if (_current_cmd.mode == cmd_base_t::mode_t::ACTIVE && (!_ctx.data.flag.leg_is_should_restart))
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
        leg.L_wp               = evaluate_polynomial_ascending(
            leg.current_leg_length, L_WP_POLY_COEF, L_WP_POLY_DEGREE);
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

void wl_chassis_t::_manual_control()
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

void wl_chassis_t::_gain_calculate()
{
    // const float norm_delta_L =
    //     std::clamp((_ctx.data.leg[leg_def::L].current_leg_length -
    //                 _ctx.data.leg[leg_def::R].current_leg_length) *
    //                    (1.0f / 0.2f),
    //                -1.0f, 1.0f);
    const float norm_L1 =
        std::clamp((_ctx.data.leg[leg_def::L].current_leg_length -
                    0.5f * (MAX_LEG_LENGTH + MIN_LEG_LENGTH)) *
                       (1.0f / (0.5f * (MAX_LEG_LENGTH - MIN_LEG_LENGTH))),
                   -1.0f, 1.0f);
    const float norm_L2 =
        std::clamp((_ctx.data.leg[leg_def::R].current_leg_length -
                    0.5f * (MAX_LEG_LENGTH + MIN_LEG_LENGTH)) *
                       (1.0f / (0.5f * (MAX_LEG_LENGTH - MIN_LEG_LENGTH))),
                   -1.0f, 1.0f);

    for (uint32_t input = 0; input < INPUT_DIM; ++input)
    {
        for (uint32_t state = 0; state < STATE_DIM; ++state)
        {
            float p_terms[K_POLY_DEGREE + 1];
            for (uint32_t p = 0; p <= K_POLY_DEGREE; ++p)
            {
                p_terms[p] = evaluate_polynomial_ascending(
                    norm_L2, K_POLY_COEF[input][state][p], K_POLY_DEGREE);
            }
            _ctx.data.K[input][state] =
                evaluate_polynomial_ascending(norm_L1, p_terms, K_POLY_DEGREE);
        }
    }

    _ctx.data.U0[lqr_input_def::F_L1] =
        evaluate_polynomial_ascending(norm_L1, FL_U0_POLY_COEF, U0_POLY_DEGREE);
    _ctx.data.U0[lqr_input_def::F_L2] =
        evaluate_polynomial_ascending(norm_L2, FL_U0_POLY_COEF, U0_POLY_DEGREE);
    // _ctx.data.target_state.beta1 = evaluate_polynomial_ascending(
    //     _ctx.data.leg[leg_def::L].current_leg_length, BETA_TRIM_POLY_COEF,
    //     BETA_TRIM_POLY_DEGREE);
    // _ctx.data.target_state.beta2 = evaluate_polynomial_ascending(
    //     _ctx.data.leg[leg_def::R].current_leg_length, BETA_TRIM_POLY_COEF,
    //     BETA_TRIM_POLY_DEGREE);
}

void wl_chassis_t::_balance_control()
{
    float error[STATE_DIM];
    for (uint8_t state = 0; state < STATE_DIM; ++state)
    {
        error[state] = _ctx.data.target_state.data[state] -
                       _ctx.data.current_state.data[state];
    }
    error[lqr_state_def::PSI] =
        loop_fp32_constrain(error[lqr_state_def::PSI], -PI, PI);

    // calculate control data

    for (uint8_t input = 0; input < INPUT_DIM; ++input)
    {
        _ctx.data.control.data[input] = _ctx.data.U0[input];
        for (uint8_t state = 0; state < STATE_DIM; ++state)
        {
            _ctx.data.control.data[input] +=
                _ctx.data.K[input][state] * error[state];
        }
    }

    _ctx.data.control.T_w1 =
        std::clamp(_ctx.data.control.T_w1, -MAX_T_W, MAX_T_W);
    _ctx.data.control.T_w2 =
        std::clamp(_ctx.data.control.T_w2, -MAX_T_W, MAX_T_W);
    _ctx.data.control.T_p1 =
        std::clamp(_ctx.data.control.T_p1, -MAX_T_P, MAX_T_P);
    _ctx.data.control.T_p2 =
        std::clamp(_ctx.data.control.T_p2, -MAX_T_P, MAX_T_P);
    _ctx.data.control.F_l1 =
        std::clamp(_ctx.data.control.F_l1, -MAX_F_L, MAX_F_L);
    _ctx.data.control.F_l2 =
        std::clamp(_ctx.data.control.F_l2, -MAX_F_L, MAX_F_L);

    _ctx.data.leg[leg_def::L].out_T_p   = _ctx.data.control.T_p1;
    _ctx.data.leg[leg_def::R].out_T_p   = _ctx.data.control.T_p2;
    _ctx.data.leg[leg_def::L].out_F_L   = _ctx.data.control.F_l1;
    _ctx.data.leg[leg_def::R].out_F_L   = _ctx.data.control.F_l2;

    _ctx.data.wheel[leg_def::L].out_T_w = _ctx.data.control.T_w1;
    _ctx.data.wheel[leg_def::R].out_T_w = _ctx.data.control.T_w2;
    for (auto &wheel : _ctx.data.wheel)
    {
        wheel.out_current =
            std::clamp(wheel.out_T_w * (1 / K_t), -MAX_CURRENT, MAX_CURRENT);
    }
}


void wl_chassis_t::_vmc_trans_v2j()
{
    constexpr float MAX_MOTOR_TORQUE = 40.0f;

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

void wl_chassis_t::_send_joint_torque() const
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

void wl_chassis_t::_send_wheel_torque() const
{
    _ctx.motor.wheel[leg_def::L]->send_torque(
        _ctx.data.wheel[leg_def::L].direction *
        _ctx.data.wheel[leg_def::L].out_current);
    _ctx.motor.wheel[leg_def::R]->send_torque(
        _ctx.data.wheel[leg_def::R].direction *
        _ctx.data.wheel[leg_def::R].out_current);

    // _ctx.motor.wheel[leg_def::L]->send_torque(0);
    // _ctx.motor.wheel[leg_def::R]->send_torque(0);
}

} // namespace pyro
