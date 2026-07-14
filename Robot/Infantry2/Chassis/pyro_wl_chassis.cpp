/*
 * @Author: Vod vod0575@outlook
 * @Date: 2026-02-06 15:27:37
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-05-29 04:42:03
 * @Description: Wheel-Legged Chassis module source file / 杞吙搴曠洏妯″潡婧愭枃浠? *
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved.
 */

#include "pyro_wl_chassis.h"
#include "pyro_dwt_drv.h"
#include "pyro_algo_common.h"
#include "pyro_vofa.h"
#include "pyro_referee.h"
#include <cmath>

#define WHEEL_DISTANCE 0.424f             /* Distance between left and right wheels (m) / 宸﹀彸杞窛 (m) */
#define SUPPORT_FORCE_ACC_LPF_RC 0.01f   /* LPF time constant for vertical acceleration / 鍨傜洿鍔犻€熷害浣庨€氭护娉㈡椂闂村父鏁?*/

/* IMU offset from yaw rotation center (midpoint of two wheels) along body x-axis.
   Positive = IMU is in front of wheel axis. Measure and adjust this value.
   IMU 鐩稿鍋忚埅鏃嬭浆涓績锛堜袱杞酱绾夸腑鐐癸級娌胯溅浣?X 杞寸殑鍋忕Щ閲忋€傚墠鍋忎负姝ｃ€?*/
#define IMU_OFFSET_X  0.2f

namespace pyro
{
float time;
float last_time;
float test_wl_chassis_power_cap{};
float test_wl_cap_power_cap{};
float test_wl_cap_vot{};
supercap_drv_t::cap_feedback_t test_wl_cap_feedback{};

static float wl_power_predict(const power_fit_params_t &params,
                              const float target_cmd,
                              const float uncontrolled_cmd,
                              const float rpm,
                              const float temp)
{
    float temp_factor = 1.0f + params.alpha * (temp - 20.0f);
    if (temp_factor < 1.0f)
    {
        temp_factor = 1.0f;
    }

    const float total_cmd = target_cmd + uncontrolled_cmd;
    const float copper = params.k2 * temp_factor * total_cmd * total_cmd;
    const float controlled_mechanical = params.k1 * rpm * target_cmd;
    const float uncontrolled_mechanical = params.k1 * rpm * uncontrolled_cmd;
    const float static_loss = params.k3 * rpm * rpm +
                              params.k4 * fabsf(rpm) +
                              params.k5;

    return copper +
           ((controlled_mechanical > 0.0f) ? controlled_mechanical : 0.0f) +
           ((uncontrolled_mechanical > 0.0f) ? uncontrolled_mechanical : 0.0f) +
           static_loss;
}

/* Constructor / 鏋勯€犲嚱鏁帮紝璁惧畾浠诲姟鍚嶇О鍙婂爢鏍堝ぇ灏忥紝鍒濆鍖?2 缁勮疆閫熷崱灏旀浖婊ゆ尝鍣?*/
wl_chassis_t::wl_chassis_t() : module_base_t("wl_chassis", 0, 2048),
    _wheel_kf{kf_t(3, 1, 3, 2), kf_t(3, 1, 3, 2)}
{
}

/* Get current leg angle / 鑾峰彇褰撳墠鏋佸潗鏍囨憜瑙?alpha */
status_t wl_chassis_t::get_cur_angle(float *r_angle, float *l_angle)
{
    if(!l_angle || !r_angle)
    {
        return PYRO_PARAM_ERROR;
    }
    *r_angle = _ctx.data.leg[R].alpha;
    *l_angle = _ctx.data.leg[L].alpha;
    return PYRO_OK;
}

/* Get current leg angular velocity / 鑾峰彇褰撳墠鏋佸潗鏍囨憜瑙掑彉鍖栫巼 d_alpha */
status_t wl_chassis_t::get_cur_d_angle(float *r_d_angle, float *l_d_angle)
{
    if(!l_d_angle || !r_d_angle)
    {
        return PYRO_PARAM_ERROR;
    }
    *r_d_angle = _ctx.data.leg[R].d_alpha;
    *l_d_angle = _ctx.data.leg[L].d_alpha;
    return PYRO_OK;
}

/* Get current leg length / 鑾峰彇褰撳墠鏋佽酱鍗婂緞锛堟憜鑵块暱搴︼級 l */
status_t wl_chassis_t::get_cur_length(float *r_leg, float *l_leg)
{
    if(!l_leg || !r_leg)
    {
        return PYRO_PARAM_ERROR;
    }
    *r_leg = _ctx.data.leg[R].l;
    *l_leg = _ctx.data.leg[L].l;
    return PYRO_OK;
}

/* Get current virtual hip torque / 鑾峰彇褰撳墠铏氭嫙鎽嗗姩鎵煩 F[1] */
status_t wl_chassis_t::get_cur_p_torque(float *r_torque, float *l_torque)
{
    if(!r_torque || !l_torque)
    {
        return PYRO_PARAM_ERROR;
    }
    *r_torque = _ctx.data.leg[R].F[1];
    *l_torque = _ctx.data.leg[L].F[1];
    return PYRO_OK;
}

/* Get current INS Yaw angle / 鑾峰彇褰撳墠鎯鍋忚埅瑙?*/
status_t wl_chassis_t::get_cur_ins_yaw(float* temp_yaw)
{
    if(!temp_yaw)
    {
        return PYRO_PARAM_ERROR;
    }
    *temp_yaw = _ctx.data.yaw;
    return PYRO_OK;
}

/* Get current displacement error bias / 鑾峰彇褰撳墠浣嶇Щ宸亸宸噺 */
status_t wl_chassis_t::get_cur_x_bias(float* r_x_bias, float* l_x_bias)
{
    if(!r_x_bias || !l_x_bias)
    {
        return PYRO_PARAM_ERROR;
    }
    *r_x_bias = _ctx.data.leg[R].x_bias;
    *l_x_bias = _ctx.data.leg[L].x_bias;
    return PYRO_OK;
}

/* Get current tilt angle bias / 鑾峰彇褰撳墠鎽嗚吙鍋忚鍋忓樊閲?*/
status_t wl_chassis_t::get_cur_beta_bias(float* r_beta_bias, float* l_beta_bias)
{
    if(!r_beta_bias || !l_beta_bias)
    {
        return PYRO_PARAM_ERROR;
    }
    *r_beta_bias = _ctx.data.leg[R].beta_bias;
    *l_beta_bias = _ctx.data.leg[L].beta_bias;
    return PYRO_OK;
}

/* Get current pitch angle bias / 鑾峰彇褰撳墠搴曠洏淇话瑙掑亸宸噺 */
status_t wl_chassis_t::get_cur_gamma_bias(float* gamma_bias)
{
    if(!gamma_bias)
    {
        return PYRO_PARAM_ERROR;
    }
    *gamma_bias = _ctx.data.leg[R].gamma_bias;
    return PYRO_OK;
}

/* Get sub-state FSM completion flags / 鏌ヨ鍚勫瓙鐘舵€佸氨缁爣蹇?*/
uint8_t wl_chassis_t::get_status_flag(wl_cmd_t::active_mode_t mode)
{
    switch(mode)
    {
        case wl_cmd_t::READY:
            return _ctx.data.active_mode_flag.ready;
        case wl_cmd_t::TEST:
            return _ctx.data.active_mode_flag.test;
        case wl_cmd_t::REVERSE:
            return _ctx.data.active_mode_flag.reverse;
        case wl_cmd_t::OVER_STEP:
            return _ctx.data.active_mode_flag.over_step;
        case wl_cmd_t::OVER_STEP_RESET:
            return _ctx.data.active_mode_flag.over_step_reset;
        case wl_cmd_t::NORMAL:
            return _ctx.data.active_mode_flag.normal;
        case wl_cmd_t::CONTROL:
            return _ctx.data.active_mode_flag.control;
        default:
            return 0;
    }
}

/* Clear FSM completion flags / 娓呴櫎鍚勫瓙鐘舵€佸氨缁爣蹇?*/
status_t wl_chassis_t::clear_status_flag(wl_cmd_t::active_mode_t mode)
{
    switch(mode)
    {
        case wl_cmd_t::READY:
            _ctx.data.active_mode_flag.ready = 0;
            break;
        case wl_cmd_t::TEST:
            _ctx.data.active_mode_flag.test = 0;
            break;
        case wl_cmd_t::REVERSE:
            _ctx.data.active_mode_flag.reverse = 0;
            break;
        case wl_cmd_t::OVER_STEP:
            _ctx.data.active_mode_flag.over_step = 0;
            break;
        case wl_cmd_t::OVER_STEP_RESET:
            _ctx.data.active_mode_flag.over_step_reset = 0;
            break;
        case wl_cmd_t::NORMAL:
            _ctx.data.active_mode_flag.normal = 0;
            break;
        case wl_cmd_t::CONTROL:
            _ctx.data.active_mode_flag.control = 0;
            break;
        default:
            return PYRO_PARAM_ERROR;
    }
    return PYRO_OK;
}

/* Initialize chassis drivers and controllers / 鍒濆鍖栧簳鐩橀┍鍔ㄣ€佸弬鏁板拰鎺у埗鍣?*/
status_t wl_chassis_t::_init()
{
    status_t ret;
    _ctx = {};

    /* Initialize kinematic solver with given coefficients
       鐢ㄧ粰瀹氱殑杩愬姩瀛﹀弬鏁板垵濮嬪寲浜旇繛鏉嗚В鏋愭眰瑙ｅ櫒 */
    ret = _kinematic_solver.init(&_module_deps.phi_k,
             &_module_deps.polar_k, &_module_deps.vmc_k);
    CHECK_PYRO_RET(ret);

    /* Save LQR coefficients
       鎷疯礉淇濆瓨 LQR 鍙嶉澧炵泭鐨勫椤瑰紡鎷熷悎绯绘暟琛?*/
    memcpy(_lqr_cof, _module_deps.lqr_coef, sizeof(float) * 48);
    memcpy(_lqr_cof_over_step, _module_deps.lqr_coef_over_step, sizeof(float) * 48);

    /* Save wheel radius and reduction ratio / 瀛樺叆杞崐寰勫拰杞數鏈哄噺閫熸瘮 */
    _ctx.data.wheel_radius = _module_deps.wheel_radius;
    _ctx.data.reduction_ratio = _module_deps.reduction_ratio;

    power_controller_t &power_ctrl = power_controller_t::get_instance();
    power_ctrl.config_buffer_loop(_module_deps.power_limit_cfg.buffer_safe_energy,
                                  _module_deps.power_limit_cfg.buffer_kp,
                                  _module_deps.power_limit_cfg.buffer_ki,
                                  _module_deps.power_limit_cfg.buffer_kd);
    for (uint8_t i = 0; i < 2; ++i)
    {
        _ctx.power.wheel_node[i] =
            power_ctrl.register_motor(_module_deps.power_limit_cfg.fit_params[i]);
        if (_ctx.power.wheel_node[i] == nullptr)
            return PYRO_ERROR;
    }

    /* Initialize joint motor driver (DM Motors)
       寰幆鍒濆鍖?4 涓洿椹辫揪濡欏叧鑺傜數鏈洪┍鍔?*/
    for(uint8_t i = 0; i < 4; i++)
    {
        _ctx.motor.joint[i] = new dm_motor_drv_t(_module_deps.joint_motor_cfg[i].tx_id,
                                           _module_deps.joint_motor_cfg[i].rx_id,
                                           _module_deps.joint_motor_cfg[i].can);
        if(!_ctx.motor.joint[i])
        {
            return PYRO_NO_MEMORY;
        }
        _motor_offset[i] = _module_deps.joint_motor_cfg[i].offset_angle;

        /* Set joint motor operation limits / 璁剧疆杈惧鍏宠妭鐢垫満鐨勭墿鐞嗗畨鍏ㄩ檺鍒?*/
        _ctx.motor.joint[i]->set_rotate_range(_module_deps.rotate_min,
                                                   _module_deps.rotate_max);
        _ctx.motor.joint[i]->set_position_range(_module_deps.position_min,
                                                   _module_deps.position_max);
        _ctx.motor.joint[i]->set_torque_range(_module_deps.torque_min,
                                                   _module_deps.torque_max);
    }

    /* Initialize wheel motor driver (DJI M3508)
       寰幆鍒濆鍖栧乏鍙?2 涓ぇ鐤嗚疆姣傜數鏈洪┍鍔?*/
    for(uint8_t i = 0; i < 2; i++)
    {
        _ctx.motor.wheel[i] =
               new dji_m3508_motor_drv_t(_module_deps.wheel_motor_cfg[i].tx_id,
                                     _module_deps.wheel_motor_cfg[i].can);
        if(!_ctx.motor.wheel[i])
        {
            return PYRO_NO_MEMORY;
        }
    }

    /* Initialize gimbal yaw motor driver (DJI GM6020)
       鍒濆鍖栧簳鐩樹笌浜戝彴鐩稿鏃嬭浆鐨勫亸鑸鐢垫満椹卞姩 */
    _ctx.motor.yaw = new dji_gm_6020_motor_drv_t(_module_deps.yaw_motor_cfg.tx_id,
                                                  _module_deps.yaw_motor_cfg.can);
    _yaw_offset = _module_deps.yaw_offset;
    if(!_ctx.motor.yaw)
    {
        return PYRO_NO_MEMORY;
    }

    /* Initialize VMC matrix / 鍒濆鍖栧乏鍙冲崟鑵跨煩闃佃繍绠楃粨鏋勶紙鏋佸潗鏍囧埌鍏宠妭鍔涚煩鏄犲皠锛?*/
    for(uint8_t i = 0; i < 2; i++)
    {
        arm_mat_init_f32(&_ctx.data.leg[i].T_mat, 2, 2,
                                             _ctx.data.leg[i].T_mat_val);
    }

    /* Initialize PID controllers / 鍒濆鍖栧悇鎺у埗 PID */
    for(uint8_t i = 0; i < 2; i++)
    {
        /* Init T pid (Joint torque / angle) / 鎽嗚澶栫幆 PID */
        _ctx.pid.T[i] = new pid_t(_module_deps.T_pid_cfg[i].kp, _module_deps.T_pid_cfg[i].ki,
                            _module_deps.T_pid_cfg[i].kd,
                            _module_deps.T_pid_cfg[i].integral_limit,
                            _module_deps.T_pid_cfg[i].max_out);
        if(!_ctx.pid.T[i])
        {
            return PYRO_NO_MEMORY;
        }
        /* Init d_T pid (Joint rate) / 鎽嗚瑙掗€熷害鍐呯幆 PID */
        _ctx.pid.d_T[i] = new pid_t(_module_deps.d_T_pid_cfg[i].kp, _module_deps.d_T_pid_cfg[i].ki,
                            _module_deps.d_T_pid_cfg[i].kd,
                            _module_deps.d_T_pid_cfg[i].integral_limit,
                            _module_deps.d_T_pid_cfg[i].max_out);
        if(!_ctx.pid.d_T[i])
        {
            return PYRO_NO_MEMORY;
        }
        /* Init F pid (Leg length) / 鎽嗛暱澶栫幆 PID */
        _ctx.pid.F[i] = new pid_t(_module_deps.F_pid_cfg[i].kp, _module_deps.F_pid_cfg[i].ki,
                            _module_deps.F_pid_cfg[i].kd,
                            _module_deps.F_pid_cfg[i].integral_limit,
                            _module_deps.F_pid_cfg[i].max_out);
        if(!_ctx.pid.F[i])
        {
            return PYRO_NO_MEMORY;
        }
        /* Init d_F pid (Leg length rate) / 鎽嗛暱閫熷害鍐呯幆 PID */
        _ctx.pid.d_F[i] = new pid_t(_module_deps.d_F_pid_cfg[i].kp, _module_deps.d_F_pid_cfg[i].ki,
                            _module_deps.d_F_pid_cfg[i].kd,
                            _module_deps.d_F_pid_cfg[i].integral_limit,
                            _module_deps.d_F_pid_cfg[i].max_out);
        if(!_ctx.pid.d_F[i])
        {
            return PYRO_NO_MEMORY;
        }
    }
    _ctx.pid.yaw = new pid_t(_module_deps.yaw_pid_cfg.kp, _module_deps.yaw_pid_cfg.ki,
                            _module_deps.yaw_pid_cfg.kd,
                            _module_deps.yaw_pid_cfg.integral_limit,
                            _module_deps.yaw_pid_cfg.max_out);
    if(!_ctx.pid.yaw)
    {
        return PYRO_NO_MEMORY;
    }
    _ctx.pid.g_yaw = new pid_t(_module_deps.g_yaw_pid_cfg.kp, _module_deps.g_yaw_pid_cfg.ki,
                            _module_deps.g_yaw_pid_cfg.kd,
                            _module_deps.g_yaw_pid_cfg.integral_limit,
                            _module_deps.g_yaw_pid_cfg.max_out);
    if(!_ctx.pid.g_yaw)
    {
        return PYRO_NO_MEMORY;
    }
    _ctx.pid.delta = new pid_t(_module_deps.delta_pid_cfg.kp, _module_deps.delta_pid_cfg.ki,
                            _module_deps.delta_pid_cfg.kd,
                            _module_deps.delta_pid_cfg.integral_limit,
                            _module_deps.delta_pid_cfg.max_out);
    if(!_ctx.pid.delta)
    {
        return PYRO_NO_MEMORY;
    }
    _ctx.pid.d_delta = new pid_t(_module_deps.d_delta_pid_cfg.kp, _module_deps.d_delta_pid_cfg.ki,
                            _module_deps.d_delta_pid_cfg.kd,
                            _module_deps.d_delta_pid_cfg.integral_limit,
                            _module_deps.d_delta_pid_cfg.max_out);
    if(!_ctx.pid.d_delta)
    {
        return PYRO_NO_MEMORY;
    }

    _ctx.pid.roll = new pid_t(_module_deps.roll_pid_cfg.kp, _module_deps.roll_pid_cfg.ki,
                            _module_deps.roll_pid_cfg.kd,
                            _module_deps.roll_pid_cfg.integral_limit,
                            _module_deps.roll_pid_cfg.max_out);
    if(!_ctx.pid.roll)
    {
        return PYRO_NO_MEMORY;
    }

    /* Initialize Kalman filter for the wheel velocity
       鍒濆鍖栬溅杞殑绾挎€у钩鍔ㄨ浆閫?鍔犻€熷害瑙傛祴鍗″皵鏇兼护娉㈠櫒 */
    for(uint8_t i = 0; i < 2; i++)
    {
        ret = _wheel_kf[i].init(_module_deps.wheel_kf_cfg[i].A,
                                 _module_deps.wheel_kf_cfg[i].B,
                                 _module_deps.wheel_kf_cfg[i].H,
                                 _module_deps.wheel_kf_cfg[i].G,
                                 _module_deps.wheel_kf_cfg[i].Q,
                                 _module_deps.wheel_kf_cfg[i].R,
                                 _module_deps.wheel_kf_cfg[i].x_init,
                                 _module_deps.wheel_kf_cfg[i].P_init);
        CHECK_PYRO_RET(ret);
    }

    /* Get INS instance / 缁戝畾鍗曚緥鐨勬儻鎬у鑸郴缁?*/
    _ctx.ins = ins_drv_t::get_instance();
    _ctx.data.yaw = _ctx.data.pitch = _ctx.data.roll = 0.0f;
    _ctx.data.g_yaw = _ctx.data.g_pitch = _ctx.data.g_roll = 0.0f;
    _ctx.data.accel_x = _ctx.data.accel_y = _ctx.data.accel_z = 0.0f;
    _ctx.data.accel_forward = _ctx.data.accel_upward =
        _ctx.data.accel_upward_lpf = 0.0f;

    _ctx.cmd = &_current_cmd;
    return ret;
}

float power;
float energy;
float limit;

extern referee_drv_t *referee_drv;

/* Update feedback loop / 浼犳劅鍣ㄤ笌鐢垫満鍙嶉鍛ㄦ湡鎬ф暟鎹埛鏂?*/
void wl_chassis_t::_update_feedback()
{
    power = referee_drv->get_data().robot_status.chassis_power_limit;
    last_time = dwt_drv_t::get_timeline_ms();

    /* Get feedback from supercapacitor board
       浠庤秴绾х數瀹规澘鐨?CAN 鍙嶉涓В鏋愮湡瀹炵殑搴曠洏鐢佃兘鏁版嵁锛堢摝鐗广€佺數鍘嬬瓑锛?*/
    supercap_drv_t::cap_feedback_t cap_feedback = supercap_drv_t::get_instance()->get_feedback();
    _ctx.power.cap_feedback = cap_feedback;
    _ctx.power.chassis_power = cap_feedback.chassis_power_cap / 100.0f;
    _ctx.power.cap_power = cap_feedback.cap_power_cap / 100.0f - 250;
    _ctx.power.voltage = cap_feedback.vot_cap / 100.0f;
    _ctx.power.limit = referee_drv->get_data().robot_status.chassis_power_limit;
    _ctx.power.buffer_energy = referee_drv->get_data().power_heat.buffer_energy;

    /* Pack referee data to send back to supercapacitor
       鎵撳寘瑁佸垽绯荤粺鐨勫疄鏃跺姛鐜囬檺鍒跺拰缂撳啿鑳介噺鏁版嵁锛岀敤浠ュ彂閫佺粰瓒呯骇鐢靛鍋氳嚜閫傚簲鍏呮斁鐢垫帶鍒?*/
    _ctx.power.supercap_cmd.power_referee = 0;
    _ctx.power.supercap_cmd.power_limit_referee = _ctx.power.limit;
    _ctx.power.supercap_cmd.power_buffer_limit_referee = 60.0f;
    _ctx.power.supercap_cmd.power_buffer_referee = _ctx.power.buffer_energy;
    _ctx.power.supercap_cmd.kill_chassis_user = 0;
    _ctx.power.supercap_cmd.speed_up_user_now = 0;

    static uint32_t dwt_cnt;
    static float last_dx[2];

    /* Update INS sensor data
       鏇存柊鏈轰綋濮挎€佹鎷夎锛堜刊浠般€佹í婊氥€佽埅鍚戯級銆佽閫熷害浠ュ強娑堥櫎閲嶅姏鍔犻€熷害鍚庣殑鍔犻€熷害 */
    if(_ctx.ins)
    {
        _ctx.ins->get_rads_b(&_ctx.data.yaw, &_ctx.data.pitch, &_ctx.data.roll);
        _ctx.ins->get_gyro_b(&_ctx.data.g_yaw, &_ctx.data.g_pitch,
                             &_ctx.data.g_roll);
        _ctx.ins->get_accel_without_g_b(&_ctx.data.accel_x,
                                         &_ctx.data.accel_y,
                                         &_ctx.data.accel_z);
    }

    /* Update joint motor feedback
       鑾峰彇鐩撮┍浜旇繛鏉嗙數鏈虹殑瀹為檯寮у害鍜岄€熷害鍊?*/
    for(uint8_t i = 0; i < 4; i++)
    {
        _ctx.motor.joint[i]->update_feedback();
    }

    /* Map joint motor angles to linkage coordinates (theta1, theta2)
       theta 瑙掗『鏃堕拡涓烘锛屼互姘村钩鍚戝墠鏂瑰悜涓?0 寮у害绾裤€傚洜涓哄乏鍙充晶鐢垫満鏈濆悜瀹夎鐩稿弽锛?       鎵€浠ュ彸鑵块渶瑕佸皢鐢垫満鐨勭紪鐮佸櫒鍊煎彇鍙嶏紝骞跺姞鍏ョ墿鐞嗛浂鐐瑰亸绉诲畨瑁呬慨姝?*/
    _ctx.data.leg[R].theta1 = -_ctx.motor.joint[RF]->get_current_position()
                                                        + _motor_offset[RF];
    _ctx.data.leg[R].theta2 = -_ctx.motor.joint[RB]->get_current_position()
                                                        + _motor_offset[RB];
    _ctx.data.leg[L].theta1 = _ctx.motor.joint[LF]->get_current_position()
                                                        + _motor_offset[LF];
    _ctx.data.leg[L].theta2 = _ctx.motor.joint[LB]->get_current_position()
                                                        + _motor_offset[LB];

    /* Map joint velocities / 杩炴潌瀹為檯杞€熷€艰浆鎹?*/
    _ctx.data.leg[R].d_theta1 = -_ctx.motor.joint[RF]->get_current_rotate();
    _ctx.data.leg[R].d_theta2 = -_ctx.motor.joint[RB]->get_current_rotate();
    _ctx.data.leg[L].d_theta1 = _ctx.motor.joint[LF]->get_current_rotate();
    _ctx.data.leg[L].d_theta2 = _ctx.motor.joint[LB]->get_current_rotate();

    /* Update wheel motor feedback
       鏇存柊澶х枂 M3508 杞瘋鐢垫満鐨勫弽棣堛€傚乏渚х數鏈烘湞鍚戜笌搴曠洏鍓嶈繘鏂瑰悜鐩稿弽锛?       鎵€浠ュ乏杞嚎閫熷害 dx 璁＄畻闇€娣诲姞璐熷彿銆傞€氳繃骞冲姩绾块€熷害绉垎璁＄畻鍑鸿疆寮忕粷瀵归噷绋嬭 x銆?*/
    _ctx.motor.wheel[R]->update_feedback();
    _ctx.data.leg[R].w = _ctx.motor.wheel[R]->get_current_rotate() / _ctx.data.reduction_ratio;
    _ctx.data.leg[R].T_w_real = _ctx.motor.wheel[R]->get_current_torque()*0.3f/(3591.0f/187.0f) * _ctx.data.reduction_ratio;
    _ctx.data.leg[R].dx = _ctx.data.leg[R].w * _ctx.data.wheel_radius;
    _ctx.data.leg[R].x += (_ctx.data.leg[R].dx + last_dx[R])/2 * 0.001f;
    last_dx[R] = _ctx.data.leg[R].dx;

    _ctx.motor.wheel[L]->update_feedback();
    _ctx.data.leg[L].w = _ctx.motor.wheel[L]->get_current_rotate() / _ctx.data.reduction_ratio;
    _ctx.data.leg[L].T_w_real = _ctx.motor.wheel[L]->get_current_torque()*0.3f/(3591.0f/187.0f) * _ctx.data.reduction_ratio;
    _ctx.data.leg[L].dx = - _ctx.data.leg[L].w * _ctx.data.wheel_radius;
    _ctx.data.leg[L].x += (_ctx.data.leg[L].dx + last_dx[L])/2 * 0.001f;
    last_dx[L] = _ctx.data.leg[L].dx;

    /* Update Gimbal Yaw tracking / 鍒锋柊搴曠洏瀵归綈浜戝彴鍋忚埅瑙掔數鏈虹殑瑙掑害鍜岃閫熷害 */
    _ctx.motor.yaw->update_feedback();
    _ctx.data.gimbal_yaw =
        wrap2pi_f32_normalized(_ctx.motor.yaw->get_current_position() + _yaw_offset);
    _ctx.data.gimbal_g_yaw = _ctx.motor.yaw->get_current_rotate();

    time = dwt_drv_t::get_delta_t(&dwt_cnt);

    /* Execute 5-bar kinematics solver
       浣跨敤瑙ｆ瀽鍑犱綍锛屾牴鎹繛鏉嗙殑涓や釜鍏宠妭瑙掓眰瑙ｆ瀬杞翠笅鐨勬瀬寰?l锛堣吙闀匡級鍙婃瀬杞村亸瑙?alpha銆?       鍚屾椂鎺ㄥ鍑哄綋鍓嶆敹缂╅€熷害 d_l 涓庢憜鍔ㄩ€熷害 d_alpha銆?*/
    for(uint8_t i = 0; i < 2; i++)
    {
        status_t ret;
        float last_d_l = _ctx.data.leg[i].d_l;
        float last_d_beta = _ctx.data.leg[i].d_beta;
        ret = _kinematic_solver.solve(_ctx.data.leg[i].theta1,
                                      _ctx.data.leg[i].theta2,
                                    _ctx.data.leg[i].d_theta1,
                                    _ctx.data.leg[i].d_theta2,
                                        &_ctx.data.leg[i].phi1,
                                        &_ctx.data.leg[i].phi2,
                                      &_ctx.data.leg[i].alpha,
                                       &_ctx.data.leg[i].l,
                                    &_ctx.data.leg[i].d_l,
                                     &_ctx.data.leg[i].d_alpha,
                                     &_ctx.data.leg[i].d_jx,
                                     &_ctx.data.leg[i].d_jy,
                                     &_ctx.data.leg[i].jx,
                                     &_ctx.data.leg[i].jy);
        if(ret != PYRO_OK)
        {
            _ctx.data.cnt.solver_error++;
        }

        /* Calculate tilt coordinates beta relative to vertical gravity line
           璁＄畻搴曠洏铏氭嫙鑵垮拰閲嶅姏鍨傜洿绾跨殑鍋忚 beta锛屼互鍙婂畠鐨勫彉鍖栬閫熷害 d_beta */
        _ctx.data.leg[i].beta = PI / 2 - _ctx.data.leg[i].alpha - _ctx.data.pitch;
        _ctx.data.leg[i].d_beta = -_ctx.data.leg[i].d_alpha - _ctx.data.g_pitch;
        _ctx.data.leg[i].gamma = -_ctx.data.pitch;
        _ctx.data.leg[i].d_gamma = -_ctx.data.g_pitch;

        /* Estimate second-order derivatives (accelerations) / 閫氳繃鏁板€煎樊鍒嗘眰浜岄樁鍔犻€熷害椤?*/
        _ctx.data.leg[i].d2_beta = (_ctx.data.leg[i].d_beta - last_d_beta) / time;
        _ctx.data.leg[i].d2_l = (_ctx.data.leg[i].d_l - last_d_l) / time;
    }

    /* Update VMC Jacobian matrix
       姹傝В杞吙鍑犱綍鐨?VMC 鏄犲皠闆呭彲姣旂煩闃碉紙鐢ㄤ簬灏嗘瀬鍧愭爣铏氭嫙鍔涜浆鎹负鍏宠妭鐢垫満鐨勬壄鐭╋級 */
    for(uint8_t i = 0; i < 2; i++)
    {
        status_t ret;

        ret = _kinematic_solver.get_VMC_value(_ctx.data.leg[i].theta1,
                                             _ctx.data.leg[i].theta2,
                                             _ctx.data.leg[i].phi1,
                                             _ctx.data.leg[i].phi2,
                                             _ctx.data.leg[i].l,
                                             _ctx.data.leg[i].alpha,
                                             _ctx.data.leg[i].T_mat.pData);
        if(ret != PYRO_OK)
        {
            _ctx.data.cnt.solver_error++;
        }
    }

    /* Project body-frame acceleration to ground-aligned axes
       灏嗘満浣撳潗鏍囩郴鐨勫姞閫熷害鎶曞奖鍒板湴鐞嗗瀭鐩磋酱涓庣旱鍚戞按骞宠酱銆?       淇话瑙掍互浣庡ご涓烘锛屽洜姝ょ敱浜?IMU 闆朵綅鍋忕疆娌?X 杞村墠绉?`IMU_OFFSET_X`锛屽湪鏃嬭浆鍋忚埅鏃朵細浜х敓绂诲績鍔犻€熷害锛岄渶鍔犺ˉ鍋裤€?*/
    float kf_u = 0.0f;
    float kf_z[3] = {0.0f, 0.0f, 0.0f};
    float kf_estimated[3] = {0.0f, 0.0f, 0.0f};

    _ctx.data.accel_forward =
        _ctx.data.accel_x * arm_cos_f32(_ctx.data.pitch)
        + _ctx.data.accel_z * arm_sin_f32(_ctx.data.pitch)
        + _ctx.data.g_yaw * _ctx.data.g_yaw * IMU_OFFSET_X;
    _ctx.data.accel_upward =
        -_ctx.data.accel_x * arm_sin_f32(_ctx.data.pitch)
        + _ctx.data.accel_z * arm_cos_f32(_ctx.data.pitch);
    _ctx.data.accel_upward_lpf =
        _ctx.data.accel_upward_lpf * SUPPORT_FORCE_ACC_LPF_RC /
            (time + SUPPORT_FORCE_ACC_LPF_RC)
        + _ctx.data.accel_upward * time /
            (time + SUPPORT_FORCE_ACC_LPF_RC);

    /* Run Kalman filter for linear velocity
       浣跨敤骞冲潎宸﹀彸杞€熶綔涓哄钩鍔ㄨ娴嬪€?v_obs锛岃緭鍏ュ崱灏旀浖婊ゆ尝鍣紝浼拌鏈轰綋褰撳墠鐨勭粷瀵归€熷害銆佸姞閫熷害鍜屼綅绉婚噺 */
    float v_obs = (_ctx.data.leg[R].dx + _ctx.data.leg[L].dx) / 2.0f;
    for(uint8_t i = 0; i < 2; i++)
    {
        kf_u = 0.0f;
        kf_z[0] = v_obs;
        kf_z[1] = _ctx.data.accel_forward;
        kf_z[2] = _ctx.data.g_yaw;
        _wheel_kf[i].update(kf_z, &kf_u, kf_estimated);
        _ctx.data.leg[i].kf_v = kf_estimated[0];
        _ctx.data.leg[i].kf_a = kf_estimated[1];
        _ctx.data.leg[i].kf_w = kf_estimated[2];
        _ctx.data.leg[i].kf_x += _ctx.data.leg[i].kf_v * 0.001f;
    }
}

/* FSM entry point called by scheduler / 搴曠洏涓绘帶浠诲姟鐨?FSM 鍒锋柊鍏ュ彛 */
void wl_chassis_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;

    /* Decouple Passive (disable) and Active mode / 琚姩妯″紡涓庝富鍔ㄨ繍琛屾ā寮忓垏鎹?*/
    if (cmd_base_t::mode_t::PASSIVE == _ctx.cmd->mode)
        _fsm.change_state(&_state_passive)  ;
    else if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _fsm.change_state(&_state_active);

    __decide_cap(); /* Process supercapacitor board FSM / 瓒呯骇鐢靛琛屼负鍒ゆ柇 */
    _fsm.execute(this); /* Run FSM execute loop / 鎵ц鐘舵€佹満鐨勫姩浣?*/
    time = dwt_drv_t::get_timeline_ms() - last_time;
}

void wl_chassis_t::_solve_wheel_power_limit(const float tau_motion[2],
                                            const float tau_uncontrolled[2],
                                            const float omega[2],
                                            float tau_out[2])
{
    power_controller_t &power_ctrl = power_controller_t::get_instance();

    for (uint8_t i = 0; i < 2; ++i)
    {
        power_node_t *node = _ctx.power.wheel_node[i];
        if (node == nullptr)
        {
            tau_out[i] = tau_motion[i] + tau_uncontrolled[i];
            _ctx.data.leg[i].predict_power = 0.0f;
            continue;
        }

        node->target_cmd = tau_motion[i];
        node->uncontrolled_cmd = tau_uncontrolled[i];
        node->rpm = omega[i];
        node->temp = 20.0f;
    }

    const float cap_extra_power =
        fp32_constrain(_ctx.power.cap_power, 0.0f,
                       _module_deps.power_limit_cfg.cap_extra_power_limit);
    power_ctrl.solve(_ctx.power.limit, _ctx.power.buffer_energy, cap_extra_power);

    for (uint8_t i = 0; i < 2; ++i)
    {
        power_node_t *node = _ctx.power.wheel_node[i];
        if (node == nullptr)
            continue;

        tau_out[i] = node->safe_cmd;
        _ctx.data.leg[i].predict_power =
            wl_power_predict(node->params,
                             node->safe_cmd - node->uncontrolled_cmd,
                             node->uncontrolled_cmd,
                             node->rpm,
                             node->temp);
    }
}

void wl_chassis_t::__send_supercap_command() const
{
    supercap_drv_t::get_instance()->send_cmd(_ctx.power.supercap_cmd);
}

/* Supercapacitor state manager / 瓒呯骇鐢靛鐘舵€佺鐞嗛€昏緫 */
void wl_chassis_t::__decide_cap()
{
    static bool _last_status = false;
    static uint32_t _timer   = 0;
    static bool _delay_done  = false;

    /* Get chassis power output state from referee system / 鑾峰彇瑁佸垽绯荤粺搴曠洏鏄惁鏈夌數婧愯緭鍑虹殑鎸囩ず */
    bool current_status = referee_drv->get_data().robot_status.power_management_chassis_output;

    if (current_status)
    {
        /* Case A: Chassis power output is active / 鎯呭喌 A锛氬簳鐩樼數婧愭湁杈撳嚭 */
        if (!_last_status)
        {
            /* Reset delay timers upon status transition / 鍒氬垏鎹㈣嚦鏈夎緭鍑虹姸鎬侊紝閲嶇疆闃叉姈鍙婂彂閫佽鏃跺櫒 */
            _timer      = 0;
            _delay_done = false;
        }

        if (!_delay_done)
        {
            /* Process 1000 tick initial delay for cap bootstrap / 澶勭悊瓒呯骇鐢靛鍚姩 1000 娆″懆鏈熺殑鍒濆闃叉姈寤舵椂 */
            if (++_timer >= 1000)
            {
                _delay_done           = true;
                _timer                = 0;
                _ctx.power.supercap_cmd.use_cap = 1; /* Request to discharge capacitor / 鍏佽寮€濮嬩娇鐢ㄨ秴绾х數瀹瑰姛鑰?*/
                __send_supercap_command();
            }
        }
        else
        {
            /* Emit CAP active command every 10 ticks / 寤舵椂閫氳繃鍚庯紝姣?10 娆″懆鏈熺粰瓒呯數椹卞姩涓嬪彂涓€娆?use_cap=1 */
            if (++_timer >= 10)
            {
                _timer                = 0;
                _ctx.power.supercap_cmd.use_cap = 1;
                __send_supercap_command();
            }
        }
    }
    else
    {
        /* Case B: Power cut detected / 鎯呭喌 B锛氬簳鐩樻柇鐢垫垨鐢垫簮闄愬埗杈撳嚭 */
        if (_last_status)
        {
            /* Immediately request bypass / disable capacitor usage / 鍒囨崲鑷虫棤鐢垫簮杈撳嚭锛岀珛鍒诲皢 use_cap 璁句负 0 骞跺彂閫?*/
            _ctx.power.supercap_cmd.use_cap = 0;
            __send_supercap_command();

            _delay_done = false;
            _timer      = 0;
        }
    }

    _last_status = current_status;
}
}
