/*
 * @Author: Vod vod0575@outlook
 * @Date: 2026-02-06 15:27:37
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-05-29 04:42:03
 * @Description: Wheel-Legged Chassis module source file / 轮腿底盘模块源文件
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */

#include "pyro_wl_chassis.h"
#include "pyro_dwt_drv.h"
#include "pyro_algo_common.h"
#include "pyro_vofa.h"
#include "pyro_referee.h"
#include <cmath>

#define WHEEL_DISTANCE 0.424f             /* Distance between left and right wheels (m) / 左右轮距 (m) */
#define SUPPORT_FORCE_ACC_LPF_RC 0.01f   /* LPF time constant for vertical acceleration / 垂直加速度低通滤波时间常数 */

/* IMU offset from yaw rotation center (midpoint of two wheels) along body x-axis.
   Positive = IMU is in front of wheel axis. Measure and adjust this value.
   IMU 相对偏航旋转中心（两轮轴线中点）沿车体 X 轴的偏移量。前偏为正。 */
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

/* Constructor / 构造函数，设定任务名称及堆栈大小，初始化 2 组轮速卡尔曼滤波器 */
wl_chassis_t::wl_chassis_t() : module_base_t("wl_chassis", 0, 2048),
    _wheel_power_node{nullptr, nullptr},
    _wheel_kf{kf_t(3, 1, 3, 2), kf_t(3, 1, 3, 2)}
{
}

/* Get current leg angle / 获取当前极坐标摆角 alpha */
status_t wl_chassis_t::get_cur_angle(float *r_angle, float *l_angle)
{
    if(!l_angle || !r_angle)
    {
        return PYRO_PARAM_ERROR;
    }
    *r_angle = _leg_data[R].alpha;
    *l_angle = _leg_data[L].alpha;
    return PYRO_OK;
}

/* Get current leg angular velocity / 获取当前极坐标摆角变化率 d_alpha */
status_t wl_chassis_t::get_cur_d_angle(float *r_d_angle, float *l_d_angle)
{
    if(!l_d_angle || !r_d_angle)
    {
        return PYRO_PARAM_ERROR;
    }
    *r_d_angle = _leg_data[R].d_alpha;
    *l_d_angle = _leg_data[L].d_alpha;
    return PYRO_OK;
}

/* Get current leg length / 获取当前极轴半径（摆腿长度） l */
status_t wl_chassis_t::get_cur_length(float *r_leg, float *l_leg)
{
    if(!l_leg || !r_leg)
    {
        return PYRO_PARAM_ERROR;
    }
    *r_leg = _leg_data[R].l;
    *l_leg = _leg_data[L].l;
    return PYRO_OK;
}

/* Get current virtual hip torque / 获取当前虚拟摆动扭矩 F[1] */
status_t wl_chassis_t::get_cur_p_torque(float *r_torque, float *l_torque)
{
    if(!r_torque || !l_torque)
    {
        return PYRO_PARAM_ERROR;
    }
    *r_torque = _leg_data[R].F[1];
    *l_torque = _leg_data[L].F[1];
    return PYRO_OK;
}

/* Get current INS Yaw angle / 获取当前惯导偏航角 */
status_t wl_chassis_t::get_cur_ins_yaw(float* temp_yaw)
{
    if(!temp_yaw)
    {
        return PYRO_PARAM_ERROR;
    }
    *temp_yaw = yaw;
    return PYRO_OK;
}

/* Get current displacement error bias / 获取当前位移差偏差量 */
status_t wl_chassis_t::get_cur_x_bias(float* r_x_bias, float* l_x_bias)
{
    if(!r_x_bias || !l_x_bias)
    {
        return PYRO_PARAM_ERROR;
    }
    *r_x_bias = _leg_data[R].x_bias;
    *l_x_bias = _leg_data[L].x_bias;
    return PYRO_OK;
}

/* Get current tilt angle bias / 获取当前摆腿偏角偏差量 */
status_t wl_chassis_t::get_cur_beta_bias(float* r_beta_bias, float* l_beta_bias)
{
    if(!r_beta_bias || !l_beta_bias)
    {
        return PYRO_PARAM_ERROR;
    }
    *r_beta_bias = _leg_data[R].beta_bias;
    *l_beta_bias = _leg_data[L].beta_bias;
    return PYRO_OK;
}

/* Get current pitch angle bias / 获取当前底盘俯仰角偏差量 */
status_t wl_chassis_t::get_cur_gamma_bias(float* gamma_bias)
{
    if(!gamma_bias)
    {
        return PYRO_PARAM_ERROR;
    }
    *gamma_bias = _leg_data[R].gamma_bias;
    return PYRO_OK;
}

/* Get sub-state FSM completion flags / 查询各子状态就绪标志 */
uint8_t wl_chassis_t::get_status_flag(wl_cmd_t::active_mode_t mode)
{
    switch(mode)
    {
        case wl_cmd_t::READY:
            return _active_mode_flag.ready;
        case wl_cmd_t::TEST:
            return _active_mode_flag.test;
        case wl_cmd_t::REVERSE:
            return _active_mode_flag.reverse;
        case wl_cmd_t::OVER_STEP:
            return _active_mode_flag.over_step;
        case wl_cmd_t::OVER_STEP_RESET:
            return _active_mode_flag.over_step_reset;
        case wl_cmd_t::NORMAL:
            return _active_mode_flag.normal;
        case wl_cmd_t::CONTROL:
            return _active_mode_flag.control;
        default:
            return 0;
    }
}

/* Clear FSM completion flags / 清除各子状态就绪标志 */
status_t wl_chassis_t::clear_status_flag(wl_cmd_t::active_mode_t mode)
{
    switch(mode)
    {
        case wl_cmd_t::READY:
            _active_mode_flag.ready = 0;
            break;
        case wl_cmd_t::TEST:
            _active_mode_flag.test = 0;
            break;
        case wl_cmd_t::REVERSE:
            _active_mode_flag.reverse = 0;
            break;
        case wl_cmd_t::OVER_STEP:
            _active_mode_flag.over_step = 0;
            break;
        case wl_cmd_t::OVER_STEP_RESET:
            _active_mode_flag.over_step_reset = 0;
            break;
        case wl_cmd_t::NORMAL:
            _active_mode_flag.normal = 0;
            break;
        case wl_cmd_t::CONTROL:
            _active_mode_flag.control = 0;
            break;
        default:
            return PYRO_PARAM_ERROR;
    }
    return PYRO_OK;
}

/* Initialize chassis drivers and controllers / 初始化底盘驱动、参数和控制器 */
status_t wl_chassis_t::_init()
{
    status_t ret;

    /* Initialize kinematic solver with given coefficients
       用给定的运动学参数初始化五连杆解析求解器 */
    ret = _kinematic_solver.init(&_module_deps.phi_k, 
             &_module_deps.polar_k, &_module_deps.vmc_k);
    CHECK_PYRO_RET(ret);
    
    /* Save LQR coefficients
       拷贝保存 LQR 反馈增益的多项式拟合系数表 */
    memcpy(_lqr_cof, _module_deps.lqr_coef, sizeof(float) * 48);
    memcpy(_lqr_cof_over_step, _module_deps.lqr_coef_over_step, sizeof(float) * 48);

    /* Save wheel radius and reduction ratio / 存入轮半径和轮电机减速比 */
    _wheel_radius = _module_deps.wheel_radius;
    _reduction_ratio = _module_deps.reduction_ratio;

    power_controller_t &power_ctrl = power_controller_t::get_instance();
    power_ctrl.config_buffer_loop(_module_deps.power_limit_cfg.buffer_safe_energy,
                                  _module_deps.power_limit_cfg.buffer_kp,
                                  _module_deps.power_limit_cfg.buffer_ki,
                                  _module_deps.power_limit_cfg.buffer_kd);
    for (uint8_t i = 0; i < 2; ++i)
    {
        _wheel_power_node[i] =
            power_ctrl.register_motor(_module_deps.power_limit_cfg.fit_params[i]);
        if (_wheel_power_node[i] == nullptr)
            return PYRO_ERROR;
    }

    /* Initialize joint motor driver (DM Motors)
       循环初始化 4 个直驱达妙关节电机驱动 */
    for(uint8_t i = 0; i < 4; i++)
    {
        _motor_drv[i] = new dm_motor_drv_t(_module_deps.joint_motor_cfg[i].tx_id,
                                           _module_deps.joint_motor_cfg[i].rx_id,
                                           _module_deps.joint_motor_cfg[i].can);
        if(!_motor_drv[i])
        {
            return PYRO_NO_MEMORY;
        }
        _motor_offset[i] = _module_deps.joint_motor_cfg[i].offset_angle;
        
        /* Set joint motor operation limits / 设置达妙关节电机的物理安全限制 */
        _motor_drv[i]->set_rotate_range(_module_deps.rotate_min, 
                                                   _module_deps.rotate_max);
        _motor_drv[i]->set_position_range(_module_deps.position_min,
                                                   _module_deps.position_max);
        _motor_drv[i]->set_torque_range(_module_deps.torque_min,
                                                   _module_deps.torque_max);
    }
    
    /* Initialize wheel motor driver (DJI M3508)
       循环初始化左右 2 个大疆轮毂电机驱动 */
    for(uint8_t i = 0; i < 2; i++)
    {
        _wheel_drv[i] = 
               new dji_m3508_motor_drv_t(_module_deps.wheel_motor_cfg[i].tx_id,
                                     _module_deps.wheel_motor_cfg[i].can);
        if(!_wheel_drv[i])
        {
            return PYRO_NO_MEMORY;
        }
    }
    
    /* Initialize gimbal yaw motor driver (DJI GM6020)
       初始化底盘与云台相对旋转的偏航角电机驱动 */
    _yaw_motor_drv = new dji_gm_6020_motor_drv_t(_module_deps.yaw_motor_cfg.tx_id,
                                                  _module_deps.yaw_motor_cfg.can);
    _yaw_offset = _module_deps.yaw_offset;
    if(!_yaw_motor_drv)
    {
        return PYRO_NO_MEMORY;
    }
    
    /* Initialize VMC matrix / 初始化左右单腿矩阵运算结构（极坐标到关节力矩映射） */
    for(uint8_t i = 0; i < 2; i++)
    {
        arm_mat_init_f32(&_leg_data[i].T_mat, 2, 2, 
                                             _leg_data[i].T_mat_val);
    }
    
    /* Initialize PID controllers / 初始化各控制 PID */
    for(uint8_t i = 0; i < 2; i++)
    {
        /* Init T pid (Joint torque / angle) / 摆角外环 PID */
        _T_pid[i] = new pid_t(_module_deps.T_pid_cfg[i].kp, _module_deps.T_pid_cfg[i].ki, 
                            _module_deps.T_pid_cfg[i].kd, 
                            _module_deps.T_pid_cfg[i].integral_limit,
                            _module_deps.T_pid_cfg[i].max_out);
        if(!_T_pid[i])
        {
            return PYRO_NO_MEMORY;
        }
        /* Init d_T pid (Joint rate) / 摆角角速度内环 PID */
        _d_T_pid[i] = new pid_t(_module_deps.d_T_pid_cfg[i].kp, _module_deps.d_T_pid_cfg[i].ki, 
                            _module_deps.d_T_pid_cfg[i].kd, 
                            _module_deps.d_T_pid_cfg[i].integral_limit,
                            _module_deps.d_T_pid_cfg[i].max_out);
        if(!_d_T_pid[i])
        {
            return PYRO_NO_MEMORY;
        }
        /* Init F pid (Leg length) / 摆长外环 PID */
        _F_pid[i] = new pid_t(_module_deps.F_pid_cfg[i].kp, _module_deps.F_pid_cfg[i].ki, 
                            _module_deps.F_pid_cfg[i].kd, 
                            _module_deps.F_pid_cfg[i].integral_limit,
                            _module_deps.F_pid_cfg[i].max_out);
        if(!_F_pid[i])        
        {
            return PYRO_NO_MEMORY;
        }
        /* Init d_F pid (Leg length rate) / 摆长速度内环 PID */
        _d_F_pid[i] = new pid_t(_module_deps.d_F_pid_cfg[i].kp, _module_deps.d_F_pid_cfg[i].ki, 
                            _module_deps.d_F_pid_cfg[i].kd, 
                            _module_deps.d_F_pid_cfg[i].integral_limit,
                            _module_deps.d_F_pid_cfg[i].max_out);
        if(!_d_F_pid[i])
        {
            return PYRO_NO_MEMORY;
        }
    }
    _yaw_pid = new pid_t(_module_deps.yaw_pid_cfg.kp, _module_deps.yaw_pid_cfg.ki, 
                            _module_deps.yaw_pid_cfg.kd, 
                            _module_deps.yaw_pid_cfg.integral_limit,
                            _module_deps.yaw_pid_cfg.max_out);
    if(!_yaw_pid)
    {
        return PYRO_NO_MEMORY;
    }
    _g_yaw_pid = new pid_t(_module_deps.g_yaw_pid_cfg.kp, _module_deps.g_yaw_pid_cfg.ki, 
                            _module_deps.g_yaw_pid_cfg.kd, 
                            _module_deps.g_yaw_pid_cfg.integral_limit,
                            _module_deps.g_yaw_pid_cfg.max_out);
    if(!_g_yaw_pid)    
    {
        return PYRO_NO_MEMORY;
    }
    _delta_pid = new pid_t(_module_deps.delta_pid_cfg.kp, _module_deps.delta_pid_cfg.ki, 
                            _module_deps.delta_pid_cfg.kd, 
                            _module_deps.delta_pid_cfg.integral_limit,
                            _module_deps.delta_pid_cfg.max_out);
    if(!_delta_pid)    
    {
        return PYRO_NO_MEMORY;
    }
    _d_delta_pid = new pid_t(_module_deps.d_delta_pid_cfg.kp, _module_deps.d_delta_pid_cfg.ki, 
                            _module_deps.d_delta_pid_cfg.kd, 
                            _module_deps.d_delta_pid_cfg.integral_limit,
                            _module_deps.d_delta_pid_cfg.max_out);
    if(!_d_delta_pid)
    {
        return PYRO_NO_MEMORY;
    }

    _roll_pid = new pid_t(_module_deps.roll_pid_cfg.kp, _module_deps.roll_pid_cfg.ki, 
                            _module_deps.roll_pid_cfg.kd, 
                            _module_deps.roll_pid_cfg.integral_limit,
                            _module_deps.roll_pid_cfg.max_out);
    if(!_roll_pid)
    {
        return PYRO_NO_MEMORY;
    }
    
    /* Initialize Kalman filter for the wheel velocity
       初始化车轮的线性平动转速/加速度观测卡尔曼滤波器 */
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
    
    /* Get INS instance / 绑定单例的惯性导航系统 */
    _ins_drv = ins_drv_t::get_instance();
    yaw = pitch = roll = 0.0f;
    g_yaw = g_pitch = g_roll = 0.0f;
    a_x = a_y = a_z = 0.0f;
    a_forward = a_upward = a_upward_lpf = 0.0f;

    return ret;
}

float power;
float energy;
float limit;

extern referee_drv_t *referee_drv;

/* Update feedback loop / 传感器与电机反馈周期性数据刷新 */
void wl_chassis_t::_update_feedback()
{
    power = referee_drv->get_data().robot_status.chassis_power_limit;
    last_time = dwt_drv_t::get_timeline_ms();
   
    /* Get feedback from supercapacitor board
       从超级电容板的 CAN 反馈中解析真实的底盘电能数据（瓦特、电压等） */
    supercap_drv_t::cap_feedback_t cap_feedback = supercap_drv_t::get_instance()->get_feedback();
    _power_data.chassis_power = cap_feedback.chassis_power_cap / 100.0f; 
    _power_data.cap_power = cap_feedback.cap_power_cap / 100.0f - 250; 
    _power_data.voltage = cap_feedback.vot_cap / 100.0f; 
    _power_data.limit = referee_drv->get_data().robot_status.chassis_power_limit;
    _power_data.buffer_energy = referee_drv->get_data().power_heat.buffer_energy;

    /* Pack referee data to send back to supercapacitor
       打包裁判系统的实时功率限制和缓冲能量数据，用以发送给超级电容做自适应充放电控制 */
    _supercap_cmd.power_referee = 0;
    _supercap_cmd.power_limit_referee = _power_data.limit;
    _supercap_cmd.power_buffer_limit_referee = 60.0f;
    _supercap_cmd.power_buffer_referee = _power_data.buffer_energy;
    _supercap_cmd.kill_chassis_user = 0;
    _supercap_cmd.speed_up_user_now = 0;

    static uint32_t dwt_cnt;
    static float last_dx[2];
    
    /* Update INS sensor data
       更新机体姿态欧拉角（俯仰、横滚、航向）、角速度以及消除重力加速度后的加速度 */
    if(_ins_drv)
    {
        _ins_drv->get_rads_b(&yaw, &pitch, &roll);
        _ins_drv->get_gyro_b(&g_yaw, &g_pitch, &g_roll);
        _ins_drv->get_accel_b(&a_x, &a_y, &a_z);
    }
    
    /* Update joint motor feedback
       获取直驱五连杆电机的实际弧度和速度值 */
    for(uint8_t i = 0; i < 4; i++)
    {
        _motor_drv[i]->update_feedback();
    }
    
    /* Map joint motor angles to linkage coordinates (theta1, theta2)
       theta 角顺时针为正，以水平向前方向为 0 弧度线。因为左右侧电机朝向安装相反，
       所以右腿需要将电机的编码器值取反，并加入物理零点偏移安装修正 */
    _leg_data[R].theta1 = -_motor_drv[RF]->get_current_position() 
                                                        + _motor_offset[RF];
    _leg_data[R].theta2 = -_motor_drv[RB]->get_current_position() 
                                                        + _motor_offset[RB];
    _leg_data[L].theta1 = _motor_drv[LF]->get_current_position() 
                                                        + _motor_offset[LF];
    _leg_data[L].theta2 = _motor_drv[LB]->get_current_position() 
                                                        + _motor_offset[LB];
                                                        
    /* Map joint velocities / 连杆实际转速值转换 */
    _leg_data[R].d_theta1 = -_motor_drv[RF]->get_current_rotate();
    _leg_data[R].d_theta2 = -_motor_drv[RB]->get_current_rotate();
    _leg_data[L].d_theta1 = _motor_drv[LF]->get_current_rotate();
    _leg_data[L].d_theta2 = _motor_drv[LB]->get_current_rotate();

    /* Update wheel motor feedback
       更新大疆 M3508 轮毂电机的反馈。左侧电机朝向与底盘前进方向相反，
       所以左轮线速度 dx 计算需添加负号。通过平动线速度积分计算出轮式绝对里程计 x。 */
    _wheel_drv[R]->update_feedback();
    _leg_data[R].w = _wheel_drv[R]->get_current_rotate() / _reduction_ratio;
    _leg_data[R].T_w_real = _wheel_drv[R]->get_current_torque()*0.3f/(3591.0f/187.0f) * _reduction_ratio;
    _leg_data[R].dx = _leg_data[R].w * _wheel_radius;
    _leg_data[R].x += (_leg_data[R].dx + last_dx[R])/2 * 0.001f;
    last_dx[R] = _leg_data[R].dx;
    
    _wheel_drv[L]->update_feedback();
    _leg_data[L].w = _wheel_drv[L]->get_current_rotate() / _reduction_ratio;
    _leg_data[L].T_w_real = _wheel_drv[L]->get_current_torque()*0.3f/(3591.0f/187.0f) * _reduction_ratio;
    _leg_data[L].dx = - _leg_data[L].w * _wheel_radius;
    _leg_data[L].x += (_leg_data[L].dx + last_dx[L])/2 * 0.001f;
    last_dx[L] = _leg_data[L].dx;

    /* Update Gimbal Yaw tracking / 刷新底盘对齐云台偏航角电机的角度和角速度 */
    _yaw_motor_drv->update_feedback();
    gimbal_yaw = wrap2pi_f32(_yaw_motor_drv->get_current_position() + _yaw_offset);
    gimbal_g_yaw = _yaw_motor_drv->get_current_rotate();

    time = dwt_drv_t::get_delta_t(&dwt_cnt);
    
    /* Execute 5-bar kinematics solver
       使用解析几何，根据连杆的两个关节角求解极轴下的极径 l（腿长）及极轴偏角 alpha。
       同时推导出当前收缩速度 d_l 与摆动速度 d_alpha。 */
    for(uint8_t i = 0; i < 2; i++)
    {
        status_t ret;
        float last_d_l = _leg_data[i].d_l;
        float last_d_beta = _leg_data[i].d_beta;
        ret = _kinematic_solver.solve(_leg_data[i].theta1, 
                                      _leg_data[i].theta2,
                                    _leg_data[i].d_theta1,
                                    _leg_data[i].d_theta2,
                                        &_leg_data[i].phi1,
                                        &_leg_data[i].phi2,
                                      &_leg_data[i].alpha,
                                       &_leg_data[i].l,
                                    &_leg_data[i].d_l,
                                     &_leg_data[i].d_alpha,
                                     &_leg_data[i].d_jx,
                                     &_leg_data[i].d_jy,
                                     &_leg_data[i].jx,
                                     &_leg_data[i].jy);
        if(ret != PYRO_OK)
        {
            _cnt.solver_error++;
        }
         
        /* Calculate tilt coordinates beta relative to vertical gravity line
           计算底盘虚拟腿和重力垂直线的偏角 beta，以及它的变化角速度 d_beta */
        _leg_data[i].beta = PI / 2 - _leg_data[i].alpha - pitch;
        _leg_data[i].d_beta = -_leg_data[i].d_alpha - g_pitch;
        _leg_data[i].gamma = -pitch;
        _leg_data[i].d_gamma = -g_pitch;

        /* Estimate second-order derivatives (accelerations) / 通过数值差分求二阶加速度项 */
        _leg_data[i].d2_beta = (_leg_data[i].d_beta - last_d_beta) / time;
        _leg_data[i].d2_l = (_leg_data[i].d_l - last_d_l) / time;
    }

    /* Update VMC Jacobian matrix
       求解轮腿几何的 VMC 映射雅可比矩阵（用于将极坐标虚拟力转换为关节电机的扭矩） */
    for(uint8_t i = 0; i < 2; i++)
    {
        status_t ret;

        ret = _kinematic_solver.get_VMC_value(_leg_data[i].theta1, 
                                             _leg_data[i].theta2,
                                             _leg_data[i].phi1,
                                             _leg_data[i].phi2,
                                             _leg_data[i].l,
                                             _leg_data[i].alpha,
                                             _leg_data[i].T_mat.pData);
        if(ret != PYRO_OK)
        {
            _cnt.solver_error++;
        }
    }
    
    /* Project body-frame acceleration to ground-aligned axes
       将机体坐标系的加速度投影到地理垂直轴与纵向水平轴。
       俯仰角以低头为正，因此由于 IMU 零位偏置沿 X 轴前移 `IMU_OFFSET_X`，在旋转偏航时会产生离心加速度，需加补偿。 */
    float kf_u = 0.0f;
    float kf_z[3] = {0.0f, 0.0f, 0.0f};
    float kf_estimated[3] = {0.0f, 0.0f, 0.0f};
    
    a_forward = a_x * arm_cos_f32(pitch)
                 + a_z * arm_sin_f32(pitch)
                 + g_yaw * g_yaw * IMU_OFFSET_X;
    a_upward = -a_x * arm_sin_f32(pitch)
                + a_z * arm_cos_f32(pitch);
    a_upward_lpf = a_upward_lpf * SUPPORT_FORCE_ACC_LPF_RC / (time + SUPPORT_FORCE_ACC_LPF_RC)
                 + a_upward * time / (time + SUPPORT_FORCE_ACC_LPF_RC);
                 
    /* Run Kalman filter for linear velocity
       使用平均左右轮速作为平动观测值 v_obs，输入卡尔曼滤波器，估计机体当前的绝对速度、加速度和位移量 */
    float v_obs = (_leg_data[R].dx + _leg_data[L].dx) / 2.0f;
    for(uint8_t i = 0; i < 2; i++)
    {
        kf_u = 0.0f;
        kf_z[0] = v_obs;
        kf_z[1] = a_forward;
        kf_z[2] = g_yaw;   
        _wheel_kf[i].update(kf_z, &kf_u, kf_estimated);
        _leg_data[i].kf_v = kf_estimated[0];
        _leg_data[i].kf_a = kf_estimated[1];
        _leg_data[i].kf_w = kf_estimated[2];
        _leg_data[i].kf_x += _leg_data[i].kf_v * 0.001f;
    }
}

/* FSM entry point called by scheduler / 底盘主控任务的 FSM 刷新入口 */
void wl_chassis_t::_fsm_execute()
{
    _cmd = &_current_cmd;
    
    /* Decouple Passive (disable) and Active mode / 被动模式与主动运行模式切换 */
    if (cmd_base_t::mode_t::PASSIVE == _cmd->mode)
        _fsm.change_state(&_state_passive)  ;
    else if (cmd_base_t::mode_t::ACTIVE == _cmd->mode)
        _fsm.change_state(&_state_active);

    __decide_cap(); /* Process supercapacitor board FSM / 超级电容行为判断 */
    _fsm.execute(this); /* Run FSM execute loop / 执行状态机的动作 */
    time = dwt_drv_t::get_timeline_ms() - last_time;
}

/* Transmit CMD packet to supercapacitor board / 向超电板发送数据包 */
void wl_chassis_t::_solve_wheel_power_limit(const float tau_motion[2],
                                            const float tau_uncontrolled[2],
                                            const float omega[2],
                                            float tau_out[2])
{
    power_controller_t &power_ctrl = power_controller_t::get_instance();

    for (uint8_t i = 0; i < 2; ++i)
    {
        power_node_t *node = _wheel_power_node[i];
        if (node == nullptr)
        {
            tau_out[i] = tau_motion[i] + tau_uncontrolled[i];
            _leg_data[i].predict_power = 0.0f;
            continue;
        }

        node->target_cmd = tau_motion[i];
        node->uncontrolled_cmd = tau_uncontrolled[i];
        node->rpm = omega[i];
        node->temp = 20.0f;
    }

    const float cap_extra_power =
        fp32_constrain(_power_data.cap_power, 0.0f,
                       _module_deps.power_limit_cfg.cap_extra_power_limit);
    power_ctrl.solve(_power_data.limit, _power_data.buffer_energy, cap_extra_power);

    for (uint8_t i = 0; i < 2; ++i)
    {
        power_node_t *node = _wheel_power_node[i];
        if (node == nullptr)
            continue;

        tau_out[i] = node->safe_cmd;
        _leg_data[i].predict_power =
            wl_power_predict(node->params,
                             node->safe_cmd - node->uncontrolled_cmd,
                             node->uncontrolled_cmd,
                             node->rpm,
                             node->temp);
    }
}

void wl_chassis_t::__send_supercap_command() const
{
    supercap_drv_t::get_instance()->send_cmd(_supercap_cmd);
}

/* Supercapacitor state manager / 超级电容状态管理逻辑 */
void wl_chassis_t::__decide_cap()
{
    static bool _last_status = false;
    static uint32_t _timer   = 0;
    static bool _delay_done  = false;

    /* Get chassis power output state from referee system / 获取裁判系统底盘是否有电源输出的指示 */
    bool current_status = referee_drv->get_data().robot_status.power_management_chassis_output;

    if (current_status)
    {
        /* Case A: Chassis power output is active / 情况 A：底盘电源有输出 */
        if (!_last_status)
        {
            /* Reset delay timers upon status transition / 刚切换至有输出状态，重置防抖及发送计时器 */
            _timer      = 0;
            _delay_done = false;
        }

        if (!_delay_done)
        {
            /* Process 1000 tick initial delay for cap bootstrap / 处理超级电容启动 1000 次周期的初始防抖延时 */
            if (++_timer >= 1000)
            {
                _delay_done           = true;
                _timer                = 0; 
                _supercap_cmd.use_cap = 1; /* Request to discharge capacitor / 允许开始使用超级电容功耗 */
                __send_supercap_command();
            }
        }
        else
        {
            /* Emit CAP active command every 10 ticks / 延时通过后，每 10 次周期给超电驱动下发一次 use_cap=1 */
            if (++_timer >= 10)
            {
                _timer                = 0;
                _supercap_cmd.use_cap = 1;
                __send_supercap_command();
            }
        }
    }
    else
    {
        /* Case B: Power cut detected / 情况 B：底盘断电或电源限制输出 */
        if (_last_status)
        {
            /* Immediately request bypass / disable capacitor usage / 切换至无电源输出，立刻将 use_cap 设为 0 并发送 */
            _supercap_cmd.use_cap = 0;
            __send_supercap_command();

            _delay_done = false;
            _timer      = 0;
        }
    }

    _last_status = current_status;
}
}
