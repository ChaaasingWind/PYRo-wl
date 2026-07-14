/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-07 15:14:47
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-05-28 09:01:23
 * @Description: Wheel-Legged Chassis module header file / 轮腿底盘模块头文件
 *
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved.
 */
#ifndef __PYRO_WL_CHASSIS_H__
#define __PYRO_WL_CHASSIS_H__

#include "pyro_module_base.h"
#include "pyro_kin.wl.h"
#include "pyro_supercap_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_ins.h"
#include "pyro_algo_pid.h"
#include "kf.h"
#include "pyro_power_control.h"

namespace pyro
{
/**
 * @brief DM joint motor configuration structure / 达妙关节电机配置结构体
 */
struct wl_dm_motor_cfg_t
{
    /* can tx id of motor / 电机 CAN 发送 ID */
    uint8_t tx_id;
    /* can rx id of motor / 电机 CAN 接收 ID */
    uint8_t rx_id;
    /* Which CAN bus the motor is connected to / 电机挂载的 CAN 总线 */
    bsp_can::which_can can;
    /* Offset angle to eliminate the bias of installation(rad) / 消除安装偏差的偏移角度(rad) */
    float offset_angle;
};

/**
 * @brief DJI 3508 configuration structure / 大疆 3508 电机配置结构体
 */
struct wl_dji_motor_cfg_t
{
    /* can tx id of motor / 电机 CAN 发送 ID */
    dji_motor_tx_frame_t::register_id_t tx_id;
    /* Which CAN bus the motor is connected to / 电机挂载的 CAN 总线 */
    bsp_can::which_can can;
};

/**
 * @brief PID configuration structure / PID 控制器配置结构体
 */
struct wl_pid_cfg_t
{
    float kp;
    float ki;
    float kd;
    float integral_limit; /* Integral limit / 积分限幅 */
    float max_out;        /* Maximum output limit / 最大输出限幅 */
};

/**
 * @brief Kalman filter configuration structure / 卡尔曼滤波器配置结构体
 */
struct wl_kf_cfg_t
{
   float *x_init; /* Initial state estimate / 状态初始化值 */
   float *P_init; /* Initial covariance estimate / 协方差初始化值 */
   float *A;      /* State transition matrix / 状态转移矩阵 */
   float *B;      /* Control input matrix / 控制输入矩阵 */
   float *H;      /* Observation matrix / 观测矩阵 */
   float *G;      /* Process noise gain matrix / 过程噪声增益矩阵 */
   float *Q;      /* Process noise covariance / 过程噪声协方差 */
   float *R;      /* Measurement noise covariance / 测量噪声协方差 */
};

/**
 * @brief Configuration structure for wheel-legged chassis / 轮腿底盘全局参数配置结构体
 */
struct wl_power_limit_cfg_t
{
    power_fit_params_t fit_params[2];
    float buffer_safe_energy;
    float buffer_kp;
    float buffer_ki;
    float buffer_kd;
    float cap_extra_power_limit;
};

struct wl_chassis_cfg_t
{
    /* Kinematic coefficients for the chassis / 底盘运动学常数 */
    wheel_legged_kin_t::phi_k_t phi_k;
    wheel_legged_kin_t::polar_k_t polar_k;
    wheel_legged_kin_t::vmc_k_t vmc_k;

    /* Joint motor configuration. The order is: front-right, rear-right, front-left, rear-left
       关节电机配置。数组顺序为：右前、右后、左前、左后 */
    wl_dm_motor_cfg_t joint_motor_cfg[4];

    /* Wheel motor configuration. The order is: right wheel, left wheel
       轮毂电机配置。数组顺序为：右轮、左轮 */
    wl_dji_motor_cfg_t wheel_motor_cfg[2];

    wl_dji_motor_cfg_t yaw_motor_cfg; /* Yaw motor (Gimbal) / 云台偏航电机配置 */
    float yaw_offset;                 /* Installation offset of yaw motor / 偏航角电机安装偏差 */

    /* PID configuration for the chassis control. The order is: right leg, left leg
       用于底盘控制的 PID 配置。数组顺序为：右腿、左腿 */
    wl_pid_cfg_t T_pid_cfg[2];   /* Joint torque PID / 关节力矩 PID */
    wl_pid_cfg_t d_T_pid_cfg[2]; /* Derivative joint torque PID / 关节力矩变化率 PID */
    wl_pid_cfg_t F_pid_cfg[2];   /* Expansion force PID / 伸缩力 PID */
    wl_pid_cfg_t d_F_pid_cfg[2]; /* Derivative expansion force PID / 伸缩力变化率 PID */

    /* Yaw PID configuration for the chassis control / 偏航角 PID 控制器配置 */
    wl_pid_cfg_t yaw_pid_cfg;
    wl_pid_cfg_t g_yaw_pid_cfg;

    /* PID controllers to control the bias between right leg angle and left leg angle
       用于控制左右腿摆角偏差的 PID 控制器配置 */
    wl_pid_cfg_t delta_pid_cfg;
    wl_pid_cfg_t d_delta_pid_cfg;

    wl_pid_cfg_t roll_pid_cfg; /* Roll angle compensation PID / 横滚角补偿 PID */

    /* Kalman filter configuration for the wheel velocity / 轮速卡尔曼滤波配置 */
    wl_kf_cfg_t wheel_kf_cfg[2];

    /* LQR coefficients for the chassis control. 2 rows x 6 columns, 12 values in total.
       Every value has 4 polynomial coefficients.
       底盘控制 of LQR 增益多项式系数。2行 x 6列 = 12个控制增益，每个控制增益包含 4 个三阶多项式拟合系数。 */
    float *lqr_coef;
    float *lqr_coef_over_step; /* LQR coefficients used in over-step stance / 跨步平衡姿态下的 LQR 系数 */

    float wheel_radius;     /* Wheel radius(m) / 车轮半径(m) */
    float reduction_ratio;  /* Gear reduction ratio of the wheel motor / 轮机减速比 */

    /* Rotation speed range of the joint motor(rad/s) / 关节电机转速范围限制 */
    float rotate_min;
    float rotate_max;

    /* Position range of the joint motor(rad) / 关节电机位置范围限制 */
    float position_min;
    float position_max;

    /* Torque range of the joint motor(Nm) / 关节电机力矩范围限制 */
    float torque_min;
    float torque_max;

    wl_power_limit_cfg_t power_limit_cfg; /* Power controller configuration / 功率限制控制器配置 */
};

/**
 * @brief Command structure for wheel-legged chassis / 轮腿底盘控制指令结构体
 */
struct wl_cmd_t final : public cmd_base_t
{
    /* Linear velocity in x direction(m/s), positive toward the front of chassis
       底盘 X 方向目标线速度(m/s)，向前为正，向后为负 */
    float vx;
    /* Linear velocity in y direction(m/s), positive toward the left of chassis
       底盘 Y 方向目标线速度(m/s)，向左为正，向右为负 */
    float vy;
    /* Angular velocity in z direction(rad/s), positive for counter-clockwise
       底盘 Z 方向（偏航角）目标角速度(rad/s)，逆时针为正，顺时针为负 */
    float vz;
    /* Yaw angle of the chassis(rad), counter-clockwise positive
       底盘目标偏航角(rad)，逆时针为正 */
    float yaw;
    /* Leg length of both sides after normalization, value is between 0 and 1
       归一化后的左右腿长，取值在 0 到 1 之间 */
    float l_leg;
    float r_leg;

    /* Manual control leg angles / 手动控制目标腿角 */
    float l_angle;
    float r_angle;

    /* Active chassis states enum / 主动控制状态枚举 */
    enum active_mode_t
    {
        NORMAL = 0,          /* Normal balance/movement / 正常站立自平衡与行驶 */
        READY = 1,           /* Standing up stance transition / 起身准备与过渡 */
        TEST = 2,            /* Zero torque motor test / 电机零力矩测试模式 */
        REVERSE = 3,         /* Flip/reverse leg sweep / 翻转/倒地恢复摆腿 */
        OVER_STEP = 4,       /* Climb step / over-step action / 跃障/跨步翻越动作 */
        OVER_STEP_READY = 5, /* Over-step balanced stance / 跃障障上 LQR 平衡姿态 */
        OVER_STEP_RESET = 6, /* Reset after over-step / 跃障复位恢复 */
        CONTROL = 7,         /* Manual joint debug / 手动关节直控调试 */
        SPIN = 8             /* Spin-top (top spin) mode / 小陀螺旋转模式 */
    } active_mode, last_active_mode;

    /* Constructor / 构造函数，赋初始零值 */
    wl_cmd_t() : vx(0), vy(0), vz(0), yaw(0), l_leg(0), r_leg(0), l_angle(0), r_angle(0), active_mode(TEST), last_active_mode(TEST)
    {
    }
};

/**
 * @brief Public runtime context for wheel-legged chassis diagnostics.
 */
struct wl_leg_ctx_t
{
    /* Angle of front/rear linkages with forward horizontal direction (rad) */
    float theta1{0.0f};
    float theta2{0.0f};
    float d_theta1{0.0f};
    float d_theta2{0.0f};
    float phi1{0.0f};
    float phi2{0.0f};

    float l{0.0f};
    float ref_l{0.0f};
    float d_l{0.0f};
    float d2_l{0.0f};
    float ref_d_l{0.0f};

    float alpha{0.0f};
    float d_alpha{0.0f};
    float ref_d_alpha{0.0f};

    float beta{0.0f};
    float d_beta{0.0f};
    float d2_beta{0.0f};
    float gamma{0.0f};
    float d_gamma{0.0f};

    float x{0.0f};
    float dx{0.0f};
    float w{0.0f};

    arm_matrix_instance_f32 T_mat{};
    float T_mat_val[4]{};

    float T[2]{};
    float F[2]{};

    float T_w{0.0f};
    float T_w_balance{0.0f};
    float T_w_move{0.0f};
    float T_w_turn{0.0f};
    float T_w_real{0.0f};
    float T_w_out{0.0f};

    float lqr_gain[12]{};
    float x_gain{0.0f};
    float d_x_gain{0.0f};

    float P{0.0f};

    float jx{0.0f};
    float jy{0.0f};
    float d_jx{0.0f};
    float d_jy{0.0f};

    float kf_v{0.0f};
    float kf_a{0.0f};
    float kf_w{0.0f};
    float kf_x{0.0f};

    float predict_power{0.0f};

    float beta_bias{0.0f};
    float d_beta_bias{0.0f};
    float gamma_bias{0.0f};
    float d_gamma_bias{0.0f};
    float x_bias{0.0f};
    float d_x_bias{0.0f};
};

struct wl_chassis_data_ctx_t
{
    wl_leg_ctx_t leg[2]{};

    float yaw{0.0f};
    float pitch{0.0f};
    float roll{0.0f};
    float g_yaw{0.0f};
    float g_pitch{0.0f};
    float g_roll{0.0f};

    float accel_x{0.0f};
    float accel_y{0.0f};
    float accel_z{0.0f};
    float accel_forward{0.0f};
    float accel_upward{0.0f};
    float accel_upward_lpf{0.0f};

    float gimbal_yaw{0.0f};
    float gimbal_g_yaw{0.0f};

    float yaw_ref{0.0f};
    float g_yaw_ref{0.0f};
    float delta_mea{0.0f};
    float d_delta_mea{0.0f};
    float d_delta_ref{0.0f};
    float T_l_gain{0.0f};
    float roll_gain{0.0f};

    float wheel_radius{0.0f};
    float reduction_ratio{0.0f};

    struct
    {
        uint16_t solver_error{0};
    } cnt;

    struct
    {
        uint8_t is_aerial{0};
        uint8_t aerial_cnt{0};
        float test{0.0f};
    } flag;

    struct
    {
        uint8_t normal{0};
        uint8_t ready{0};
        uint8_t test{0};
        uint8_t reverse{0};
        uint8_t over_step{0};
        uint8_t over_step_ready{0};
        uint8_t over_step_reset{0};
        uint8_t control{0};
        uint8_t spin{0};
    } active_mode_flag;
};

struct wl_chassis_ctx_t
{
    struct
    {
        dm_motor_drv_t *joint[4]{};
        dji_m3508_motor_drv_t *wheel[2]{};
        dji_gm_6020_motor_drv_t *yaw{nullptr};
    } motor;

    struct
    {
        pid_t *T[2]{};
        pid_t *d_T[2]{};
        pid_t *F[2]{};
        pid_t *d_F[2]{};
        pid_t *yaw{nullptr};
        pid_t *g_yaw{nullptr};
        pid_t *delta{nullptr};
        pid_t *d_delta{nullptr};
        pid_t *roll{nullptr};
    } pid;

    struct
    {
        power_node_t *wheel_node[2]{};
        supercap_drv_t::chassis_cmd_t supercap_cmd{};
        supercap_drv_t::cap_feedback_t cap_feedback{};
        float chassis_power{0.0f};
        float voltage{0.0f};
        float cap_power{0.0f};
        float limit{0.0f};
        float buffer_energy{0.0f};
        float total_predicted_power{0.0f};
    } power;

    wl_chassis_data_ctx_t data{};
    ins_drv_t *ins{nullptr};
    wl_cmd_t *cmd{nullptr};
};

struct wl_chassis_module_params_t
{
    using CmdType    = wl_cmd_t;
    using ModuleDeps = wl_chassis_cfg_t;
    using ModuleCtx  = wl_chassis_ctx_t;
};

/**
 * @brief Wheel-legged chassis control module class / 轮腿底盘控制模块类
 */
class wl_chassis_t final : public module_base_t<wl_chassis_t, wl_chassis_module_params_t>
{
   friend class module_base_t<wl_chassis_t, wl_chassis_module_params_t>;
   friend class vofa_drv_t;

public:
   wl_chassis_t(const wl_chassis_t &)            = delete;
   wl_chassis_t &operator=(const wl_chassis_t &) = delete;
   using data_ctx_t   = wl_chassis_data_ctx_t;
   using wl_context_t = pyro::wl_chassis_ctx_t;

   /* Telemetry and state query interfaces / 传感器与状态数据查询接口 */
   status_t get_cur_angle(float *r_angle, float *l_angle);
   status_t get_cur_d_angle(float *r_angle, float *l_angle);
   status_t get_cur_length(float *r_leg, float *l_leg);
   status_t get_cur_p_torque(float *r_torque, float *l_torque);
   status_t get_cur_ins_yaw(float* temp_yaw);
   status_t get_cur_x_bias(float* r_x_bias, float* l_x_bias);
   status_t get_cur_beta_bias(float* r_beta_bias, float* l_beta_bias);
   status_t get_cur_gamma_bias(float* gamma_bias);

   /* Finite State Machine status management / 有限状态机状态标志获取与清除 */
   uint8_t get_status_flag(wl_cmd_t::active_mode_t mode);
   status_t clear_status_flag(wl_cmd_t::active_mode_t mode);

private:
    /**
     * @brief Constructor, initializes configuration / 构造函数，隐式调用
     */
    wl_chassis_t();
    ~wl_chassis_t() override = default;

    /* Base class interface overrides / 基类重写接口 */
    /**
     * @brief Initialize the module dependencies / 初始化模块驱动、PID和滤波器依赖
     */
    status_t _init() override;

    /**
     * @brief Update chassis feedback states / 周期性更新底盘传感器、电机和运动学状态
     */
    void _update_feedback() override;

    /**
     * @brief Run the FSM iteration / 有限状态机决策与轮腿控制计算主入口
     */
    void _fsm_execute() override;

    /* Array index definitions / 数组索引定义 */
    enum
    {
        RF = 0, /* Front Right joint motor / 右前直驱关节电机 */
        RB = 1, /* Rear Right joint motor / 右后直驱关节电机 */
        LF = 2, /* Front Left joint motor / 左前直驱关节电机 */
        LB = 3, /* Rear Left joint motor / 左后直驱关节电机 */
    };
    enum
    {
        R = 0, /* Right leg / 右腿 */
        L = 1, /* Left leg / 左腿 */
    };

    /* Kinematic solver for the 5-bar linkage / 五连杆运动学解析求解器 */
    wheel_legged_kin_t _kinematic_solver;









    /* Internal LQR coefficients lookup tables / LQR 状态反馈增益系数多项式表 */
    float _lqr_cof[48];            /* Normal state LQR / 正常态 LQR 系数 */
    float _lqr_cof_over_step[48];  /* Over-step stance LQR / 跃障态 LQR 系数 */




    float _yaw_offset;
    float _motor_offset[4]; /* Joint installation calibration offset / 连杆关节绝对零位偏置 */



    /* Wheel velocity Kalman state estimators / 轮速卡尔曼估计器 */
    kf_t _wheel_kf[2];




    /* 有限状态机 - 主动运行状态容器类 */
    class fsm_active_t : public fsm_t<wl_chassis_t>
    {
    public:
        /* state_test_t: Sends zero torque to all motors for debugging / 零力矩使能测试态 */
        class state_test_t : public state_t<wl_chassis_t>
        {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
        } _state_test;

        /* state_ready_t: Prepares the robot to stand up smoothly / 起身准备状态（规划起立） */
        class state_ready_t : public state_t<wl_chassis_t>
        {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
            void calc_target_value(wl_chassis_t *owner); /* Interpolates stand-up targets / 起身轨迹插值 */
        } _state_ready;

        /* state_normal_t: Main state handling balancing, LQR, power capping and driving / 核心自平衡与运动状态 */
        class state_normal_t : public state_t<wl_chassis_t>
        {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
            void calc_support_force(wl_chassis_t *owner); /* Estimates wheel vertical support force / 支持力估算 */
        } _state_normal;

        /* state_reverse_t: Flips joint positions for recovery preparation / 翻转/反向摆腿状态 */
        class state_reverse_t : public state_t<wl_chassis_t>
        {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
        } _state_reverse;

        /* state_over_step_t: Controls sweep joints and wheel speeds to climb high step / 跃障动作状态 */
        class state_over_step_t : public state_t<wl_chassis_t>
        {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
            void calc_target_value(wl_chassis_t *owner); /* Plans sweep trajectories / 规划越障摆腿轨迹 */
        } _state_over_step;

        /* state_over_step_ready_t: Stance LQR balance using over-step coefficients / 跃障后特殊的 LQR 平衡姿态 */
        class state_over_step_ready_t : public state_t<wl_chassis_t>
        {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
        } _state_over_step_ready;

        /* state_over_step_reset_t: Restores post-jump geometry to ready height / 越障后的复位恢复状态 */
        class state_over_step_reset_t : public state_t<wl_chassis_t>
        {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
            void calc_target_value(wl_chassis_t *owner);
        } _state_over_step_reset;

        /* state_control_t: Bypasses balancing to direct control joint polar target / 关节直控调试状态 */
        class state_control_t : public state_t<wl_chassis_t>
        {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
        } _state_control;

        /* state_spin_t: Executes chassis rapid turning (top-spin) / 小陀螺旋转模式 */
        class state_spin_t : public state_t<wl_chassis_t>
        {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
        } _state_spin;

        /* Base active state transition handlers / 主动模式全局生命周期方法 */
        void on_enter(wl_chassis_t *owner) override;
        void on_execute(wl_chassis_t *owner) override;
        void on_exit(wl_chassis_t *owner) override;
    } _state_active;

    /* state_passive_t: Safely shuts down and brakes the robot wheels / 安全被动失能状态 */
    class state_passive_t : public state_t<wl_chassis_t>
    {
    public:
        void enter(wl_chassis_t *owner) override;
        void execute(wl_chassis_t *owner) override;
        void exit(wl_chassis_t *owner) override;
    } _state_passive;

    friend class fsm_active_t;
    friend class state_passive_t;

    fsm_t<wl_chassis_t> _fsm; /* Finite State Machine manager / 有限状态机管理器 */

    void _solve_wheel_power_limit(const float tau_motion[2],
                                  const float tau_uncontrolled[2],
                                  const float omega[2],
                                  float tau_out[2]);
    void __send_supercap_command() const; /* Sends command to cap driver / 发送数据包到超级电容驱动 */
    void __decide_cap();                   /* Supercapacitor logic state machine / 超级电容充放电行为机 */
};

}
#endif // __PYRO_WL_CHASSIS_H__
