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
 * @brief Wheel-legged chassis control module class / 轮腿底盘控制模块类
 */
struct wl_chassis_ctx_t
{
};

struct wl_chassis_module_params_t
{
    using CmdType    = wl_cmd_t;
    using ModuleDeps = wl_chassis_cfg_t;
    using ModuleCtx  = wl_chassis_ctx_t;
};

class wl_chassis_t final : public module_base_t<wl_chassis_t, wl_chassis_module_params_t>
{
   friend class module_base_t<wl_chassis_t, wl_chassis_module_params_t>;
   friend class vofa_drv_t;

public:
   wl_chassis_t(const wl_chassis_t &)            = delete;
   wl_chassis_t &operator=(const wl_chassis_t &) = delete;

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
   
    /* Power controller instance / 电机功率限制控制器 */
    power_node_t *_wheel_power_node[2];

    /* INS (Inertial Navigation System) driver / 惯导系统驱动指针 */
    ins_drv_t *_ins_drv;

    /* Chassis pitch, roll, yaw angles and rates / 底盘欧拉角以及角速度反馈 */
    float yaw, pitch, roll;
    float g_yaw, g_pitch, g_roll;
    
    /* Linear accelerations / 纵向、法向及滤波惯性加速度 */
    float a_x, a_y, a_z, a_forward, a_upward, a_upward_lpf;
    
    /* Yaw gimbal motor feedbacks / 云台偏航电机的绝对位置与速度 */
    float gimbal_yaw, gimbal_g_yaw;

    /* PID controllers for Yaw orientation tracking / 偏航角追踪控制 PID */
    pid_t* _yaw_pid;
    pid_t* _g_yaw_pid;
    float _yaw_ref;
    float _g_yaw_ref;

    /* PID controllers to control the leg angle bias / 左右腿角度平衡（防劈叉）控制器 */
    pid_t* _delta_pid;
    pid_t* _d_delta_pid;
    float _delta_mea;
    float _d_delta_mea;
    float _d_delta_ref;

    /* Roll stabilization PID / 横滚稳定控制 PID */
    pid_t* _roll_pid;

    /* Internal LQR coefficients lookup tables / LQR 状态反馈增益系数多项式表 */
    float _lqr_cof[48];            /* Normal state LQR / 正常态 LQR 系数 */
    float _lqr_cof_over_step[48];  /* Over-step stance LQR / 跃障态 LQR 系数 */

    /* DM joint motors driver / 直驱连杆关节电机驱动 */
    dm_motor_drv_t *_motor_drv[4];

    /* DJI wheel motors driver / 大疆 3508 轮毂驱动 */
    dji_m3508_motor_drv_t *_wheel_drv[2];
    
    /* DJI gimbal yaw motor driver / 大疆 6020 云台偏航电机驱动 */
    dji_gm_6020_motor_drv_t *_yaw_motor_drv;
    
    float _wheel_radius;
    float _reduction_ratio;
    float _yaw_offset;
    float _motor_offset[4]; /* Joint installation calibration offset / 连杆关节绝对零位偏置 */
    
    float _T_w_gain;
    float _x_gain;
    float T_l_gain; 
    float roll_gain;

    /**
     * @brief Detailed mechanical and kinematic data of each leg / 单侧轮腿部状态结构体
     */
    struct leg_data_t
    {
        /* Angle of front/rear linkages with forward horizontal direction (rad)
           前部摆杆与后部摆杆分别与正前方的夹角 (rad) */
        float theta1, theta2;
        /* Angular rates of the linkages / 连杆旋转角速度 (rad/s) */
        float d_theta1, d_theta2;
        /* Inner structural linkage angles / 五连杆下半部分小连杆的辅助几何计算角 */
        float phi1, phi2;
        
        /* Virtual leg polar coordinates / 虚拟摆腿极坐标表示 */
        float l;      /* Polar radius (leg length, m) / 实际极轴半径（腿长，m） */
        float ref_l;  /* Reference leg length target / 设定极轴半径（目标腿长） */
        float d_l;    /* Polar velocity (leg contraction rate) / 腿长伸缩速度 */
        float d2_l;   /* Polar acceleration / 腿部伸缩加速度 */
        float ref_d_l;/* Reference contraction rate / 摆长闭环反馈计算出的速度目标值 */

        float alpha;      /* Polar angle of the leg (rad) / 实际极坐标摆角 */
        float d_alpha;    /* Polar angular rate / 极坐标摆摆动角速度 */
        float ref_d_alpha;/* Reference angular rate / 摆角闭环计算出的目标摆角速度 */

        /* Leg tilt coordinate relative to vertical direction / 虚拟腿相对于重力垂直线的倾斜角 */
        float beta;       /* Tilt angle (rad) / 倾斜角（$\beta = \pi/2 - \alpha - pitch$） */
        float d_beta;     /* Angular rate / 倾斜角速度 */
        float d2_beta;    /* Angular acceleration / 倾斜角速度变化率 */
        
        /* Yaw/yaw-derived coordinates / 航向角微分状态 */
        float gamma;
        float d_gamma;
        
        /* Cartesian displacement of the contact point / 接地轮中心沿车身的横向位移 */
        float x;          /* Displacement (m) / 位移累计 (m) */
        float dx;         /* Linear speed (m/s) / 轮中心横向平动速度 (m/s) */
        float w;          /* Wheel angular velocity / 车轮实际旋转角速度 */
        
        /* VMC Jacobian / VMC 力矩转换雅可比矩阵 */
        arm_matrix_instance_f32 T_mat;
        float T_mat_val[4];
        
        /* Motor torques output: 0 is front joint, 1 is rear joint
           分配给关节电机的实际指令力矩：0 为前直驱关节，1 为后直驱关节 */
        float T[2];
        /* Virtual force vector: 0 is expansion force F_leg, 1 is hip torque T_hip
           极坐标系虚拟合力向量：0 为极轴方向作用力，1 为旋转切向作用力 */
        float F[2];
        
        /* Decomposed wheel commands / 轮毂电机的合成指令扭矩 */
        float T_w;         /* Commanded total wheel torque / 轮机总输出指令力矩 */
        float T_w_balance; /* Balance term / 俯仰与姿态平衡项 */
        float T_w_move;    /* Motion term / 前进平移驱动项 */
        float T_w_turn;    /* Yaw turn term / 转向差分力矩项 */
        float T_w_real;    /* Feed-forward real torque / 电机实际输出力矩估值 */
        float T_w_out;     /* Capped output torque from power controller / 经过功率限制最终下发的扭矩 */
        
        /* Gain scheduled LQR gains for this leg / 经腿长插值解算出的 12 个 LQR 反馈控制增益 */
        float lqr_gain[12];
        float x_gain;
        float d_x_gain;
        
        float P; /* Vertical ground reaction support force / 轮对地垂直支持力估计值 (N) */
        
        /* Contact positions in body coordinate / 接地触点沿机体坐标系的位移与速度估计 */
        float jx, jy;
        float d_jx, d_jy;

        /* Wheel velocity state observer / 轮速观测器滤波输出 */
        float kf_v; /* Filtered velocity / 滤波速度 */
        float kf_a; /* Filtered acceleration / 滤波加速度 */
        float kf_w; /* Filtered angular rate / 滤波角速度 */
        float kf_x; /* Filtered position / 滤波位移 */
        
        float predict_power; /* Predicted motor electrical power / 预测功耗 */

        /* LQR state errors / LQR 状态机状态偏差项 */
        float beta_bias;
        float d_beta_bias;
        float gamma_bias;
        float d_gamma_bias;
        float x_bias;
        float d_x_bias;
    } _leg_data[2];

    /* Wheel velocity Kalman state estimators / 轮速卡尔曼估计器 */
    kf_t _wheel_kf[2];
    
    /* VMC joint PIDs / 极坐标虚拟力伺服 PID 控制器 */
    pid_t *_T_pid[2];
    pid_t *_d_T_pid[2];
    pid_t *_F_pid[2];
    pid_t *_d_F_pid[2];

    /* Statistics counter / 运动学解算异常监测计数器 */
    struct
    {
        uint16_t solver_error;
    } _cnt;

    /* Aerial/Takeoff flags / 腾空检测标志位 */
    struct
    {
      uint8_t is_aerial = 0;   /* Is the robot currently flying / 底盘是否处于腾空状态 */
      uint8_t aerial_cnt = 0;  /* Debounce counter / 空中/着陆滤波防抖计数器 */
      float test = 0;
    } _flag;

    /* FSM transition state flags / 底盘各子状态的就绪/完成标志位 */
    struct
    {
      uint8_t normal = 0;
      uint8_t ready = 0;
      uint8_t test = 0;
      uint8_t reverse = 0;
      uint8_t over_step = 0;
      uint8_t over_step_reset = 0;
      uint8_t control = 0;
    } _active_mode_flag;

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
    wl_cmd_t *_cmd;           /* Received cmd pointer / 当前生效指令指针 */

    supercap_drv_t::chassis_cmd_t _supercap_cmd; /* Supercapacitor control command / 降压超级电容控制命令包 */
    
    /* Referee power metrics / 裁判系统反馈的真实电源与能量数据 */
    struct
    {
      float chassis_power;
      float voltage;
      float cap_power;
      float limit;
      float buffer_energy;
    } _power_data;
   
    void _solve_wheel_power_limit(const float tau_motion[2],
                                  const float tau_uncontrolled[2],
                                  const float omega[2],
                                  float tau_out[2]);

    void __send_supercap_command() const; /* Sends command to cap driver / 发送数据包到超级电容驱动 */
    void __decide_cap();                   /* Supercapacitor logic state machine / 超级电容充放电行为机 */
};

}
#endif // __PYRO_WL_CHASSIS_H__
