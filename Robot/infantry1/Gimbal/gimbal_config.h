#ifndef PYRO_ROBOT_CONFIG_H
#define PYRO_ROBOT_CONFIG_H

//自瞄模式下-------------------------------------------

//yaw轴速度环pid参数
#define AUTO_YAW_SPEED_PID_KP 20.0f
#define AUTO_YAW_SPEED_PID_KI 0.0f
#define AUTO_YAW_SPEED_PID_KD 0.0f

//yaw轴位置环pid参数
#define AUTO_YAW_POS_PID_KP 23.0f
#define AUTO_YAW_POS_PID_KI 0.0f
#define AUTO_YAW_POS_PID_KD 0.0f

//pitch轴达妙mit控制阻抗系数
#define AUTO_DM_MOT_PITCH_KP 20.0f
#define AUTO_DM_MOT_PITCH_KI 0.0f
#define AUTO_DM_MOT_PITCH_KD 0.7f

//pitch轴达妙mit控制的重力补偿的pid的参数
#define AUTO_PITCH_DM_MOT_KP 28.0f
#define AUTO_PITCH_DM_MOT_KI 0.0f
#define AUTO_PITCH_DM_MOT_KD 0.5f

//手动模式下------------------------------------------

//yaw轴速度环pid参数
#define YAW_SPEED_PID_KP 15.0f
#define YAW_SPEED_PID_KI 0.0f
#define YAW_SPEED_PID_KD 0.0f

//yaw轴位置环pid参数
#define YAW_POS_PID_KP 8.0f
#define YAW_POS_PID_KI 0.0f
#define YAW_POS_PID_KD 0.0f

  


//pitch轴达妙mit控制阻抗系数
#define DM_MOT_PITCH_KP 28.0f
#define DM_MOT_PITCH_KI 0.0f
#define DM_MOT_PITCH_KD 0.7f

//pitch轴物理限幅参数
#define PITCH_LIMIT_MAX 1.60f
#define PITCH_LIMIT_MIN 2.80f


//--------------------------------------------------------

//拨弹盘单发情况速度环pid参数
#define TRIGGER_SINGLE_SPEED_PID_KP 0.16f
#define TRIGGER_SINGLE_SPEED_PID_KI 0.0f
#define TRIGGER_SINGLE_SPEED_PID_KD 0.0f

//拨弹盘单发情况位置环pid参数
#define TRIGGER_SINGLE_POS_PID_KP 2300.0f
#define TRIGGER_SINGLE_POS_PID_KI 0.0f
#define TRIGGER_SINGLE_POS_PID_KD 0.0f

//拨弹盘连发情况速度环pid参数
#define TRIGGER_BURST_SPEED_PID_KP 0.16f
#define TRIGGER_BURST_SPEED_PID_KI 0.0f
#define TRIGGER_BURST_SPEED_PID_KD 0.0002f

//摩擦轮速度环pid参数
#define FRIC_SPEED_PID_KP 0.3f
#define FRIC_SPEED_PID_KI 0.0f
#define FRIC_SPEED_PID_KD 0.00002f




constexpr float PITCH_ALIGN_TARGET_RAD = 0.0f;
constexpr float YAW_ALIGN_TARGET_RAD = 1.97f;

constexpr float PITCH_K_GRAVITY_COS = 1.0f; // 水平方向质心补偿
constexpr float PITCH_K_GRAVITY_SIN = -0.5f; // 垂直方向质心补偿



#endif
