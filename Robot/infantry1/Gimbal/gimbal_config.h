#ifndef PYRO_ROBOT_CONFIG_H
#define PYRO_ROBOT_CONFIG_H


//手动模式下------------------------------------------

//yaw轴速度环pid参数
constexpr float YAW_SPEED_PID_KP = 3.5f;
constexpr float YAW_SPEED_PID_KI = 0.0f;
constexpr float YAW_SPEED_PID_KD = 0.0f;

//yaw轴位置环pid参数
constexpr float YAW_POS_PID_KP = 18.0f;
constexpr float YAW_POS_PID_KI = 0.0f;
constexpr float YAW_POS_PID_KD = 0.0f;

  
//pitch轴力矩输出部分的pid参数
constexpr float DM_POS_PITCH_KP = 25.0f;
constexpr float DM_POS_PITCH_KD = 0.3f;

//pitch轴达妙mit控制阻抗系数
constexpr float DM_MOT_PITCH_KP = 25.0f;
constexpr float DM_MOT_PITCH_KD = 0.7f;

//pitch轴物理限幅参数
constexpr float PITCH_LIMIT_MAX = 2.1f;
constexpr float PITCH_LIMIT_MIN = 2.90f;


constexpr float PITCH_ALIGN_TARGET_RAD = 2.1f;
constexpr float YAW_ALIGN_TARGET_RAD   = -2.1f;

constexpr float PITCH_K_GRAVITY_COS = -0.8f; // 水平方向质心补偿
constexpr float PITCH_K_GRAVITY_SIN = 0.0f; // 垂直方向质心补偿


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






#endif
