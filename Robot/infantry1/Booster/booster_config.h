#ifndef PYRO_ROBOT_BOOSTER_CONFIG_H
#define PYRO_ROBOT_BOOSTER_CONFIG_H





//拨弹盘速度环pid参数
constexpr float TRIGGER_SPEED_PID_KP = 0.16f;
constexpr float TRIGGER_SPEED_PID_KD = 0.0f;

//拨弹盘位置环pid参数
constexpr float TRIGGER_POS_PID_KP = 2300.0f;
constexpr float TRIGGER_POS_PID_KD = 0.0002f;

//摩擦轮速度环pid参数
constexpr float FRIC_SPEED_PID_KP = 0.3f;
constexpr float FRIC_SPEED_PID_KD = 0.00002f;




//云台差的东西
//1.热量控制
//2.弹速闭环
//3.自瞄控制
//4.ui绘制
//5.功率控制

#endif