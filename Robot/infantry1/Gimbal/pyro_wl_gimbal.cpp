#include "pyro_wl_gimbal.h"
#include "gimbal_config.h"
#include "pyro_algo_common.h"
#include "pyro_dwt_drv.h"
#include "pyro_ins.h"
#include "dsp/fast_math_functions.h"
#include <algorithm>

namespace pyro
{

wl_gimbal_t::wl_gimbal_t() : module_base_t("wl_gimbal")
{
}

status_t wl_gimbal_t::_init()
{
    _ctx                                      = {};
    _ctx.motor                                = _module_deps.motor_deps;
    _ctx.pid                                  = _module_deps.pid_deps;

    return PYRO_OK;
}

void wl_gimbal_t::_update_feedback()
{
    // 读取 IMU 数据作为云台姿态反馈
    ins_drv_t::get_instance()->get_rads_n(&_ctx.data.imu.yaw,
                                          &_ctx.data.imu.pitch,
                                          &_ctx.data.imu.roll);

    ins_drv_t::get_instance()->get_gyro_b(&_ctx.data.imu.gyro[0],
                                          &_ctx.data.imu.gyro[1],
                                          &_ctx.data.imu.gyro[2]);

    ins_drv_t::get_instance()->get_accel_b(&_ctx.data.imu.accel[0], 
                                           &_ctx.data.imu.accel[1], 
                                           &_ctx.data.imu.accel[2]);

    _ctx.data.imu.timestamp = xTaskGetTickCount();

    //更新电机反馈
    _module_deps.motor_deps.pitch->update_feedback();
    _module_deps.motor_deps.yaw->update_feedback();


    _ctx.data.state.pitch.online = _module_deps.motor_deps.pitch->is_online();
    _ctx.data.state.pitch.pos    = _module_deps.motor_deps.pitch->get_current_position();
    _ctx.data.state.pitch.vel    = _module_deps.motor_deps.pitch->get_current_rotate();
    _ctx.data.state.pitch.temp   = _module_deps.motor_deps.pitch->get_temperature();
    _ctx.data.state.pitch.torque = _module_deps.motor_deps.pitch->get_current_torque();
    

    _ctx.data.state.yaw.online   = _module_deps.motor_deps.yaw->is_online();
    _ctx.data.state.yaw.pos      = _module_deps.motor_deps.yaw->get_current_position();
    _ctx.data.state.yaw.vel      = _module_deps.motor_deps.yaw->get_current_rotate();
    _ctx.data.state.yaw.temp     = _module_deps.motor_deps.yaw->get_temperature();
    _ctx.data.state.yaw.torque   = _module_deps.motor_deps.yaw->get_current_torque();

    _ctx.data.target_yaw_vel     = _current_cmd.yawVel;
    _ctx.data.target_pitch_vel   = _current_cmd.pitchVel;
    _ctx.data.motionState        = _current_cmd.state_cmd;
    _ctx.data.mode               = _current_cmd.mode;

    static uint32_t dwtCnt;
    _ctx.data.dt = pyro::dwt_drv_t::get_delta_t(&dwtCnt);
    
}



void wl_gimbal_t::updatePitch()
{
    // 在线检测
    if (!_module_deps.motor_deps.pitch->is_online())
    {
        _module_deps.pid_deps.pitch_pos->clear();
        _ctx.data.output.targetPitchSpeed       = 0.0f;
        _ctx.data.output.pitchFeedforwardTorque = 0.0f;
        _ctx.data.output.pitchEn                = false;
        return;
    }
        
    // IMU and motor encoder move in the same direction. Keep their measured
    // relative offset when converting the target IMU angle to motor angle.
    float pitch_pos = _ctx.data.state.pitch.pos;
    float offsetPitch = pitch_pos - _ctx.data.imu.pitch;
    float targetMotorRaw = _ctx.data.telem.targetPitchRad + offsetPitch;


    // 角度限制
    if (targetMotorRaw > PITCH_LIMIT_MIN)
    {
        targetMotorRaw = PITCH_LIMIT_MIN;
    }
        
    else if (targetMotorRaw < PITCH_LIMIT_MAX)
    {
        targetMotorRaw = PITCH_LIMIT_MAX;
    }

    // Keep the clamped target consistent with the IMU coordinate.
    _ctx.data.telem.targetPitchRad = targetMotorRaw - offsetPitch;

    //云台俯仰轴的重力补偿和位置控制
    float gravityFf           = PITCH_K_GRAVITY_COS * arm_cos_f32(_ctx.data.imu.pitch) + PITCH_K_GRAVITY_SIN * arm_sin_f32(_ctx.data.imu.pitch);


    float pitchIntegralTorque = _module_deps.pid_deps.pitch_pos->calculate(
                    _ctx.data.telem.targetPitchRad, _ctx.data.imu.pitch);
    float totalFf             = gravityFf + pitchIntegralTorque;//总扭矩合成


    float targetPitchSpeed =  _ctx.data.target_pitch_vel;


    if (std::abs(targetPitchSpeed) > 0.1f)//如果俯仰速度过大，限幅
        targetPitchSpeed = targetPitchSpeed > 0.0f ? 0.1f : -0.1f;

    _ctx.data.output.targetPitchPos         = targetMotorRaw;
    _ctx.data.output.targetPitchSpeed       = targetPitchSpeed;
    _ctx.data.output.pitchFeedforwardTorque = -totalFf;
    _ctx.data.output.pitchEn                = true;
}


void wl_gimbal_t::updateYaw()
{
    /*1. 在线检测 */
    if (!_ctx.data.state.yaw.online) {
        _ctx.data.telem.targetYawRad = _ctx.data.imu.yaw;
        _module_deps.pid_deps.yaw_pos->clear();
        _module_deps.pid_deps.yaw_spd->clear();
        _ctx.data.output.yawCurrent = 0.0f;
        return;
    }


    _module_deps.pid_deps.yaw_pos->
        set_gains(YAW_POS_PID_KP, YAW_POS_PID_KI, YAW_POS_PID_KD);
    _module_deps.pid_deps.yaw_spd->
        set_gains(YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD);


    
    //角度归一化
    float alignedTgtYaw   = _ctx.data.imu.yaw + wrapAngle(_ctx.data.telem.targetYawRad - _ctx.data.imu.yaw);
    float yawPosOut       = _module_deps.pid_deps.yaw_pos->calculate(alignedTgtYaw, _ctx.data.imu.yaw);
    float tgtYawSpd       = yawPosOut;
    //后期要加上小陀螺转速补偿
    float yawSpdOut       = _module_deps.pid_deps.yaw_spd->calculate(tgtYawSpd, _ctx.data.imu.gyro[0]);
    _ctx.data.output.yawCurrent = yawSpdOut;
}



void wl_gimbal_t::_fsm_execute()
{

    // if (_ctx.data.mode == cmd_base_t::mode_t::ACTIVE)
    // {
    //     _main_fsm.change_state(&_state_active);
    // }
    // else
    // {
    //     _main_fsm.change_state(&_state_passive);
    // }
    _main_fsm.execute(this);
    wl_gimbal_t::_send_motor_command();
}

void wl_gimbal_t::set_pitchstate(bool enable)
{
    if (enable)
    {
        instance()->_module_deps.motor_deps.pitch->enable();
    }
    else
    {
        instance()->_module_deps.motor_deps.pitch->disable();
    }
}

void wl_gimbal_t::set_yawstate(bool enable)
{
    if (enable)
    {
        instance()->_module_deps.motor_deps.yaw->enable();
    }
    else
    {
        instance()->_module_deps.motor_deps.yaw->disable();
    }
}

void wl_gimbal_t::_send_motor_command()
{
    _module_deps.motor_deps.pitch->send_mit_ctrl(_ctx.data.output.targetPitchPos, 
                                                 _ctx.data.output.targetPitchSpeed,
                                                 _ctx.data.output.pitchFeedforwardTorque);
    _module_deps.motor_deps.yaw->send_torque(_ctx.data.output.yawCurrent);
}

float wl_gimbal_t::wrapAngle(float angle) {
    while(angle<-PI)
    {
        angle+=2*PI;
    }
    while(angle>PI)
    {
        angle-=2*PI;
    }
    return angle;
}
}
