#include "pyro_wl_gimbal.h"

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
    

    _ctx.data.state.yaw.online = _module_deps.motor_deps.yaw->is_online();
    _ctx.data.state.yaw.pos    = _module_deps.motor_deps.yaw->get_current_position();
    _ctx.data.state.yaw.vel    = _module_deps.motor_deps.yaw->get_current_rotate();
    _ctx.data.state.yaw.temp   = _module_deps.motor_deps.yaw->get_temperature();
    _ctx.data.state.yaw.torque = _module_deps.motor_deps.yaw->get_current_torque();

    static uint32_t dwtCnt;
    _ctx.data.dt = pyro::dwt_drv_t::get_delta_t(&dwtCnt);
    
}


void wl_gimbal_t::_fsm_execute()
{

    _main_fsm.execute(this);
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
}