#include "pyro_wl_booster.h"
#include "gimbal_config.h"
#include "pyro_dwt_drv.h"
#include "pyro_ins.h"
#include "dsp/fast_math_functions.h"


namespace pyro
{

wl_booster_t::wl_booster_t() : module_base_t("wl_booster")
{
}

status_t wl_booster_t::_init()
{
    _ctx                                      = {};
    _ctx.motor                                = _module_deps.motor_deps;
    _ctx.pid                                  = _module_deps.pid_deps;

    return PYRO_OK;
}

void wl_booster_t::_update_feedback()
{
    
    //更新电机反馈
    _module_deps.motor_deps.fric1->update_feedback();
    _module_deps.motor_deps.fric2->update_feedback();
    _module_deps.motor_deps.trigger->update_feedback();




    _ctx.data.motor_state.fric[0].online = _module_deps.motor_deps.fric1->is_online();
    _ctx.data.motor_state.fric[0].pos    = _module_deps.motor_deps.fric1->get_current_position();
    _ctx.data.motor_state.fric[0].vel    = _module_deps.motor_deps.fric1->get_current_rotate();
    _ctx.data.motor_state.fric[0].temp   = _module_deps.motor_deps.fric1->get_temperature();
    _ctx.data.motor_state.fric[0].torque = _module_deps.motor_deps.fric1->get_current_torque();
    

    _ctx.data.motor_state.fric[1].online   = _module_deps.motor_deps.fric2->is_online();
    _ctx.data.motor_state.fric[1].pos      = _module_deps.motor_deps.fric2->get_current_position();
    _ctx.data.motor_state.fric[1].vel      = _module_deps.motor_deps.fric2->get_current_rotate();
    _ctx.data.motor_state.fric[1].temp     = _module_deps.motor_deps.fric2->get_temperature();
    _ctx.data.motor_state.fric[1].torque   = _module_deps.motor_deps.fric2->get_current_torque();

    _ctx.data.motor_state.trigger.online   = _module_deps.motor_deps.trigger->is_online();
    _ctx.data.motor_state.trigger.pos      = _module_deps.motor_deps.trigger->get_current_position();
    _ctx.data.motor_state.trigger.vel      = _module_deps.motor_deps.trigger->get_current_rotate();
    _ctx.data.motor_state.trigger.temp     = _module_deps.motor_deps.trigger->get_temperature();
    _ctx.data.motor_state.trigger.torque   = _module_deps.motor_deps.trigger->get_current_torque();


    _ctx.data.mode                         = _current_cmd.mode;
    _ctx.data.cmd_event                    = _current_cmd.event;
    _ctx.data.target_state.burst_state                  = _current_cmd.burst_state.burstShot;

    static uint32_t dwtCnt;
    _ctx.data.dt = pyro::dwt_drv_t::get_delta_t(&dwtCnt);
    
}




void wl_booster_t::_fsm_execute()
{

    if (_ctx.data.mode == cmd_base_t::mode_t::ACTIVE)
    {
        _main_fsm.change_state(&_state_active);
    }
    else
    {
        _main_fsm.change_state(&_state_passive);
    }
    _main_fsm.execute(this);
    sendCurrents();
}


void wl_booster_t::calculateFricCurrents()
{

}

void wl_booster_t::calculateTriggerCurrents(bool useTriggerSpeedLoopOnly)
{

}

void wl_booster_t::sendCurrents()
{
    _module_deps.motor_deps.fric1->send_torque(_ctx.data.output.fric1Current);
    _module_deps.motor_deps.fric2->send_torque(_ctx.data.output.fric2Current);
    _module_deps.motor_deps.trigger->send_torque(_ctx.data.output.triggerCurrent);
}


}
