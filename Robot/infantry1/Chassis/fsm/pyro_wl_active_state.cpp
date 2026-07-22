#include "pyro_dm_motor_drv.h"
#include "pyro_wl_chassis.h"

namespace pyro
{

void wl_chassis_t::fsm_active_t::on_enter(wl_chassis_t *owner)
{
    static_cast<dm_motor_drv_t*>(owner->_ctx.motor.joint[leg_def::L][motor_def::HIP])->clear_error();
    static_cast<dm_motor_drv_t*>(owner->_ctx.motor.joint[leg_def::L][motor_def::KNEE])->clear_error();
    owner->_ctx.motor.joint[leg_def::L][motor_def::HIP]->enable();
    owner->_ctx.motor.joint[leg_def::L][motor_def::KNEE]->enable();
    owner->_ctx.motor.joint[leg_def::R][motor_def::HIP]->enable();
    owner->_ctx.motor.joint[leg_def::R][motor_def::KNEE]->enable();

    change_state(&_state_manual);
}

void wl_chassis_t::fsm_active_t::on_execute(wl_chassis_t *ctx)
{
    (void)ctx;
}

void wl_chassis_t::fsm_active_t::on_exit(wl_chassis_t *ctx)
{
    (void)ctx;
}

} // namespace pyro
