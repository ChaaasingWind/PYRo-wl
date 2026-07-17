#include "pyro_wl_chassis.h"

namespace pyro
{

void wl_chassis_t::fsm_active_t::on_enter(wl_chassis_t *owner)
{
    owner->_ctx.motor.joint[leg_def::LEFT][motor_def::HIP]->enable();
    owner->_ctx.motor.joint[leg_def::LEFT][motor_def::KNEE]->enable();
    owner->_ctx.motor.joint[leg_def::RIGHT][motor_def::HIP]->disable();
    owner->_ctx.motor.joint[leg_def::RIGHT][motor_def::KNEE]->disable();

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
