#include "pyro_dm_motor_drv.h"
#include "pyro_wl_chassis.h"

namespace pyro
{

void wl_chassis_t::fsm_active_t::state_normal_t::enter(wl_chassis_t *owner)
{
    owner->_ctx.data.target_state = {};
    owner->_ctx.data.current_state[0].x = 0;
    owner->_ctx.data.current_state[1].x = 0;
}

void wl_chassis_t::fsm_active_t::state_normal_t::execute(wl_chassis_t *ctx)
{
    (void)ctx;
}

void wl_chassis_t::fsm_active_t::state_normal_t::exit(wl_chassis_t *ctx)
{
    (void)ctx;
}

} // namespace pyro
