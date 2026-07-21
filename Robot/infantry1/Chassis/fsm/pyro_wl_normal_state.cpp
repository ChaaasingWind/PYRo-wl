#include "pyro_dm_motor_drv.h"
#include "pyro_wl_chassis.h"

namespace pyro
{

void wl_chassis_t::fsm_active_t::state_normal_t::enter(wl_chassis_t *owner)
{

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
