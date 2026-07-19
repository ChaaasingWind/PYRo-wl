#include "pyro_algo_common.h"
#include "pyro_wl_chassis.h"

namespace pyro
{

void wl_chassis_t::fsm_active_t::state_manual_t::enter(wl_chassis_t *owner)
{
    (void)owner;
}

void wl_chassis_t::fsm_active_t::state_manual_t::execute(wl_chassis_t *owner)
{

    owner->_ctx.data.leg[leg_def::LEFT].target_leg_length +=
        owner->_current_cmd.delta_leg_length[leg_def::LEFT];
    if (owner->_ctx.data.leg[leg_def::LEFT].target_leg_length < MIN_LEG_LENGTH)
    {
        owner->_ctx.data.leg[leg_def::LEFT].target_leg_length = MIN_LEG_LENGTH;
    }
    else if (owner->_ctx.data.leg[leg_def::LEFT].target_leg_length >
             MAX_LEG_LENGTH)
    {
        owner->_ctx.data.leg[leg_def::LEFT].target_leg_length = MAX_LEG_LENGTH;
    }
    owner->_ctx.data.leg[leg_def::LEFT].target_leg_rad =
        fp32_constrain(owner->_ctx.data.leg[leg_def::LEFT].target_leg_rad +
                           owner->_current_cmd.delta_leg_rad[leg_def::LEFT],
                       -PI, PI);
    owner->_ctx.data.leg[leg_def::LEFT].error_leg_rad = loop_fp32_constrain(
        owner->_ctx.data.leg[leg_def::LEFT].target_leg_rad -
            owner->_ctx.data.leg[leg_def::LEFT].current_leg_rad,
        -PI, PI);

    owner->_ctx.data.leg[leg_def::RIGHT].target_leg_length +=
        owner->_current_cmd.delta_leg_length[leg_def::RIGHT];
    if (owner->_ctx.data.leg[leg_def::RIGHT].target_leg_length < MIN_LEG_LENGTH)
    {
        owner->_ctx.data.leg[leg_def::RIGHT].target_leg_length = MIN_LEG_LENGTH;
    }
    else if (owner->_ctx.data.leg[leg_def::RIGHT].target_leg_length >
             MAX_LEG_LENGTH)
    {
        owner->_ctx.data.leg[leg_def::RIGHT].target_leg_length = MAX_LEG_LENGTH;
    }
    owner->_ctx.data.leg[leg_def::RIGHT].target_leg_rad =
        fp32_constrain(owner->_ctx.data.leg[leg_def::RIGHT].target_leg_rad +
                           owner->_current_cmd.delta_leg_rad[leg_def::RIGHT],
                       -PI, PI);
    owner->_ctx.data.leg[leg_def::RIGHT].error_leg_rad = loop_fp32_constrain(
        owner->_ctx.data.leg[leg_def::RIGHT].target_leg_rad -
            owner->_ctx.data.leg[leg_def::RIGHT].current_leg_rad,
        -PI, PI);


    owner->_calculate();
    owner->_vmc_control();
    owner->_send_torque();
}

void wl_chassis_t::fsm_active_t::state_manual_t::exit(wl_chassis_t *owner)
{
    (void)owner;
}

} // namespace pyro
