#include "pyro_algo_common.h"
#include "pyro_wl_chassis.h"

namespace pyro
{

void wl_chassis_t::fsm_active_t::state_manual_t::enter(wl_chassis_t *owner)
{
    for (auto &leg : owner->_ctx.data.leg)
    {
        leg.target_leg_length = leg.current_leg_length;
        leg.target_leg_rad = leg.current_leg_rad;
        leg.target_leg_speed = leg.current_leg_speed;
        leg.target_leg_radps = leg.current_leg_radps;
        leg.out_F_L = 0;
        leg.out_T_p = 0;
        leg.out_joint_torque[motor_def::HIP]  = 0;
        leg.out_joint_torque[motor_def::KNEE] = 0;
    }
}

void wl_chassis_t::fsm_active_t::state_manual_t::execute(wl_chassis_t *owner)
{

    owner->_ctx.data.leg[leg_def::L].target_leg_length +=
        owner->_current_cmd.delta_leg_length[leg_def::L];
    if (owner->_ctx.data.leg[leg_def::L].target_leg_length < MIN_LEG_LENGTH)
    {
        owner->_ctx.data.leg[leg_def::L].target_leg_length = MIN_LEG_LENGTH;
    }
    else if (owner->_ctx.data.leg[leg_def::L].target_leg_length >
             MAX_LEG_LENGTH)
    {
        owner->_ctx.data.leg[leg_def::L].target_leg_length = MAX_LEG_LENGTH;
    }
    owner->_ctx.data.leg[leg_def::L].target_leg_rad =
        fp32_constrain(owner->_ctx.data.leg[leg_def::L].target_leg_rad +
                           owner->_current_cmd.delta_leg_rad[leg_def::L],
                       -PI, PI);
    owner->_ctx.data.leg[leg_def::L].error_leg_rad = loop_fp32_constrain(
        owner->_ctx.data.leg[leg_def::L].current_leg_rad -
            owner->_ctx.data.leg[leg_def::L].target_leg_rad,
        -PI, PI);

    owner->_ctx.data.leg[leg_def::R].target_leg_length +=
        owner->_current_cmd.delta_leg_length[leg_def::R];
    if (owner->_ctx.data.leg[leg_def::R].target_leg_length < MIN_LEG_LENGTH)
    {
        owner->_ctx.data.leg[leg_def::R].target_leg_length = MIN_LEG_LENGTH;
    }
    else if (owner->_ctx.data.leg[leg_def::R].target_leg_length >
             MAX_LEG_LENGTH)
    {
        owner->_ctx.data.leg[leg_def::R].target_leg_length = MAX_LEG_LENGTH;
    }
    owner->_ctx.data.leg[leg_def::R].target_leg_rad =
        fp32_constrain(owner->_ctx.data.leg[leg_def::R].target_leg_rad +
                           owner->_current_cmd.delta_leg_rad[leg_def::R],
                       -PI, PI);
    owner->_ctx.data.leg[leg_def::R].error_leg_rad = loop_fp32_constrain(
        owner->_ctx.data.leg[leg_def::R].current_leg_rad -
            owner->_ctx.data.leg[leg_def::R].target_leg_rad,
        -PI, PI);


    owner->_calculate();
    owner->_vmc_trans_v2j();
    owner->_send_joint_torque();
}

void wl_chassis_t::fsm_active_t::state_manual_t::exit(wl_chassis_t *owner)
{
    (void)owner;
}

} // namespace pyro
