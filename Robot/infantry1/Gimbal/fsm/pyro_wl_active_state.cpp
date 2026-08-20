#include "pyro_wl_gimbal.h"

void pyro::wl_gimbal_t::fsm_active_t::on_enter(pyro::wl_gimbal_t* ctx) {
    ctx->_ctx.data.output.pitchEn                = true;
    ctx->_ctx.data.output.yawEn                  = true;

    instance()->set_pitchstate(ctx->_ctx.data.output.pitchEn);
    instance()->set_yawstate(ctx->_ctx.data.output.yawEn);

    change_state(&ctx->_state_active._state_align);
}

void pyro::wl_gimbal_t::fsm_active_t::on_execute(pyro::wl_gimbal_t* ctx)
{
    if(ctx->_ctx.data.mode == cmd_base_t::mode_t::PASSIVE)
    {
        request_switch(&instance()->_state_passive);
    }
}

void pyro::wl_gimbal_t::fsm_active_t::on_exit(pyro::wl_gimbal_t* ctx)
{

}