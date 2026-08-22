#include "pyro_wl_gimbal.h"

void pyro::wl_gimbal_t::state_passive_t::enter(pyro::wl_gimbal_t* ctx) {
    ctx->_ctx.data.output.targetPitchPos         = ctx->_ctx.data.state.pitch.pos;
    ctx->_ctx.data.output.targetPitchSpeed       = 0.0f;
    ctx->_ctx.data.output.pitchFeedforwardTorque = 0.0f;
    ctx->_ctx.data.output.pitchEn                = false;
    ctx->_ctx.data.output.yawCurrent             = 0.0f;
    ctx->_ctx.data.output.yawEn                  = false;
    ctx->_ctx.data.telem.targetYawRad            = ctx->_ctx.data.imu.yaw;
    ctx->_ctx.data.telem.targetPitchRad          = ctx->_ctx.data.imu.pitch;

    
    ctx->_module_deps.pid_deps.yaw_pos->clear();
    ctx->_module_deps.pid_deps.yaw_spd->clear();
    ctx->_module_deps.pid_deps.pitch_pos->clear();
    ctx->_module_deps.pid_deps.pitch_spd->clear();
    ctx->set_pitchstate(ctx->_ctx.data.output.pitchEn);
    ctx->set_yawstate(ctx->_ctx.data.output.yawEn);
}

void pyro::wl_gimbal_t::state_passive_t::execute(pyro::wl_gimbal_t* ctx) 
{
    ctx->_ctx.data.telem.targetYawRad            = ctx->_ctx.data.imu.yaw;
    ctx->_ctx.data.telem.targetPitchRad          = ctx->_ctx.data.imu.pitch;
    if(ctx->_ctx.data.mode == cmd_base_t::mode_t::ACTIVE)
    {
        request_switch(&instance()->_state_active);
    }
}

void pyro::wl_gimbal_t::state_passive_t::exit(pyro::wl_gimbal_t* ctx)
{

}