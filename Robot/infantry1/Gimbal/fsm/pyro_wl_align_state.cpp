#include "pyro_wl_gimbal.h"
#include "gimbal_config.h"




void pyro::wl_gimbal_t::fsm_active_t::state_align_t::enter(owner *owner)
{
    owner->_module_deps.pid_deps.yaw_pos->clear();
    owner->_module_deps.pid_deps.yaw_spd->clear();
    owner->_module_deps.pid_deps.pitch_pos->clear();
    owner->_module_deps.pid_deps.pitch_spd->clear();
    owner->_ctx.data.output.pitchEn    = true;
    owner->_ctx.data.output.yawEn      = true;
    owner->_ctx.data.output.yawCurrent = 0.0f;
    instance()->set_pitchstate(owner->_ctx.data.output.pitchEn);
    instance()->set_yawstate(owner->_ctx.data.output.yawEn);

}

void pyro::wl_gimbal_t::fsm_active_t::state_align_t::execute(owner *owner) 
{
    owner->_ctx.data.telem.targetPitchRad = PITCH_ALIGN_TARGET_RAD;
    owner->updatePitch();

    if(owner->_ctx.data.state.pitch.pos<PITCH_LIMIT_MIN-0.3f)
    {
        float error_rad = YAW_ALIGN_TARGET_RAD - owner->_ctx.motor.yaw->get_current_position();
        float yawSpdCmd     = owner->_ctx.pid.yaw_pos->calculate(0.0f,  -error_rad);
        owner->_ctx.data.output.yawCurrent = owner->_ctx.pid.yaw_spd->calculate(yawSpdCmd, owner->_ctx.data.imu.gyro[2]);
        owner->updateYaw();

        static int count = 0;
        if(error_rad < 0.3f)
        {
            if(count >= 50)
            {
                request_switch(&owner->_state_active._state_manual);
            }
            count++;
        }
        else
        {
            count = 0;
        }
    }
 
}

void pyro::wl_gimbal_t::fsm_active_t::state_align_t::exit(owner *owner)
{
    (void)owner;
}