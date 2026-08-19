#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_rc_core.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_wl_gimbal.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_bsp_can.h"
#include "pyro_dji_motor_drv.h"
#include "gimbal_config.h"

using namespace pyro;




extern "C"
{
    void infantry1_gimbal_init(void *argument)
    {

    }
}