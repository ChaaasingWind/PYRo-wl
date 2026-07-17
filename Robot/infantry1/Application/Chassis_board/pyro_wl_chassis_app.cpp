#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_wl_chassis.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_bsp_can.h"
#include "wl_config.h"

using namespace pyro;


static TaskHandle_t chassis_task_handle                = nullptr;
static pyro::wl_chassis_t *wl_chassis_ptr             = nullptr;
static pyro::wl_chassis_cmd_t *wl_chassis_cmd_ptr     = nullptr;
static pyro::wl_chassis_deps_t *wl_chassis_deps       = nullptr;

static void chassis_dr162cmd();
static void deps_init();

extern "C"
{
    void infantry1_chassis_thread(void *argument)
    {
        static bool leg_toggle_state = false;

        while (true)
        {
            uint32_t notify_val = 0;
            // 接收任务通知事件（不阻塞等待，0 tick延时）
            xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);

            // 当前没有板间通信，直接检测并使用遥控器控制
            if (dr16_drv_t::instance().check_online())
            {
                chassis_dr162cmd();
            }
            else
            {
                wl_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
            }

            wl_chassis_ptr->set_command(*wl_chassis_cmd_ptr);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    void infantry1_chassis_init(void *argument)
    {
        wl_chassis_cmd_ptr = new pyro::wl_chassis_cmd_t();
        wl_chassis_ptr     = pyro::wl_chassis_t::instance();

        deps_init();
        wl_chassis_ptr->configure(*wl_chassis_deps);
        wl_chassis_ptr->start();

        xTaskCreate(infantry1_chassis_thread, "chassis_app_thread", 256, nullptr,
                    configMAX_PRIORITIES - 1, &chassis_task_handle);

        auto &vrc = pyro::rc_drv_t::read();


        vTaskDelete(nullptr);
    }
}

void chassis_dr162cmd()
{
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    // 右开关控制底盘使能模式：不处于MID或DOWN时，失能
    if (pyro::sw_pos_t::MID != vrc.switches.right.current_pos)
    {
        wl_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
        wl_chassis_cmd_ptr->delta_leg_length[leg_def::LEFT]  = 0.0f;
        wl_chassis_cmd_ptr->delta_leg_length[leg_def::RIGHT] = 0.0f;
        wl_chassis_cmd_ptr->delta_leg_rad[leg_def::LEFT]     = 0.0f;
        wl_chassis_cmd_ptr->delta_leg_rad[leg_def::RIGHT]    = 0.0f;
        return;
    }

    wl_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;

    // 手动通道输入控制腿长和腿度（角度）的偏置量

    wl_chassis_cmd_ptr->delta_leg_length[leg_def::LEFT]  = vrc.axes.ly * 0.0001f;
    wl_chassis_cmd_ptr->delta_leg_rad[leg_def::LEFT] = vrc.axes.lx * 0.00001f;

    wl_chassis_cmd_ptr->delta_leg_length[leg_def::RIGHT]  = vrc.axes.ry * 0.0001f;
    wl_chassis_cmd_ptr->delta_leg_rad[leg_def::RIGHT] = vrc.axes.rx * 0.00001f;
}

void deps_init()
{
    wl_chassis_deps = new pyro::wl_chassis_deps_t();

    // 1. 初始化二维数组形式的 4 个关节达妙电机 (使用 CAN1)
    wl_chassis_deps->motor.joint[leg_def::LEFT][motor_def::HIP] = new pyro::dm_motor_drv_t(0x04, 0x14, pyro::bsp_can::can2);
    wl_chassis_deps->motor.joint[leg_def::LEFT][motor_def::KNEE] = new pyro::dm_motor_drv_t(0x03, 0x13, pyro::bsp_can::can2);
    wl_chassis_deps->motor.joint[leg_def::RIGHT][motor_def::HIP] = new pyro::dm_motor_drv_t(0x02, 0x12, pyro::bsp_can::can1);
    wl_chassis_deps->motor.joint[leg_def::RIGHT][motor_def::KNEE] = new pyro::dm_motor_drv_t(0x01, 0x11, pyro::bsp_can::can1);

    static_cast<dm_motor_drv_t* >(wl_chassis_deps->motor.joint[leg_def::LEFT][motor_def::HIP])->set_position_range(-PI,PI);
    static_cast<dm_motor_drv_t* >(wl_chassis_deps->motor.joint[leg_def::LEFT][motor_def::HIP])->set_rotate_range(-45.0f, 45.0f);
    static_cast<dm_motor_drv_t* >(wl_chassis_deps->motor.joint[leg_def::LEFT][motor_def::HIP])->set_torque_range(-54.0f, 54.0f);

    static_cast<dm_motor_drv_t* >(wl_chassis_deps->motor.joint[leg_def::LEFT][motor_def::KNEE])->set_position_range(-PI,PI);
    static_cast<dm_motor_drv_t* >(wl_chassis_deps->motor.joint[leg_def::LEFT][motor_def::KNEE])->set_rotate_range(-45.0f, 45.0f);
    static_cast<dm_motor_drv_t* >(wl_chassis_deps->motor.joint[leg_def::LEFT][motor_def::KNEE])->set_torque_range(-54.0f, 54.0f);

    static_cast<dm_motor_drv_t* >(wl_chassis_deps->motor.joint[leg_def::RIGHT][motor_def::HIP])->set_position_range(-PI,PI);
    static_cast<dm_motor_drv_t* >(wl_chassis_deps->motor.joint[leg_def::RIGHT][motor_def::HIP])->set_rotate_range(-45.0f, 45.0f);
    static_cast<dm_motor_drv_t* >(wl_chassis_deps->motor.joint[leg_def::RIGHT][motor_def::HIP])->set_torque_range(-54.0f, 54.0f);

    static_cast<dm_motor_drv_t* >(wl_chassis_deps->motor.joint[leg_def::RIGHT][motor_def::KNEE])->set_position_range(-PI,PI);
    static_cast<dm_motor_drv_t* >(wl_chassis_deps->motor.joint[leg_def::RIGHT][motor_def::KNEE])->set_rotate_range(-45.0f, 45.0f);
    static_cast<dm_motor_drv_t* >(wl_chassis_deps->motor.joint[leg_def::RIGHT][motor_def::KNEE])->set_torque_range(-54.0f, 54.0f);


    // 2. 初始化 PIDs
    // 腿长 PID
    wl_chassis_deps->pid.leg_length[leg_def::LEFT] = new pyro::pid_t(1.0f, 0.0f, 0.1f, 1.0f, 50.0f);
    wl_chassis_deps->pid.leg_length[leg_def::RIGHT] = new pyro::pid_t(1.0f, 0.0f, 0.1f, 1.0f, 50.0f);

    // 腿角度 PID
    wl_chassis_deps->pid.leg_rad[leg_def::LEFT] = new pyro::pid_t(0.5f, 0.0f, 0.0f, 2.0f, 100.0f);
    wl_chassis_deps->pid.leg_rad[leg_def::RIGHT] = new pyro::pid_t(0.5f, 0.0f, 0.0f, 2.0f, 100.0f);
}
