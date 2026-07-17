#ifndef __PYRO_WL_CHASSIS_H__
#define __PYRO_WL_CHASSIS_H__

#include "pyro_algo_pid.h"
#include "pyro_module_base.h"
#include "pyro_motor_base.h"
#include "wl_config.h"

namespace pyro
{
struct wl_chassis_deps_t
{
    struct motor_deps_t
    {
        motor_base_t *joint[2][2];
    };

    struct pid_deps_t
    {
        pid_t *leg_length[2];
        pid_t *leg_rad[2];
    };
    motor_deps_t motor;
    pid_deps_t pid;
};

struct wl_chassis_cmd_t final : public cmd_base_t
{
    float delta_leg_length[2];
    float delta_leg_rad[2];


    enum class wl_chassis_mode_t : uint8_t
    {
        MANUAL,
    };
};

struct leg_ctx
{
    float target_leg_length;
    float target_leg_speed;
    float target_leg_rad;
    float target_leg_radps;
    float current_leg_length; // current_leg_length = f((θ1 - θ2) / 2)
    float current_leg_speed;
    float current_leg_rad; // current_leg_rad = (θ1 + θ2) / 2
    float current_leg_radps;

    float J_L;
    float out_F_L; // F_L * J_L = tau_2 - tau_1
    float out_T_p; // T_p = tau_2 + tau_1
    float current_F_L;
    float current_T_p;

    float current_motor_rad[2];// θ1，θ2
    float current_motor_radps[2];

    float out_motor_torque[2];
};

struct wl_chassis_data_ctx_t
{
    leg_ctx leg[2];
};

struct wl_chassis_ctx_t
{
    wl_chassis_deps_t::motor_deps_t motor;
    wl_chassis_deps_t::pid_deps_t pid;
    wl_chassis_data_ctx_t data;
};

struct wl_chassis_param_t
{

    using CmdType    = wl_chassis_cmd_t;
    using ModuleDeps = wl_chassis_deps_t;
    using ModuleCtx  = wl_chassis_ctx_t;
};

class wl_chassis_t final
    : public module_base_t<wl_chassis_t, wl_chassis_param_t>
{
    friend class module_base_t<wl_chassis_t, wl_chassis_param_t>;

  public:
    wl_chassis_t(const wl_chassis_t &)            = delete;
    wl_chassis_t &operator=(const wl_chassis_t &) = delete;
    using data_ctx_t                              = wl_chassis_data_ctx_t;
    using ctx_t                                   = wl_chassis_ctx_t;

  private:
    wl_chassis_t();
    ~wl_chassis_t() override = default;

    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // 私有成员变量

    // 辅助函数
    void _vmc_trans();
    void _calculate();
    void _vmc_control();
    void _send_torque();

    using owner = wl_chassis_t;


    struct state_passive_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };
    struct fsm_active_t final : public fsm_t<owner>
    {
        struct state_manual_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        void on_enter(wl_chassis_t *ctx) override;
        void on_execute(wl_chassis_t *ctx) override;
        void on_exit(wl_chassis_t *ctx) override;

      private:
        state_manual_t _state_manual;
    };

    fsm_t<owner> _main_fsm;
    state_passive_t _state_passive;
    fsm_active_t _state_active;
};


} // namespace pyro
#endif // __PYRO_WL_CHASSIS_H__
