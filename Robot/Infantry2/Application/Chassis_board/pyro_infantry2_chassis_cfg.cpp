/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-27 20:38:05
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-05-30 00:44:42
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
//the cofficients for phi calculation
#define PHI_K0 22506.0f
#define PHI_K1 475976946.0f
#define PHI_K2 296955904.0f
#define PHI_K3 179021042.0f
#define PHI_K4 18922.0f
//the cofficients for position in polar coordinates
#define POLAR_K0 236976927.0f
#define POLAR_K1 21059.0f
#define POLAR_K2 946100000.0f
#define POLAR_K3 100000.0f
//the cofficients for VMC transformation matrix
#define TRANS_K0 21059.0f
#define TRANS_K1 100000.0f
//the offsets of motors, which is the angle between the zero point of motor and
// the forward direction of robot, counter clockwise is positive(rad)
// -0.5536 0.770
#if ROBOT_ID == INFANTRY1_ID
#define R_MOTOR1_OFFSET 0.4424f 
#define R_MOTOR2_OFFSET  0.2270f 
#define L_MOTOR1_OFFSET  -0.0236f 
#define L_MOTOR2_OFFSET  3.8210f 
#elif ROBOT_ID == INFANTRY2_ID
#define R_MOTOR1_OFFSET  1.2664f
#define R_MOTOR2_OFFSET 0.2100f
#define L_MOTOR1_OFFSET 0.5134f
#define L_MOTOR2_OFFSET 2.2900f
#endif


#define WHEEL_DISTANCE 0.424f
#define CONTROL_PERIOD 0.001f
// the cofficients of lqr gain, 48 values in total, every value has 3 cofficients,
#define LQR_GAIN \
-0.9822, 14.2356, -25.1651, 15.4754,-4.0889, 61.2729, -106.1286, 64.5492,-18.8575, -18.5331, 192.4497, -254.9521,-3.4886, -5.0196, 37.7243, -47.2479,6.1991, -140.3664, 90.9413, 26.5639,0.8399, -11.9744, -29.4522, 36.8439,2.1984, -133.0088, 506.8353, -585.9221,9.1394, -577.8490, 2198.9349, -2542.0398,-1.6755, -317.2803, -101.5236, 787.0448,0.4825, -20.7444, -167.8482, 324.9636,-55.8928, 1690.0016, -5614.6267, 6071.1369,-9.3538, 239.3199, -674.7390, 677.7541
#define LQR_COEF_OVER_STEP \
-1.0981, 19.8929, -49.9394, 46.5243,-4.5492, 84.2808, -207.8295, 192.5193,-21.8057, 21.1337, 84.8880, -164.0295,-3.9946, 0.0755, 27.1950, -41.8240,5.3853, -148.0953, 163.4375, -81.1506,0.7690, -12.9720, -22.7003, 27.9048,-6.2843, -22.0035, 143.7480, -194.7848,-26.6169, -97.0868, 616.6843, -830.3942,17.2288, -565.2392, 1236.8181, -998.1581,6.4658, -104.0931, 205.8637, -147.1518,7.8346, 561.3459, -1928.7994, 2121.5519,-0.8096, 90.8998, -240.2076, 235.7069
#include "pyro_wl_chassis.h"



using namespace pyro;

float infantry2_lqr_coef[48] = {LQR_GAIN};
float infantry2_lqr_coef_over_step[48] = {LQR_COEF_OVER_STEP};
wl_chassis_cfg_t infantry2_chassis_cfg = {
    .phi_k = {
        .k0 = PHI_K0,
        .k1 = PHI_K1,
        .k2 = PHI_K2,
        .k3 = PHI_K3,
        .k4 = PHI_K4,
    },
    .polar_k = {
        .k0 = POLAR_K0,
        .k1 = POLAR_K1,
        .k2 = POLAR_K2,
        .k3 = POLAR_K3,
    },
    .vmc_k = {
        .k0 = TRANS_K0,
        .k1 = TRANS_K1,
    },
    .joint_motor_cfg = {
        {
            .tx_id = 0x01,
            .rx_id = 0x11,
            .can = bsp_can::can1,
            .offset_angle = R_MOTOR1_OFFSET,
        },
        {
            .tx_id = 0x02,
            .rx_id = 0x12,
            .can = bsp_can::can1,
            .offset_angle = R_MOTOR2_OFFSET,
        },
        {
            .tx_id = 0x03,
            .rx_id = 0x13,
            .can = bsp_can::can2,
            .offset_angle = L_MOTOR1_OFFSET,
        },
        {
            .tx_id = 0x04,
            .rx_id = 0x14,
            .can = bsp_can::can2,
            .offset_angle = L_MOTOR2_OFFSET,
        }
    },
    .wheel_motor_cfg = {
        {
            .tx_id = dji_motor_tx_frame_t::id_3,
            .can = bsp_can::can1,
        },
        {
            .tx_id = dji_motor_tx_frame_t::id_2,
            .can = bsp_can::can2,
        }
    },
    .yaw_motor_cfg = {
        .tx_id = dji_motor_tx_frame_t::id_5,
        .can = bsp_can::can3,
    },
#if ROBOT_ID == INFANTRY1_ID
    .yaw_offset = -1.9f,
#elif ROBOT_ID == INFANTRY2_ID 
    .yaw_offset = -1.765f,
#endif
    .T_pid_cfg = {
        {
            .kp = 20.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 40.0f,
        },
        {
            .kp = 20.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 40.0f,
        }
    },
    .d_T_pid_cfg = {
        {
            .kp = 30.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 20.0f,
        },
        {
            .kp = 30.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 20.0f,
        }
    },
#if ROBOT_ID == INFANTRY1_ID
    .F_pid_cfg = {
        {
            .kp = 70.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 50.0f,
            .max_out = 200.0f,
        },
        {
            .kp = 70.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 50.0f,
            .max_out = 200.0f,
        }
    },

    .d_F_pid_cfg = {
        {
            .kp = 200.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 200.0f,
        },
        {
            .kp = 200.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 200.0f,
        }
    },
#elif ROBOT_ID == INFANTRY2_ID
    .F_pid_cfg = {
        {
            .kp = 80.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 50.0f,
            .max_out = 200.0f,
        },
        {
            .kp = 80.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 50.0f,
            .max_out = 200.0f,
        }
    },
    .d_F_pid_cfg = {
        {
            .kp = 240.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 200.0f,
        },
        {
            .kp = 240.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 200.0f,
        }
    },
#endif
    .yaw_pid_cfg = {
        .kp = 4.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integral_limit = 0.0f,
        .max_out = 30.0f,
    },
    .g_yaw_pid_cfg = {
        .kp = 2.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integral_limit = 0.0f,
        .max_out = 40.0f
    },
    .delta_pid_cfg = {
          .kp = 4.0f,
          .ki = 0.0f,
          .kd = 0.0f,
          .integral_limit = 0.0f,
          .max_out = 20.0f,
     },
    .d_delta_pid_cfg = {
          .kp = 3.0f,
          .ki = 0.0f,
          .kd = 0.0f,
          .integral_limit = 0.0f,
          .max_out = 20.0f,
     },
    .roll_pid_cfg = {
          .kp = 0.0f,
          .ki = 0.0f,
          .kd = 0.0f,
          .integral_limit = 0.0f,
          .max_out = 30.0f,
     },
    .wheel_kf_cfg = {
        {
            .x_init = (float[3]){0.0f, 0.0f, 0.0f},
            .P_init = (float[9]){
                1.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 1.0f
            },
            .A = (float[9]){
                1.0f,  CONTROL_PERIOD, 0.0f,
                0.0f,  1.0f,   0.0f,
                0.0f,  0.0f,   1.0f
            },
            .B = (float[3]){
                0.0f, 0.0f, 0.0f
            },
            .H = (float[9]){
                1.0f,  0.0f, 0.0f,
                0.0f,  1.0f, 0.0f,
                0.0f,  0.0f, 1.0f
            },
            .G = (float[6]){
                CONTROL_PERIOD * CONTROL_PERIOD / 2.0f, 0.0f,
                CONTROL_PERIOD,                         0.0f,
                0.0f,                                   CONTROL_PERIOD
            },
            .Q = (float[4]){
                10000.0f, 0.0f,
                0.0f,     10000.0f
            },
            .R = (float[9]){
                0.05f, 0.0f,  0.0f,
                0.0f,  0.5f,  0.0f,
                0.0f,  0.0f,  0.005f
            }
        },
        {
            .x_init = (float[3]){0.0f, 0.0f, 0.0f},
            .P_init = (float[9]){
                1.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 1.0f
            },
            .A = (float[9]){
                1.0f,  CONTROL_PERIOD,  0.0f,
                0.0f,  1.0f,   0.0f,
                0.0f,  0.0f,   1.0f
            },
            .B = (float[3]){
                0.0f, 0.0f, 0.0f
            },
            .H = (float[9]){
                1.0f,  0.0f, 0.0f,
                0.0f,  1.0f, 0.0f,
                0.0f,  0.0f, 1.0f
            },
            .G = (float[6]){
                CONTROL_PERIOD * CONTROL_PERIOD / 2.0f, 0.0f,
                CONTROL_PERIOD,                         0.0f,
                0.0f,                                   CONTROL_PERIOD
            },
            .Q = (float[4]){
                10000.0f, 0.0f,
                0.0f,     10000.0f
            },
            .R = (float[9]){
                0.05f, 0.0f,  0.0f,
                0.0f,  0.5f,  0.0f,
                0.0f,  0.0f,  0.005f
            },
        }
    },


    .lqr_coef = infantry2_lqr_coef,
    .lqr_coef_over_step = infantry2_lqr_coef_over_step,
    .wheel_radius = 0.06f,
    .reduction_ratio = 13.94f,
    .rotate_min = -45.0f,
    .rotate_max = 45.0f,
    .position_min = -3.141593f,
    .position_max = 3.141593f,
    .torque_min = -54.0f,
    .torque_max = 54.0f,
#if ROBOT_ID == INFANTRY1_ID
    .power_limit_cfg = {
        .fit_params = {
            {.k1 = 1.0f, .k2 = 2.44f, .k3 = 0.0f, .k4 = 0.115f, .k5 = 2.0f},
            {.k1 = 1.0f, .k2 = 2.26f, .k3 = 0.0f, .k4 = 0.167f, .k5 = 2.0f},
        },
        .buffer_safe_energy = 60.0f,
        .buffer_kp = 0.0f,
        .buffer_ki = 0.0f,
        .buffer_kd = 0.0f,
        .cap_extra_power_limit = 0.0f,
    },
#elif ROBOT_ID == INFANTRY2_ID
    .power_limit_cfg = {
        .fit_params = {
            {.k1 = 1.0f, .k2 = 2.44f, .k3 = 0.0f, .k4 = 0.115f, .k5 = 2.0f},
            {.k1 = 1.0f, .k2 = 2.26f, .k3 = 0.0f, .k4 = 0.167f, .k5 = 2.0f},
        },
        .buffer_safe_energy = 60.0f,
        .buffer_kp = 1.0f,
        .buffer_ki = 0.0f,
        .buffer_kd = 0.0f,
        .cap_extra_power_limit = 120.0f,
    },
#endif
};
