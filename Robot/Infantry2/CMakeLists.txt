set(BOARD "CHASSIS_BOARD")

add_library(Infantry2 OBJECT)

target_compile_definitions(Infantry2 PRIVATE
        BOARD=${BOARD}
        CHASSIS_BOARD=2
)

target_compile_definitions(PYRo PUBLIC
        REFEREE_UART=PYRO_UART1
        SUPERCAP_UART=PYRO_UART7
)

target_sources(Infantry2
        PRIVATE
        pyro_init_thread.cpp
        pyro_mission_planer.cpp

        Application/Chassis_board/pyro_infantry2_chassis_app.cpp
        Application/Chassis_board/pyro_infantry2_chassis_cfg.cpp
        Application/Chassis_board/pyro_infantry2_ui_app.cpp

        ${CMAKE_SOURCE_DIR}/PYRo/Algorithm/KF/kf.cpp

        Chassis/pyro_wl_chassis.cpp
        Chassis/fsm/pyro_wl_passive_state.cpp
        Chassis/fsm/pyro_wl_active_state/pyro_wl_active_state.cpp
        Chassis/fsm/pyro_wl_active_state/pyro_wl_test_state.cpp
        Chassis/fsm/pyro_wl_active_state/pyro_wl_ready_state.cpp
        Chassis/fsm/pyro_wl_active_state/pyro_wl_normal_state.cpp
        Chassis/fsm/pyro_wl_active_state/pyro_wl_reverse_state.cpp
        Chassis/fsm/pyro_wl_active_state/pyro_wl_over_step_state.cpp
        Chassis/fsm/pyro_wl_active_state/pyro_wl_over_step_ready_state.cpp
        Chassis/fsm/pyro_wl_active_state/pyro_wl_over_step_reset_state.cpp
        Chassis/fsm/pyro_wl_active_state/pyro_wl_control_state.cpp
        Chassis/fsm/pyro_wl_active_state/pyro_wl_spin_state.cpp

        Communication/General/pyro_com_canrx.cpp
        Communication/General/pyro_com_cantx.cpp
)

target_include_directories(Infantry2
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/Application/Chassis_board
        ${CMAKE_CURRENT_SOURCE_DIR}/Chassis
        ${CMAKE_CURRENT_SOURCE_DIR}/Communication/General
)

target_link_libraries(Infantry2
        PUBLIC
        stm32cubemx
        dsppp_lib
        PYRo
)
