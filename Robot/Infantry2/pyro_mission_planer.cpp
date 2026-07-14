#include "FreeRTOS.h"
#include "pyro_core_config.h"
#include "pyro_core_def.h"
#include "task.h"

extern "C"
{
extern void pyro_init_thread(void *argument);
extern void start_debug_task(void *arg);
extern pyro::status_t infantry2_chassis_init(void *argument);

void start_mission_planer_task(void const *argument)
{
    (void)argument;

    xTaskCreate(pyro_init_thread, "pyro_init_thread", 512, nullptr,
                configMAX_PRIORITIES - 1, nullptr);
    vTaskDelay(pdMS_TO_TICKS(20));

    infantry2_chassis_init(nullptr);

#if DEBUG_MODE
    xTaskCreate(start_debug_task, "start_debug_task", 512, nullptr,
                configMAX_PRIORITIES - 3, nullptr);
#endif

    vTaskDelete(nullptr);
}
}
