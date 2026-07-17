#include "FreeRTOS.h"
#include "pyro_core_config.h"
#include "pyro_core_def.h"
#include "task.h"

extern "C"
{
extern void pyro_init_thread(void *argument);
extern void start_debug_task(void *arg);
extern void infantry1_chassis_init(void *argument);

void start_mission_planer_task(void const *argument)
{
    xTaskCreate(pyro_init_thread, "pyro_init_thread", 512, nullptr,
                configMAX_PRIORITIES - 1, nullptr);
    vTaskDelay(pdMS_TO_TICKS(20));

    xTaskCreate(infantry1_chassis_init, "infantry1_chassis_init", 512, nullptr,
                configMAX_PRIORITIES - 2, nullptr);

#if DEBUG_MODE
    xTaskCreate(start_debug_task, "start_debug_task", 512, nullptr,
                configMAX_PRIORITIES - 3, nullptr);
#endif

    vTaskDelete(nullptr);
}
}
