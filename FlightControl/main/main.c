#include "FreeRTOS.h"
#include "task.h"
#include "MissionControl.h"

void app_main(void) {
    // Create a FreeRTOS task for MissionControl
    xTaskCreate(
        MissionControl_StateHandler, // Function that implements the task
        "MissionControl",            // Name of the task for debugging
        2048,                        // Stack size
        NULL,                        // Parameters passed to the task
        tskIDLE_PRIORITY,            // Priority
        NULL                         // Task handle
    );

    // Initialize any other tasks or resources here

    // Start the scheduler
    vTaskStartScheduler();

    // Should never reach here
    while(1) {
    }
}
