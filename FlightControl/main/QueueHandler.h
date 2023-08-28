#ifndef QUEUE_HANDLER_H
#define QUEUE_HANDLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    TEST_SUCCESS,
    TEST_FAIL,
    STABLE,
    UNSTABLE,
    FALL
} StatusEnum;

// Function to initialize all queues
void Queues_Init(QueueHandle_t *missionControlToGuidance, QueueHandle_t *guidanceToMissionControl);

#endif // QUEUE_HANDLER_H
