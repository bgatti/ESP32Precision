#ifndef MISSIONCONTROL_H
#define MISSIONCONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Enumerated type to represent the different states of Mission Control
typedef enum {
    SLEEP,
    TEST,
    ERROR,
    STANDBY,
    STABLE,
    ACTIVE
} MissionState;

// Function to initialize MissionControl
void MissionControl_Init(void);

// Function to handle transitions between states
void MissionControl_StateHandler(void *pvParameters);

// Queue to interface between MissionControl and Guidance modules
extern QueueHandle_t MissionControltoGuidance;

#endif // MISSIONCONTROL_H
