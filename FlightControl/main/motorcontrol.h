#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "motorCommand.h" // MotorCommand for incoming motor commands

// Configuration structure for a motor
typedef struct {
    QueueHandle_t commandQueue;  // Queue for motor commands
    uint8_t pin1;                // Pin for motor driver input 1
    uint8_t pin2;                // Pin for motor driver input 2
    uint8_t feedbackPin;         // Pin for feedback from motor (encoder or other)
    MotorIndex motorIndex;       // Index identifying the motor
} MotorControlConfig;

/**
 * @brief Initialize the motor with the provided configuration
 *
 * @param config A pointer to a MotorControlConfig structure containing the required configuration.
 */
void MotorControl_Init(MotorControlConfig *config);

/**
 * @brief Motor control task function.
 *
 * @param arg Task argument, should point to a MotorControlConfig structure.
 */
void MotorControl_Task(void *arg);

#endif // MOTOR_CONTROL_H
