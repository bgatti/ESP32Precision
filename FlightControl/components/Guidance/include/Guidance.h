#ifndef GUIDANCE_H_
#define GUIDANCE_H_

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "IMU.h"
#include "MotorDriver.h"
#include "pid_ctrl.h"

#define GUIDANCE_TAG "GUIDANCE"

#define ANGLE_MOTOR_0  0.0
#define ANGLE_MOTOR_1  120.0
#define ANGLE_MOTOR_2 -120.0

#define MAX_POSITION 45  // max swing of Motor

// methods
void Guidance_Init();
void Guidance_Center(); // centers the motors (finds and sets to 0
void Guidance_DeInit();
void Guidance_Run(); // starts GuidanceRun task
void Guidance_Loop(); // single loop 

float ComputeMotorControl(float control_x, float control_y, float angle_degrees);


#endif // GUIDANCE_H_
