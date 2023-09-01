#ifndef IMU_H
#define IMU_H

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Enumerations to handle the state of the IMU sensor
typedef enum {
    Power_Down,
    Low_Power,
    Calibrate,
    Full_Speed
} IMUState;

// Struct to hold the 9 DOF (Degrees of Freedom) data
typedef struct {
    float accelerometer_x;
    float accelerometer_y;
    float accelerometer_z;

    float gyroscope_x;
    float gyroscope_y;
    float gyroscope_z;

    float magnetometer_x;
    float magnetometer_y;
    float magnetometer_z;
} IMUData;

// Function prototypes
/**
 * Initialize the IMU sensor
 * 
 * @return 0 on success, -1 on error
 */
int IMU_Init();

/**
 * Update the IMU data
 * 
 * @return 0 on success, -1 on error
 */
int IMU_Update(IMUData* data) ;

/**
 * Change the IMU sensor state
 * 
 * @param state The new state
 */
void IMU_SetState(IMUState state);

/**
 * Retrieve the current state of the IMU sensor
 * 
 * @return The current state
 */
IMUState IMU_GetState();


#endif // IMU_H
