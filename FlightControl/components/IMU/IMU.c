#include "IMU.h"
#include "LIS2DW12.h"

static IMUData current_data = {0};
static IMUState current_state = Power_Down;

// Initialize the IMU sensor
int IMU_Init() {
    LIS2DW12StatusTypeDef status = accel_begin();
    if (status != LIS2DW12_STATUS_OK) {
        ESP_ERROR_CHECK(-1);
    }
    return 0;
}

// Update the IMU data
int IMU_Update(IMUData* data) {
    LIS2DW12StatusTypeDef status;    
    lis_axis3bit16_t raw_data;
    
    if (current_state != Power_Down) {
        status = read_accel_raw(&raw_data);
        if (status != LIS2DW12_STATUS_OK) {
            ESP_ERROR_CHECK(-1);
        }
        
        // Convert the raw data to actual data here and update `current_data`.
        // For simplicity, using raw_data as is
        current_data.accelerometer_x = raw_data.i16bit[0];
        current_data.accelerometer_y = raw_data.i16bit[1];
        current_data.accelerometer_z = raw_data.i16bit[2];
    }
    *data = current_data;    
    return 0;
}

// Change the IMU sensor state
void IMU_SetState(IMUState state) {
    LIS2DW12StatusTypeDef status = LIS2DW12_STATUS_OK;

    switch(state) {
        case Power_Down:
            status = set_accel_mode(LIS2DW12_LOW_POWER_MODE1); // Just an example
            break;
        case Low_Power:
            status = set_accel_mode(LIS2DW12_LOW_POWER_MODE2); // Just an example
            break;
        case Calibrate:
            // Add calibration logic here
            break;
        case Full_Speed:
            status = set_accel_mode(LIS2DW12_HIGH_PERFORMANCE_MODE);
            break;
    }

    if (status != LIS2DW12_STATUS_OK) {
        ESP_ERROR_CHECK(-1);
    }
    current_state = state;
}

// Retrieve the current state of the IMU sensor
IMUState IMU_GetState() {
    return current_state;
}
