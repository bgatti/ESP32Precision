#include "IMU.h"
#include "LIS2DW12.h"
#include "qmc5883l.h"
#include "esp_err.h"
#include "esp_log.h"
#include <math.h>

static IMUData current_data = {0};
static IMUState current_state = Power_Down;

static const char *TAG = "IMU";
static qmc5883l_t qmc5883l_dev;

// Initialize the IMU sensor
int IMU_Init() {    

    if(false){
        i2cdev_init();
        // Initialize the QMC5883L magnetometer
        esp_err_t ret = qmc5883l_init_desc(&qmc5883l_dev, QMC5883L_I2C_ADDR_DEF, I2C_NUM_0, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize QMC5883L magnetometer");
        }

        ret = qmc5883l_set_config(&qmc5883l_dev,  QMC5883L_DR_50, QMC5883L_OSR_512, QMC5883L_RNG_2 );
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set QMC5883L magnetometer config");
        }

        ret = qmc5883l_set_mode(&qmc5883l_dev, QMC5883L_MODE_CONTINUOUS);
        if (ret != ESP_OK) {        
            ESP_LOGE(TAG, "Failed to set QMC5883L magnetometer mode");
        }
    }

    LIS2DW12StatusTypeDef status = accel_begin(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    if (status != LIS2DW12_STATUS_OK) {
        ESP_LOGE(TAG, "Failed to initialize LISDW12 acceleromater");
        ESP_ERROR_CHECK(-1);
    }

    ESP_LOGI(TAG, "IMU initialization successful");
    return ESP_OK;
}

// Deinitialize the IMU sensor
int IMU_Deinit() {
    if(false){

        // Deinitialize the LIS2DW12 accelerometer
        LIS2DW12StatusTypeDef status = LIS2DW12_STATUS_OK;
        status = set_accel_mode(LIS2DW12_LOW_POWER_MODE1);
        if (status != LIS2DW12_STATUS_OK) {
            ESP_ERROR_CHECK(-1);
        }

        // Deinitialize the QMC5883L magnetometer
        esp_err_t ret = qmc5883l_set_mode(&qmc5883l_dev, QMC5883L_MODE_STANDBY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set QMC5883L magnetometer mode");
        }

        //deinit i2c
        i2cdev_done();
    }

    

    ESP_LOGI(TAG, "IMU deinitialization successful");
    return ESP_OK;
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

        if(false){
            // Update the magnetometer data
            qmc5883l_raw_data_t raw_mag_data;
            qmc5883l_data_t mag_data;
            esp_err_t ret = qmc5883l_get_raw_data(&qmc5883l_dev, &raw_mag_data);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to get raw magnetometer data");
            }
            ret = qmc5883l_raw_to_mg(&qmc5883l_dev, &raw_mag_data, &mag_data);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to convert raw magnetometer data");
            }
            current_data.magnetometer_x = mag_data.x;
            current_data.magnetometer_y = mag_data.y;
            current_data.magnetometer_z = mag_data.z;
            //calulate heading
            current_data.heading = atan2(current_data.magnetometer_y, current_data.magnetometer_x) * 180 / M_PI;
        }

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
