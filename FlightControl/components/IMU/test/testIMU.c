#include "unity.h"
#include "IMU.h"
#include <stdio.h>
#include <string.h>
#include "esp_timer.h"
#include "esp_log.h"


static const char *TAG = "testIMU";


// Testing the initialization function
TEST_CASE("IMU_Init", "[IMU]")
{
    int ret = IMU_Init();
    TEST_ASSERT_EQUAL_INT(0, ret);
}

TEST_CASE("IMU_Deinit", "[IMU]")
{
    int ret = IMU_Deinit();
    TEST_ASSERT_EQUAL_INT(0, ret);
}

// Testing the IMU Update function
TEST_CASE("IMU_Update", "[IMU]")
{
    IMUData data;

    // Initialize IMU
    IMU_Init();
    // Set the IMU to full-speed mode
    IMU_SetState(Full_Speed);

    // Loop for 100 reads
    for (int i = 0; i < 30; ++i) {
        // Update IMU data
        int ret = IMU_Update(&data);

        TEST_ASSERT_EQUAL_INT(0, ret);

        // Debug the values
        ESP_LOGI(TAG, "Accel X: %f, Y: %f, Z: %f", data.accelerometer_x, data.accelerometer_y, data.accelerometer_z);
        //ESP_LOGI(TAG, "Gyro X: %f, Y: %f, Z: %f", data.gyroscope_x, data.gyroscope_y, data.gyroscope_z);
        ESP_LOGI(TAG, "Mag X: %f, Y: %f, Z: %f", data.magnetometer_x, data.magnetometer_y, data.magnetometer_z);
        //heading
        ESP_LOGI(TAG, "Heading: %f", data.heading);

        //reset values
        data.accelerometer_x = 0;
        data.accelerometer_y = 0;
        data.accelerometer_z = 0;
        data.gyroscope_x = 0;
        data.gyroscope_y = 0;
        data.gyroscope_z = 0;
        data.magnetometer_x = 0;
        data.magnetometer_y = 0;
        data.magnetometer_z = 0;
        data.heading = 0;


        // Wait for 20 ms
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
