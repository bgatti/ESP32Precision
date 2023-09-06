#include "unity.h"
#include "Guidance.h"
#include <stdio.h>
#include <string.h>
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "testGuidance";

// Test the initialization function
TEST_CASE("Guidance_Init", "[Guidance]") {
    Guidance_Init();
    // Add checks to ensure IMU and MotorDriver are initialized.
    // This may involve exposing some internal state or adding some getters in the IMU/MotorDriver modules
    // E.g. TEST_ASSERT_EQUAL_INT(STATE_INITIALIZED, IMU_GetState());
    // E.g. TEST_ASSERT_EQUAL_INT(STATE_INITIALIZED, MotorDriver_GetState());
    Guidance_DeInit();

}

// Test the ComputeMotorControl function
TEST_CASE("ComputeMotorControl_Test", "[Guidance]") {
    float control_x = 100.0f;
    float control_y = 200.0f;
    float result;

    // Test for motor at 0 degrees
    result = ComputeMotorControl(control_x, control_y, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 100.0f, result);

    // Test for motor at 120 degrees
    result = ComputeMotorControl(control_x, control_y, 120.0f);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, -73.2f, result);

    // Test for motor at -120 degrees
    result = ComputeMotorControl(control_x, control_y, -120.0f);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 173.2f, result);
}

TEST_CASE("Guidance_Centering", "[Guidance]") {

    Guidance_Init();
    Guidance_Center();
    Guidance_DeInit();
}


TEST_CASE("Guidance_Loop", "[Guidance]") {

    Guidance_Init();
// loop to run Loop() 20 times with 20ms delay
    for (int i = 0; i < 300; ++i) {
        Guidance_Loop();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    Guidance_DeInit();
}

