#include "unity.h"
#include "MotorDriver.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>


static MotorDriverContext* test_motor_ctx;
static MotorDriverContext** all_motor_ctx;

int num_motors = 2;

//TAG
static const char *TAG = "testMotorDriver";

// A sample MotorDriverConfig array for two motors
MotorDriverConfig motorConfigs[] = {
    { .pwma_gpio_num = 48, .pwmb_gpio_num = 35, .encoder_gpio = 36, .pid_params = {.kp = 12.0, .ki = 3.2, .kd = 18.0}, .label = "motor1" },
    { .pwma_gpio_num = 37, .pwmb_gpio_num = 38, .encoder_gpio = 39, .pid_params = {.kp = 12.0, .ki = 3.2, .kd = 18.0}, .label = "motor2" }
};

// Testing the initialization function
TEST_CASE("MotorDriver_Init", "[MotorDriver]")
{
    all_motor_ctx = MotorDriver_Init(motorConfigs, num_motors);
    TEST_ASSERT_NOT_NULL(all_motor_ctx);
    test_motor_ctx = all_motor_ctx[0];
    TEST_ASSERT_NOT_NULL(test_motor_ctx);
    TEST_ASSERT_NOT_NULL(test_motor_ctx->pid_ctrl);
    TEST_ASSERT_NOT_NULL(test_motor_ctx->pcnt_encoder);
}

// Testing the SetPosition function
TEST_CASE("MotorDriver_Update_Position", "[MotorDriver]")
{
    int test_position = 100;
    int32_t error = 0;

    for(int count = 0; count < 100; count++) {
        test_position = (int32_t)(150 * sin((float)esp_timer_get_time() / 1000000.0f * 2 * 3.1415926f / 1.0f)); 

        for(int i = 0; i < num_motors; ++i) {
            MotorDriverContext* ctx = all_motor_ctx[i];
            MotorDriver_Update_Position(ctx, test_position);
            error += abs(test_position - ctx->position);
        }        
        vTaskDelay(pdMS_TO_TICKS(40));
    }

    //report error as value between 0 and 1000
//    print_banner(("Error: %d", error));
//log error
    ESP_LOGI(TAG, "Error: %d", error);


    TEST_ASSERT_GREATER_OR_EQUAL_INT32(1000, error); // error should be less than 1000

}
