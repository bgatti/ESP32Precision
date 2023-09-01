#include "unity.h"
#include "MotorDriver.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"


static MotorDriverContext* test_motor_ctx;
static MotorDriverContext** all_motor_ctx;

#define num_motors 3

//        .kp = 0.6 / 100,
//        .ki = 0.4 / 100,
//        .kd = 0.2 /100,
// PID const



//TAG
static const char *TAG = "testMotorDriver";



// A sample MotorDriverConfig array for two motors
MotorDriverConfig motorConfigs[num_motors];  // An array of two MotorDriverConfig




// Testing the initialization function
TEST_CASE("MotorDriver_Init", "[MotorDriver]")
{
    pid_ctrl_parameter_t pid_runtime_param = {
        .kp = 0.6  * 20,
        .ki = 0.4  * 8,
        .kd = 0.2 * 90,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = BDC_MCPWM_DUTY_TICK_MAX - 1,
        .min_output   = (BDC_MCPWM_DUTY_TICK_MAX - 1) * -1,    //0,
        .max_integral = 1000 /4,
        .min_integral = -1000/4,
    };

    MotorDriverConfig motorConfigs[] = {
        { .pwma_gpio_num = 48, .pwmb_gpio_num = 35, .encoder_gpio = 36, .pid_params = pid_runtime_param, .label = "motorA" },
        { .pwma_gpio_num = 37, .pwmb_gpio_num = 38, .encoder_gpio = 26, .pid_params = pid_runtime_param, .label = "motorB" },
        { .pwma_gpio_num = 41, .pwmb_gpio_num = 40, .encoder_gpio = 39, .pid_params = pid_runtime_param, .label = "motorC" },
    };
    
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
    int test_position = 0;
    int32_t error = 0;


    //disable all motors
    for(int i = 0; i < num_motors; ++i) {
        MotorDriver_Disable(all_motor_ctx[i]);
    }

    for(int i = 0; i < num_motors; ++i) {
        MotorDriver_Update_Position(all_motor_ctx[i], 0);
    }

    //store start millis
    uint64_t start_time = esp_timer_get_time();

    for(int i = 0; i < num_motors; ++i) {
        MotorDriver_Enable(all_motor_ctx[i]);
        start_time = esp_timer_get_time();

        for(int count = 0; count < 100; count++) {
            //get time since start
            uint64_t time_since_start = esp_timer_get_time() - start_time;
            //create sine sequence  
            test_position = (int32_t)(150 * sin((float)time_since_start / 1000000.0f * 2 * 3.1415926f / 1.0f)); 

            MotorDriverContext* ctx = all_motor_ctx[i];
            MotorDriver_Update_Position(ctx, test_position);
            error += abs(test_position - ctx->position);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        //disable motor
        MotorDriver_Disable(all_motor_ctx[i]);

        //log error
        ESP_LOGI(TAG, "Error: %d", (int)error);
        //TEST_ASSERT_LESS_OR_EQUAL_INT32(1000, error); // error should be less than 2000
        error=0;
    }        


    for(int i = 0; i < num_motors; ++i) {
        MotorDriver_Enable(all_motor_ctx[i]);
    }

    start_time = esp_timer_get_time();
    for(int count = 0; count < 100; count++) {
        //get time since start
        uint64_t time_since_start = esp_timer_get_time() - start_time;
        //create sine sequence  
        test_position = (int32_t)(150 * sin((float)time_since_start / 1000000.0f * 2 * 3.1415926f / 1.0f)); 

        for(int i = 0; i < num_motors; ++i) {
            MotorDriverContext* ctx = all_motor_ctx[i];
            MotorDriver_Update_Position(ctx, test_position);
            error += abs(test_position - ctx->position);
        }        
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    //disable all motors
    for(int i = 0; i < num_motors; ++i) {
        MotorDriver_Disable(all_motor_ctx[i]);
    }


    //log error
    ESP_LOGI(TAG, "Error: %d", (int)error);


    TEST_ASSERT_GREATER_OR_EQUAL_INT32(100, error); // error should be greater than 100
    TEST_ASSERT_LESS_OR_EQUAL_INT32(1000, error); // error should be less than 2000

}
