#include "unity.h"
#include "MotorDriver.h"

static MotorDriverContext* test_motor_ctx;
static MotorDriverContext** all_motor_ctx;

// A sample MotorDriverConfig array for two motors
MotorDriverConfig motorConfigs[] = {
    { .pwma_gpio_num = 48, .pwmb_gpio_num = 35, .encoder_gpio = 36, .pid_params = {.kp = 12.0, .ki = 3.2, .kd = 18.0} },
    { .pwma_gpio_num = 37, .pwmb_gpio_num = 38, .encoder_gpio = 39, .pid_params = {.kp = 12.0, .ki = 3.2, .kd = 18.0} }
};

// Testing the initialization function
TEST_CASE("MotorDriver_Init", "[MotorDriver]")
{
    all_motor_ctx = MotorDriver_Init(motorConfigs, 2);
    TEST_ASSERT_NOT_NULL(all_motor_ctx);
    test_motor_ctx = all_motor_ctx[0];
    TEST_ASSERT_NOT_NULL(test_motor_ctx);
    TEST_ASSERT_NOT_NULL(test_motor_ctx->pid_ctrl);
    TEST_ASSERT_NOT_NULL(test_motor_ctx->pcnt_encoder);
}

// Testing the SetPosition function
TEST_CASE("MotorDriver_SetPosition", "[MotorDriver]")
{
    int test_position = 100;
    MotorDriver_SetPosition(test_motor_ctx, test_position);
    //delay for 500ms


    // You may add assertions based on what SetPosition should do.
    // For instance, if it changes a variable inside the context, you could check that.
    TEST_ASSERT_EQUAL_INT(test_position, test_motor_ctx->position);
}
