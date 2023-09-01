/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"
#include "math.h"

static const char *TAG = "example";

// Enable this config,  we will print debug formated string, which in return can be captured and parsed by Serial-Studio
#define SERIAL_STUDIO_DEBUG           CONFIG_SERIAL_STUDIO_DEBUG

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define BDC_MCPWM_GPIO_A              41 //37 //41 //48
#define BDC_MCPWM_GPIO_B              40 //38 //40 //35

#define BDC_ENCODER_GPIO_A            39 //26 //39 //26 //36
#define BDC_ENCODER_GPIO_B            -1
#define BDC_ENCODER_PCNT_HIGH_LIMIT   1000
#define BDC_ENCODER_PCNT_LOW_LIMIT    -1000

#define BDC_PID_LOOP_PERIOD_MS        20   // calculate the motor speed every 10ms
#define BDC_PID_MAX_SPEED          5  // expected motor speed, in the pulses counted by the rotary encoder

typedef struct {
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    pid_ctrl_block_handle_t pos_pid_ctrl;
    int report_pulses;
    int last_direction;
    int set_position;
    int position;
} motor_control_context_t;

static void pid_loop_cb(void *args)
{
    static int last_pulse_count = 0;
    motor_control_context_t *ctx = (motor_control_context_t *)args;
    pcnt_unit_handle_t pcnt_unit = ctx->pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl = ctx->pid_ctrl;
    pid_ctrl_block_handle_t pos_pid_ctrl = ctx->pos_pid_ctrl;
    bdc_motor_handle_t motor = ctx->motor;

    // get the result from rotary encoder
    int cur_pulse_count = 0;
    pcnt_unit_get_count(pcnt_unit, &cur_pulse_count);
    int real_pulses = cur_pulse_count - last_pulse_count;
    last_pulse_count = cur_pulse_count;
    ctx->report_pulses = real_pulses;
    // support servo mode
    // set speed and direction based on position PID
    //calculate position_error
    if(ctx->last_direction){
        ctx->position+= real_pulses;
    }    
    else{
        ctx->position-= real_pulses;
    }

    float position_error = ctx->set_position - ctx->position;

    // set the new speed
    float new_speed = 0;
    pid_compute(pid_ctrl, position_error, &new_speed);

    //set direction
    int set_direction = new_speed > 0;

    //wait for 0 pulses before change directions
    if(set_direction != ctx->last_direction && real_pulses == 0){
        ctx->last_direction = set_direction;
    }

    //calculate the target speed
    float new_target_speed = 0;

    // calculate the speed error as absolute value
    float error = fabs(new_target_speed) - real_pulses; 

    if( set_direction == ctx->last_direction) {
//        pid_compute(pos_pid_ctrl, position_error, &new_target_speed);
        bdc_motor_set_speed(motor, (uint32_t)fabs(new_speed));
        if (set_direction) {
            bdc_motor_forward(motor);  
        } else {
            bdc_motor_reverse(motor);  
        }

        // print debug info
        //direction, speed, position, set position, error, new speed, real speed, target speed, position error
        ESP_LOGI(TAG, "PID: %d, %d, %d, %d,      %d, %d, %d", ctx->set_position, ctx->position, ctx->last_direction, (int)position_error,  real_pulses, (int)error,  (int)new_speed);

    }
    else
    {
//        pid_compute(pid_ctrl, real_pulses * 100, &new_speed); // should reduce pid speed to 0
        bdc_motor_brake(motor);

        ESP_LOGI(TAG, "Brake: %d, %d, %d, %d,      %d, %d, %d", ctx->set_position, ctx->position, ctx->last_direction, (int)position_error,  real_pulses, (int)error,  (int)new_speed);

    }


}

void app_main(void)
{
    static motor_control_context_t motor_ctrl_ctx = {
        .pcnt_encoder = NULL,
    };

    ESP_LOGI(TAG, "Create DC motor");
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_A,
        .pwmb_gpio_num = BDC_MCPWM_GPIO_B,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
    motor_ctrl_ctx.motor = motor;

    ESP_LOGI(TAG, "Init pcnt driver to decode single pin pulse signal");

    pcnt_unit_config_t unit_config = {
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true, // enable counter accumulation
    };
    pcnt_unit_handle_t pcnt_unit = NULL;

    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    // Set up channel A configuration for pulse counting
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = BDC_ENCODER_GPIO_A,  // Assuming this is your pin
        .level_gpio_num = -1,  // Not used for single pin config
    };

    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    // Set edge action to count both rising and falling edges if desired
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
    motor_ctrl_ctx.pcnt_encoder = pcnt_unit;


    ESP_LOGI(TAG, "Create PID control block");
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
    pid_ctrl_block_handle_t pid_ctrl = NULL;
    pid_ctrl_config_t pid_config = {
        .init_param = pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl));
    motor_ctrl_ctx.pid_ctrl = pid_ctrl;

    //unused
    ESP_LOGI(TAG, "Create Position PID control block");
    pid_ctrl_parameter_t pos_pid_runtime_param = {
        .kp = 0.6 / 100,
        .ki = 0.4 / 100,
        .kd = 0.2 /100,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = BDC_PID_MAX_SPEED,
        .min_output   = BDC_PID_MAX_SPEED * -1,
        .max_integral = 1000,
        .min_integral = -1000,
    };
    pid_ctrl_block_handle_t pos_pid_ctrl = NULL;
    pid_ctrl_config_t pos_pid_config = {
        .init_param = pos_pid_runtime_param,
    };

    ESP_ERROR_CHECK(pid_new_control_block(&pos_pid_config, &pos_pid_ctrl));
    motor_ctrl_ctx.pos_pid_ctrl = pos_pid_ctrl;

    ESP_LOGI(TAG, "Create a timer to do PID calculation periodically");
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = pid_loop_cb,
        .arg = &motor_ctrl_ctx,
        .name = "pid_loop"
    };
    esp_timer_handle_t pid_loop_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &pid_loop_timer));

    ESP_LOGI(TAG, "Enable motor");
    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    ESP_LOGI(TAG, "Forward motor");
    ESP_ERROR_CHECK(bdc_motor_forward(motor));

    ESP_LOGI(TAG, "Reverse motor");
    ESP_ERROR_CHECK(bdc_motor_reverse(motor));

    ESP_LOGI(TAG, "Start motor speed loop");
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer, BDC_PID_LOOP_PERIOD_MS * 1000));

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        // the following logging format is according to the requirement of serial-studio frame format
        // also see the dashboard config file `serial-studio-dashboard.json` for more information

        // calculate a sine wave as float based on time, and set the setPosition of the motor to move the motor back and forth 100 pulses over 3 second period
        motor_ctrl_ctx.set_position = (int32_t)(150 * sin((float)esp_timer_get_time() / 1000000.0f * 2 * 3.1415926f / 1.0f)); 


#if SERIAL_STUDIO_DEBUG
        printf("/*%d*/\r\n", motor_ctrl_ctx.report_pulses);
#endif
    }
}
