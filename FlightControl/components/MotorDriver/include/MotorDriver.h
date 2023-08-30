#ifndef _MOTORDRIVER_H_
#define _MOTORDRIVER_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"
#include "esp_timer.h"
#include "math.h"


#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define BDC_ENCODER_PCNT_HIGH_LIMIT   1000
#define BDC_ENCODER_PCNT_LOW_LIMIT    -1000

#define BDC_PID_LOOP_PERIOD_MS        20   // calculate the motor speed every 10ms
#define BDC_PID_MAX_SPEED          5  // expected motor speed, in the pulses counted by the rotary encoder



typedef struct {
    uint32_t pwma_gpio_num;
    uint32_t pwmb_gpio_num;
    uint32_t encoder_gpio;
    pid_ctrl_parameter_t pid_params;
    char label[10];
} MotorDriverConfig;

typedef struct {
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    int report_pulses;
    int last_direction;
    int set_position;
    int position;
    //add label for debug purposes
    char label[10];
} MotorDriverContext;


MotorDriverContext** MotorDriver_Init(MotorDriverConfig configs[], int num_motors);
void MotorDriver_Update_Position(MotorDriverContext* ctx, int position);
static pid_ctrl_block_handle_t pid_ctrl_init(pid_ctrl_parameter_t *init_params);
static bdc_motor_handle_t motor_init(MotorDriverConfig* config);
static pcnt_unit_handle_t pcnt_init(int encoder_gpio);

#endif // _MOTORDRIVER_H_
