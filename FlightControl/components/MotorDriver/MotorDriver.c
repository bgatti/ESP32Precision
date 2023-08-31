#include "math.h"
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "pid_ctrl.h"
#include "MotorDriver.h"

static const char *TAG = "MotorDriver";

typedef struct pid_ctrl_block_t pid_ctrl_block_t;
typedef float (*pid_cal_func_t)(pid_ctrl_block_t *pid, float error);

struct pid_ctrl_block_t {
    float Kp; // PID Kp value
    float Ki; // PID Ki value
    float Kd; // PID Kd value
    float previous_err1; // e(k-1)
    float previous_err2; // e(k-2)
    float integral_err;  // Sum of error
    float last_output;  // PID output in last control period
    float max_output;   // PID maximum output limitation
    float min_output;   // PID minimum output limitation
    float max_integral; // PID maximum integral value limitation
    float min_integral; // PID minimum integral value limitation
    pid_cal_func_t calculate_func; // calculation function, depends on actual PID type set by user
};


//Update Position and process PID Loop - run often
void MotorDriver_Update_Position(MotorDriverContext* ctx, int position) {
    ctx->set_position = position;

//    motor_control_context_t *ctx = (motor_control_context_t *)args;
    pcnt_unit_handle_t pcnt_unit = ctx->pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl = ctx->pid_ctrl;
    bdc_motor_handle_t motor = ctx->motor;

    // get the result from rotary encoder
    int cur_pulse_count = 0;
    pcnt_unit_get_count(pcnt_unit, &cur_pulse_count);
    int real_pulses = cur_pulse_count - ctx->last_pulse_count;
    ctx->last_pulse_count = cur_pulse_count;
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

    //ESP_LOGI(TAG, "PID compute: %s, %f, %f, %f", ctx->label, pid_ctrl->Kp, pid_ctrl->Ki, pid_ctrl->Kd);

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
        if (set_direction) {
            bdc_motor_forward(motor);  
        } else {
            bdc_motor_reverse(motor);  
        }
        bdc_motor_set_speed(motor, (uint32_t)fabs(new_speed));

        // print debug info
        //direction, speed, position, set position, error, new speed, real speed, target speed, position error
        //add label for debug purposes
        //ESP_LOGI(TAG, "Drive: %d, %d, %d, \t%d, \t%d, %d, %d", ctx->set_position, ctx->position, ctx->last_direction, (int)position_error,  real_pulses, (int)error,  (int)new_speed);
        ESP_LOGI(TAG, "Drive: %s, %d, %d, %d, %d, %d, %d, %d", ctx->label, ctx->set_position, ctx->position, ctx->last_direction, (int)position_error,  real_pulses, (int)error,  (int)new_speed);
    }
    else
    {
//        pid_compute(pid_ctrl, real_pulses * 100, &new_speed); // should reduce pid speed to 0
        bdc_motor_brake(motor);
        //add label for debug purposes
        //ESP_LOGI(TAG, "Brake: %d, %d, %d, %d, %d, %d, %d", ctx->set_position, ctx->position, ctx->last_direction, (int)position_error,  real_pulses, (int)error,  (int)new_speed);
        ESP_LOGI(TAG, "Brake: %s, %d, %d, %d, %d, %d, %d, %d", ctx->label, ctx->set_position, ctx->position, ctx->last_direction, (int)position_error,  real_pulses, (int)error,  (int)new_speed);
    }


}



// Function to initialize PCNT for a motor
static pcnt_unit_handle_t pcnt_init(int encoder_gpio) {

    //include encoder_gpio
    ESP_LOGI(TAG, "Init pcnt driver to decode single pin pulse signal on pin: %d", encoder_gpio);

    pcnt_unit_config_t unit_config = {
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;

    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = encoder_gpio, 
        .level_gpio_num = -1,
    };

    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    return pcnt_unit;
}


// Helper function to initialize a single motor
static bdc_motor_handle_t motor_init(MotorDriverConfig* config) {
    ESP_LOGI(TAG, "Create DC motor");

    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = config->pwma_gpio_num,
        .pwmb_gpio_num = config->pwmb_gpio_num,
    };

    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };

    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));

    ESP_LOGI(TAG, "Enable motor");
    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    
    return motor;
}


static pid_ctrl_block_handle_t pid_ctrl_init(pid_ctrl_parameter_t *init_params) {
    ESP_LOGI(TAG, "Create PID control block");

    pid_ctrl_block_handle_t pid_ctrl = NULL;
    pid_ctrl_config_t pid_config = {
        .init_param = *init_params
    };

    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl));

    return pid_ctrl;
}

MotorDriverContext** MotorDriver_Init(MotorDriverConfig configs[], int num_motors) {
    MotorDriverContext** contexts = malloc(num_motors * sizeof(MotorDriverContext*));

    for (int i = 0; i < num_motors; ++i) {
        MotorDriverContext* ctx = malloc(sizeof(MotorDriverContext));

        // Initialization of BDC Motor        
        ctx->motor = motor_init(&configs[i]);

        // Initialize PCNT
        ctx->pcnt_encoder = pcnt_init(configs[i].encoder_gpio);

        // Initialize PID using configs[i].pid_params
        ctx->pid_ctrl = pid_ctrl_init(&configs[i].pid_params);

        strncpy(ctx->label, configs[i].label, 10);

        //log pid K values as float
        ESP_LOGI(TAG, "PID instantiate: %s, %f, %f, %f", ctx->label, ctx->pid_ctrl->Kp, ctx->pid_ctrl->Ki, ctx->pid_ctrl->Kd);

        ctx->position = 0;
        ctx->last_pulse_count = 0;
        ctx->last_direction = 1;
        ctx->set_position = 0;
        ctx->report_pulses = 0;
        

        contexts[i] = ctx;
    }

    // ESP_LOGI(TAG, "Create a timer to do PID calculation periodically");
    // const esp_timer_create_args_t periodic_timer_args = {
    //     .callback = pid_loop_cb,
    //     .arg = &contexts,
    //     .name = "pid_loop"
    // };
    // esp_timer_handle_t pid_loop_timer = NULL;
    // ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &pid_loop_timer));

    // ESP_LOGI(TAG, "Start motor speed loop");
    // ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer, BDC_PID_LOOP_PERIOD_MS * 1000));


    return contexts;
}
