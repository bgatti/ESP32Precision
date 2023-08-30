#include "MotorDriver.h"
#include "esp_log.h"
#include "esp_err.h"
#include "pid_ctrl.h"

static const char *TAG = "MotorDriver";

// Function to initialize PCNT for a motor
static pcnt_unit_handle_t pcnt_init(int encoder_gpio) {
    ESP_LOGI(TAG, "Init pcnt driver to decode single pin pulse signal");

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

        contexts[i] = ctx;
    }

    return contexts;
}

void MotorDriver_SetPosition(MotorDriverContext* ctx, int position) {
    ctx->set_position = position;
}