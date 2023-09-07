#include "Guidance.h"
#include "esp_timer.h"

static const char *TAG = "Guidance";

//pid
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

static pid_ctrl_block_handle_t pid_ctrl_init(pid_ctrl_parameter_t *init_params) {
    ESP_LOGI(TAG, "Create PID control block");
    pid_ctrl_block_handle_t pid_ctrl = NULL;
    pid_ctrl_config_t pid_config = {
        .init_param = *init_params
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl));
    return pid_ctrl;
}

typedef struct {
    pid_ctrl_block_handle_t pid_ctrl;
    float average_corrected;
    float nominal_accel;
    float pid_output;
} axis_guidance;

// declare static axis_guidance for x,y;
static axis_guidance axis_guidance_x;
static axis_guidance axis_guidance_y;

static IMUData initial_data;
static IMUData data;

//Motors
static MotorDriverContext* test_motor_ctx;
static MotorDriverContext** all_motor_ctx;

#define num_motors 3
MotorDriverConfig motorConfigs[num_motors];


#define MAX_FIND_POSITION 220

// Center a single motor
void Center_Motor(MotorDriverContext* ctx) {


    for (int i = 0; i < MAX_FIND_POSITION; i=i+15) // update motor each 20ms
    {
        MotorDriver_Update_Position(ctx, i);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    for (int i = MAX_FIND_POSITION; i > -MAX_FIND_POSITION; i = i-15) // update motor each 20ms
    {
        MotorDriver_Update_Position(ctx, i);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // get range of motion from min/max
    int range = ctx->max_position - ctx->min_position;
    ESP_LOGI(TAG, "range: %d", range);

    //debug min/max
    ESP_LOGI(TAG, "min_position: %d", ctx->min_position);

    //update current position to 0 - range/2
    ctx->position = 0 - range/2;
    ctx->min_position = 0 - range/2;
    ctx->max_position = range/2;

    //debug new position
    ESP_LOGI(TAG, "new position: %d", ctx->position);
    MotorDriver_Disable(ctx);
    MotorDriver_Enable(ctx);

    //update set position to 0
    for (int i = 0; i < 50; ++i) 
    {
        MotorDriver_Update_Position(ctx, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    MotorDriver_Disable(ctx);

}



void Guidance_Center() { // centers the motors (finds and sets to 0
    // for i = 0 to num_motors-1 { call subroutine Center_Motor(all_motor_ctx[i])}
    for (int i = 0; i < num_motors; ++i) {
        // Update Motor Drivers
        Center_Motor(all_motor_ctx[i]);
    }

} 



void UpdateAccumulatedError(axis_guidance* axis_guidance, float current_accel) {
    //debug passed values
//    ESP_LOGI(TAG, "current_accel: %f", current_accel);
//    ESP_LOGI(TAG, "nominal_accel: %f", axis_guidance->nominal_accel);
//    ESP_LOGI(TAG, "accumulated_error: %f", axis_guidance->accumulated_error);

//    axis_guidance->accumulated_error += (current_accel - axis_guidance->nominal_accel);
    float weight = 5;
    float last = axis_guidance->average_corrected * weight;
    float current = current_accel - axis_guidance->nominal_accel;
    axis_guidance->average_corrected = (last + current) / (weight + 1);

    float input = axis_guidance->average_corrected * -1 ;


    //debug accumulated error
    //ESP_LOGI(TAG, "accumulated_error: %f", axis_guidance->accumulated_error);

    //calculate pid use pid_compute 
    float pid_output;
    pid_compute(axis_guidance->pid_ctrl, input, &pid_output);
    axis_guidance->pid_output = pid_output;
    //debug
    //ESP_LOGI(TAG, "pid_output: %f", axis_guidance->pid_output);

}

// Initialize Guidance
void Guidance_Init() {
    // Initialize IMU to HighPower
    IMU_Init();
    IMU_SetState(Full_Speed);

    // initiate Motor pid parameters
    pid_ctrl_parameter_t pid_motor_param = {
        .kp = 0.6 * 85, //75,
        .ki = 0.4 * 10,
        .kd = 0.2 * 73,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = BDC_MCPWM_DUTY_TICK_MAX*5/6 - 1,
        .min_output   = (BDC_MCPWM_DUTY_TICK_MAX*5/6 - 1) * -1,    //0,
        .max_integral = BDC_MCPWM_DUTY_TICK_MAX /2,
        .min_integral = -BDC_MCPWM_DUTY_TICK_MAX/2,
    };


    // initiate Guidance pid parameters
    pid_ctrl_parameter_t pid_guidance_param = {
        .kp = 0.6 * 0.2,
        .ki = 0.4 * 0.0,
        .kd = 0.2 * 0.0,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = MAX_POSITION,
        .min_output   = MAX_POSITION * -1,
        .max_integral = MAX_POSITION * 1,
        .min_integral = -MAX_POSITION * 1,
    };

    axis_guidance_x.pid_ctrl = pid_ctrl_init(&pid_guidance_param);
    axis_guidance_x.average_corrected = 0;
    axis_guidance_x.nominal_accel = 0;

    axis_guidance_y.pid_ctrl = pid_ctrl_init(&pid_guidance_param);
    axis_guidance_y.average_corrected = 0;
    axis_guidance_y.nominal_accel = 0;


    // Initialize MotorDriver
    MotorDriverConfig motorConfigs[] = {
        { .pwma_gpio_num = 48, .pwmb_gpio_num = 35, .encoder_gpio = 36, .pid_params = pid_motor_param, .label = "motorA" },
        { .pwma_gpio_num = 37, .pwmb_gpio_num = 38, .encoder_gpio = 26, .pid_params = pid_motor_param, .label = "motorB" },
        { .pwma_gpio_num = 41, .pwmb_gpio_num = 40, .encoder_gpio = 39, .pid_params = pid_motor_param, .label = "motorC" },
    };
    
    all_motor_ctx = MotorDriver_Init(motorConfigs, num_motors);
    test_motor_ctx = all_motor_ctx[0];

    //small delay to allow IMU to initialize
    //vTaskDelay(pdMS_TO_TICKS(20));
    // Read the initial "nominal" IMU Data
    //IMU_Update(&initial_data);  
    initial_data.accelerometer_x = 0;
    initial_data.accelerometer_y = 0;

    //repeat the above and average of 10 readings
    for (int i = 0; i < 10; ++i) {
        IMU_Update(&data);
        initial_data.accelerometer_x += data.accelerometer_x;
        initial_data.accelerometer_y += data.accelerometer_y;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    initial_data.accelerometer_x = initial_data.accelerometer_x / 10;
    initial_data.accelerometer_y = initial_data.accelerometer_y / 10;


    // Set nominal accel values
    axis_guidance_x.nominal_accel = initial_data.accelerometer_x;
    axis_guidance_y.nominal_accel = initial_data.accelerometer_y;

    //debug nominal accel
    ESP_LOGI(TAG, "axis_guidance_x.nominal_accel: %f", axis_guidance_x.nominal_accel);
    ESP_LOGI(TAG, "axis_guidance_y.nominal_accel: %f", axis_guidance_y.nominal_accel);

    MotorDriver_Enable(all_motor_ctx[0]);
    MotorDriver_Enable(all_motor_ctx[1]);
    MotorDriver_Enable(all_motor_ctx[2]);

    //short delay
    vTaskDelay(pdMS_TO_TICKS(20));

    // Update Motor Drivers
    MotorDriver_Update_Position(all_motor_ctx[0], 0);
    MotorDriver_Update_Position(all_motor_ctx[1], 0);
    MotorDriver_Update_Position(all_motor_ctx[2], 0);

}

// Halt Guidance
void Guidance_DeInit() {
    // Halt IMU
    IMU_Deinit();

    // Halt MotorDriver
    MotorDriver_DeInit(all_motor_ctx, num_motors);
    
    //free(all_motor_ctx);

    
}

// Compute control value for given angle
float ComputeMotorControl(float control_x, float control_y, float angle_degrees) {
    float angle_radians = angle_degrees * (M_PI / 180.0);
    return control_x * cos(angle_radians) + control_y * sin(angle_radians);
}

void Guidance_Loop(){
        //read imu 10 times with random delay between 0 and 1 ms
        IMUData average_data;
        average_data.accelerometer_x = 0;
        average_data.accelerometer_y = 0;
        for (int i = 0; i < 20; ++i) {
            IMU_Update(&data);
            average_data.accelerometer_x += data.accelerometer_x;
            average_data.accelerometer_y += data.accelerometer_y;            
            vTaskDelay(pdMS_TO_TICKS(rand() % 2));
        }
        data.accelerometer_x = average_data.accelerometer_x / 20;
        data.accelerometer_y = average_data.accelerometer_y / 20;

//        IMU_Update(&data);

        //debug on single line
//        ESP_LOGI(TAG, "data.accelerometer_x: %f, data.accelerometer_y: %f", data.accelerometer_x, data.accelerometer_y);

        // Update accumulated error in method
        UpdateAccumulatedError(&axis_guidance_x, data.accelerometer_x);
        UpdateAccumulatedError(&axis_guidance_y, data.accelerometer_y);

        float control_x = axis_guidance_x.pid_output;
        float control_y = axis_guidance_y.pid_output;

        //Motor A Controls X Axis

        // Compute controls for motors at 0, 120, and -120 degrees
        int control_motor_0 = (int)ComputeMotorControl(control_x, control_y, ANGLE_MOTOR_0);
        int control_motor_1 = (int)ComputeMotorControl(control_x, control_y, ANGLE_MOTOR_1);
        int control_motor_2 = (int)ComputeMotorControl(control_x, control_y, ANGLE_MOTOR_2);

        // Update Motor Drivers
        MotorDriver_Update_Position(all_motor_ctx[0], control_x);
//        MotorDriver_Update_Position(all_motor_ctx[1], control_y);
//        MotorDriver_Update_Position(all_motor_ctx[2], control_motor_2);

// debug nominalx, accelx, accum x and control x
        ESP_LOGI(TAG, "nominal: %f, x: %f, avg: %f, control_x: %f", axis_guidance_x.nominal_accel, data.accelerometer_x, axis_guidance_x.average_corrected, control_x);

        //debug on a single line         
//        ESP_LOGI(TAG, "control_x: %f, control_y: %f, control_motor_0: %d, control_motor_1: %d, control_motor_2: %d", control_x, control_y, control_motor_0, control_motor_1, control_motor_2);

    
}

// Guidance Main Run Loop
void Guidance_Run(void *pvParameters) {
    while (1) {
        Guidance_Loop();

        // Logging and Error handling could go here.

        // Delay for 20ms
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Main Function
int main(void) {
    // Initialize Guidance
    Guidance_Init();

    // Create Guidance Task
    xTaskCreate(Guidance_Run, "GuidanceRun", 2048, NULL, 5, NULL);

    // Start FreeRTOS Scheduler
    vTaskStartScheduler();

    return 0;
}
