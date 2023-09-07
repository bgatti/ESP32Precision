#include "LIS2DW12.h"
#include "esp_log.h"
#include "lis2dw12_reg.h"
#include "driver/i2c.h"
//#include "hardware.h" // in future push local defines to hardware.h
//#include "sdkconfig.h"
// include freertos
#include "freertos/FreeRTOS.h"

//file: components\LIS2DW12\LIS2DW12.c

// Define constants as needed
#define I2C_MASTER_NUM     I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define LIS2DW12TR_ADDR    0x18

#define WHO_AM_I_REG        0x0F      /*!< WHO_AM_I register address */


// Function implementations
int32_t i2c_write(void *ctx, uint8_t devAddr, uint8_t *data, uint16_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIS2DW12TR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, devAddr, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

int32_t i2c_read(void *ctx, uint8_t regAddr, uint8_t *data, uint16_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIS2DW12TR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, regAddr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIS2DW12TR_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void i2c_master_init(int sda, int scl) {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;
    
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

lis2dw12_ctx_t reg_ctx;

LIS2DW12StatusTypeDef accel_end(void) {

    /* Output data rate selection - power down. */
    if (lis2dw12_data_rate_set(&reg_ctx, LIS2DW12_XL_ODR_OFF) != 0)
    {
        return LIS2DW12_STATUS_ERROR;
    }


    i2c_driver_delete(I2C_MASTER_NUM);
    return LIS2DW12_STATUS_OK;
}


// Initialize the accelerometer
LIS2DW12StatusTypeDef accel_begin(int sda, int scl) {
    i2c_master_init(sda, scl);

    reg_ctx.write_reg = i2c_write;
    reg_ctx.read_reg = i2c_read;
    reg_ctx.handle = &reg_ctx;

    //settling delay
    vTaskDelay(10 / portTICK_PERIOD_MS);


    ESP_LOGI("LIS2DW12TR", "Starting LIS2DW12TR");

    uint8_t who_am_i;
    if(i2c_read(&reg_ctx, WHO_AM_I_REG, &who_am_i, 1)== ESP_OK ) {
        ESP_LOGI("LIS2DW12TR", "WHO_AM_I register: 0x%x", who_am_i);
    } else {
        ESP_LOGE("LIS2DW12TR", "Failed to read WHO_AM_I register");
    }

    /* Restore default configuration */
    if (lis2dw12_reset_set(&reg_ctx, PROPERTY_ENABLE) != 0)
    {
        return LIS2DW12_STATUS_ERROR;
    }

    uint8_t rst;
    do {
        lis2dw12_reset_get(&reg_ctx, &rst);
    } while (rst);

    /* Full scale selection. */
    if (lis2dw12_full_scale_set(&reg_ctx, LIS2DW12_2g) != 0)
    {
        return LIS2DW12_STATUS_ERROR;
    }

     /* Enable register address automatically incremented during a multiple byte
    access with a serial interface. */
    if (lis2dw12_auto_increment_set(&reg_ctx, PROPERTY_ENABLE) != 0)
    {
        return LIS2DW12_STATUS_ERROR;
    }

    /* Enable BDU */
    if (lis2dw12_block_data_update_set(&reg_ctx, PROPERTY_ENABLE) != 0)
    {
        return LIS2DW12_STATUS_ERROR;
    }

    /* FIFO mode selection */
    if (lis2dw12_fifo_mode_set(&reg_ctx, LIS2DW12_BYPASS_MODE) != 0)
    {
        return LIS2DW12_STATUS_ERROR;
    }


    /* Output data rate selection - power down. */
    if (lis2dw12_data_rate_set(&reg_ctx, LIS2DW12_XL_ODR_400Hz) != 0)
    {
        ESP_LOGE("LIS2DW12TR", "Failed to Set Data Rate");
    }    

    /* Power mode selection */
    if (lis2dw12_power_mode_set(&reg_ctx, LIS2DW12_HIGH_PERFORMANCE) != 0)
    {
        return LIS2DW12_STATUS_ERROR;
    }


    return LIS2DW12_STATUS_OK;
}

//Reads the accelerometer and returns the raw values
LIS2DW12StatusTypeDef read_accel_raw(lis_axis3bit16_t *accel_data) {
    /* Read raw data values. */
    axis3bit16_t data_raw;
    if (lis2dw12_acceleration_raw_get(&reg_ctx, data_raw.u8bit) != 0)
    {
        ESP_LOGE("LIS2DW12TR", "Failed to read raw X");
    }

    // debug the values
    //ESP_LOGI("LIS2DW12TR", "Raw X: %d, Y: %d, Z: %d", data_raw.i16bit[0], data_raw.i16bit[1], data_raw.i16bit[2]);

    
    //int16_t value[3];
    /* Data format 14 bits. */
    accel_data->i16bit[0] = (data_raw.i16bit[0] / 4) * 244 / 1000;
    accel_data->i16bit[1] = (data_raw.i16bit[1] / 4) * 244 / 1000;
    accel_data->i16bit[2] = (data_raw.i16bit[2] / 4) * 244 / 1000;

    return LIS2DW12_STATUS_OK;
}

LIS2DW12StatusTypeDef set_accel_mode(LIS2DW12_Operating_Mode_t mode)
{
    if (lis2dw12_power_mode_set(&reg_ctx, mode) != 0)
    {
        return LIS2DW12_STATUS_ERROR;
    }
    return LIS2DW12_STATUS_OK;
}


int testable_mean(const int* values, int count)
{
    if (count == 0) {
        return 0;
    }
    int sum = 0;
    for (int i = 0; i < count; ++i) {
        sum += values[i];
    }
    return sum / count;
}
