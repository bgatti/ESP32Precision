#include "driver/i2c.h"
#include "esp_log.h"

#include "lis2dw12_reg.h"

#define I2C_MASTER_SDA_IO   4    /*!< gpio number for I2C master data  */
#define I2C_MASTER_SCL_IO   5    /*!< gpio number for I2C master clock */
#define I2C_MASTER_NUM      I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ  100000    /*!< I2C master clock frequency */
#define LIS2DW12TR_ADDR     0x18      /*!< slave address for LIS2DW12TR sensor */

#define WHO_AM_I_REG        0x0F      /*!< WHO_AM_I register address */

#define CTRL4_INT1_PAD_CTRL_REG  0x23
#define CTRL5_INT2_PAD_CTRL_REG  0x24
#define CTRL6_REG                0x25
#define STATUS_REG               0x27

#define portTICK_RATE_MS         portTICK_PERIOD_MS
#define CONFIG_FREERTOS_HZ        100   

typedef enum
{
  LIS2DW12_STATUS_OK = 0,
  LIS2DW12_STATUS_ERROR
} LIS2DW12StatusTypeDef;

typedef struct
{
  unsigned int WakeUpStatus : 1;
  unsigned int D6DOrientationStatus : 1;
  unsigned int SleepStatus : 1;
} LIS2DW12_Event_Status_t;

typedef enum
{
  LIS2DW12_HIGH_PERFORMANCE_MODE,
  LIS2DW12_LOW_POWER_MODE4,
  LIS2DW12_LOW_POWER_MODE3,
  LIS2DW12_LOW_POWER_MODE2,
  LIS2DW12_LOW_POWER_MODE1
} LIS2DW12_Operating_Mode_t;

typedef enum
{
  LIS2DW12_LOW_NOISE_DISABLE,
  LIS2DW12_LOW_NOISE_ENABLE
} LIS2DW12_Low_Noise_t;

lis2dw12_ctx_t reg_ctx;

int32_t i2c_write(void *ctx, uint8_t devAddr, uint8_t *data, uint16_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIS2DW12TR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, devAddr, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
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
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void i2c_master_init() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;
    
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

/**
 * @brief  Configure the sensor in order to be used
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DW12StatusTypeDef begin(){
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

  /* Power mode selection */
  if (lis2dw12_power_mode_set(&reg_ctx, LIS2DW12_HIGH_PERFORMANCE) != 0)
  {
    return LIS2DW12_STATUS_ERROR;
  }

  /* Output data rate selection - power down. */
  if (lis2dw12_data_rate_set(&reg_ctx, LIS2DW12_XL_ODR_OFF) != 0)
  {
    return LIS2DW12_STATUS_ERROR;
  }

  /* Full scale selection. */
  if (lis2dw12_full_scale_set(&reg_ctx, LIS2DW12_2g) != 0)
  {
    return LIS2DW12_STATUS_ERROR;
  }

  /* Select default output data rate. */
//  X_Last_ODR = 100.0f;

  //X_Last_Operating_Mode = LIS2DW12_HIGH_PERFORMANCE_MODE;

  //X_Last_Noise = LIS2DW12_LOW_NOISE_DISABLE;

//  X_isEnabled = 0;

  return LIS2DW12_STATUS_OK;
}



void app_main() {
    i2c_master_init();

    reg_ctx.write_reg = i2c_write;
    reg_ctx.read_reg = i2c_read;
    reg_ctx.handle = &reg_ctx;

    ESP_LOGI("MainApp", "Starting LIS2DW12TR");

    if(begin(&reg_ctx) != LIS2DW12_STATUS_OK) {
        ESP_LOGE("LIS2DW12TR", "Failed to initialize LIS2DW12TR");
    }

    uint8_t who_am_i;
    if(i2c_read(&reg_ctx, WHO_AM_I_REG, &who_am_i, 1)== ESP_OK ) {
        ESP_LOGI("LIS2DW12TR", "WHO_AM_I register: 0x%x", who_am_i);
    } else {
        ESP_LOGE("LIS2DW12TR", "Failed to read WHO_AM_I register");
    }

    /* Output data rate selection - power down. */
    if (lis2dw12_data_rate_set(&reg_ctx, LIS2DW12_XL_ODR_100Hz) != 0)
    {
        ESP_LOGE("LIS2DW12TR", "Failed to Set Data Rate");
    }    




    while (true) {

        /* Read raw data values. */
        axis3bit16_t data_raw;
        if (lis2dw12_acceleration_raw_get(&reg_ctx, data_raw.u8bit) != 0)
        {
            ESP_LOGE("LIS2DW12TR", "Failed to read raw X");
        }

//        ESP_LOGI("LIS2DW12TR", "X: %d", data_raw.i16bit[0]);
        
        int16_t value[3];
        /* Data format 14 bits. */
        value[0] = (data_raw.i16bit[0] / 4) * 244 / 1000;
        value[1] = (data_raw.i16bit[1] / 4) * 244 / 1000;
        value[2] = (data_raw.i16bit[2] / 4) * 244 / 1000;

        //report xyz on a single line tab separated
        ESP_LOGI("LIS2DW12TR", "X: %d\tY: %d\t\tZ: %d", value[0], value[1], value[2]);

        vTaskDelay(pdMS_TO_TICKS(50)); // delay for 50ms
    }

}
