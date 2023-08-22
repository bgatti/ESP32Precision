#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SDA_IO   4    /*!< gpio number for I2C master data  */
#define I2C_MASTER_SCL_IO   5    /*!< gpio number for I2C master clock */
#define I2C_MASTER_NUM      I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ  100000    /*!< I2C master clock frequency */
#define LIS2DW12TR_ADDR     0x18      /*!< slave address for LIS2DW12TR sensor */

#define WHO_AM_I_REG        0x0F      /*!< WHO_AM_I register address */

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

esp_err_t custom_i2c_master_read_byte(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t* data_rd) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( LIS2DW12TR_ADDR << 1 ) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( LIS2DW12TR_ADDR << 1 ) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data_rd, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 /portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void app_main() {
    i2c_master_init();

    uint8_t who_am_i;
    if(custom_i2c_master_read_byte(I2C_MASTER_NUM, WHO_AM_I_REG, &who_am_i) == ESP_OK) {
        ESP_LOGI("LIS2DW12TR", "WHO_AM_I register: 0x%x", who_am_i);
    } else {
        ESP_LOGE("LIS2DW12TR", "Failed to read WHO_AM_I register");
    }

    // Here you can read acceleration data and determine the orientation
    // You'll need to refer to the LIS2DW12TR datasheet to understand the data format
    // and which registers to read for acceleration values.

    // ...

}
