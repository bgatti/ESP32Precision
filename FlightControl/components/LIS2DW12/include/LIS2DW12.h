#ifndef LIS2DW12_H
#define LIS2DW12_H

//file: components\LIS2DW12\include\LIS2DW12.h

#include "driver/i2c.h"

typedef enum {
    LIS2DW12_STATUS_OK = 0,
    LIS2DW12_STATUS_ERROR
} LIS2DW12StatusTypeDef;

typedef struct {
    unsigned int WakeUpStatus : 1;
    unsigned int D6DOrientationStatus : 1;
    unsigned int SleepStatus : 1;
} LIS2DW12_Event_Status_t;

typedef enum {
    LIS2DW12_HIGH_PERFORMANCE_MODE,
    LIS2DW12_LOW_POWER_MODE4,
    LIS2DW12_LOW_POWER_MODE3,
    LIS2DW12_LOW_POWER_MODE2,
    LIS2DW12_LOW_POWER_MODE1
} LIS2DW12_Operating_Mode_t;

typedef enum {
    LIS2DW12_LOW_NOISE_DISABLE,
    LIS2DW12_LOW_NOISE_ENABLE
} LIS2DW12_Low_Noise_t;


//duplicated here for convenience
typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} lis_axis3bit16_t;


//void i2c_master_init(void);
//int32_t i2c_write(void *ctx, uint8_t devAddr, uint8_t *data, uint16_t len);
//int32_t i2c_read(void *ctx, uint8_t regAddr, uint8_t *data, uint16_t len);

LIS2DW12StatusTypeDef accel_begin(int sda, int scl);
LIS2DW12StatusTypeDef accel_end(void);
LIS2DW12StatusTypeDef read_accel_raw(lis_axis3bit16_t *accel_data);
LIS2DW12StatusTypeDef set_accel_mode(LIS2DW12_Operating_Mode_t mode);

int testable_mean(const int* values, int count);

#endif // LIS2DW12_H
