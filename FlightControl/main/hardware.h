#ifndef HARDWARE_H
#define HARDWARE_H
//filename: hardware.h

// Motor Driver Pins
#define MOTOR_A_INPUT1  30
#define MOTOR_A_INPUT2  31
#define MOTOR_A_FEEDBACK 32

#define MOTOR_B_INPUT1  33
#define MOTOR_B_INPUT2  34
#define MOTOR_B_FEEDBACK 35

#define MOTOR_C_INPUT1  37
#define MOTOR_C_INPUT2  36
#define MOTOR_C_FEEDBACK 26

// IMU Pins
//#define I2C_MASTER_SDA_IO   4    defined in LISDW12
//#define I2C_MASTER_SCL_IO   5    
#define ACCEL_INTERRUPT1  29
#define ACCEL_INTERRUPT2  28

// Misc Pins
#define LED1_PIN        7
#define VOLTAGE_MEASURE_PIN  18
#define STRAP_PIN       41

#endif // HARDWARE_H
