#ifndef MOTOR_COMMAND_H
#define MOTOR_COMMAND_H

#include <stdint.h>

// Enum for identifying the motor index
typedef enum {
    MOTOR_INDEX_1,
    MOTOR_INDEX_2,
    MOTOR_INDEX_3,
    // Add more motors here
} MotorIndex;

// Enum for specifying the type of motor command
typedef enum {
    MOTOR_COMMAND_POSITION,
    MOTOR_COMMAND_SPEED,
    // Add more commands here
} MotorCommandType;

// Structure for holding a motor command
typedef struct {
    MotorIndex index;        // Which motor the command is for
    MotorCommandType command; // What type of command it is (position, speed)
    int32_t value;           // Value for the command
} MotorCommand;

#endif // MOTOR_COMMAND_H
