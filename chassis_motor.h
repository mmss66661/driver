#ifndef CHASSIS_MOTOR_H_
#define CHASSIS_MOTOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "chassis_motor_mapping.h"

#define CHASSIS_FEEDBACK_REGISTERS (8U)

typedef struct {
    int16_t leftCommand;
    int16_t rightCommand;
    int16_t motorCommand[CHASSIS_PHYSICAL_MOTOR_COUNT];
    int16_t feedback[CHASSIS_FEEDBACK_REGISTERS];
    int16_t motorEncoderCount[CHASSIS_PHYSICAL_MOTOR_COUNT];
    int32_t leftEncoderCount;
    int32_t rightEncoderCount;
    uint32_t feedbackFrameCount;
    uint32_t acknowledgementCount;
    uint32_t crcErrorCount;
    uint32_t uartErrorCount;
    bool commandTimedOut;
} ChassisMotor_Status;

void ChassisMotor_init(void);
void ChassisMotor_process(uint32_t nowMs);
void ChassisMotor_setWheelSpeeds(
    int16_t leftSpeed, int16_t rightSpeed, uint32_t nowMs);
void ChassisMotor_setDifferential(
    int16_t forwardSpeed, int16_t turnSpeed, uint32_t nowMs);
void ChassisMotor_stop(uint32_t nowMs);
bool ChassisMotor_isClosedLoopEnabled(void);
void ChassisMotor_getStatus(ChassisMotor_Status *status);

#endif
