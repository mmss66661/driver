#ifndef STEPPER_MOTOR_H_
#define STEPPER_MOTOR_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    STEPPER_MOTOR_1    = 1U, /* Pitch */
    STEPPER_MOTOR_2    = 2U, /* Yaw */
    STEPPER_MOTOR_BOTH = 3U
} StepperMotor_Select;

typedef enum {
    STEPPER_FORWARD = 0U,
    STEPPER_REVERSE = 1U
} StepperMotor_Direction;

void StepperMotor_init(void);

/* Enable the driver (if needed) and start STEP pulses. */
void StepperMotor_start(StepperMotor_Select motor);

/* Stop STEP but keep the driver enabled so the axis retains holding torque. */
void StepperMotor_hold(StepperMotor_Select motor);

/* Stop STEP and drive ENABLE low to put the selected driver to sleep. */
void StepperMotor_stop(StepperMotor_Select motor);

void StepperMotor_setDirection(
    StepperMotor_Select motor, StepperMotor_Direction direction);
void StepperMotor_toggleDirection(StepperMotor_Select motor);

/* Set both axes to the same rate (compatibility helper). */
bool StepperMotor_setStepRate(uint32_t stepsPerSecond);
uint32_t StepperMotor_getStepRate(void);

/* Set/read one independent axis rate. Valid range: 100..20000 steps/s. */
bool StepperMotor_setMotorStepRate(
    StepperMotor_Select motor, uint32_t stepsPerSecond);
uint32_t StepperMotor_getMotorStepRate(StepperMotor_Select motor);

#endif /* STEPPER_MOTOR_H_ */
