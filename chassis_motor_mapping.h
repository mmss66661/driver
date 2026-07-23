#ifndef CHASSIS_MOTOR_MAPPING_H_
#define CHASSIS_MOTOR_MAPPING_H_

#include <stdint.h>

#define CHASSIS_PHYSICAL_MOTOR_COUNT (4U)

typedef struct {
    int16_t motor[CHASSIS_PHYSICAL_MOTOR_COUNT];
} ChassisMotor_PhysicalCommand;

static inline int16_t ChassisMotor_negateSaturated(int16_t value)
{
    return (value == INT16_MIN) ? INT16_MAX : (int16_t) -value;
}

static inline void ChassisMotor_mapSideSpeeds(
    int16_t left, int16_t right, ChassisMotor_PhysicalCommand *command)
{
    command->motor[0] = left;
    command->motor[1] = ChassisMotor_negateSaturated(right);
    command->motor[2] = ChassisMotor_negateSaturated(right);
    command->motor[3] = ChassisMotor_negateSaturated(left);
}

static inline int16_t ChassisMotor_averageWrappedEncoder(
    int16_t first, int16_t second)
{
    uint16_t difference = (uint16_t) second - (uint16_t) first;
    int32_t signedDifference = (difference <= INT16_MAX) ?
        (int32_t) difference : (int32_t) difference - 65536;
    int32_t midpoint =
        (int32_t) (uint16_t) first + signedDifference / 2;

    if (midpoint < 0) {
        midpoint += 65536;
    } else if (midpoint > UINT16_MAX) {
        midpoint -= 65536;
    }

    return (midpoint <= INT16_MAX) ? (int16_t) midpoint :
        (int16_t) (midpoint - 65536);
}

#endif
