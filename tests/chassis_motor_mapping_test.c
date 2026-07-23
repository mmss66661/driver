#include <assert.h>
#include <limits.h>
#include <stdio.h>

#include "chassis_motor_mapping.h"

static void expectMapping(int16_t left, int16_t right,
    int16_t motor0, int16_t motor1, int16_t motor2, int16_t motor3)
{
    ChassisMotor_PhysicalCommand command;

    ChassisMotor_mapSideSpeeds(left, right, &command);

    assert(command.motor[0] == motor0);
    assert(command.motor[1] == motor1);
    assert(command.motor[2] == motor2);
    assert(command.motor[3] == motor3);
}

int main(void)
{
    expectMapping(10, 10, 10, -10, -10, -10);
    expectMapping(-10, -10, -10, 10, 10, 10);
    expectMapping(5, 15, 5, -15, -15, -5);
    expectMapping(0, 0, 0, 0, 0, 0);
    expectMapping(INT16_MIN, INT16_MIN,
        INT16_MIN, INT16_MAX, INT16_MAX, INT16_MAX);

    assert(ChassisMotor_averageWrappedEncoder(100, 104) == 102);
    assert(ChassisMotor_averageWrappedEncoder(-100, -104) == -102);
    assert(ChassisMotor_averageWrappedEncoder(32766, INT16_MIN) == 32767);
    assert(ChassisMotor_averageWrappedEncoder(INT16_MIN, 32766) == 32767);

    puts("chassis motor mapping tests passed");
    return 0;
}
