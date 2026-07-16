#include "line_follower.h"

#include "chassis_motor.h"
#include "line_sensor.h"

#define LINE_FOLLOW_PERIOD_MS          (20U)
#define LINE_FOLLOW_BASE_SPEED         (300)
#define LINE_FOLLOW_TURN_GAIN          (20)
#define LINE_FOLLOW_MAX_TURN           (220)
#define LINE_FOLLOW_STEERING_INVERTED  (0)

/* LINE0 is assumed to be the leftmost sensor and LINE11 the rightmost. */
static const int8_t gSensorWeights[LINE_SENSOR_COUNT] = {
    -11, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 11
};

static bool gEnabled;
static uint32_t gLastProcessTime;

static int16_t clampTurn(int32_t turn)
{
    if (turn > LINE_FOLLOW_MAX_TURN) {
        return LINE_FOLLOW_MAX_TURN;
    }
    if (turn < -LINE_FOLLOW_MAX_TURN) {
        return -LINE_FOLLOW_MAX_TURN;
    }
    return (int16_t) turn;
}

void LineFollower_init(void)
{
    gEnabled = false;
    gLastProcessTime = 0U;
}

void LineFollower_process(uint32_t nowMs)
{
    uint16_t sensors;
    int32_t weightedSum = 0;
    uint8_t activeCount = 0U;
    uint8_t i;
    int32_t errorTenths;
    int32_t turn;

    if (!gEnabled ||
        ((nowMs - gLastProcessTime) < LINE_FOLLOW_PERIOD_MS)) {
        return;
    }
    gLastProcessTime = nowMs;
    sensors = LineSensor_readRaw();

    for (i = 0U; i < LINE_SENSOR_COUNT; i++) {
        if ((sensors & (uint16_t) (1U << i)) != 0U) {
            weightedSum += gSensorWeights[i];
            activeCount++;
        }
    }

    if (activeCount == 0U) {
        ChassisMotor_stop(nowMs);
        return;
    }

    errorTenths = (weightedSum * 10) / activeCount;
    turn = -(errorTenths * LINE_FOLLOW_TURN_GAIN) / 10;
#if LINE_FOLLOW_STEERING_INVERTED
    turn = -turn;
#endif
    ChassisMotor_setDifferential(LINE_FOLLOW_BASE_SPEED,
        clampTurn(turn), nowMs);
}

void LineFollower_setEnabled(bool enabled, uint32_t nowMs)
{
    gEnabled = enabled;
    if (enabled) {
        gLastProcessTime = nowMs - LINE_FOLLOW_PERIOD_MS;
    } else {
        ChassisMotor_stop(nowMs);
    }
}

bool LineFollower_isEnabled(void)
{
    return gEnabled;
}
