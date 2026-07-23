#include "line_follower.h"

#include <stddef.h>

#include "chassis_motor.h"
#include "line_sensor.h"

#define LINE_FOLLOW_PERIOD_MS              (20U)
#define LINE_FOLLOW_BASE_SPEED             (10)
#define LINE_FOLLOW_MIN_SPEED              (6)
#define LINE_FOLLOW_LOST_SPEED             (3)
#define LINE_FOLLOW_LOST_TURN              (20)//5
#define LINE_FOLLOW_MAX_TURN               (20)//10
#define LINE_FOLLOW_INTERSECTION_COUNT     (6U)
#define LINE_FOLLOW_LOST_TIMEOUT_MS        (700U)

/* Error is measured in tenths of sensor positions: -110 to +110. */
#define LINE_FOLLOW_ERROR_LIMIT             (110)
#define LINE_FOLLOW_EDGE_TURN_ERROR         (110)

/* Integer PID coefficients use a common divisor of 100. */
#define LINE_PID_KP                         (15)//3
#define LINE_PID_KI                         (0)
#define LINE_PID_KD                         (20)//20
#define LINE_PID_DIVISOR                    (100)
#define LINE_PID_INTEGRAL_LIMIT             (100)
#define LINE_PID_DERIVATIVE_DIVISOR        (2)

/* LINE0 is leftmost and LINE11 is rightmost. */
static const int8_t gSensorWeights[LINE_SENSOR_COUNT] = {
    -11, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 11
};

static bool gEnabled;
static uint32_t gLastProcessTime;
static uint32_t gLineLostStartedAt;
static int16_t gLastErrorTenths;
static int16_t gPreviousErrorTenths;
static int16_t gIntegralError;
static int16_t gFilteredDerivative;
static int16_t gLastTurnCommand;

static int16_t clampTurn(int32_t value)
{
    if (value > LINE_FOLLOW_MAX_TURN) {
        return LINE_FOLLOW_MAX_TURN;
    }
    if (value < -LINE_FOLLOW_MAX_TURN) {
        return (int16_t) -LINE_FOLLOW_MAX_TURN;
    }
    return (int16_t) value;
}

static uint8_t countBits(uint16_t value)
{
    uint8_t count = 0U;

    while (value != 0U) {
        count = (uint8_t) (count + (uint8_t) (value & 1U));
        value >>= 1U;
    }
    return count;
}

static int16_t absoluteError(int16_t error)
{
    return (error < 0) ? (int16_t) -error : error;
}

static void resetPid(void)
{
    gLastErrorTenths = 0;
    gPreviousErrorTenths = 0;
    gIntegralError = 0;
    gFilteredDerivative = 0;
    gLastTurnCommand = 0;
}

static int16_t calculateError(uint16_t sensors, uint8_t activeCount)
{
    int32_t weightedSum = 0;
    uint8_t leftCount = countBits((uint16_t) (sensors & 0x003FU));
    uint8_t rightCount = countBits((uint16_t) ((sensors >> 6U) & 0x003FU));
    uint8_t i;

    /* A square-corner edge is more useful than the center of a wide line. */
    if ((activeCount >= LINE_FOLLOW_INTERSECTION_COUNT) &&
        (leftCount > (uint8_t) (rightCount + 1U))) {
        return (int16_t) -LINE_FOLLOW_EDGE_TURN_ERROR;
    }
    if ((activeCount >= LINE_FOLLOW_INTERSECTION_COUNT) &&
        (rightCount > (uint8_t) (leftCount + 1U))) {
        return LINE_FOLLOW_EDGE_TURN_ERROR;
    }

    for (i = 0U; i < LINE_SENSOR_COUNT; i++) {
        if ((sensors & (uint16_t) (1U << i)) != 0U) {
            weightedSum += gSensorWeights[i];
        }
    }
    if (activeCount == 0U) {
        return gLastErrorTenths;
    }
    weightedSum = (weightedSum * 10) / activeCount;
    if (weightedSum > LINE_FOLLOW_ERROR_LIMIT) {
        weightedSum = LINE_FOLLOW_ERROR_LIMIT;
    } else if (weightedSum < -LINE_FOLLOW_ERROR_LIMIT) {
        weightedSum = -LINE_FOLLOW_ERROR_LIMIT;
    }
    return (int16_t) weightedSum;
}

static int16_t calculatePidTurn(int16_t errorTenths)
{
    int16_t derivative;
    int32_t output;

    if (absoluteError(errorTenths) < 80) {
        gIntegralError = (int16_t) (gIntegralError + errorTenths);
        if (gIntegralError > LINE_PID_INTEGRAL_LIMIT) {
            gIntegralError = LINE_PID_INTEGRAL_LIMIT;
        } else if (gIntegralError < -LINE_PID_INTEGRAL_LIMIT) {
            gIntegralError = -LINE_PID_INTEGRAL_LIMIT;
        }
    } else {
        /* Do not carry a large integral through a sharp line displacement. */
        gIntegralError = (int16_t) (gIntegralError / 2);
    }

    derivative = (int16_t) (errorTenths - gPreviousErrorTenths);
    gFilteredDerivative = (int16_t) (
        gFilteredDerivative +
        (derivative - gFilteredDerivative) /
            LINE_PID_DERIVATIVE_DIVISOR);
    gPreviousErrorTenths = errorTenths;

    output = ((int32_t) LINE_PID_KP * errorTenths) +
        ((int32_t) LINE_PID_KI * gIntegralError) +
        ((int32_t) LINE_PID_KD * gFilteredDerivative);

    /* Positive sensor error means the line is on the right. */
    output = -output / LINE_PID_DIVISOR;
    gLastErrorTenths = errorTenths;
    gLastTurnCommand = clampTurn(output);
    return gLastTurnCommand;
}

static int16_t calculateLostTurn(void)
{
    if (gLastTurnCommand != 0) {
        return (gLastTurnCommand > 0) ?
            LINE_FOLLOW_LOST_TURN : (int16_t) -LINE_FOLLOW_LOST_TURN;
    }
    return (gLastErrorTenths >= 0) ?
        (int16_t) -LINE_FOLLOW_LOST_TURN : LINE_FOLLOW_LOST_TURN;
}

static int16_t calculateForwardSpeed(int16_t errorTenths)
{
    int16_t magnitude = absoluteError(errorTenths);
    int32_t speed;

    speed = LINE_FOLLOW_BASE_SPEED -
        ((int32_t) (LINE_FOLLOW_BASE_SPEED - LINE_FOLLOW_MIN_SPEED) *
         magnitude) / LINE_FOLLOW_ERROR_LIMIT;
    if (speed < LINE_FOLLOW_MIN_SPEED) {
        speed = LINE_FOLLOW_MIN_SPEED;
    }
    return (int16_t) speed;
}

void LineFollower_init(void)
{
    gEnabled = false;
    gLastProcessTime = 0U;
    gLineLostStartedAt = 0U;
    resetPid();
}

void LineFollower_process(uint32_t nowMs)
{
    LineFollower_Observation observation;
    int16_t forwardSpeed;

    if (!gEnabled ||
        ((nowMs - gLastProcessTime) < LINE_FOLLOW_PERIOD_MS)) {
        return;
    }
    gLastProcessTime = nowMs;
    LineFollower_getObservation(&observation);

    if (observation.state == LINE_FOLLOW_LOST) {
        if (gLineLostStartedAt == 0U) {
            gLineLostStartedAt = nowMs;
        }
        if ((nowMs - gLineLostStartedAt) >
            LINE_FOLLOW_LOST_TIMEOUT_MS) {
            ChassisMotor_stop(nowMs);
        } else {
            ChassisMotor_setDifferential(
                LINE_FOLLOW_LOST_SPEED, calculateLostTurn(), nowMs);
        }
        return;
    }

    gLineLostStartedAt = 0U;
    forwardSpeed = calculateForwardSpeed(observation.errorTenths);
    ChassisMotor_setDifferential(
        forwardSpeed, observation.turnCommand, nowMs);
}

void LineFollower_setEnabled(bool enabled, uint32_t nowMs)
{
    gEnabled = enabled;
    gLastProcessTime = enabled ?
        nowMs - LINE_FOLLOW_PERIOD_MS : nowMs;
    gLineLostStartedAt = 0U;
    resetPid();
    if (!enabled) {
        ChassisMotor_stop(nowMs);
    }
}

bool LineFollower_isEnabled(void)
{
    return gEnabled;
}

void LineFollower_getObservation(LineFollower_Observation *observation)
{
    uint16_t sensors;
    uint8_t activeCount;
    int16_t error;

    if (observation == NULL) {
        return;
    }
    sensors = LineSensor_readRaw();
    activeCount = countBits(sensors);
    observation->rawSensors = sensors;
    observation->activeCount = activeCount;

    if (activeCount == 0U) {
        observation->errorTenths = gLastErrorTenths;
        observation->turnCommand = 0;
        observation->state = LINE_FOLLOW_LOST;
        return;
    }

    error = calculateError(sensors, activeCount);
    observation->errorTenths = error;
    observation->turnCommand = calculatePidTurn(error);
    observation->state = (activeCount >= LINE_FOLLOW_INTERSECTION_COUNT) ?
        LINE_FOLLOW_INTERSECTION : LINE_FOLLOW_TRACK;
}
