#include "line_follower.h"

#include <stddef.h>

#include "chassis_motor.h"
#include "imu601.h"
#include "line_sensor.h"

#define LINE_FOLLOW_PERIOD_MS          (20U)
#define LINE_FOLLOW_CORNER_PERIOD_MS   (10U)
#define LINE_FOLLOW_BASE_SPEED         (10)
#define LINE_FOLLOW_MIN_FORWARD_SPEED  (6)
#define LINE_FOLLOW_INTERSECTION_SPEED (8)
#define LINE_FOLLOW_MAX_TURN           (5)
#define LINE_FOLLOW_STEERING_DEADBAND  (25)
#define LINE_FOLLOW_ERROR_FILTER_DIVISOR (3)
#define LINE_FOLLOW_TURN_ATTACK_PER_CYCLE (1)
#define LINE_FOLLOW_TURN_RELEASE_PER_CYCLE (1)
#define LINE_FOLLOW_INTERSECTION_COUNT (6U)
#define LINE_FOLLOW_STEERING_INVERTED  (0)
#define LINE_FOLLOW_SQUARE_CCW         (1)
#define LINE_FOLLOW_CORNER_ADVANCE_SPEED (8)
#define LINE_FOLLOW_CORNER_ADVANCE_TICKS (400U)
#define LINE_FOLLOW_CORNER_ADVANCE_TIMEOUT_MS (1000U)
#define LINE_FOLLOW_CORNER_TURN_SPEED  (6)
#define LINE_FOLLOW_CORNER_ALIGN_SPEED (3)
#define LINE_FOLLOW_CORNER_ALIGN_FORWARD_SPEED (5)
#define LINE_FOLLOW_CORNER_TARGET_YAW_CD (9000U)
#define LINE_FOLLOW_CORNER_YAW_TOLERANCE_CD (200U)
#define LINE_FOLLOW_CORNER_SLOWDOWN_YAW_CD (7500U)
#define LINE_FOLLOW_CORNER_HEADING_DEADBAND_CD (300U)
#define LINE_FOLLOW_CORNER_HEADING_CORRECTION (1)
#define LINE_FOLLOW_CORNER_FALLBACK_TURN_MS (450U)
#define LINE_FOLLOW_CORNER_CENTER_CONFIRM_CYCLES (2U)
#define LINE_FOLLOW_CORNER_TURN_TIMEOUT_MS (1200U)
#define LINE_FOLLOW_CORNER_ALIGN_TIMEOUT_MS (700U)
#define LINE_FOLLOW_LOST_FORWARD_SPEED (3)
#define LINE_FOLLOW_LOST_TURN_SPEED    (7)
#define LINE_FOLLOW_LOST_TIMEOUT_MS    (700U)
#define LINE_FOLLOW_CENTER_MASK        ((uint16_t) ((1U << 5U) | (1U << 6U)))

/* LINE0 is assumed to be the leftmost sensor and LINE11 the rightmost. */
static const int8_t gSensorWeights[LINE_SENSOR_COUNT] = {
    -9, -7, -5, -3, -1, -0, 0, 1, 3, 5, 7, 9
};

typedef enum {
    CORNER_IDLE = 0,
    CORNER_ADVANCING,
    CORNER_TURNING,
    CORNER_ALIGNING
} CornerState;

static bool gEnabled;
static uint32_t gLastProcessTime;
static CornerState gCornerState;
static bool gCornerArmed;
static uint32_t gCornerStageStartedAt;
static int32_t gCornerStartLeftEncoder;
static int32_t gCornerStartRightEncoder;
static bool gCornerEncoderValid;
static uint16_t gCornerStartYaw;
static bool gCornerYawValid;
static uint8_t gCornerCenteredCycles;
static uint32_t gLineLostStartedAt;
static int8_t gLastTurnDirection;
static int16_t gFilteredErrorTenths;
static bool gErrorFilterInitialized;
static int16_t gAppliedTurn;

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

static uint32_t encoderDelta(int32_t current, int32_t previous)
{
    int16_t delta = (int16_t) ((uint16_t) current - (uint16_t) previous);
    return (delta < 0) ? (uint32_t) (-(int32_t) delta) :
                         (uint32_t) delta;
}

static int32_t yawDelta(uint16_t current, uint16_t previous)
{
    int32_t delta = (int32_t) current - (int32_t) previous;
    if (delta > 18000) {
        delta -= 36000;
    } else if (delta < -18000) {
        delta += 36000;
    }
    return delta;
}

static uint32_t absoluteInt32(int32_t value)
{
    return (value < 0) ? (uint32_t) (-(int64_t) value) :
                         (uint32_t) value;
}

static int16_t lookupTrackTurn(int16_t errorTenths)
{
    uint32_t magnitude = absoluteInt32(errorTenths);
    int16_t turnMagnitude;

    if (magnitude <= LINE_FOLLOW_STEERING_DEADBAND) {
        return 0;
    }
    if (magnitude <= 25U) {
        turnMagnitude = 1;
    } else if (magnitude <= 45U) {
        turnMagnitude = 2;
    } else if (magnitude <= 65U) {
        turnMagnitude = 3;
    } else if (magnitude <= 85U) {
        turnMagnitude = 4;
    } else {
        turnMagnitude = LINE_FOLLOW_MAX_TURN;
    }

    /* A line on the right requires a right turn, which is negative here. */
    return (errorTenths > 0) ? (int16_t) -turnMagnitude : turnMagnitude;
}

static int16_t calculateForwardSpeed(int16_t errorTenths)
{
    uint32_t magnitude = absoluteInt32(errorTenths);
    uint32_t speedRange = (uint32_t) (LINE_FOLLOW_BASE_SPEED -
        LINE_FOLLOW_MIN_FORWARD_SPEED);
    uint32_t reduction;

    if (magnitude > 110U) {
        magnitude = 110U;
    }
    reduction = (magnitude * speedRange) / 110U;
    return (int16_t) ((uint32_t) LINE_FOLLOW_BASE_SPEED - reduction);
}

static void resetSteeringState(void)
{
    gAppliedTurn = 0;
    gFilteredErrorTenths = 0;
    gErrorFilterInitialized = false;
}

static void beginCornerAdvance(uint32_t nowMs)
{
    ChassisMotor_Status chassis;

    ChassisMotor_getStatus(&chassis);
    gCornerState = CORNER_ADVANCING;
    gCornerArmed = false;
    gCornerStageStartedAt = nowMs;
    gCornerStartLeftEncoder = chassis.leftEncoderCount;
    gCornerStartRightEncoder = chassis.rightEncoderCount;
    gCornerEncoderValid = chassis.feedbackFrameCount != 0U;
    gCornerCenteredCycles = 0U;
    resetSteeringState();
}

static bool cornerAdvanceComplete(uint32_t nowMs)
{
    ChassisMotor_Status chassis;

    ChassisMotor_getStatus(&chassis);
    if (gCornerEncoderValid && (chassis.feedbackFrameCount != 0U)) {
        uint32_t left = encoderDelta(
            chassis.leftEncoderCount, gCornerStartLeftEncoder);
        uint32_t right = encoderDelta(
            chassis.rightEncoderCount, gCornerStartRightEncoder);
        if (((left + right) / 2U) >=
            LINE_FOLLOW_CORNER_ADVANCE_TICKS) {
            return true;
        }
    }
    return (nowMs - gCornerStageStartedAt) >=
        LINE_FOLLOW_CORNER_ADVANCE_TIMEOUT_MS;
}

static void beginCornerTurn(uint32_t nowMs)
{
    IMU601_Attitude attitude;

    gCornerState = CORNER_TURNING;
    gCornerStageStartedAt = nowMs;
    gCornerYawValid = IMU601_getAttitude(&attitude);
    gCornerStartYaw = attitude.yawCentiDegrees;
    gCornerCenteredCycles = 0U;
    resetSteeringState();
}

static bool getCornerYawTravel(uint32_t *turned)
{
    IMU601_Attitude attitude;

    if ((turned != NULL) && gCornerYawValid &&
        IMU601_getAttitude(&attitude)) {
        *turned = absoluteInt32(
            yawDelta(attitude.yawCentiDegrees, gCornerStartYaw));
        return true;
    }
    return false;
}

static bool processCornerTurn(uint32_t nowMs)
{
    uint32_t elapsed = nowMs - gCornerStageStartedAt;
    uint32_t turned = 0U;
    bool yawAvailable = getCornerYawTravel(&turned);
    bool turnComplete = yawAvailable ?
        (turned >= (LINE_FOLLOW_CORNER_TARGET_YAW_CD -
            LINE_FOLLOW_CORNER_YAW_TOLERANCE_CD)) :
        (elapsed >= LINE_FOLLOW_CORNER_FALLBACK_TURN_MS);

    if (turnComplete) {
        gCornerState = CORNER_ALIGNING;
        gCornerStageStartedAt = nowMs;
        gCornerCenteredCycles = 0U;
        ChassisMotor_setDifferential(0, 0, nowMs);
        return true;
    }

    if (elapsed >= LINE_FOLLOW_CORNER_TURN_TIMEOUT_MS) {
        gCornerState = CORNER_IDLE;
        gCornerArmed = false;
        gCornerCenteredCycles = 0U;
        resetSteeringState();
        ChassisMotor_stop(nowMs);
        return true;
    }

    ChassisMotor_setDifferential(0,
        (yawAvailable &&
         (turned >= LINE_FOLLOW_CORNER_SLOWDOWN_YAW_CD)) ?
            LINE_FOLLOW_CORNER_ALIGN_SPEED :
            LINE_FOLLOW_CORNER_TURN_SPEED,
        nowMs);
    return true;
}

static bool processCornerAlignment(
    const LineFollower_Observation *observation, uint32_t nowMs)
{
    uint32_t elapsed = nowMs - gCornerStageStartedAt;
    uint32_t turned = 0U;
    int16_t headingCorrection = 0;
    bool centered =
        (observation->rawSensors & LINE_FOLLOW_CENTER_MASK) != 0U;

    if (getCornerYawTravel(&turned)) {
        if ((turned + LINE_FOLLOW_CORNER_HEADING_DEADBAND_CD) <
            LINE_FOLLOW_CORNER_TARGET_YAW_CD) {
            headingCorrection = LINE_FOLLOW_CORNER_HEADING_CORRECTION;
        } else if (turned > (LINE_FOLLOW_CORNER_TARGET_YAW_CD +
                   LINE_FOLLOW_CORNER_HEADING_DEADBAND_CD)) {
            headingCorrection = -LINE_FOLLOW_CORNER_HEADING_CORRECTION;
        }
    }

    if (centered) {
        if (gCornerCenteredCycles <
            LINE_FOLLOW_CORNER_CENTER_CONFIRM_CYCLES) {
            gCornerCenteredCycles++;
        }
    } else {
        gCornerCenteredCycles = 0U;
    }

    if (gCornerCenteredCycles >=
        LINE_FOLLOW_CORNER_CENTER_CONFIRM_CYCLES) {
        gCornerState = CORNER_IDLE;
        gCornerArmed = false;
        gCornerCenteredCycles = 0U;
        resetSteeringState();
        ChassisMotor_setDifferential(
            LINE_FOLLOW_CORNER_ALIGN_FORWARD_SPEED, 0, nowMs);
        return true;
    }

    if (elapsed >= LINE_FOLLOW_CORNER_ALIGN_TIMEOUT_MS) {
        gCornerState = CORNER_IDLE;
        gCornerArmed = false;
        gCornerCenteredCycles = 0U;
        resetSteeringState();
        ChassisMotor_stop(nowMs);
        return true;
    }

    ChassisMotor_setDifferential(
        LINE_FOLLOW_CORNER_ALIGN_FORWARD_SPEED, headingCorrection, nowMs);
    return true;
}

void LineFollower_init(void)
{
    gEnabled = false;
    gLastProcessTime = 0U;
    gCornerState = CORNER_IDLE;
    gCornerArmed = true;
    gCornerStageStartedAt = 0U;
    gCornerStartLeftEncoder = 0;
    gCornerStartRightEncoder = 0;
    gCornerEncoderValid = false;
    gCornerStartYaw = 0U;
    gCornerYawValid = false;
    gCornerCenteredCycles = 0U;
    gLineLostStartedAt = 0U;
    gLastTurnDirection = 1;
    resetSteeringState();
}

static int16_t slewTurn(int16_t requestedTurn)
{
    int32_t delta = (int32_t) requestedTurn - (int32_t) gAppliedTurn;
    int32_t limit = LINE_FOLLOW_TURN_ATTACK_PER_CYCLE;

    if ((requestedTurn == 0) ||
        ((requestedTurn > 0) && (gAppliedTurn < 0)) ||
        ((requestedTurn < 0) && (gAppliedTurn > 0))) {
        gAppliedTurn = 0;
        return 0;
    }
    if (((gAppliedTurn > 0) && (requestedTurn < gAppliedTurn)) ||
        ((gAppliedTurn < 0) && (requestedTurn > gAppliedTurn))) {
        limit = LINE_FOLLOW_TURN_RELEASE_PER_CYCLE;
    }
    if (delta > limit) {
        delta = limit;
    } else if (delta < -limit) {
        delta = -limit;
    }
    gAppliedTurn = (int16_t) ((int32_t) gAppliedTurn + delta);
    return gAppliedTurn;
}

void LineFollower_process(uint32_t nowMs)
{
    LineFollower_Observation observation;
    int16_t forwardSpeed = LINE_FOLLOW_BASE_SPEED;
    uint32_t period = ((gCornerState == CORNER_TURNING) ||
        (gCornerState == CORNER_ALIGNING)) ?
            LINE_FOLLOW_CORNER_PERIOD_MS : LINE_FOLLOW_PERIOD_MS;

    if (!gEnabled ||
        ((nowMs - gLastProcessTime) < period)) {
        return;
    }
    gLastProcessTime = nowMs;
    LineFollower_getObservation(&observation);

    if (observation.turnCommand > 0) {
        gLastTurnDirection = 1;
    } else if (observation.turnCommand < 0) {
        gLastTurnDirection = -1;
    }

#if LINE_FOLLOW_SQUARE_CCW
    if (gCornerState == CORNER_ADVANCING) {
        if (cornerAdvanceComplete(nowMs)) {
            beginCornerTurn(nowMs);
            ChassisMotor_setDifferential(0,
                LINE_FOLLOW_CORNER_TURN_SPEED, nowMs);
        } else {
            ChassisMotor_setDifferential(
                LINE_FOLLOW_CORNER_ADVANCE_SPEED, 0, nowMs);
        }
        return;
    }

    if (gCornerState == CORNER_TURNING) {
        (void) processCornerTurn(nowMs);
        return;
    }

    if (gCornerState == CORNER_ALIGNING) {
        (void) processCornerAlignment(&observation, nowMs);
        return;
    }
#endif

    if (observation.state == LINE_FOLLOW_LOST) {
        resetSteeringState();
        if (gLineLostStartedAt == 0U) {
            gLineLostStartedAt = nowMs;
        }
        if ((nowMs - gLineLostStartedAt) > LINE_FOLLOW_LOST_TIMEOUT_MS) {
            gAppliedTurn = 0;
            ChassisMotor_stop(nowMs);
        } else {
            gAppliedTurn = 0;
            ChassisMotor_setDifferential(LINE_FOLLOW_LOST_FORWARD_SPEED,
                (int16_t) (gLastTurnDirection *
                    LINE_FOLLOW_LOST_TURN_SPEED), nowMs);
        }
        return;
    }

    gLineLostStartedAt = 0U;

#if LINE_FOLLOW_SQUARE_CCW
    if ((observation.activeCount < LINE_FOLLOW_INTERSECTION_COUNT) &&
        (gCornerState == CORNER_IDLE)) {
        gCornerArmed = true;
    }
    if ((observation.state == LINE_FOLLOW_INTERSECTION) &&
        gCornerArmed) {
        beginCornerAdvance(nowMs);
        ChassisMotor_setDifferential(
            LINE_FOLLOW_CORNER_ADVANCE_SPEED, 0, nowMs);
        return;
    }
#endif

    forwardSpeed = calculateForwardSpeed(observation.errorTenths);
    if (observation.state == LINE_FOLLOW_INTERSECTION) {
        forwardSpeed = LINE_FOLLOW_INTERSECTION_SPEED;
    }
    ChassisMotor_setDifferential(
        forwardSpeed, slewTurn(observation.turnCommand), nowMs);
}

void LineFollower_setEnabled(bool enabled, uint32_t nowMs)
{
    gEnabled = enabled;
    if (enabled) {
        gLastProcessTime = nowMs - LINE_FOLLOW_PERIOD_MS;
        gCornerState = CORNER_IDLE;
        gCornerArmed = true;
        gCornerStageStartedAt = 0U;
        gCornerEncoderValid = false;
        gCornerYawValid = false;
        gCornerCenteredCycles = 0U;
        gLineLostStartedAt = 0U;
        gLastTurnDirection = 1;
        resetSteeringState();
    } else {
        gCornerState = CORNER_IDLE;
        gCornerArmed = true;
        gCornerEncoderValid = false;
        gCornerYawValid = false;
        gCornerCenteredCycles = 0U;
        gLineLostStartedAt = 0U;
        resetSteeringState();
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
    int32_t weightedSum = 0;
    uint8_t activeCount = 0U;
    uint8_t i;
    int16_t turn;

    if (observation == NULL) {
        return;
    }
    sensors = LineSensor_readRaw();
    for (i = 0U; i < LINE_SENSOR_COUNT; i++) {
        if ((sensors & (uint16_t) (1U << i)) != 0U) {
            weightedSum += gSensorWeights[i];
            activeCount++;
        }
    }

    observation->rawSensors = sensors;
    observation->activeCount = activeCount;
    if (activeCount == 0U) {
        observation->errorTenths = 0;
        observation->turnCommand = 0;
        observation->state = LINE_FOLLOW_LOST;
        return;
    }

    {
        int16_t rawError = (int16_t) ((weightedSum * 10) / activeCount);
        if (!gErrorFilterInitialized) {
            gFilteredErrorTenths = rawError;
            gErrorFilterInitialized = true;
        } else {
            gFilteredErrorTenths = (int16_t) (
                gFilteredErrorTenths +
                (rawError - gFilteredErrorTenths) /
                    LINE_FOLLOW_ERROR_FILTER_DIVISOR);
        }
        observation->errorTenths = gFilteredErrorTenths;
    }
    turn = lookupTrackTurn(observation->errorTenths);
#if LINE_FOLLOW_STEERING_INVERTED
    turn = (int16_t) -turn;
#endif
    observation->turnCommand = clampTurn(turn);
    observation->state = (activeCount >= LINE_FOLLOW_INTERSECTION_COUNT) ?
        LINE_FOLLOW_INTERSECTION : LINE_FOLLOW_TRACK;
}
