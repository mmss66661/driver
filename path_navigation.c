#include "path_navigation.h"

#include <stddef.h>
#include <string.h>

#include "chassis_motor.h"
#include "imu601.h"
#include "line_follower.h"
#include "ti_msp_dl_config.h"

#define PATH_FLASH_ADDRESS                 (0x0001E000UL)
#define PATH_FLASH_SIZE                    (0x00002000UL)
#define PATH_FLASH_SECTOR_COUNT            (8U)
#define PATH_MAGIC                         (0x3141564EU)
#define PATH_VERSION                       (1U)

#define PATH_RECORD_DISTANCE_STEP_TICKS    (100U)
#define PATH_MIN_DISTANCE_TICKS            (100U)
#define PATH_RECORD_HEADING_STEP_CD        (100)
#define PATH_RECORD_MAX_INTERVAL_MS        (500U)
#define PATH_SENSOR_TIMEOUT_MS             (1000U)
#define PATH_CONTROL_PERIOD_MS             (20U)
#define PATH_REPLAY_FORWARD_SPEED          (10)
#define PATH_REPLAY_SLOW_SPEED             (6)
#define PATH_REPLAY_CURVE_SPEED            (7)
#define PATH_REPLAY_INTERSECTION_SPEED     (6)
#define PATH_REPLAY_LINE_LOST_SPEED        (3)
#define PATH_REPLAY_HEADING_LOOKAHEAD_TICKS (60U)
#define PATH_REPLAY_SPEED_LOOKAHEAD_TICKS  (180U)
#define PATH_REPLAY_CURVE_ANGLE_CD          (1000)
#define PATH_REPLAY_SLOW_ANGLE_CD           (1800)
#define PATH_REPLAY_TURN_ONLY_ANGLE_CD      (3500)
#define PATH_REPLAY_CURVE_ERROR_TENTHS     (60)
#define PATH_LINE_LOST_TIMEOUT_MS          (800U)
#define PATH_HEADING_GAIN_DIVISOR          (50)
#define PATH_MAX_TURN_SPEED                (6)
#define PATH_TURN_ONLY_MAX_SPEED           (8)
#define PATH_TURN_SLEW_PER_CYCLE           (1)
#define PATH_FUSION_WEIGHT_TOTAL           (4)
#define PATH_NORMAL_LINE_WEIGHT            (3)
#define PATH_INTERSECTION_LINE_WEIGHT      (1)
#define PATH_STEERING_INVERTED             (0)

typedef struct {
    uint32_t distanceTicks;
    int32_t headingCentiDegrees;
} PathPoint;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t pointCount;
    uint32_t totalDistanceTicks;
    uint32_t pointsCrc;
} PathHeader;

#define PATH_MAX_POINTS \
    ((PATH_FLASH_SIZE - sizeof(PathHeader)) / sizeof(PathPoint))

_Static_assert(sizeof(PathPoint) == 8U, "PathPoint must be one flash word");
_Static_assert(sizeof(PathHeader) == 16U, "PathHeader layout changed");

static PathPoint gPoints[PATH_MAX_POINTS];
__attribute__((used, section(".path_flash_reserve")))
static const uint8_t gPathFlashReserve[PATH_FLASH_SIZE];
static PathNavigation_Status gStatus;
static uint32_t gLastFeedbackFrame;
static uint32_t gLastImuFrame;
static uint32_t gLastFeedbackTime;
static uint32_t gLastImuTime;
static uint32_t gLastControlTime;
static uint32_t gLastRecordTime;
static uint32_t gLastLineSeenTime;
static uint32_t gLastPointDistance;
static int32_t gLastPointHeading;
static int32_t gPreviousLeftEncoder;
static int32_t gPreviousRightEncoder;
static uint16_t gPreviousYaw;
static bool gYawInitialized;
static int16_t gAppliedTurnCommand;

static uint32_t crc32(const void *data, uint32_t length)
{
    const uint8_t *bytes = (const uint8_t *) data;
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;

    while (length-- != 0U) {
        crc ^= *bytes++;
        for (i = 0U; i < 8U; i++) {
            crc = ((crc & 1U) != 0U) ?
                (crc >> 1U) ^ 0xEDB88320UL : crc >> 1U;
        }
    }
    return ~crc;
}

static uint32_t absoluteDelta(int32_t current, int32_t previous)
{
    int16_t delta = (int16_t) ((uint16_t) current - (uint16_t) previous);
    return (delta < 0) ? (uint32_t) (-(int32_t) delta) : (uint32_t) delta;
}

static int32_t absoluteInt32(int32_t value)
{
    return (value < 0) ? (int32_t) (-(int64_t) value) : value;
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

static int16_t clampTurn(int32_t turn)
{
    if (turn > PATH_MAX_TURN_SPEED) {
        return PATH_MAX_TURN_SPEED;
    }
    if (turn < -PATH_MAX_TURN_SPEED) {
        return -PATH_MAX_TURN_SPEED;
    }
    return (int16_t) turn;
}

static int16_t clampTurnOnly(int32_t turn)
{
    if (turn > PATH_TURN_ONLY_MAX_SPEED) {
        return PATH_TURN_ONLY_MAX_SPEED;
    }
    if (turn < -PATH_TURN_ONLY_MAX_SPEED) {
        return -PATH_TURN_ONLY_MAX_SPEED;
    }
    return (int16_t) turn;
}

static int16_t fuseTurns(
    int16_t lineTurn, int16_t navigationTurn, int32_t lineWeight)
{
    int32_t fused = (int32_t) navigationTurn +
        ((int32_t) lineTurn * lineWeight) / PATH_FUSION_WEIGHT_TOTAL;
    return clampTurn(fused);
}

static int16_t slewTurn(int16_t requested)
{
    int32_t delta = (int32_t) requested - gAppliedTurnCommand;

    if (delta > PATH_TURN_SLEW_PER_CYCLE) {
        delta = PATH_TURN_SLEW_PER_CYCLE;
    } else if (delta < -PATH_TURN_SLEW_PER_CYCLE) {
        delta = -PATH_TURN_SLEW_PER_CYCLE;
    }
    gAppliedTurnCommand = (int16_t) (gAppliedTurnCommand + delta);
    return gAppliedTurnCommand;
}

static uint32_t addTicksSaturated(uint32_t distance, uint32_t increment)
{
    return ((UINT32_MAX - distance) < increment) ?
        UINT32_MAX : distance + increment;
}

static int32_t pathHeadingAtDistance(uint32_t distanceTicks)
{
    uint16_t upper = 1U;
    uint16_t lower;
    uint32_t span;
    uint32_t offset;
    int32_t headingSpan;

    if ((gStatus.pointCount == 0U) ||
        (distanceTicks <= gPoints[0].distanceTicks)) {
        return (gStatus.pointCount == 0U) ? 0 :
            gPoints[0].headingCentiDegrees;
    }
    while ((upper < gStatus.pointCount) &&
           (gPoints[upper].distanceTicks < distanceTicks)) {
        upper++;
    }
    if (upper >= gStatus.pointCount) {
        return gPoints[gStatus.pointCount - 1U].headingCentiDegrees;
    }

    lower = upper - 1U;
    while ((lower > 0U) &&
           (gPoints[lower].distanceTicks ==
            gPoints[upper].distanceTicks)) {
        lower--;
    }
    span = gPoints[upper].distanceTicks -
        gPoints[lower].distanceTicks;
    if (span == 0U) {
        return gPoints[upper].headingCentiDegrees;
    }
    offset = distanceTicks - gPoints[lower].distanceTicks;
    headingSpan = gPoints[upper].headingCentiDegrees -
        gPoints[lower].headingCentiDegrees;
    return gPoints[lower].headingCentiDegrees + (int32_t)
        (((int64_t) headingSpan * offset) / span);
}

static void updateLineStatus(
    const LineFollower_Observation *line, uint32_t nowMs)
{
    gStatus.lineActiveCount = line->activeCount;
    gStatus.lineErrorTenths = line->errorTenths;
    gStatus.lineTurnCommand = line->turnCommand;
    gStatus.lineDetected = line->state != LINE_FOLLOW_LOST;
    gStatus.intersectionDetected =
        line->state == LINE_FOLLOW_INTERSECTION;
    if (gStatus.lineDetected) {
        gLastLineSeenTime = nowMs;
        gStatus.lineLostMilliseconds = 0U;
    } else {
        gStatus.lineLostMilliseconds = nowMs - gLastLineSeenTime;
    }
}

static bool loadPath(void)
{
    const PathHeader *header =
        (const PathHeader *) (uintptr_t) PATH_FLASH_ADDRESS;
    const PathPoint *points = (const PathPoint *)
        (uintptr_t) (PATH_FLASH_ADDRESS + sizeof(PathHeader));
    uint32_t byteCount;

    if ((header->magic != PATH_MAGIC) ||
        (header->version != PATH_VERSION) ||
        (header->pointCount < 2U) ||
        (header->pointCount > PATH_MAX_POINTS)) {
        gStatus.pathValid = false;
        gStatus.pointCount = 0U;
        return false;
    }
    byteCount = (uint32_t) header->pointCount * sizeof(PathPoint);
    if (crc32(points, byteCount) != header->pointsCrc) {
        gStatus.pathValid = false;
        gStatus.pointCount = 0U;
        return false;
    }
    memcpy(gPoints, points, byteCount);
    gStatus.pathValid = true;
    gStatus.pointCount = header->pointCount;
    return true;
}

static bool programFlashWord(uint32_t address, const void *source)
{
    uint32_t word[2];
    DL_FLASHCTL_COMMAND_STATUS result;

    memcpy(word, source, sizeof(word));
    DL_FlashCTL_executeClearStatus(FLASHCTL);
    DL_FlashCTL_unprotectSector(
        FLASHCTL, address, DL_FLASHCTL_REGION_SELECT_MAIN);
    result = DL_FlashCTL_programMemoryFromRAM64WithECCGenerated(
        FLASHCTL, address, word);
    return result == DL_FLASHCTL_COMMAND_STATUS_PASSED;
}

static bool savePath(void)
{
    PathHeader header;
    uint32_t address;
    uint32_t i;
    uint32_t primask;
    bool success = true;

    header.magic = PATH_MAGIC;
    header.version = PATH_VERSION;
    header.pointCount = gStatus.pointCount;

    header.totalDistanceTicks =
        gPoints[gStatus.pointCount - 1U].distanceTicks;
    header.pointsCrc = crc32(gPoints,
        (uint32_t) gStatus.pointCount * sizeof(PathPoint));

    primask = __get_PRIMASK();
    __disable_irq();

    for (i = 0U; i < PATH_FLASH_SECTOR_COUNT; i++) {
        address = PATH_FLASH_ADDRESS + i * DL_FLASHCTL_SECTOR_SIZE;
        DL_FlashCTL_executeClearStatus(FLASHCTL);
        DL_FlashCTL_unprotectSector(
            FLASHCTL, address, DL_FLASHCTL_REGION_SELECT_MAIN);
        if (DL_FlashCTL_eraseMemoryFromRAM(FLASHCTL, address,
                DL_FLASHCTL_COMMAND_SIZE_SECTOR) !=
            DL_FLASHCTL_COMMAND_STATUS_PASSED) {
            success = false;
            break;
        }
    }

    address = PATH_FLASH_ADDRESS + sizeof(PathHeader);

    for (i = 0U; success && (i < gStatus.pointCount); i++) {
        success = programFlashWord(address, &gPoints[i]);
        address += sizeof(PathPoint);
    }
    if (success) {
        success = programFlashWord(PATH_FLASH_ADDRESS, &header) &&
                  programFlashWord(PATH_FLASH_ADDRESS + 8U,
                      (const uint8_t *) &header + 8U);
    }
    DL_FlashCTL_protectMainMemory(FLASHCTL);

    if (primask == 0U) {
        __enable_irq();
    }
    return success && loadPath();
}

static bool readSensors(uint32_t nowMs, bool initialize)
{
    ChassisMotor_Status chassis;
    IMU601_Attitude attitude;

    ChassisMotor_getStatus(&chassis);
    (void) IMU601_getAttitude(&attitude);
    if ((chassis.feedbackFrameCount == 0U) ||
        (attitude.validFrameCount == 0U)) {
        return false;
    }

    if (initialize) {
        gPreviousLeftEncoder = chassis.leftEncoderCount;
        gPreviousRightEncoder = chassis.rightEncoderCount;
        gPreviousYaw = attitude.yawCentiDegrees;
        gYawInitialized = true;
        gLastFeedbackFrame = chassis.feedbackFrameCount;
        gLastImuFrame = attitude.validFrameCount;
        gLastFeedbackTime = nowMs;
        gLastImuTime = nowMs;
        return true;
    }

    if (attitude.validFrameCount != gLastImuFrame) {
        gLastImuFrame = attitude.validFrameCount;
        gLastImuTime = nowMs;
        if (gYawInitialized) {
            gStatus.headingCentiDegrees +=
                yawDelta(attitude.yawCentiDegrees, gPreviousYaw);
        }
        gPreviousYaw = attitude.yawCentiDegrees;
        gYawInitialized = true;
    }

    if (chassis.feedbackFrameCount != gLastFeedbackFrame) {
        uint32_t leftTravel = absoluteDelta(
            chassis.leftEncoderCount, gPreviousLeftEncoder);
        uint32_t rightTravel = absoluteDelta(
            chassis.rightEncoderCount, gPreviousRightEncoder);
        uint32_t increment = (leftTravel + rightTravel) / 2U;

        gLastFeedbackFrame = chassis.feedbackFrameCount;
        gLastFeedbackTime = nowMs;
        gPreviousLeftEncoder = chassis.leftEncoderCount;
        gPreviousRightEncoder = chassis.rightEncoderCount;
        if ((UINT32_MAX - gStatus.distanceTicks) < increment) {
            gStatus.distanceTicks = UINT32_MAX;
        } else {
            gStatus.distanceTicks += increment;
        }
    }
    return ((nowMs - gLastFeedbackTime) <= PATH_SENSOR_TIMEOUT_MS) &&
           ((nowMs - gLastImuTime) <= PATH_SENSOR_TIMEOUT_MS);
}

static bool addPoint(uint32_t nowMs)
{
    PathPoint *point;

    if (gStatus.pointCount >= PATH_MAX_POINTS) {
        return false;
    }
    point = &gPoints[gStatus.pointCount++];
    point->distanceTicks = gStatus.distanceTicks;
    point->headingCentiDegrees = gStatus.headingCentiDegrees;
    gLastPointDistance = gStatus.distanceTicks;
    gLastPointHeading = gStatus.headingCentiDegrees;
    gLastRecordTime = nowMs;
    return true;
}

static void fail(PathNavigation_Error error, uint32_t nowMs)
{
    ChassisMotor_stop(nowMs);
    gAppliedTurnCommand = 0;
    gStatus.mode = PATH_NAV_ERROR;
    gStatus.error = error;
    gStatus.headingErrorCentiDegrees = 0;
}

void PathNavigation_init(void)
{
    memset(&gStatus, 0, sizeof(gStatus));
    gAppliedTurnCommand = 0;
    gStatus.mode = PATH_NAV_IDLE;
    gStatus.error = PATH_NAV_ERROR_NONE;
    gStatus.lineDetected = false;
    gStatus.intersectionDetected = false;
    (void) loadPath();
}

bool PathNavigation_startRecording(uint32_t nowMs)
{
    LineFollower_Observation line;

    gStatus.error = PATH_NAV_ERROR_NONE;
    if (ChassisMotor_isClosedLoopEnabled()) {
        fail(PATH_NAV_ERROR_DRIVE_ACTIVE, nowMs);
        return false;
    }
    if (!readSensors(nowMs, true)) {
        fail(PATH_NAV_ERROR_SENSORS, nowMs);
        return false;
    }
    gStatus.distanceTicks = 0U;
    gStatus.headingCentiDegrees = 0;
    gStatus.targetHeadingCentiDegrees = 0;
    gStatus.headingErrorCentiDegrees = 0;
    gStatus.lineActiveCount = 0U;
    gStatus.lineErrorTenths = 0;
    gStatus.lineTurnCommand = 0;
    gStatus.navigationTurnCommand = 0;
    gStatus.fusedTurnCommand = 0;
    gStatus.lineLostMilliseconds = 0U;
    gStatus.lineDetected = false;
    gStatus.intersectionDetected = false;
    LineFollower_getObservation(&line);
    gLastLineSeenTime = nowMs;
    updateLineStatus(&line, nowMs);
    gStatus.currentPoint = 0U;
    gStatus.pointCount = 0U;
    gStatus.pathValid = false;
    gLastControlTime = nowMs - PATH_CONTROL_PERIOD_MS;
    gAppliedTurnCommand = 0;
    ChassisMotor_stop(nowMs);
    gStatus.mode = PATH_NAV_RECORDING;
    return addPoint(nowMs);
}

bool PathNavigation_finishRecording(uint32_t nowMs)
{
    if (gStatus.mode != PATH_NAV_RECORDING) {
        return false;
    }
    ChassisMotor_stop(nowMs);
    if ((gStatus.pointCount == 0U) ||
        ((gPoints[gStatus.pointCount - 1U].distanceTicks !=
          gStatus.distanceTicks) ||
         (gPoints[gStatus.pointCount - 1U].headingCentiDegrees !=
          gStatus.headingCentiDegrees))) {
        if (!addPoint(nowMs)) {
            fail(PATH_NAV_ERROR_PATH_FULL, nowMs);
            return false;
        }
    }
    if ((gStatus.pointCount < 2U) ||
        (gStatus.distanceTicks < PATH_MIN_DISTANCE_TICKS)) {
        fail(PATH_NAV_ERROR_PATH_SHORT, nowMs);
        return false;
    }
    if (!savePath()) {
        fail(PATH_NAV_ERROR_FLASH, nowMs);
        return false;
    }
    gStatus.mode = PATH_NAV_COMPLETE;
    gStatus.error = PATH_NAV_ERROR_NONE;
    gStatus.currentPoint = gStatus.pointCount - 1U;
    return true;
}

bool PathNavigation_startReplay(uint32_t nowMs)
{
    LineFollower_Observation line;
    if (!gStatus.pathValid || (gStatus.pointCount < 2U)) {
        fail(PATH_NAV_ERROR_NO_PATH, nowMs);
        return false;
    }
    gStatus.distanceTicks = 0U;
    gStatus.headingCentiDegrees = 0;
    gStatus.targetHeadingCentiDegrees = gPoints[1].headingCentiDegrees;
    gStatus.headingErrorCentiDegrees = 0;
    gStatus.currentPoint = 1U;
    gStatus.error = PATH_NAV_ERROR_NONE;
    if (!readSensors(nowMs, true)) {
        fail(PATH_NAV_ERROR_SENSORS, nowMs);
        return false;
    }
    LineFollower_getObservation(&line);
    gLastLineSeenTime = nowMs;
    updateLineStatus(&line, nowMs);
    gLastControlTime = nowMs - PATH_CONTROL_PERIOD_MS;
    gAppliedTurnCommand = 0;
    gStatus.mode = PATH_NAV_REPLAYING;
    return true;
}

void PathNavigation_stop(uint32_t nowMs)
{
    bool wasRecording = gStatus.mode == PATH_NAV_RECORDING;
    ChassisMotor_stop(nowMs);
    gAppliedTurnCommand = 0;
    gStatus.mode = PATH_NAV_IDLE;
    gStatus.error = PATH_NAV_ERROR_NONE;
    gStatus.headingErrorCentiDegrees = 0;
    if (wasRecording) {
        (void) loadPath();
    }
}

void PathNavigation_process(uint32_t nowMs)
{
    LineFollower_Observation line;
    int32_t pathHeadingNow;
    int32_t speedPreviewHeading;
    int32_t upcomingHeadingChange;
    uint32_t targetDistance;
    uint32_t speedPreviewDistance;

    if ((gStatus.mode != PATH_NAV_RECORDING) &&
        (gStatus.mode != PATH_NAV_REPLAYING)) {
        return;
    }
    if (!readSensors(nowMs, false)) {
        fail(PATH_NAV_ERROR_SENSORS, nowMs);
        return;
    }

    if (gStatus.mode == PATH_NAV_RECORDING) {
        if ((nowMs - gLastControlTime) >= PATH_CONTROL_PERIOD_MS) {
            gLastControlTime = nowMs;
            LineFollower_getObservation(&line);
            updateLineStatus(&line, nowMs);
        }
        if (((gStatus.distanceTicks - gLastPointDistance) >=
             PATH_RECORD_DISTANCE_STEP_TICKS) ||
            (absoluteInt32(gStatus.headingCentiDegrees - gLastPointHeading) >=
             PATH_RECORD_HEADING_STEP_CD) ||
            ((nowMs - gLastRecordTime) >= PATH_RECORD_MAX_INTERVAL_MS)) {
            if (!addPoint(nowMs)) {
                fail(PATH_NAV_ERROR_PATH_FULL, nowMs);
            } else if (gStatus.pointCount >= PATH_MAX_POINTS) {
                (void) PathNavigation_finishRecording(nowMs);
            }
        }
        return;
    }

    if ((nowMs - gLastControlTime) < PATH_CONTROL_PERIOD_MS) {
        return;
    }
    gLastControlTime = nowMs;
    LineFollower_getObservation(&line);
    updateLineStatus(&line, nowMs);

    while ((gStatus.currentPoint < gStatus.pointCount) &&
           (gStatus.distanceTicks >=
            gPoints[gStatus.currentPoint].distanceTicks)) {
        gStatus.currentPoint++;
    }
    if (gStatus.currentPoint >= gStatus.pointCount) {
        ChassisMotor_stop(nowMs);
        gStatus.currentPoint = gStatus.pointCount - 1U;
        gStatus.mode = PATH_NAV_COMPLETE;
        gStatus.headingErrorCentiDegrees = 0;
        return;
    }

    targetDistance = addTicksSaturated(gStatus.distanceTicks,
        PATH_REPLAY_HEADING_LOOKAHEAD_TICKS);
    speedPreviewDistance = addTicksSaturated(gStatus.distanceTicks,
        PATH_REPLAY_SPEED_LOOKAHEAD_TICKS);
    pathHeadingNow = pathHeadingAtDistance(gStatus.distanceTicks);
    gStatus.targetHeadingCentiDegrees =
        pathHeadingAtDistance(targetDistance);
    speedPreviewHeading = pathHeadingAtDistance(speedPreviewDistance);
    upcomingHeadingChange = absoluteInt32(
        speedPreviewHeading - pathHeadingNow);
    gStatus.headingErrorCentiDegrees =
        gStatus.targetHeadingCentiDegrees - gStatus.headingCentiDegrees;
    gStatus.navigationTurnCommand = clampTurnOnly(
        gStatus.headingErrorCentiDegrees / PATH_HEADING_GAIN_DIVISOR);
#if PATH_STEERING_INVERTED
    gStatus.navigationTurnCommand =
        (int16_t) -gStatus.navigationTurnCommand;
#endif

    if (!gStatus.lineDetected &&
        (gStatus.lineLostMilliseconds > PATH_LINE_LOST_TIMEOUT_MS)) {
        fail(PATH_NAV_ERROR_LINE_LOST, nowMs);
        return;
    }

    {
        int32_t absoluteError = absoluteInt32(
            gStatus.headingErrorCentiDegrees);
        int16_t forward = PATH_REPLAY_FORWARD_SPEED;

        if (!gStatus.lineDetected) {
            forward = PATH_REPLAY_LINE_LOST_SPEED;
            gStatus.fusedTurnCommand = gStatus.navigationTurnCommand;
        } else if (gStatus.intersectionDetected) {
            forward = PATH_REPLAY_INTERSECTION_SPEED;
            gStatus.fusedTurnCommand = fuseTurns(
                gStatus.lineTurnCommand, gStatus.navigationTurnCommand,
                PATH_INTERSECTION_LINE_WEIGHT);
        } else {
            gStatus.fusedTurnCommand = fuseTurns(
                gStatus.lineTurnCommand, gStatus.navigationTurnCommand,
                PATH_NORMAL_LINE_WEIGHT);
            if ((upcomingHeadingChange >= PATH_REPLAY_CURVE_ANGLE_CD) ||
                (gStatus.lineErrorTenths >=
                 PATH_REPLAY_CURVE_ERROR_TENTHS) ||
                (gStatus.lineErrorTenths <=
                 -PATH_REPLAY_CURVE_ERROR_TENTHS)) {
                forward = PATH_REPLAY_CURVE_SPEED;
            }
        }

        if (absoluteError >= PATH_REPLAY_TURN_ONLY_ANGLE_CD) {
            forward = 0;
            gStatus.fusedTurnCommand = clampTurnOnly(
                gStatus.headingErrorCentiDegrees /
                    PATH_HEADING_GAIN_DIVISOR);
        } else if (((absoluteError >= PATH_REPLAY_SLOW_ANGLE_CD) ||
                    (upcomingHeadingChange >=
                     PATH_REPLAY_SLOW_ANGLE_CD)) &&
                   (forward > PATH_REPLAY_SLOW_SPEED)) {
            forward = PATH_REPLAY_SLOW_SPEED;
        }
        gStatus.fusedTurnCommand = slewTurn(
            gStatus.fusedTurnCommand);
        ChassisMotor_setDifferential(
            forward, gStatus.fusedTurnCommand, nowMs);
    }
}

bool PathNavigation_hasPath(void)
{
    return gStatus.pathValid;
}

bool PathNavigation_isActive(void)
{
    return (gStatus.mode == PATH_NAV_RECORDING) ||
           (gStatus.mode == PATH_NAV_REPLAYING);
}

void PathNavigation_getStatus(PathNavigation_Status *status)
{
    if (status != NULL) {
        *status = gStatus;
    }
}
