#include "path_navigation.h"

#include <stddef.h>
#include <string.h>

#include "chassis_motor.h"
#include "imu601.h"
#include "line_follower.h"
#include "ti_msp_dl_config.h"

#define PATH_FLASH_ADDRESS                  (0x0001E000UL)
#define PATH_FLASH_SIZE                     (0x00002000UL)
#define PATH_MAGIC                          (0x32515653UL)
#define PATH_VERSION                        (3U)

#define SQUARE_SIDE_COUNT                   (4U)
#define SQUARE_CORNER_COUNT                 (3U)
#define SQUARE_CONTROL_PERIOD_MS             (20U)
#define SQUARE_SENSOR_TIMEOUT_MS             (1000U)
#define SQUARE_MIN_SIDE_TICKS                (100U)
#define SQUARE_MIN_CORNER_PROGRESS_DIVISOR   (2U)
#define SQUARE_REARM_DISTANCE_TICKS          (20U)
#define SQUARE_CORNER_SENSOR_COUNT           (4U)
#define SQUARE_TEACH_TURN_ENTER_CD           (300)
#define SQUARE_TEACH_FALLBACK_TURN_ENTER_CD  (800)
#define SQUARE_TEACH_TURN_COMPLETE_CD        (8800)
#define SQUARE_TURN_SLOWDOWN_CD              (7500U)
#define SQUARE_TURN_COMPLETE_CD              (8800U)
#define SQUARE_TURN_TIMEOUT_MS               (1400U)
#define SQUARE_ALIGN_TIMEOUT_MS              (700U)
#define SQUARE_ADVANCE_TIMEOUT_MS            (1200U)
#define SQUARE_MIN_REPLAY_ADVANCE_TICKS      (10U)
#define SQUARE_LINE_LOST_TIMEOUT_MS           (800U)
#define SQUARE_STRAIGHT_SLOWDOWN_TICKS       (120U)
#define SQUARE_TURN_ENCODER_MARGIN_TICKS     (20U)
#define SQUARE_CENTER_CONFIRM_CYCLES         (3U)
#define SQUARE_CENTER_MASK                   ((uint16_t) ((1U << 5U) | (1U << 6U)))

#define SQUARE_STRAIGHT_SPEED                (10)
#define SQUARE_STRAIGHT_SLOW_SPEED           (6)
#define SQUARE_ADVANCE_SPEED                 (6)
#define SQUARE_ALIGN_SPEED                   (3)
#define SQUARE_LINE_LOST_SPEED               (3)
#define SQUARE_TURN_FAST_SPEED               (6)
#define SQUARE_TURN_SLOW_SPEED               (3)
#define SQUARE_STRAIGHT_HEADING_GAIN_CD      (100)
#define SQUARE_ALIGN_HEADING_GAIN_CD         (300)
#define SQUARE_ENCODER_BALANCE_DIVISOR       (100)
#define SQUARE_MAX_STRAIGHT_HEADING_TURN     (2)
#define SQUARE_MAX_ENCODER_BALANCE_TURN      (1)
#define SQUARE_MAX_STRAIGHT_TURN             (6)
#define SQUARE_MAX_ALIGN_TURN                (1)
#define SQUARE_TURN_SLEW_PER_CYCLE           (1)
#define SQUARE_LINE_WEIGHT_NUMERATOR         (3)
#define SQUARE_LINE_WEIGHT_DENOMINATOR       (4)

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint8_t sideCount;
    int8_t turnDirection;
    uint32_t sideTicks[SQUARE_SIDE_COUNT];
    uint32_t advanceTicks[SQUARE_CORNER_COUNT];
    uint32_t turnLeftTicks[SQUARE_CORNER_COUNT];
    uint32_t turnRightTicks[SQUARE_CORNER_COUNT];
    uint32_t reserved;
    uint32_t padding;
    uint32_t crc;
} SquareProfile;

typedef enum {
    SQUARE_STAGE_IDLE = 0,
    SQUARE_STAGE_TEACH_STRAIGHT,
    SQUARE_STAGE_TEACH_APPROACH,
    SQUARE_STAGE_TEACH_TURN,
    SQUARE_STAGE_TEACH_DONE,
    SQUARE_STAGE_REPLAY_STRAIGHT,
    SQUARE_STAGE_REPLAY_ADVANCE,
    SQUARE_STAGE_REPLAY_TURN,
    SQUARE_STAGE_REPLAY_ALIGN
} SquareStage;

_Static_assert(sizeof(SquareProfile) == 72U,
    "SquareProfile must use nine flash words");

static SquareProfile gProfile;
__attribute__((used, section(".path_flash_reserve")))
static const uint8_t gPathFlashReserve[PATH_FLASH_SIZE];
static PathNavigation_Status gStatus;
static SquareStage gStage;
static uint8_t gSideIndex;
static uint32_t gLastFeedbackFrame;
static uint32_t gLastImuFrame;
static uint32_t gLastFeedbackTime;
static uint32_t gLastImuTime;
static uint32_t gLastControlTime;
static uint32_t gLastLineSeenTime;
static int32_t gPreviousLeftEncoder;
static int32_t gPreviousRightEncoder;
static uint16_t gPreviousYaw;
static bool gYawInitialized;
static uint32_t gLeftTravelTicks;
static uint32_t gRightTravelTicks;
static uint32_t gSideStartLeftTicks;
static uint32_t gSideStartRightTicks;
static uint32_t gLandmarkLeftTicks;
static uint32_t gLandmarkRightTicks;
static uint32_t gTurnStartLeftTicks;
static uint32_t gTurnStartRightTicks;
static int32_t gSideStartHeading;
static int32_t gLandmarkHeading;
static int32_t gTurnStartHeading;
static uint32_t gStageStartedAt;
static bool gIntersectionArmed;
static uint8_t gCenteredCycles;
static int16_t gAppliedTurn;

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

static uint32_t absoluteEncoderDelta(int32_t current, int32_t previous)
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

static uint32_t averageTravelFrom(uint32_t leftStart, uint32_t rightStart)
{
    uint32_t left = gLeftTravelTicks - leftStart;
    uint32_t right = gRightTravelTicks - rightStart;
    return (left / 2U) + (right / 2U) +
        (((left & 1U) + (right & 1U)) / 2U);
}

static int16_t clampSigned(int32_t value, int16_t limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return (int16_t) -limit;
    }
    return (int16_t) value;
}

static int16_t slewTurn(int16_t requested)
{
    int32_t delta = (int32_t) requested - gAppliedTurn;

    if (delta > SQUARE_TURN_SLEW_PER_CYCLE) {
        delta = SQUARE_TURN_SLEW_PER_CYCLE;
    } else if (delta < -SQUARE_TURN_SLEW_PER_CYCLE) {
        delta = -SQUARE_TURN_SLEW_PER_CYCLE;
    }
    gAppliedTurn = (int16_t) (gAppliedTurn + delta);
    return gAppliedTurn;
}

static int16_t headingTurn(int32_t error, int32_t divisor, int16_t limit)
{
    int32_t command;

    if (gProfile.turnDirection == 0) {
        return 0;
    }
    command = (error / divisor) * gProfile.turnDirection;
    return clampSigned(command, limit);
}

static bool cornerDetected(const LineFollower_Observation *line)
{
    return line->activeCount >= SQUARE_CORNER_SENSOR_COUNT;
}

static uint32_t profileCrc(const SquareProfile *profile)
{
    return crc32(profile, (uint32_t) offsetof(SquareProfile, crc));
}

static bool profileValid(const SquareProfile *profile)
{
    uint8_t i;

    if ((profile->magic != PATH_MAGIC) ||
        (profile->version != PATH_VERSION) ||
        (profile->sideCount != SQUARE_SIDE_COUNT) ||
        ((profile->turnDirection != 1) &&
         (profile->turnDirection != -1)) ||
        (profile->crc != profileCrc(profile))) {
        return false;
    }
    for (i = 0U; i < SQUARE_SIDE_COUNT; i++) {
        if (profile->sideTicks[i] < SQUARE_MIN_SIDE_TICKS) {
            return false;
        }
    }
    for (i = 0U; i < SQUARE_CORNER_COUNT; i++) {
        if ((profile->turnLeftTicks[i] == 0U) &&
            (profile->turnRightTicks[i] == 0U)) {
            return false;
        }
    }
    return true;
}

static bool loadPath(void)
{
    const SquareProfile *stored =
        (const SquareProfile *) (uintptr_t) PATH_FLASH_ADDRESS;

    if (!profileValid(stored)) {
        memset(&gProfile, 0, sizeof(gProfile));
        gStatus.pathValid = false;
        gStatus.pointCount = 0U;
        return false;
    }
    gProfile = *stored;
    gStatus.pathValid = true;
    gStatus.pointCount = SQUARE_SIDE_COUNT;
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
    const uint8_t *bytes = (const uint8_t *) &gProfile;
    uint32_t offset;
    uint32_t primask;
    bool success = true;

    gProfile.magic = PATH_MAGIC;
    gProfile.version = PATH_VERSION;
    gProfile.sideCount = SQUARE_SIDE_COUNT;
    gProfile.reserved = 0U;
    gProfile.padding = 0U;
    gProfile.crc = profileCrc(&gProfile);

    primask = __get_PRIMASK();
    __disable_irq();
    DL_FlashCTL_executeClearStatus(FLASHCTL);
    DL_FlashCTL_unprotectSector(
        FLASHCTL, PATH_FLASH_ADDRESS, DL_FLASHCTL_REGION_SELECT_MAIN);
    if (DL_FlashCTL_eraseMemoryFromRAM(FLASHCTL, PATH_FLASH_ADDRESS,
            DL_FLASHCTL_COMMAND_SIZE_SECTOR) !=
        DL_FLASHCTL_COMMAND_STATUS_PASSED) {
        success = false;
    }
    for (offset = 0U; success && (offset < sizeof(gProfile));
         offset += 8U) {
        success = programFlashWord(PATH_FLASH_ADDRESS + offset,
            bytes + offset);
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
        gLastFeedbackFrame = chassis.feedbackFrameCount;
        gLastImuFrame = attitude.validFrameCount;
        gLastFeedbackTime = nowMs;
        gLastImuTime = nowMs;
        gLeftTravelTicks = 0U;
        gRightTravelTicks = 0U;
        gYawInitialized = true;
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
        gLastFeedbackFrame = chassis.feedbackFrameCount;
        gLastFeedbackTime = nowMs;
        gLeftTravelTicks += absoluteEncoderDelta(
            chassis.leftEncoderCount, gPreviousLeftEncoder);
        gRightTravelTicks += absoluteEncoderDelta(
            chassis.rightEncoderCount, gPreviousRightEncoder);
        gPreviousLeftEncoder = chassis.leftEncoderCount;
        gPreviousRightEncoder = chassis.rightEncoderCount;
        gStatus.distanceTicks =
            (gLeftTravelTicks / 2U) + (gRightTravelTicks / 2U) +
            (((gLeftTravelTicks & 1U) +
              (gRightTravelTicks & 1U)) / 2U);
    }
    return ((nowMs - gLastFeedbackTime) <= SQUARE_SENSOR_TIMEOUT_MS) &&
           ((nowMs - gLastImuTime) <= SQUARE_SENSOR_TIMEOUT_MS);
}

static void updateLineStatus(
    const LineFollower_Observation *line, uint32_t nowMs)
{
    gStatus.lineActiveCount = line->activeCount;
    gStatus.lineErrorTenths = line->errorTenths;
    gStatus.lineTurnCommand = line->turnCommand;
    gStatus.lineDetected = line->state != LINE_FOLLOW_LOST;
    gStatus.intersectionDetected = cornerDetected(line);
    if (gStatus.lineDetected) {
        gLastLineSeenTime = nowMs;
        gStatus.lineLostMilliseconds = 0U;
    } else {
        gStatus.lineLostMilliseconds = nowMs - gLastLineSeenTime;
    }
}

static void resetRuntime(uint32_t nowMs)
{
    gStage = SQUARE_STAGE_IDLE;
    gSideIndex = 0U;
    gSideStartLeftTicks = 0U;
    gSideStartRightTicks = 0U;
    gLandmarkLeftTicks = 0U;
    gLandmarkRightTicks = 0U;
    gTurnStartLeftTicks = 0U;
    gTurnStartRightTicks = 0U;
    gSideStartHeading = 0;
    gLandmarkHeading = 0;
    gTurnStartHeading = 0;
    gStageStartedAt = nowMs;
    gIntersectionArmed = false;
    gCenteredCycles = 0U;
    gAppliedTurn = 0;
}

static void fail(PathNavigation_Error error, uint32_t nowMs)
{
    ChassisMotor_stop(nowMs);
    gAppliedTurn = 0;
    gStage = SQUARE_STAGE_IDLE;
    gStatus.mode = PATH_NAV_ERROR;
    gStatus.error = error;
    gStatus.headingErrorCentiDegrees = 0;
    gStatus.navigationTurnCommand = 0;
    gStatus.fusedTurnCommand = 0;
}

static bool recordSideEndpoint(uint32_t nowMs)
{
    uint32_t travel;

    if (gSideIndex >= SQUARE_SIDE_COUNT) {
        return false;
    }
    travel = averageTravelFrom(
        gSideStartLeftTicks, gSideStartRightTicks);
    if (travel < SQUARE_MIN_SIDE_TICKS) {
        return false;
    }
    gProfile.sideTicks[gSideIndex] = travel;
    gStatus.pointCount = (uint16_t) (gSideIndex + 1U);
    gStatus.segmentDistanceTicks = travel;
    gStatus.segmentTargetTicks = travel;
    gLandmarkLeftTicks = gLeftTravelTicks;
    gLandmarkRightTicks = gRightTravelTicks;
    gLandmarkHeading = gStatus.headingCentiDegrees;
    gStageStartedAt = nowMs;
    gIntersectionArmed = false;
    if (gSideIndex == (SQUARE_SIDE_COUNT - 1U)) {
        gStage = SQUARE_STAGE_TEACH_DONE;
    } else {
        gStage = SQUARE_STAGE_TEACH_APPROACH;
    }
    return true;
}

static void beginTeachTurn(void)
{
    int32_t change = gStatus.headingCentiDegrees - gLandmarkHeading;

    if (gProfile.turnDirection == 0) {
        gProfile.turnDirection = (change >= 0) ? 1 : -1;
    }
    gProfile.advanceTicks[gSideIndex] = averageTravelFrom(
        gLandmarkLeftTicks, gLandmarkRightTicks);
    gStatus.cornerAdvanceTicks =
        gProfile.advanceTicks[gSideIndex];
    gTurnStartLeftTicks = gLeftTravelTicks;
    gTurnStartRightTicks = gRightTravelTicks;
    gTurnStartHeading = gStatus.headingCentiDegrees;
    gStage = SQUARE_STAGE_TEACH_TURN;
}

static void completeTeachTurn(uint32_t nowMs)
{
    gProfile.turnLeftTicks[gSideIndex] =
        gLeftTravelTicks - gTurnStartLeftTicks;
    gProfile.turnRightTicks[gSideIndex] =
        gRightTravelTicks - gTurnStartRightTicks;
    gSideIndex++;
    gStatus.currentPoint = gSideIndex;
    gSideStartLeftTicks = gLeftTravelTicks;
    gSideStartRightTicks = gRightTravelTicks;
    gSideStartHeading = gLandmarkHeading +
        (int32_t) gProfile.turnDirection * 9000;
    gStatus.cornerAdvanceTicks = 0U;
    gStageStartedAt = nowMs;
    gIntersectionArmed = false;
    gStage = SQUARE_STAGE_TEACH_STRAIGHT;
}

static void processTeaching(
    const LineFollower_Observation *line, uint32_t nowMs)
{
    uint32_t sideTravel = averageTravelFrom(
        gSideStartLeftTicks, gSideStartRightTicks);
    uint32_t headingChange = absoluteInt32(
        gStatus.headingCentiDegrees - gSideStartHeading);

    gStatus.segmentDistanceTicks = sideTravel;
    gStatus.segmentTargetTicks =
        (gSideIndex < SQUARE_SIDE_COUNT) ?
            gProfile.sideTicks[gSideIndex] : 0U;
    if (gStage == SQUARE_STAGE_TEACH_APPROACH) {
        gStatus.cornerAdvanceTicks = averageTravelFrom(
            gLandmarkLeftTicks, gLandmarkRightTicks);
    }

    if (gStage == SQUARE_STAGE_TEACH_DONE) {
        return;
    }
    if (!cornerDetected(line) &&
        (sideTravel >= SQUARE_REARM_DISTANCE_TICKS)) {
        gIntersectionArmed = true;
    }

    if (gStage == SQUARE_STAGE_TEACH_STRAIGHT) {
        if (gIntersectionArmed &&
            cornerDetected(line) &&
            recordSideEndpoint(nowMs)) {
            return;
        }
        if ((headingChange >=
             SQUARE_TEACH_FALLBACK_TURN_ENTER_CD) &&
            recordSideEndpoint(nowMs)) {
            if (gStage != SQUARE_STAGE_TEACH_DONE) {
                gLandmarkHeading = gSideStartHeading;
                beginTeachTurn();
            }
        }
        return;
    }

    if (gStage == SQUARE_STAGE_TEACH_APPROACH) {
        uint32_t cornerHeadingChange = absoluteInt32(
            gStatus.headingCentiDegrees - gLandmarkHeading);
        if (cornerHeadingChange >= SQUARE_TEACH_TURN_ENTER_CD) {
            beginTeachTurn();
        }
        return;
    }

    if (gStage == SQUARE_STAGE_TEACH_TURN) {
        int32_t signedTurn =
            (gStatus.headingCentiDegrees - gLandmarkHeading) *
            gProfile.turnDirection;
        if (signedTurn >= SQUARE_TEACH_TURN_COMPLETE_CD) {
            completeTeachTurn(nowMs);
        }
    }
}

static int16_t straightTurnCommand(
    const LineFollower_Observation *line)
{
    uint32_t leftTravel = gLeftTravelTicks - gSideStartLeftTicks;
    uint32_t rightTravel = gRightTravelTicks - gSideStartRightTicks;
    int32_t balanceError = (int32_t) leftTravel -
        (int32_t) rightTravel;
    int16_t heading = headingTurn(
        gStatus.headingErrorCentiDegrees,
        SQUARE_STRAIGHT_HEADING_GAIN_CD,
        SQUARE_MAX_STRAIGHT_HEADING_TURN);
    int16_t balance = clampSigned(
        balanceError / SQUARE_ENCODER_BALANCE_DIVISOR,
        SQUARE_MAX_ENCODER_BALANCE_TURN);
    int32_t fused = heading + balance +
        ((int32_t) line->turnCommand *
         SQUARE_LINE_WEIGHT_NUMERATOR) /
        SQUARE_LINE_WEIGHT_DENOMINATOR;

    gStatus.navigationTurnCommand = heading + balance;
    return clampSigned(fused, SQUARE_MAX_STRAIGHT_TURN);
}

static void beginReplayAdvance(uint32_t nowMs)
{
    gLandmarkLeftTicks = gLeftTravelTicks;
    gLandmarkRightTicks = gRightTravelTicks;
    gStageStartedAt = nowMs;
    gIntersectionArmed = false;
    gAppliedTurn = 0;
    gStage = SQUARE_STAGE_REPLAY_ADVANCE;
}

static void beginReplayTurn(uint32_t nowMs)
{
    gTurnStartLeftTicks = gLeftTravelTicks;
    gTurnStartRightTicks = gRightTravelTicks;
    gTurnStartHeading = gStatus.headingCentiDegrees;
    gStageStartedAt = nowMs;
    gCenteredCycles = 0U;
    gAppliedTurn = 0;
    gStatus.targetHeadingCentiDegrees =
        (int32_t) (gSideIndex + 1U) * 9000 *
        gProfile.turnDirection;
    gStage = SQUARE_STAGE_REPLAY_TURN;
}

static void completeReplay(uint32_t nowMs)
{
    ChassisMotor_stop(nowMs);
    gAppliedTurn = 0;
    gStatus.currentPoint = SQUARE_SIDE_COUNT - 1U;
    gStatus.mode = PATH_NAV_COMPLETE;
    gStatus.headingErrorCentiDegrees = 0;
    gStatus.navigationTurnCommand = 0;
    gStatus.fusedTurnCommand = 0;
    gStatus.segmentDistanceTicks =
        gProfile.sideTicks[SQUARE_SIDE_COUNT - 1U];
    gStatus.segmentTargetTicks =
        gProfile.sideTicks[SQUARE_SIDE_COUNT - 1U];
    gStage = SQUARE_STAGE_IDLE;
}

static void processReplayStraight(
    const LineFollower_Observation *line, uint32_t nowMs)
{
    uint32_t sideTravel = averageTravelFrom(
        gSideStartLeftTicks, gSideStartRightTicks);
    uint32_t targetTicks = gProfile.sideTicks[gSideIndex];
    uint32_t minimumCornerProgress =
        targetTicks / SQUARE_MIN_CORNER_PROGRESS_DIVISOR;
    uint32_t remaining = (sideTravel < targetTicks) ?
        targetTicks - sideTravel : 0U;
    bool atLandmark;
    int16_t forward;

    gStatus.segmentDistanceTicks = sideTravel;
    gStatus.segmentTargetTicks = targetTicks;
    gStatus.cornerAdvanceTicks =
        (gSideIndex < SQUARE_CORNER_COUNT) ?
            gProfile.advanceTicks[gSideIndex] : 0U;
    if (!cornerDetected(line) &&
        (sideTravel >= SQUARE_REARM_DISTANCE_TICKS)) {
        gIntersectionArmed = true;
    }
    atLandmark = (sideTravel >= targetTicks) ||
        (gIntersectionArmed &&
         (sideTravel >= minimumCornerProgress) &&
         cornerDetected(line));
    if (atLandmark) {
        if (gSideIndex == (SQUARE_SIDE_COUNT - 1U)) {
            completeReplay(nowMs);
        } else {
            beginReplayAdvance(nowMs);
            ChassisMotor_setDifferential(
                SQUARE_ADVANCE_SPEED, 0, nowMs);
        }
        return;
    }

    gStatus.targetHeadingCentiDegrees =
        (int32_t) gSideIndex * 9000 * gProfile.turnDirection;
    gStatus.headingErrorCentiDegrees =
        gStatus.targetHeadingCentiDegrees -
        gStatus.headingCentiDegrees;
    if (!gStatus.lineDetected) {
        if (gStatus.lineLostMilliseconds >
            SQUARE_LINE_LOST_TIMEOUT_MS) {
            fail(PATH_NAV_ERROR_LINE_LOST, nowMs);
            return;
        }
        forward = SQUARE_LINE_LOST_SPEED;
        gStatus.fusedTurnCommand = headingTurn(
            gStatus.headingErrorCentiDegrees,
            SQUARE_STRAIGHT_HEADING_GAIN_CD,
            SQUARE_MAX_STRAIGHT_HEADING_TURN);
    } else {
        forward = (remaining <= SQUARE_STRAIGHT_SLOWDOWN_TICKS) ?
            SQUARE_STRAIGHT_SLOW_SPEED : SQUARE_STRAIGHT_SPEED;
        gStatus.fusedTurnCommand = straightTurnCommand(line);
    }
    gStatus.fusedTurnCommand = slewTurn(
        gStatus.fusedTurnCommand);
    ChassisMotor_setDifferential(
        forward, gStatus.fusedTurnCommand, nowMs);
}

static void processReplayAdvance(uint32_t nowMs)
{
    uint32_t traveled = averageTravelFrom(
        gLandmarkLeftTicks, gLandmarkRightTicks);
    uint32_t target = gProfile.advanceTicks[gSideIndex];

    if (target < SQUARE_MIN_REPLAY_ADVANCE_TICKS) {
        target = SQUARE_MIN_REPLAY_ADVANCE_TICKS;
    }
    gStatus.segmentDistanceTicks = traveled;
    gStatus.segmentTargetTicks = target;
    gStatus.cornerAdvanceTicks = target;

    gStatus.headingErrorCentiDegrees =
        gStatus.targetHeadingCentiDegrees -
        gStatus.headingCentiDegrees;
    if ((traveled >= target) ||
        ((nowMs - gStageStartedAt) >= SQUARE_ADVANCE_TIMEOUT_MS)) {
        beginReplayTurn(nowMs);
        ChassisMotor_setDifferential(
            0, SQUARE_TURN_FAST_SPEED, nowMs);
        return;
    }
    gStatus.navigationTurnCommand = headingTurn(
        gStatus.headingErrorCentiDegrees,
        SQUARE_STRAIGHT_HEADING_GAIN_CD,
        SQUARE_MAX_STRAIGHT_HEADING_TURN);
    gStatus.fusedTurnCommand = slewTurn(
        gStatus.navigationTurnCommand);
    ChassisMotor_setDifferential(SQUARE_ADVANCE_SPEED,
        gStatus.fusedTurnCommand, nowMs);
}

static void beginReplayAlignment(uint32_t nowMs)
{
    gStage = SQUARE_STAGE_REPLAY_ALIGN;
    gStageStartedAt = nowMs;
    gCenteredCycles = 0U;
    gAppliedTurn = 0;
    ChassisMotor_stop(nowMs);
}

static void processReplayTurn(uint32_t nowMs)
{
    uint32_t turned = absoluteInt32(
        gStatus.headingCentiDegrees - gTurnStartHeading);
    uint32_t leftTravel = gLeftTravelTicks - gTurnStartLeftTicks;
    uint32_t rightTravel = gRightTravelTicks - gTurnStartRightTicks;
    uint32_t leftLimit = gProfile.turnLeftTicks[gSideIndex] +
        gProfile.turnLeftTicks[gSideIndex] / 2U +
        SQUARE_TURN_ENCODER_MARGIN_TICKS;
    uint32_t rightLimit = gProfile.turnRightTicks[gSideIndex] +
        gProfile.turnRightTicks[gSideIndex] / 2U +
        SQUARE_TURN_ENCODER_MARGIN_TICKS;
    int16_t turnSpeed;

    gStatus.segmentDistanceTicks =
        (leftTravel / 2U) + (rightTravel / 2U) +
        (((leftTravel & 1U) + (rightTravel & 1U)) / 2U);
    gStatus.segmentTargetTicks =
        (gProfile.turnLeftTicks[gSideIndex] / 2U) +
        (gProfile.turnRightTicks[gSideIndex] / 2U) +
        (((gProfile.turnLeftTicks[gSideIndex] & 1U) +
          (gProfile.turnRightTicks[gSideIndex] & 1U)) / 2U);

    gStatus.headingErrorCentiDegrees =
        gStatus.targetHeadingCentiDegrees -
        gStatus.headingCentiDegrees;
    if (turned >= SQUARE_TURN_COMPLETE_CD) {
        beginReplayAlignment(nowMs);
        return;
    }
    if (((leftTravel >= leftLimit) ||
         (rightTravel >= rightLimit)) &&
        (turned >= SQUARE_TURN_SLOWDOWN_CD)) {
        beginReplayAlignment(nowMs);
        return;
    }
    if ((nowMs - gStageStartedAt) >= SQUARE_TURN_TIMEOUT_MS) {
        fail(PATH_NAV_ERROR_SENSORS, nowMs);
        return;
    }
    turnSpeed = (turned >= SQUARE_TURN_SLOWDOWN_CD) ?
        SQUARE_TURN_SLOW_SPEED : SQUARE_TURN_FAST_SPEED;
    gStatus.navigationTurnCommand = turnSpeed;
    gStatus.fusedTurnCommand = turnSpeed;
    ChassisMotor_setDifferential(0, turnSpeed, nowMs);
}

static void processReplayAlignment(
    const LineFollower_Observation *line, uint32_t nowMs)
{
    bool centered =
        (line->rawSensors & SQUARE_CENTER_MASK) != 0U;
    int16_t correction;

    gStatus.segmentDistanceTicks = averageTravelFrom(
        gTurnStartLeftTicks, gTurnStartRightTicks);

    gStatus.headingErrorCentiDegrees =
        gStatus.targetHeadingCentiDegrees -
        gStatus.headingCentiDegrees;
    correction = headingTurn(gStatus.headingErrorCentiDegrees,
        SQUARE_ALIGN_HEADING_GAIN_CD, SQUARE_MAX_ALIGN_TURN);
    if (centered) {
        if (gCenteredCycles < SQUARE_CENTER_CONFIRM_CYCLES) {
            gCenteredCycles++;
        }
    } else {
        gCenteredCycles = 0U;
    }
    if (gCenteredCycles >= SQUARE_CENTER_CONFIRM_CYCLES) {
        gSideIndex++;
        gStatus.currentPoint = gSideIndex;
        gSideStartLeftTicks = gLeftTravelTicks;
        gSideStartRightTicks = gRightTravelTicks;
        gSideStartHeading = gStatus.headingCentiDegrees;
        gStageStartedAt = nowMs;
        gIntersectionArmed = false;
        gCenteredCycles = 0U;
        gAppliedTurn = 0;
        gStage = SQUARE_STAGE_REPLAY_STRAIGHT;
        ChassisMotor_setDifferential(
            SQUARE_STRAIGHT_SLOW_SPEED, 0, nowMs);
        return;
    }
    if ((nowMs - gStageStartedAt) >= SQUARE_ALIGN_TIMEOUT_MS) {
        fail(PATH_NAV_ERROR_LINE_LOST, nowMs);
        return;
    }
    gStatus.navigationTurnCommand = correction;
    gStatus.fusedTurnCommand = correction;
    ChassisMotor_setDifferential(
        SQUARE_ALIGN_SPEED, correction, nowMs);
}

void PathNavigation_init(void)
{
    memset(&gStatus, 0, sizeof(gStatus));
    memset(&gProfile, 0, sizeof(gProfile));
    resetRuntime(0U);
    gStatus.mode = PATH_NAV_IDLE;
    gStatus.error = PATH_NAV_ERROR_NONE;
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
    memset(&gProfile, 0, sizeof(gProfile));
    gStatus.distanceTicks = 0U;
    gStatus.segmentDistanceTicks = 0U;
    gStatus.segmentTargetTicks = 0U;
    gStatus.cornerAdvanceTicks = 0U;
    gStatus.headingCentiDegrees = 0;
    gStatus.targetHeadingCentiDegrees = 0;
    gStatus.headingErrorCentiDegrees = 0;
    gStatus.navigationTurnCommand = 0;
    gStatus.fusedTurnCommand = 0;
    gStatus.currentPoint = 0U;
    gStatus.pointCount = 0U;
    gStatus.pathValid = false;
    resetRuntime(nowMs);
    LineFollower_getObservation(&line);
    gLastLineSeenTime = nowMs;
    updateLineStatus(&line, nowMs);
    gStage = SQUARE_STAGE_TEACH_STRAIGHT;
    gIntersectionArmed = !cornerDetected(&line);
    gLastControlTime = nowMs - SQUARE_CONTROL_PERIOD_MS;
    ChassisMotor_stop(nowMs);
    gStatus.mode = PATH_NAV_RECORDING;
    return true;
}

bool PathNavigation_finishRecording(uint32_t nowMs)
{
    if (gStatus.mode != PATH_NAV_RECORDING) {
        return false;
    }
    ChassisMotor_stop(nowMs);
    if ((gSideIndex == (SQUARE_SIDE_COUNT - 1U)) &&
        (gProfile.sideTicks[gSideIndex] == 0U)) {
        (void) recordSideEndpoint(nowMs);
    }
    if ((gProfile.sideTicks[SQUARE_SIDE_COUNT - 1U] <
         SQUARE_MIN_SIDE_TICKS) ||
        (gProfile.turnDirection == 0)) {
        fail(PATH_NAV_ERROR_PATH_SHORT, nowMs);
        return false;
    }
    gProfile.sideCount = SQUARE_SIDE_COUNT;
    gProfile.magic = PATH_MAGIC;
    gProfile.version = PATH_VERSION;
    gProfile.reserved = 0U;
    gProfile.padding = 0U;
    gProfile.crc = profileCrc(&gProfile);
    if (!profileValid(&gProfile)) {
        fail(PATH_NAV_ERROR_PATH_SHORT, nowMs);
        return false;
    }
    if (!savePath()) {
        fail(PATH_NAV_ERROR_FLASH, nowMs);
        return false;
    }
    gStatus.mode = PATH_NAV_COMPLETE;
    gStatus.error = PATH_NAV_ERROR_NONE;
    gStatus.pathValid = true;
    gStatus.pointCount = SQUARE_SIDE_COUNT;
    gStatus.currentPoint = SQUARE_SIDE_COUNT - 1U;
    gStage = SQUARE_STAGE_IDLE;
    return true;
}

bool PathNavigation_startReplay(uint32_t nowMs)
{
    LineFollower_Observation line;

    if (!gStatus.pathValid || !profileValid(&gProfile)) {
        fail(PATH_NAV_ERROR_NO_PATH, nowMs);
        return false;
    }
    if (!readSensors(nowMs, true)) {
        fail(PATH_NAV_ERROR_SENSORS, nowMs);
        return false;
    }
    gStatus.distanceTicks = 0U;
    gStatus.segmentDistanceTicks = 0U;
    gStatus.segmentTargetTicks = gProfile.sideTicks[0];
    gStatus.cornerAdvanceTicks = gProfile.advanceTicks[0];
    gStatus.headingCentiDegrees = 0;
    gStatus.targetHeadingCentiDegrees = 0;
    gStatus.headingErrorCentiDegrees = 0;
    gStatus.navigationTurnCommand = 0;
    gStatus.fusedTurnCommand = 0;
    gStatus.currentPoint = 0U;
    gStatus.pointCount = SQUARE_SIDE_COUNT;
    gStatus.error = PATH_NAV_ERROR_NONE;
    resetRuntime(nowMs);
    LineFollower_getObservation(&line);
    gLastLineSeenTime = nowMs;
    updateLineStatus(&line, nowMs);
    gStage = SQUARE_STAGE_REPLAY_STRAIGHT;
    gIntersectionArmed = !cornerDetected(&line);
    gLastControlTime = nowMs - SQUARE_CONTROL_PERIOD_MS;
    gStatus.mode = PATH_NAV_REPLAYING;
    return true;
}

void PathNavigation_stop(uint32_t nowMs)
{
    bool wasRecording = gStatus.mode == PATH_NAV_RECORDING;

    ChassisMotor_stop(nowMs);
    resetRuntime(nowMs);
    gStatus.mode = PATH_NAV_IDLE;
    gStatus.error = PATH_NAV_ERROR_NONE;
    gStatus.headingErrorCentiDegrees = 0;
    gStatus.navigationTurnCommand = 0;
    gStatus.fusedTurnCommand = 0;
    if (wasRecording) {
        (void) loadPath();
    }
}

void PathNavigation_process(uint32_t nowMs)
{
    LineFollower_Observation line;

    if ((gStatus.mode != PATH_NAV_RECORDING) &&
        (gStatus.mode != PATH_NAV_REPLAYING)) {
        return;
    }
    if (!readSensors(nowMs, false)) {
        fail(PATH_NAV_ERROR_SENSORS, nowMs);
        return;
    }
    if ((nowMs - gLastControlTime) < SQUARE_CONTROL_PERIOD_MS) {
        return;
    }
    gLastControlTime = nowMs;
    LineFollower_getObservation(&line);
    updateLineStatus(&line, nowMs);

    if (gStatus.mode == PATH_NAV_RECORDING) {
        processTeaching(&line, nowMs);
        return;
    }
    if (gStage == SQUARE_STAGE_REPLAY_STRAIGHT) {
        processReplayStraight(&line, nowMs);
    } else if (gStage == SQUARE_STAGE_REPLAY_ADVANCE) {
        processReplayAdvance(nowMs);
    } else if (gStage == SQUARE_STAGE_REPLAY_TURN) {
        processReplayTurn(nowMs);
    } else if (gStage == SQUARE_STAGE_REPLAY_ALIGN) {
        processReplayAlignment(&line, nowMs);
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
