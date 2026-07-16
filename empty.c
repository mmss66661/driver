#include "ti_msp_dl_config.h"

#include "buttons.h"
#include "chassis_motor.h"
#include "imu601.h"
#include "line_follower.h"
#include "line_sensor.h"
#include "oled.h"
#include "path_navigation.h"
#include "stepper_motor.h"
#include "tracking_controller.h"

#define UART_TX_TEST_ENABLE   (0U)
#define UART_TX_TEST_PERIOD_MS (500U)
#define OLED_REFRESH_PERIOD_MS (250U)
#define OLED_RECOVERY_PERIOD_MS (2000U)
#define OLED_PAGE_COUNT        (4U)

#if UART_TX_TEST_ENABLE
static void UART_testSendString(const char *text)
{
    while (*text != '\0') {
        DL_UART_Main_transmitDataBlocking(
            TRACKING_UART_INST, (uint8_t) *text);
        text++;
    }
}
#endif

static char *appendText(char *destination, const char *text)
{
    while (*text != '\0') {
        *destination++ = *text++;
    }
    *destination = '\0';
    return destination;
}

static char *appendUnsigned(char *destination, uint32_t value)
{
    char reverse[10];
    uint8_t count = 0U;
    do {
        reverse[count++] = (char) ('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);
    while (count != 0U) {
        *destination++ = reverse[--count];
    }
    *destination = '\0';
    return destination;
}

static char *appendSigned(char *destination, int32_t value)
{
    uint32_t magnitude;
    if (value < 0) {
        *destination++ = '-';
        magnitude = (uint32_t) (-(int64_t) value);
    } else {
        magnitude = (uint32_t) value;
    }
    return appendUnsigned(destination, magnitude);
}

static void formatSigned(char *line, const char *label, int32_t value)
{
    (void) appendSigned(appendText(line, label), value);
}

static void formatTrackingErrorPair(char *line, int16_t errX, int16_t errY)
{
    char *write = appendSigned(appendText(line, "ERR:("), errX);
    *write++ = ',';
    write = appendSigned(write, errY);
    *write++ = ')';
    *write = '\0';
}

static void formatCentiDegrees(char *line, const char *label, int16_t value)
{
    int32_t signedValue = value;
    uint32_t magnitude;
    char *write = appendText(line, label);
    if (signedValue < 0) {
        *write++ = '-';
        signedValue = -signedValue;
    }
    magnitude = (uint32_t) signedValue;
    write = appendUnsigned(write, magnitude / 100U);
    *write++ = '.';
    *write++ = (char) ('0' + ((magnitude / 10U) % 10U));
    *write++ = (char) ('0' + (magnitude % 10U));
    *write = '\0';
}

static void formatUnsignedCentiDegrees(
    char *line, const char *label, uint16_t value)
{
    char *write = appendUnsigned(appendText(line, label), value / 100U);
    *write++ = '.';
    *write++ = (char) ('0' + ((value / 10U) % 10U));
    *write++ = (char) ('0' + (value % 10U));
    *write = '\0';
}

static void formatHex12(char *line, uint16_t value)
{
    static const char digits[] = "0123456789ABCDEF";
    char *write = appendText(line, "LINE:");
    *write++ = digits[(value >> 8U) & 0x0FU];
    *write++ = digits[(value >> 4U) & 0x0FU];
    *write++ = digits[value & 0x0FU];
    *write = '\0';
}

static const char *navigationModeText(PathNavigation_Mode mode)
{
    switch (mode) {
        case PATH_NAV_RECORDING:
            return "NAV:RECORD";
        case PATH_NAV_REPLAYING:
            return "NAV:REPLAY";
        case PATH_NAV_COMPLETE:
            return "NAV:DONE";
        case PATH_NAV_ERROR:
            return "NAV:ERROR";
        default:
            return "NAV:IDLE";
    }
}

static const char *navigationErrorText(PathNavigation_Error error)
{
    switch (error) {
        case PATH_NAV_ERROR_NO_PATH:
            return "FAULT:NO PATH";
        case PATH_NAV_ERROR_SENSORS:
            return "FAULT:SENSOR";
        case PATH_NAV_ERROR_DRIVE_ACTIVE:
            return "FAULT:POWER";
        case PATH_NAV_ERROR_PATH_SHORT:
            return "FAULT:SHORT";
        case PATH_NAV_ERROR_PATH_FULL:
            return "FAULT:FULL";
        case PATH_NAV_ERROR_FLASH:
            return "FAULT:FLASH";
        default:
            return "FAULT:NONE";
    }
}

static void updateDisplay(uint8_t page)
{
    char line[22];
    IMU601_Attitude attitude;
    TrackingController_Status tracking;
    ChassisMotor_Status chassis;
    PathNavigation_Status navigation;
    uint16_t sensors = LineSensor_readRaw();

    (void) IMU601_getAttitude(&attitude);
    TrackingController_getStatus(&tracking);
    ChassisMotor_getStatus(&chassis);
    PathNavigation_getStatus(&navigation);
    OLED_Clear();

    if (page == 0U) {
        OLED_ShowString(0U, 0U,
            TrackingController_isEnabled() ? "MODE:RUN" : "MODE:HOLD");
        formatUnsignedCentiDegrees(line, "Y:", attitude.yawCentiDegrees);
        OLED_ShowString(0U, 8U, line);
        formatCentiDegrees(line, "P:", attitude.pitchCentiDegrees);
        OLED_ShowString(0U, 16U, line);
        formatCentiDegrees(line, "R:", attitude.rollCentiDegrees);
        OLED_ShowString(0U, 24U, line);
        formatHex12(line, sensors);
        OLED_ShowString(0U, 32U, line);
        (void) appendUnsigned(appendText(line, "BLACK:"),
            LineSensor_countBlack());
        OLED_ShowString(0U, 40U, line);
        formatTrackingErrorPair(line, tracking.errX, tracking.errY);
        OLED_ShowString(0U, 48U, line);
        (void) appendUnsigned(appendText(line, "CAM RX:"),
            tracking.validFrameCount);
        OLED_ShowString(0U, 56U, line);
    } else if (page == 1U) {
        formatSigned(line, "ERRX:", tracking.errX);
        OLED_ShowString(0U, 0U, line);
        formatSigned(line, "ERRY:", tracking.errY);
        OLED_ShowString(0U, 8U, line);
        (void) appendUnsigned(appendText(line, "CAM RX:"),
            tracking.validFrameCount);
        OLED_ShowString(0U, 16U, line);
        (void) appendUnsigned(appendText(line, "IMU RX:"),
            attitude.validFrameCount);
        OLED_ShowString(0U, 24U, line);
        (void) appendUnsigned(appendText(line, "IMU ERR:"),
            attitude.checksumErrorCount);
        OLED_ShowString(0U, 32U, line);
        OLED_ShowString(0U, 40U, "UP/DOWN:PAGE");
    } else if (page == 2U) {
        formatSigned(line, "LEFT:", chassis.leftCommand);
        OLED_ShowString(0U, 0U, line);
        formatSigned(line, "RIGHT:", chassis.rightCommand);
        OLED_ShowString(0U, 8U, line);
        formatSigned(line, "ENC L:", chassis.leftEncoderCount);
        OLED_ShowString(0U, 16U, line);
        formatSigned(line, "ENC R:", chassis.rightEncoderCount);
        OLED_ShowString(0U, 24U, line);
        (void) appendUnsigned(appendText(line, "MOTOR RX:"),
            chassis.feedbackFrameCount);
        OLED_ShowString(0U, 32U, line);
        if (chassis.commandTimedOut) {
            OLED_ShowString(0U, 40U, "MOTOR:TIMEOUT");
        } else if ((chassis.feedbackFrameCount == 0U) &&
                   (chassis.acknowledgementCount == 0U)) {
            OLED_ShowString(0U, 40U, "MOTOR:WAIT");
        } else {
            OLED_ShowString(0U, 40U, "MOTOR:READY");
        }
    } else {
        OLED_ShowString(0U, 0U, navigationModeText(navigation.mode));
        if (navigation.mode == PATH_NAV_ERROR) {
            OLED_ShowString(0U, 8U,
                navigationErrorText(navigation.error));
        } else {
            OLED_ShowString(0U, 8U,
                navigation.pathValid ? "PATH:SAVED" : "PATH:EMPTY");
        }
        (void) appendUnsigned(appendText(line, "POINTS:"),
            navigation.pointCount);
        OLED_ShowString(0U, 16U, line);
        (void) appendUnsigned(appendText(line, "INDEX:"),
            navigation.currentPoint);
        OLED_ShowString(0U, 24U, line);
        (void) appendUnsigned(appendText(line, "DIST:"),
            navigation.distanceTicks);
        OLED_ShowString(0U, 32U, line);
        formatSigned(line, "HEAD:", navigation.headingCentiDegrees);
        OLED_ShowString(0U, 40U, line);
        formatSigned(line, "TARGET:",
            navigation.targetHeadingCentiDegrees);
        OLED_ShowString(0U, 48U, line);
        formatSigned(line, "ERROR:",
            navigation.headingErrorCentiDegrees);
        OLED_ShowString(0U, 56U, line);
    }
    (void) OLED_Refresh();
}

int main(void)
{
#if UART_TX_TEST_ENABLE
    uint32_t lastTestTime = 0U;
    uint32_t lastRxAckTime = 0U;
    uint32_t lastAckedFrameCount = 0U;
#endif
    uint32_t lastDisplayTime = 0U;
    uint32_t lastOledRecoveryTime = 0U;
    uint8_t displayPage = 0U;

    SYSCFG_DL_init();
    StepperMotor_init();
    TrackingController_init();
    ChassisMotor_init();
    LineSensor_init();
    LineFollower_init();
    Buttons_init();
    IMU601_init();
    PathNavigation_init();
    (void) OLED_Init();

#if UART_TX_TEST_ENABLE
    /* Immediate message: UART TX testing no longer depends on SysTick. */
    UART_testSendString("UART0 TX START\r\n");
#endif

    while (1) {
        uint32_t now = TrackingController_getMilliseconds();

        Buttons_process(now);

        if (Buttons_wasPressed(BUTTON_MODE)) {
            if (PathNavigation_isActive()) {
                PathNavigation_stop(now);
                TrackingController_setEnabled(false);
                LineFollower_setEnabled(false, now);
            } else {
                bool enabled = !TrackingController_isEnabled();
                TrackingController_setEnabled(enabled);
                LineFollower_setEnabled(enabled, now);
            }
        }
        if (Buttons_wasPressed(BUTTON_UP)) {
            displayPage = (uint8_t) ((displayPage + 1U) % OLED_PAGE_COUNT);
        }
        if (Buttons_wasPressed(BUTTON_DOWN)) {
            displayPage = (displayPage == 0U) ?
                (OLED_PAGE_COUNT - 1U) : (uint8_t) (displayPage - 1U);
        }
        if (Buttons_wasPressed(BUTTON_KEY1)) {
            PathNavigation_Status navigation;
            PathNavigation_getStatus(&navigation);
            TrackingController_setEnabled(false);
            LineFollower_setEnabled(false, now);
            if (navigation.mode == PATH_NAV_RECORDING) {
                (void) PathNavigation_finishRecording(now);
            } else if (navigation.mode == PATH_NAV_REPLAYING) {
                PathNavigation_stop(now);
            } else if (Buttons_isDown(BUTTON_UP) ||
                       !PathNavigation_hasPath()) {
                (void) PathNavigation_startRecording(now);
            } else {
                (void) PathNavigation_startReplay(now);
            }
            displayPage = 3U;
        }

        TrackingController_process();
        PathNavigation_process(now);
        LineFollower_process(now);
        ChassisMotor_process(now);

        if (!OLED_IsPresent() &&
            ((now - lastOledRecoveryTime) >= OLED_RECOVERY_PERIOD_MS)) {
            lastOledRecoveryTime = now;
            (void) OLED_Init();
        }
        OLED_Process();

        if (OLED_IsPresent() &&
            ((now - lastDisplayTime) >= OLED_REFRESH_PERIOD_MS)) {
            lastDisplayTime = now;
            updateDisplay(displayPage);
        }

#if UART_TX_TEST_ENABLE
        TrackingController_Status status;

        if ((now - lastTestTime) >= UART_TX_TEST_PERIOD_MS) {
            lastTestTime = now;
            UART_testSendString("UART0 TX OK\r\n");
        }

        TrackingController_getStatus(&status);
        if ((status.validFrameCount != lastAckedFrameCount) &&
            ((now - lastRxAckTime) >= 100U)) {
            lastRxAckTime = now;
            lastAckedFrameCount = status.validFrameCount;
            UART_testSendString("UART0 RX FRAME OK\r\n");
        }
#endif

        __WFE();
    }
}
