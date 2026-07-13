#include "ti_msp_dl_config.h"

#include "buttons.h"
#include "chassis_motor.h"
#include "imu601.h"
#include "line_sensor.h"
#include "oled.h"
#include "stepper_motor.h"
#include "tracking_controller.h"

#define UART_TX_TEST_ENABLE   (0U)
#define UART_TX_TEST_PERIOD_MS (500U)
#define OLED_REFRESH_PERIOD_MS (250U)
#define OLED_PAGE_COUNT        (3U)

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

static void formatSigned(char *line, const char *label, int32_t value)
{
    char *write = appendText(line, label);
    if (value < 0) {
        *write++ = '-';
        value = -value;
    }
    (void) appendUnsigned(write, (uint32_t) value);
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

static void updateDisplay(uint8_t page)
{
    char line[22];
    IMU601_Attitude attitude;
    TrackingController_Status tracking;
    ChassisMotor_Status chassis;
    uint16_t sensors = LineSensor_readRaw();

    (void) IMU601_getAttitude(&attitude);
    TrackingController_getStatus(&tracking);
    ChassisMotor_getStatus(&chassis);
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
    } else {
        formatSigned(line, "LEFT:", chassis.leftCommand);
        OLED_ShowString(0U, 0U, line);
        formatSigned(line, "RIGHT:", chassis.rightCommand);
        OLED_ShowString(0U, 8U, line);
        formatSigned(line, "FB0:", chassis.feedback[0]);
        OLED_ShowString(0U, 16U, line);
        formatSigned(line, "FB1:", chassis.feedback[1]);
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
    uint8_t displayPage = 0U;

    SYSCFG_DL_init();
    StepperMotor_init();
    TrackingController_init();
    ChassisMotor_init();
    LineSensor_init();
    Buttons_init();
    IMU601_init();
    (void) OLED_Init();

#if UART_TX_TEST_ENABLE
    /* Immediate message: UART TX testing no longer depends on SysTick. */
    UART_testSendString("UART0 TX START\r\n");
#endif

    while (1) {
        uint32_t now = TrackingController_getMilliseconds();

        TrackingController_process();
        ChassisMotor_process(now);
        Buttons_process(now);

        if (Buttons_wasPressed(BUTTON_MODE)) {
            TrackingController_setEnabled(
                !TrackingController_isEnabled());
        }
        if (Buttons_wasPressed(BUTTON_UP)) {
            displayPage = (uint8_t) ((displayPage + 1U) % OLED_PAGE_COUNT);
        }
        if (Buttons_wasPressed(BUTTON_DOWN)) {
            displayPage = (displayPage == 0U) ?
                (OLED_PAGE_COUNT - 1U) : (uint8_t) (displayPage - 1U);
        }

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
