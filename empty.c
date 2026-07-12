#include "ti_msp_dl_config.h"

#include "stepper_motor.h"
#include "tracking_controller.h"

#define UART_TX_TEST_ENABLE   (1U)
#define UART_TX_TEST_PERIOD_MS (500U)

static void UART_testSendString(const char *text)
{
    while (*text != '\0') {
        DL_UART_Main_transmitDataBlocking(
            TRACKING_UART_INST, (uint8_t) *text);
        text++;
    }
}

int main(void)
{
    uint32_t lastTestTime = 0U;
    uint32_t lastRxAckTime = 0U;
    uint32_t lastAckedFrameCount = 0U;

    SYSCFG_DL_init();
    StepperMotor_init();
    TrackingController_init();

    while (1) {
        TrackingController_process();

#if UART_TX_TEST_ENABLE
        uint32_t now = TrackingController_getMilliseconds();
        TrackingController_Status status;

        if ((now - lastTestTime) >= UART_TX_TEST_PERIOD_MS) {
            lastTestTime = now;
            UART_testSendString("UART2 TX OK\r\n");
        }

        TrackingController_getStatus(&status);
        if ((status.validFrameCount != lastAckedFrameCount) &&
            ((now - lastRxAckTime) >= 100U)) {
            lastRxAckTime = now;
            lastAckedFrameCount = status.validFrameCount;
            UART_testSendString("UART2 RX FRAME OK\r\n");
        }
#endif

        __WFE();
    }
}
