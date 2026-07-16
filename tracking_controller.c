#include "tracking_controller.h"

#include <stdbool.h>

#include "stepper_motor.h"
#include "ti_msp_dl_config.h"

#define FRAME_HEAD_0             (0xFCU)
#define FRAME_HEAD_1             (0xCFU)
#define FRAME_TAIL_0             (0xCFU)
#define FRAME_TAIL_1             (0xFCU)
#define TRACKING_DEAD_BAND       (3)
#define TRACKING_MIN_RATE        (200U)
#define TRACKING_MAX_RATE        (4000U)
#define TRACKING_KP_STEPS_PER_PX (15U)
#define TRACKING_TIMEOUT_MS      (200U)

/* Change either value to 1 if the corresponding physical motor is reversed. */
#define PITCH_DIRECTION_INVERTED (0)
#define YAW_DIRECTION_INVERTED   (0)

static volatile uint32_t gMilliseconds;
static bool gTrackingEnabled;
static volatile bool gFrameReady;
static volatile int16_t gPendingErrX;
static volatile int16_t gPendingErrY;
static volatile uint32_t gValidFrameCount;
static volatile uint32_t gInvalidFrameCount;
static uint32_t gLastFrameTime;
static int16_t gErrX;
static int16_t gErrY;

static uint8_t gParserState;
static uint8_t gPayload[4];

static int16_t decodeSigned16(uint16_t raw)
{
    if (raw <= 0x7FFFU) {
        return (int16_t) raw;
    }
    return (int16_t) (-(int32_t) (0x10000UL - raw));
}

static void parseByte(uint8_t byte)
{
    switch (gParserState) {
        case 0U:
            gParserState = (byte == FRAME_HEAD_0) ? 1U : 0U;
            break;
        case 1U:
            if (byte == FRAME_HEAD_1) {
                gParserState = 2U;
            } else {
                gParserState = (byte == FRAME_HEAD_0) ? 1U : 0U;
            }
            break;
        case 2U:
        case 3U:
        case 4U:
        case 5U:
            gPayload[gParserState - 2U] = byte;
            gParserState++;
            break;
        case 6U:
            if (byte == FRAME_TAIL_0) {
                gParserState = 7U;
            } else {
                gInvalidFrameCount++;
                gParserState = (byte == FRAME_HEAD_0) ? 1U : 0U;
            }
            break;
        case 7U:
            if (byte == FRAME_TAIL_1) {
                uint16_t rawX = (uint16_t) gPayload[0] |
                                ((uint16_t) gPayload[1] << 8U);
                uint16_t rawY = (uint16_t) gPayload[2] |
                                ((uint16_t) gPayload[3] << 8U);
                gPendingErrX = decodeSigned16(rawX);
                gPendingErrY = decodeSigned16(rawY);
                gFrameReady  = true;
                gValidFrameCount++;
            } else {
                gInvalidFrameCount++;
            }
            gParserState = (byte == FRAME_HEAD_0) ? 1U : 0U;
            break;
        default:
            gParserState = 0U;
            break;
    }
}

static StepperMotor_Direction oppositeDirection(
    StepperMotor_Direction direction)
{
    return (direction == STEPPER_FORWARD) ? STEPPER_REVERSE :
                                            STEPPER_FORWARD;
}

static void commandAxis(StepperMotor_Select motor, int16_t error,
    StepperMotor_Direction positiveErrorDirection, bool inverted)
{
    int32_t signedError = error;
    uint32_t magnitude = (signedError < 0) ?
        (uint32_t) (-signedError) : (uint32_t) signedError;
    uint32_t rate;
    StepperMotor_Direction direction;

    if (magnitude <= (uint32_t) TRACKING_DEAD_BAND) {
        StepperMotor_hold(motor);
        return;
    }

    rate = TRACKING_MIN_RATE +
           (magnitude - TRACKING_DEAD_BAND) * TRACKING_KP_STEPS_PER_PX;
    if (rate > TRACKING_MAX_RATE) {
        rate = TRACKING_MAX_RATE;
    }

    direction = (error > 0) ? positiveErrorDirection :
                              oppositeDirection(positiveErrorDirection);
    if (inverted) {
        direction = oppositeDirection(direction);
    }

    (void) StepperMotor_setMotorStepRate(motor, rate);
    StepperMotor_setDirection(motor, direction);
    StepperMotor_start(motor);
}

void TrackingController_init(void)
{
    gMilliseconds     = 0U;
    gFrameReady       = false;
    gParserState      = 0U;
    gValidFrameCount  = 0U;
    gInvalidFrameCount = 0U;
    gLastFrameTime    = 0U;
    gErrX             = 0;
    gErrY             = 0;
    gTrackingEnabled  = false;

    /* SysConfig starts SysTick; its interrupt must be enabled separately. */
    DL_SYSTICK_enableInterrupt();
    NVIC_ClearPendingIRQ(TRACKING_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(TRACKING_UART_INST_INT_IRQN);
}

void TrackingController_process(void)
{
    bool hasFrame = false;
    int16_t errX = 0;
    int16_t errY = 0;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (gFrameReady) {
        errX       = gPendingErrX;
        errY       = gPendingErrY;
        gFrameReady = false;
        hasFrame   = true;
    }
    if (primask == 0U) {
        __enable_irq();
    }

    if (!gTrackingEnabled) {
        StepperMotor_hold(STEPPER_MOTOR_BOTH);
    } else if (hasFrame) {
        gErrX = errX;
        gErrY = errY;
        gLastFrameTime = gMilliseconds;

        /* Motor 1 = Pitch follows image Y; Motor 2 = Yaw follows image X. */
        commandAxis(STEPPER_MOTOR_1, errY, STEPPER_FORWARD,
            PITCH_DIRECTION_INVERTED != 0);
        commandAxis(STEPPER_MOTOR_2, errX, STEPPER_FORWARD,
            YAW_DIRECTION_INVERTED != 0);
    } else if ((gMilliseconds - gLastFrameTime) > TRACKING_TIMEOUT_MS) {
        /* Lost camera data: stop moving but retain gimbal holding torque. */
        StepperMotor_hold(STEPPER_MOTOR_BOTH);
    }
}

void TrackingController_setEnabled(bool enabled)
{
    gTrackingEnabled = enabled;
    if (!enabled) {
        StepperMotor_hold(STEPPER_MOTOR_BOTH);
    }
}

bool TrackingController_isEnabled(void)
{
    return gTrackingEnabled;
}

void TrackingController_getStatus(TrackingController_Status *status)
{
    if (status != NULL) {
        status->errX = gErrX;
        status->errY = gErrY;
        status->validFrameCount = gValidFrameCount;
        status->invalidFrameCount = gInvalidFrameCount;
    }
}

uint32_t TrackingController_getMilliseconds(void)
{
    return gMilliseconds;
}

void SysTick_Handler(void)
{
    gMilliseconds++;
}

void TRACKING_UART_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(TRACKING_UART_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            parseByte((uint8_t) DL_UART_Main_receiveData(TRACKING_UART_INST));
            break;
        case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
        case DL_UART_MAIN_IIDX_FRAMING_ERROR:
        case DL_UART_MAIN_IIDX_PARITY_ERROR:
        case DL_UART_MAIN_IIDX_NOISE_ERROR:
            gInvalidFrameCount++;
            gParserState = 0U;
            break;
        default:
            break;
    }
}
