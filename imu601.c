#include "imu601.h"

#include "ti_msp_dl_config.h"

#define IMU601_FRAME_SIZE (12U)

static volatile uint8_t gFrame[IMU601_FRAME_SIZE];
static volatile uint8_t gIndex;
static volatile IMU601_Attitude gAttitude;

static void sendBytes(const uint8_t *data, uint32_t length)
{
    uint32_t i;
    for (i = 0U; i < length; i++) {
        DL_UART_Main_transmitDataBlocking(IMU601_INST, data[i]);
    }
}

static void delayMs(uint32_t milliseconds)
{
    while (milliseconds-- != 0U) {
        delay_cycles(CPUCLK_FREQ / 1000U);
    }
}

static void acceptFrame(void)
{
    uint8_t checksum = 0U;
    uint8_t i;

    for (i = 2U; i < 11U; i++) {
        checksum = (uint8_t) (checksum + gFrame[i]);
    }
    if (checksum != gFrame[11]) {
        gAttitude.checksumErrorCount++;
        return;
    }

    /* AA 55 ... payload starts at byte 5; all attitude fields are LE. */
    gAttitude.yawCentiDegrees =
        ((uint16_t) gFrame[6] << 8U) | gFrame[5];
    gAttitude.pitchCentiDegrees = (int16_t)
        (((uint16_t) gFrame[8] << 8U) | gFrame[7]);
    gAttitude.rollCentiDegrees = (int16_t)
        (((uint16_t) gFrame[10] << 8U) | gFrame[9]);
    gAttitude.validFrameCount++;
}

static void parseByte(uint8_t byte)
{
    if (gIndex == 0U) {
        if (byte == 0xAAU) {
            gFrame[gIndex++] = byte;
        }
    } else if (gIndex == 1U) {
        if (byte == 0x55U) {
            gFrame[gIndex++] = byte;
        } else {
            gIndex = (byte == 0xAAU) ? 1U : 0U;
            if (gIndex != 0U) {
                gFrame[0] = byte;
            }
        }
    } else {
        gFrame[gIndex++] = byte;
        if (gIndex >= IMU601_FRAME_SIZE) {
            acceptFrame();
            gIndex = 0U;
        }
    }
}

void IMU601_init(void)
{
    static const uint8_t resetCommand[] =
        {0xAA, 0x55, 0x60, 0x12, 0x00, 0x72};
    static const uint8_t calibrationCommand[] =
        {0xAA, 0x55, 0x60, 0x14, 0x04, 0xCD, 0x4C, 0xB4, 0x43, 0x88};

    gIndex = 0U;
    gAttitude.yawCentiDegrees = 0;
    gAttitude.pitchCentiDegrees = 0;
    gAttitude.rollCentiDegrees = 0;
    gAttitude.validFrameCount = 0U;
    gAttitude.checksumErrorCount = 0U;

    sendBytes(resetCommand, sizeof(resetCommand));
    delayMs(500U);
    sendBytes(calibrationCommand, sizeof(calibrationCommand));

    NVIC_ClearPendingIRQ(IMU601_INST_INT_IRQN);
    NVIC_EnableIRQ(IMU601_INST_INT_IRQN);
}

bool IMU601_getAttitude(IMU601_Attitude *attitude)
{
    uint32_t primask;
    if (attitude == NULL) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *attitude = gAttitude;
    if (primask == 0U) {
        __enable_irq();
    }
    return attitude->validFrameCount != 0U;
}

void IMU601_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(IMU601_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            parseByte((uint8_t) DL_UART_Main_receiveData(IMU601_INST));
            break;
        case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
        case DL_UART_MAIN_IIDX_FRAMING_ERROR:
        case DL_UART_MAIN_IIDX_PARITY_ERROR:
        case DL_UART_MAIN_IIDX_NOISE_ERROR:
            gIndex = 0U;
            break;
        default:
            break;
    }
}
