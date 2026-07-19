#include "chassis_motor.h"

#include <stddef.h>

#include "ti_msp_dl_config.h"

#define MODBUS_ADDRESS                     (0x0AU)
#define MODBUS_WRITE_SINGLE                (0x06U)
#define MODBUS_WRITE_MULTIPLE              (0x10U)
#define MODBUS_READ_HOLDING                (0x03U)
#define CHASSIS_TX_PERIOD_MS               (20U)
#define CHASSIS_COMMAND_TIMEOUT_MS         (250U)
#define CHASSIS_LEFT_MOTOR_INVERTED        (0)
#define CHASSIS_RIGHT_MOTOR_INVERTED       (1)
#define CHASSIS_LEFT_ENCODER_REVERSED      (1U)
#define CHASSIS_RIGHT_ENCODER_REVERSED     (1U)
#define CHASSIS_ENCODER_BYTE_COUNT         (16U)
#define CHASSIS_RX_BUFFER_SIZE             (21U)

static volatile ChassisMotor_Status gStatus;
static volatile uint8_t gRxBuffer[CHASSIS_RX_BUFFER_SIZE];
static volatile uint8_t gRxIndex;
static volatile uint8_t gExpectedLength;
static int16_t gTargetLeft;
static int16_t gTargetRight;
static uint32_t gLastCommandTime;
static uint32_t gLastSpeedTransmitTime;
static bool gCommandValid;
static bool gTransmitImmediately;
static bool gClosedLoopEnabled;

static uint16_t crc16(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFFU;
    uint8_t i;
    while (length-- != 0U) {
        crc ^= *data++;
        for (i = 0U; i < 8U; i++) {
            crc = ((crc & 1U) != 0U) ?
                (uint16_t) ((crc >> 1U) ^ 0xA001U) :
                (uint16_t) (crc >> 1U);
        }
    }
    return crc;
}

static void sendFrame(uint8_t *frame, uint8_t lengthWithoutCrc)
{
    uint16_t crc = crc16(frame, lengthWithoutCrc);
    uint8_t length = lengthWithoutCrc;
    uint8_t i;

    frame[length++] = (uint8_t) crc;
    frame[length++] = (uint8_t) (crc >> 8U);
    for (i = 0U; i < length; i++) {
        DL_UART_Main_transmitDataBlocking(CHASSIS_UART_INST, frame[i]);
    }
}

static void writeSingleRegister(uint16_t address, uint16_t value)
{
    uint8_t frame[8];
    frame[0] = MODBUS_ADDRESS;
    frame[1] = MODBUS_WRITE_SINGLE;
    frame[2] = (uint8_t) (address >> 8U);
    frame[3] = (uint8_t) address;
    frame[4] = (uint8_t) (value >> 8U);
    frame[5] = (uint8_t) value;
    sendFrame(frame, 6U);
}

static void sendSpeeds(int16_t left, int16_t right)
{
    uint8_t frame[17];
    uint16_t values[4];
    uint8_t i;

#if CHASSIS_LEFT_MOTOR_INVERTED
    left = (left == INT16_MIN) ? INT16_MAX : (int16_t) -left;
#endif
#if CHASSIS_RIGHT_MOTOR_INVERTED
    right = (right == INT16_MIN) ? INT16_MAX : (int16_t) -right;
#endif
    values[0] = (uint16_t) left;
    values[1] = (uint16_t) right;
    values[2] = 0U;
    values[3] = 0U;

    frame[0] = MODBUS_ADDRESS;
    frame[1] = MODBUS_WRITE_MULTIPLE;
    frame[2] = 0x00U;
    frame[3] = 0x00U;
    frame[4] = 0x00U;
    frame[5] = 0x04U;
    frame[6] = 0x08U;
    for (i = 0U; i < 4U; i++) {
        frame[7U + i * 2U] = (uint8_t) (values[i] >> 8U);
        frame[8U + i * 2U] = (uint8_t) values[i];
    }
    sendFrame(frame, 15U);
}

static void delayMs(uint32_t milliseconds)
{
    while (milliseconds-- != 0U) {
        delay_cycles(CPUCLK_FREQ / 1000U);
    }
}

static int16_t saturateInt16(int32_t value)
{
    if (value > INT16_MAX) {
        return INT16_MAX;
    }
    if (value < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t) value;
}

static void resetParser(void)
{
    gRxIndex = 0U;
    gExpectedLength = 0U;
}

static void parseByte(uint8_t byte);

static void drainRxFifo(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(CHASSIS_UART_INST)) {
        uint8_t byte = (uint8_t)
            DL_UART_Main_receiveData(CHASSIS_UART_INST);
        parseByte(byte);
    }
}

static void enterClosedLoop(void)
{
    if (!gClosedLoopEnabled) {
        writeSingleRegister(0x0008U, 0x0001U);
        delayMs(50U);
        gClosedLoopEnabled = true;
    }
}

static void acceptFrame(void)
{
    uint8_t length = gExpectedLength;
    uint16_t receivedCrc = (uint16_t) gRxBuffer[length - 2U] |
        ((uint16_t) gRxBuffer[length - 1U] << 8U);
    uint8_t function = gRxBuffer[1];
    uint16_t calculatedCrc = crc16((const uint8_t *) gRxBuffer,
        (uint8_t) (length - 2U));

    if (receivedCrc != calculatedCrc) {
        gStatus.crcErrorCount++;
        return;
    }
    if (function == MODBUS_READ_HOLDING) {
        uint8_t registerCount = (uint8_t) (gRxBuffer[2] / 2U);
        uint8_t i;
        if (registerCount > CHASSIS_FEEDBACK_REGISTERS) {
            return;
        }
        for (i = 0U; i < registerCount; i++) {
            gStatus.feedback[i] = (int16_t)
                (((uint16_t) gRxBuffer[3U + i * 2U] << 8U) |
                 gRxBuffer[4U + i * 2U]);
        }
        gStatus.leftEncoderCount = gStatus.feedback[0];
        gStatus.rightEncoderCount = gStatus.feedback[1];
        gStatus.feedbackFrameCount++;
    } else {
        gStatus.acknowledgementCount++;
    }
}

static void parseByte(uint8_t byte)
{
    if (gRxIndex == 0U) {
        if (byte == MODBUS_ADDRESS) {
            gRxBuffer[gRxIndex++] = byte;
        }
        return;
    }

    if (gRxIndex >= CHASSIS_RX_BUFFER_SIZE) {
        resetParser();
        return;
    }
    gRxBuffer[gRxIndex++] = byte;

    if (gRxIndex == 2U) {
        if ((byte == MODBUS_WRITE_SINGLE) ||
            (byte == MODBUS_WRITE_MULTIPLE)) {
            gExpectedLength = 8U;
        } else if (byte != MODBUS_READ_HOLDING) {
            resetParser();
        }
    } else if ((gRxIndex == 3U) &&
               (gRxBuffer[1] == MODBUS_READ_HOLDING)) {
        if (byte != CHASSIS_ENCODER_BYTE_COUNT) {
            resetParser();
        } else {
            gExpectedLength = (uint8_t) (byte + 5U);
        }
    }

    if ((gExpectedLength != 0U) && (gRxIndex >= gExpectedLength)) {
        acceptFrame();
        resetParser();
    }
}

void ChassisMotor_init(void)
{
    uint8_t i;
    gTargetLeft = 0;
    gTargetRight = 0;
    gLastCommandTime = 0U;
    gLastSpeedTransmitTime = 0U;
    gCommandValid = false;
    gTransmitImmediately = false;
    gClosedLoopEnabled = false;
    resetParser();
    gStatus.leftCommand = 0;
    gStatus.rightCommand = 0;
    for (i = 0U; i < CHASSIS_FEEDBACK_REGISTERS; i++) {
        gStatus.feedback[i] = 0;
    }
    gStatus.feedbackFrameCount = 0U;
    gStatus.leftEncoderCount = 0;
    gStatus.rightEncoderCount = 0;
    gStatus.acknowledgementCount = 0U;
    gStatus.crcErrorCount = 0U;
    gStatus.uartErrorCount = 0U;
    gStatus.commandTimedOut = false;

    NVIC_ClearPendingIRQ(CHASSIS_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(CHASSIS_UART_INST_INT_IRQN);

    writeSingleRegister(0x0009U, CHASSIS_LEFT_ENCODER_REVERSED);
    delayMs(50U);
    writeSingleRegister(0x000AU, CHASSIS_RIGHT_ENCODER_REVERSED);
}

void ChassisMotor_process(uint32_t nowMs)
{
    if (!gClosedLoopEnabled) {
        return;
    }
    if (gCommandValid &&
        ((nowMs - gLastCommandTime) > CHASSIS_COMMAND_TIMEOUT_MS)) {
        gTargetLeft = 0;
        gTargetRight = 0;
        gStatus.leftCommand = 0;
        gStatus.rightCommand = 0;
        gStatus.commandTimedOut = true;
        gCommandValid = false;
        gTransmitImmediately = true;
    }

    if (gTransmitImmediately ||
        ((nowMs - gLastSpeedTransmitTime) >= CHASSIS_TX_PERIOD_MS)) {
        gTransmitImmediately = false;
        gLastSpeedTransmitTime = nowMs;
        sendSpeeds(gTargetLeft, gTargetRight);
    }
}

void ChassisMotor_setWheelSpeeds(
    int16_t leftSpeed, int16_t rightSpeed, uint32_t nowMs)
{
    if (((leftSpeed != 0) || (rightSpeed != 0)) &&
        !gClosedLoopEnabled) {
        enterClosedLoop();
    }
    gTargetLeft = leftSpeed;
    gTargetRight = rightSpeed;
    gLastCommandTime = nowMs;
    gCommandValid = true;
    gTransmitImmediately = gClosedLoopEnabled;
    gStatus.leftCommand = leftSpeed;
    gStatus.rightCommand = rightSpeed;
    gStatus.commandTimedOut = false;
}

void ChassisMotor_setDifferential(
    int16_t forwardSpeed, int16_t turnSpeed, uint32_t nowMs)
{
    int16_t left = saturateInt16(
        (int32_t) forwardSpeed - (int32_t) turnSpeed);
    int16_t right = saturateInt16(
        (int32_t) forwardSpeed + (int32_t) turnSpeed);
    ChassisMotor_setWheelSpeeds(left, right, nowMs);
}

void ChassisMotor_stop(uint32_t nowMs)
{
    ChassisMotor_setWheelSpeeds(0, 0, nowMs);
}

bool ChassisMotor_isClosedLoopEnabled(void)
{
    return gClosedLoopEnabled;
}

void ChassisMotor_getStatus(ChassisMotor_Status *status)
{
    uint32_t primask;
    if (status == NULL) {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *status = gStatus;
    if (primask == 0U) {
        __enable_irq();
    }
}

void CHASSIS_UART_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(CHASSIS_UART_INST)) {
        case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
        case DL_UART_MAIN_IIDX_FRAMING_ERROR:
        case DL_UART_MAIN_IIDX_PARITY_ERROR:
        case DL_UART_MAIN_IIDX_NOISE_ERROR:
            gStatus.uartErrorCount++;
            resetParser();
            break;
        default:
            break;
    }

    /* Drain the FIFO even when the pending reason is a line error. */
    drainRxFifo();
}
