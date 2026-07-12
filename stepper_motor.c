#include "stepper_motor.h"

#include "ti_msp_dl_config.h"

#define STEPPER_DEFAULT_RATE_HZ (1428U)
#define STEPPER_MIN_RATE_HZ     (100U)
#define STEPPER_MAX_RATE_HZ     (20000U)
#define STEPPER_PULSE_WIDTH_US  (5U)
#define STEPPER_DIR_SETUP_US    (5U)
#define STEPPER_DRIVER_WAKE_US  (1000U)

static uint8_t gRunningMask;
static uint8_t gEnabledMask;
static uint8_t gDirectionMask;
static uint32_t gStepRate[2] = {
    STEPPER_DEFAULT_RATE_HZ, STEPPER_DEFAULT_RATE_HZ};

static GPTIMER_Regs *getTimer(StepperMotor_Select motor)
{
    return (motor == STEPPER_MOTOR_2) ? YAW_PWM_INST : PITCH_PWM_INST;
}

static uint32_t getTimerClock(StepperMotor_Select motor)
{
    return (motor == STEPPER_MOTOR_2) ? YAW_PWM_INST_CLK_FREQ :
                                       PITCH_PWM_INST_CLK_FREQ;
}

static void setTimerOutput(GPTIMER_Regs *timer, bool enabled)
{
    DL_TimerG_setCCPOutputDisabled(timer,
        enabled ? DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL :
                  DL_TIMER_CCP_DIS_OUT_LOW,
        DL_TIMER_CCP_DIS_OUT_LOW);
}

static void applyOutputMask(uint8_t runningMask)
{
    setTimerOutput(PITCH_PWM_INST,
        (runningMask & (uint8_t) STEPPER_MOTOR_1) != 0U);
    setTimerOutput(YAW_PWM_INST,
        (runningMask & (uint8_t) STEPPER_MOTOR_2) != 0U);
}

static void applyEnableMask(uint8_t enabledMask)
{
    if ((enabledMask & (uint8_t) STEPPER_MOTOR_1) != 0U) {
        DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_DIR_MOTOR1_ENABLE_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_MOTOR1_ENABLE_PIN);
    }

    if ((enabledMask & (uint8_t) STEPPER_MOTOR_2) != 0U) {
        DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_DIR_MOTOR2_ENABLE_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_MOTOR2_ENABLE_PIN);
    }
}

static void applyDirection(uint8_t directionMask)
{
    if ((directionMask & (uint8_t) STEPPER_MOTOR_1) != 0U) {
        DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_DIR_MOTOR1_DIR_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_MOTOR1_DIR_PIN);
    }

    if ((directionMask & (uint8_t) STEPPER_MOTOR_2) != 0U) {
        DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_DIR_MOTOR2_DIR_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_MOTOR2_DIR_PIN);
    }
}

void StepperMotor_init(void)
{
    gRunningMask   = 0U;
    gEnabledMask   = 0U;
    gDirectionMask = 0U;
    applyDirection(gDirectionMask);
    applyOutputMask(gRunningMask);
    applyEnableMask(gEnabledMask);
    (void) StepperMotor_setMotorStepRate(
        STEPPER_MOTOR_1, STEPPER_DEFAULT_RATE_HZ);
    (void) StepperMotor_setMotorStepRate(
        STEPPER_MOTOR_2, STEPPER_DEFAULT_RATE_HZ);
}

void StepperMotor_start(StepperMotor_Select motor)
{
    uint8_t newlyEnabled = (uint8_t) motor & (uint8_t) ~gEnabledMask;

    gEnabledMask |= (uint8_t) motor;
    applyEnableMask(gEnabledMask);
    if (newlyEnabled != 0U) {
        delay_cycles((CPUCLK_FREQ / 1000000U) * STEPPER_DRIVER_WAKE_US);
    }

    gRunningMask |= (uint8_t) motor;
    applyOutputMask(gRunningMask);
}

void StepperMotor_hold(StepperMotor_Select motor)
{
    gRunningMask &= (uint8_t) ~(uint8_t) motor;
    applyOutputMask(gRunningMask);
}

void StepperMotor_stop(StepperMotor_Select motor)
{
    StepperMotor_hold(motor);
    gEnabledMask &= (uint8_t) ~(uint8_t) motor;
    applyEnableMask(gEnabledMask);
}

void StepperMotor_setDirection(
    StepperMotor_Select motor, StepperMotor_Direction direction)
{
    uint8_t selected = (uint8_t) motor;
    uint8_t requestedReverse =
        (direction == STEPPER_REVERSE) ? selected : 0U;
    uint8_t changed = (gDirectionMask ^ requestedReverse) & selected;
    uint8_t selectedRunning;

    if (changed == 0U) {
        return;
    }

    selectedRunning = gRunningMask & changed;
    gRunningMask &= (uint8_t) ~changed;
    applyOutputMask(gRunningMask);
    delay_cycles((CPUCLK_FREQ / 1000000U) * STEPPER_DIR_SETUP_US);

    gDirectionMask = (gDirectionMask & (uint8_t) ~changed) |
                     (requestedReverse & changed);
    applyDirection(gDirectionMask);

    delay_cycles((CPUCLK_FREQ / 1000000U) * STEPPER_DIR_SETUP_US);
    gRunningMask |= selectedRunning;
    applyOutputMask(gRunningMask);
}

void StepperMotor_toggleDirection(StepperMotor_Select motor)
{
    uint8_t selected = (uint8_t) motor;
    uint8_t selectedRunning = gRunningMask & selected;

    gRunningMask &= (uint8_t) ~selected;
    applyOutputMask(gRunningMask);
    delay_cycles((CPUCLK_FREQ / 1000000U) * STEPPER_DIR_SETUP_US);
    gDirectionMask ^= selected;
    applyDirection(gDirectionMask);
    delay_cycles((CPUCLK_FREQ / 1000000U) * STEPPER_DIR_SETUP_US);
    gRunningMask |= selectedRunning;
    applyOutputMask(gRunningMask);
}

bool StepperMotor_setMotorStepRate(
    StepperMotor_Select motor, uint32_t stepsPerSecond)
{
    GPTIMER_Regs *timer;
    uint32_t timerClock;
    uint32_t pulseClocks;
    uint32_t period;
    uint32_t compare;
    uint32_t index;

    if (((motor != STEPPER_MOTOR_1) && (motor != STEPPER_MOTOR_2)) ||
        (stepsPerSecond < STEPPER_MIN_RATE_HZ) ||
        (stepsPerSecond > STEPPER_MAX_RATE_HZ)) {
        return false;
    }

    timer       = getTimer(motor);
    timerClock  = getTimerClock(motor);
    pulseClocks = (timerClock / 1000000U) * STEPPER_PULSE_WIDTH_US;
    period      = timerClock / stepsPerSecond;
    if ((period <= pulseClocks) || (period > 65535U)) {
        return false;
    }
    compare = (period - 1U) - pulseClocks;
    index   = (motor == STEPPER_MOTOR_2) ? 1U : 0U;

    DL_TimerG_stopCounter(timer);
    DL_TimerG_setLoadValue(timer, period - 1U);
    DL_TimerG_setCaptureCompareValue(
        timer, compare, DL_TIMER_CC_0_INDEX);
    DL_TimerG_startCounter(timer);

    gStepRate[index] = timerClock / period;
    return true;
}

bool StepperMotor_setStepRate(uint32_t stepsPerSecond)
{
    bool pitchOk = StepperMotor_setMotorStepRate(
        STEPPER_MOTOR_1, stepsPerSecond);
    bool yawOk = StepperMotor_setMotorStepRate(
        STEPPER_MOTOR_2, stepsPerSecond);
    return pitchOk && yawOk;
}

uint32_t StepperMotor_getMotorStepRate(StepperMotor_Select motor)
{
    return (motor == STEPPER_MOTOR_2) ? gStepRate[1] : gStepRate[0];
}

uint32_t StepperMotor_getStepRate(void)
{
    return gStepRate[0];
}
