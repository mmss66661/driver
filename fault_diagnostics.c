#include "fault_diagnostics.h"

#include "ti_msp_dl_config.h"

enum {
    FAULT_SOURCE_NMI       = 2U,
    FAULT_SOURCE_HARDFAULT = 3U,
    FAULT_SOURCE_SVC       = 11U,
    FAULT_SOURCE_PENDSV    = 14U,
    FAULT_SOURCE_IRQ_BASE  = 16U
};

volatile FaultDiagnostics_Info gFaultInfo;
volatile uint32_t gStartupStage;

void FaultDiagnostics_setStage(uint32_t stage)
{
    gStartupStage = stage;
}

static __NO_RETURN void trap(uint32_t source, const uint32_t *stackFrame)
{
    gFaultInfo.magic           = FAULT_DIAGNOSTICS_MAGIC;
    gFaultInfo.source          = source;
    gFaultInfo.exceptionNumber = __get_IPSR();
    gFaultInfo.startupStage    = gStartupStage;
    gFaultInfo.icsr            = SCB->ICSR;

    if (stackFrame != NULL) {
        /* ARM exception frame: R0,R1,R2,R3,R12,LR,PC,xPSR. */
        gFaultInfo.stackedLR   = stackFrame[5];
        gFaultInfo.stackedPC   = stackFrame[6];
        gFaultInfo.stackedXPSR = stackFrame[7];
    } else {
        gFaultInfo.stackedLR   = 0U;
        gFaultInfo.stackedPC   = 0U;
        gFaultInfo.stackedXPSR = 0U;
    }

    __disable_irq();
    __BKPT(0);
    while (1) {
    }
}

/* Called by the naked wrapper with the original exception stack pointer. */
__NO_RETURN void FaultDiagnostics_captureHardFault(uint32_t *stackFrame)
{
    trap(FAULT_SOURCE_HARDFAULT, stackFrame);
}

__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile(
        "mrs r0, msp\n"
        "b FaultDiagnostics_captureHardFault\n");
}

void NMI_Handler(void)
{
    trap(FAULT_SOURCE_NMI, NULL);
}

void SVC_Handler(void)
{
    trap(FAULT_SOURCE_SVC, NULL);
}

void PendSV_Handler(void)
{
    trap(FAULT_SOURCE_PENDSV, NULL);
}

#define DEFINE_UNEXPECTED_HANDLER(name_, vector_) \
    void name_(void)                               \
    {                                               \
        trap(FAULT_SOURCE_IRQ_BASE + (vector_), NULL); \
    }

/* External IRQ indices follow the MSPM0G3507 startup vector table. */
DEFINE_UNEXPECTED_HANDLER(GROUP0_IRQHandler, 0U)
DEFINE_UNEXPECTED_HANDLER(GROUP1_IRQHandler, 1U)
DEFINE_UNEXPECTED_HANDLER(TIMG8_IRQHandler, 2U)
DEFINE_UNEXPECTED_HANDLER(UART3_IRQHandler, 3U)
DEFINE_UNEXPECTED_HANDLER(ADC0_IRQHandler, 4U)
DEFINE_UNEXPECTED_HANDLER(ADC1_IRQHandler, 5U)
DEFINE_UNEXPECTED_HANDLER(CANFD0_IRQHandler, 6U)
DEFINE_UNEXPECTED_HANDLER(DAC0_IRQHandler, 7U)
DEFINE_UNEXPECTED_HANDLER(SPI0_IRQHandler, 9U)
DEFINE_UNEXPECTED_HANDLER(SPI1_IRQHandler, 10U)
DEFINE_UNEXPECTED_HANDLER(UART1_IRQHandler, 13U)
/* UART2 (IRQ 14) is intentionally implemented by tracking_controller.c. */
DEFINE_UNEXPECTED_HANDLER(UART0_IRQHandler, 15U)
DEFINE_UNEXPECTED_HANDLER(TIMG0_IRQHandler, 16U)
DEFINE_UNEXPECTED_HANDLER(TIMG6_IRQHandler, 17U)
DEFINE_UNEXPECTED_HANDLER(TIMA0_IRQHandler, 18U)
DEFINE_UNEXPECTED_HANDLER(TIMA1_IRQHandler, 19U)
DEFINE_UNEXPECTED_HANDLER(TIMG7_IRQHandler, 20U)
DEFINE_UNEXPECTED_HANDLER(TIMG12_IRQHandler, 21U)
DEFINE_UNEXPECTED_HANDLER(I2C0_IRQHandler, 24U)
DEFINE_UNEXPECTED_HANDLER(I2C1_IRQHandler, 25U)
DEFINE_UNEXPECTED_HANDLER(AES_IRQHandler, 28U)
DEFINE_UNEXPECTED_HANDLER(RTC_IRQHandler, 30U)
DEFINE_UNEXPECTED_HANDLER(DMA_IRQHandler, 31U)
