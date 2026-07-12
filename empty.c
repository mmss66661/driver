#include "ti_msp_dl_config.h"

#include "fault_diagnostics.h"
#include "stepper_motor.h"
#include "tracking_controller.h"

int main(void)
{
    FaultDiagnostics_setStage(1U); /* Entered main. */
    SYSCFG_DL_init();
    FaultDiagnostics_setStage(2U); /* Clock/GPIO/peripherals initialized. */
    StepperMotor_init();
    FaultDiagnostics_setStage(3U); /* Both motor drivers initialized. */
    TrackingController_init();
    FaultDiagnostics_setStage(4U); /* UART IRQ and SysTick enabled. */

    while (1) {
        FaultDiagnostics_setStage(5U); /* Normal tracking loop. */
        TrackingController_process();
        __WFE();
    }
}
