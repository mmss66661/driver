#ifndef FAULT_DIAGNOSTICS_H_
#define FAULT_DIAGNOSTICS_H_

#include <stdint.h>

#define FAULT_DIAGNOSTICS_MAGIC (0x4641554CUL) /* "FAUL" */

typedef struct {
    uint32_t magic;
    uint32_t source;
    uint32_t exceptionNumber;
    uint32_t startupStage;
    uint32_t stackedPC;
    uint32_t stackedLR;
    uint32_t stackedXPSR;
    uint32_t icsr;
} FaultDiagnostics_Info;

/* Inspect these two globals in CCS Expressions after a fault. */
extern volatile FaultDiagnostics_Info gFaultInfo;
extern volatile uint32_t gStartupStage;

void FaultDiagnostics_setStage(uint32_t stage);

#endif /* FAULT_DIAGNOSTICS_H_ */
