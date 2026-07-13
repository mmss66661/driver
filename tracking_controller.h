#ifndef TRACKING_CONTROLLER_H_
#define TRACKING_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t errX;
    int16_t errY;
    uint32_t validFrameCount;
    uint32_t invalidFrameCount;
} TrackingController_Status;

void TrackingController_init(void);
void TrackingController_process(void);
void TrackingController_setEnabled(bool enabled);
bool TrackingController_isEnabled(void);
void TrackingController_getStatus(TrackingController_Status *status);
uint32_t TrackingController_getMilliseconds(void);

#endif /* TRACKING_CONTROLLER_H_ */
