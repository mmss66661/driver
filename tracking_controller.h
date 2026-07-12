#ifndef TRACKING_CONTROLLER_H_
#define TRACKING_CONTROLLER_H_

#include <stdint.h>

typedef struct {
    int16_t errX;
    int16_t errY;
    uint32_t validFrameCount;
    uint32_t invalidFrameCount;
} TrackingController_Status;

void TrackingController_init(void);
void TrackingController_process(void);
void TrackingController_getStatus(TrackingController_Status *status);

#endif /* TRACKING_CONTROLLER_H_ */
