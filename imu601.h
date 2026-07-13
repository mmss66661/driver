#ifndef IMU601_H_
#define IMU601_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t yawCentiDegrees;
    int16_t pitchCentiDegrees;
    int16_t rollCentiDegrees;
    uint32_t validFrameCount;
    uint32_t checksumErrorCount;
} IMU601_Attitude;

void IMU601_init(void);
bool IMU601_getAttitude(IMU601_Attitude *attitude);

#endif
