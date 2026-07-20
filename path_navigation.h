#ifndef PATH_NAVIGATION_H_
#define PATH_NAVIGATION_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    PATH_NAV_IDLE = 0,
    PATH_NAV_RECORDING,
    PATH_NAV_REPLAYING,
    PATH_NAV_COMPLETE,
    PATH_NAV_ERROR
} PathNavigation_Mode;

typedef enum {
    PATH_NAV_ERROR_NONE = 0,
    PATH_NAV_ERROR_NO_PATH,
    PATH_NAV_ERROR_SENSORS,
    PATH_NAV_ERROR_DRIVE_ACTIVE,
    PATH_NAV_ERROR_PATH_SHORT,
    PATH_NAV_ERROR_PATH_FULL,
    PATH_NAV_ERROR_FLASH,
    PATH_NAV_ERROR_LINE_LOST
} PathNavigation_Error;

typedef struct {
    PathNavigation_Mode mode;
    PathNavigation_Error error;
    bool pathValid;
    uint16_t pointCount;
    uint16_t currentPoint;
    uint32_t distanceTicks;
    uint32_t segmentDistanceTicks;
    uint32_t segmentTargetTicks;
    uint32_t cornerAdvanceTicks;
    int32_t headingCentiDegrees;
    int32_t targetHeadingCentiDegrees;
    int32_t headingErrorCentiDegrees;
    uint8_t lineActiveCount;
    int16_t lineErrorTenths;
    int16_t lineTurnCommand;
    int16_t navigationTurnCommand;
    int16_t fusedTurnCommand;
    uint32_t lineLostMilliseconds;
    bool lineDetected;
    bool intersectionDetected;
} PathNavigation_Status;

void PathNavigation_init(void);
bool PathNavigation_startRecording(uint32_t nowMs);
bool PathNavigation_finishRecording(uint32_t nowMs);
bool PathNavigation_startReplay(uint32_t nowMs);
void PathNavigation_stop(uint32_t nowMs);
void PathNavigation_process(uint32_t nowMs);
bool PathNavigation_hasPath(void);
bool PathNavigation_isActive(void);
void PathNavigation_getStatus(PathNavigation_Status *status);

#endif
