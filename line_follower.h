#ifndef LINE_FOLLOWER_H_
#define LINE_FOLLOWER_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    LINE_FOLLOW_LOST = 0,
    LINE_FOLLOW_TRACK,
    LINE_FOLLOW_INTERSECTION
} LineFollower_LineState;

typedef struct {
    uint16_t rawSensors;
    uint8_t activeCount;
    int16_t errorTenths;
    int16_t turnCommand;
    LineFollower_LineState state;
} LineFollower_Observation;

void LineFollower_init(void);
void LineFollower_process(uint32_t nowMs);
void LineFollower_setEnabled(bool enabled, uint32_t nowMs);
bool LineFollower_isEnabled(void);
void LineFollower_getObservation(LineFollower_Observation *observation);

#endif
