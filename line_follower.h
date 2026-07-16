#ifndef LINE_FOLLOWER_H_
#define LINE_FOLLOWER_H_

#include <stdbool.h>
#include <stdint.h>

void LineFollower_init(void);
void LineFollower_process(uint32_t nowMs);
void LineFollower_setEnabled(bool enabled, uint32_t nowMs);
bool LineFollower_isEnabled(void);

#endif
