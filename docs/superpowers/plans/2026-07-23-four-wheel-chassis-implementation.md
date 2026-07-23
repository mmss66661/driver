# Four-Wheel Chassis Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Drive all four physical motors with the approved direction mapping and aggregate all four encoder values into the existing logical left/right chassis interface.

**Architecture:** Keep line following and navigation on the current forward-positive logical left/right API. Add a hardware-independent mapping helper for physical commands and wrap-safe encoder averaging, then use it at the Modbus boundary while retaining raw four-motor diagnostics in chassis status.

**Tech Stack:** C11 host unit test with MinGW GCC; TI MSPM0 DriverLib firmware; Modbus RTU over UART1; CCS generated makefile.

## Global Constraints

- Do not inspect or modify gimbal code.
- Do not edit generated SysConfig or generated build files manually.
- Preserve `ChassisMotor_setWheelSpeeds(left, right, nowMs)` and logical forward-positive side semantics.
- Transmit physical speeds as `[left, -right, -right, -left]`, saturating negated `INT16_MIN` to `INT16_MAX`.
- Configure encoder-polarity registers `0x0009..0x000C` to `[0, 1, 1, 1]` before closed loop.
- Keep the existing 250 ms command watchdog and make it stop all four motors.

---

### Task 1: Pure Four-Motor Mapping

**Files:**
- Create: `tests/chassis_motor_mapping_test.c`
- Create: `chassis_motor_mapping.h`

**Interfaces:**
- Consumes: logical `int16_t leftSpeed`, `int16_t rightSpeed`, and pairs of raw `int16_t` encoder counts.
- Produces: `ChassisMotor_PhysicalCommand`, `ChassisMotor_mapSideSpeeds()`, and `ChassisMotor_averageWrappedEncoder()`.

- [ ] **Step 1: Write the failing host test**

```c
#include <assert.h>
#include <limits.h>
#include <stdio.h>

#include "chassis_motor_mapping.h"

static void expectMapping(int16_t left, int16_t right,
    int16_t m0, int16_t m1, int16_t m2, int16_t m3)
{
    ChassisMotor_PhysicalCommand command;
    ChassisMotor_mapSideSpeeds(left, right, &command);
    assert(command.motor[0] == m0);
    assert(command.motor[1] == m1);
    assert(command.motor[2] == m2);
    assert(command.motor[3] == m3);
}

int main(void)
{
    expectMapping(10, 10, 10, -10, -10, -10);
    expectMapping(-10, -10, -10, 10, 10, 10);
    expectMapping(5, 15, 5, -15, -15, -5);
    expectMapping(0, 0, 0, 0, 0, 0);
    expectMapping(INT16_MIN, INT16_MIN,
        INT16_MIN, INT16_MAX, INT16_MAX, INT16_MAX);
    assert(ChassisMotor_averageWrappedEncoder(100, 104) == 102);
    assert(ChassisMotor_averageWrappedEncoder(-100, -104) == -102);
    assert(ChassisMotor_averageWrappedEncoder(32766, INT16_MIN) == 32767);
    assert(ChassisMotor_averageWrappedEncoder(INT16_MIN, 32766) == 32767);
    puts("chassis motor mapping tests passed");
    return 0;
}
```

- [ ] **Step 2: Run the test to verify RED**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -I. tests/chassis_motor_mapping_test.c -o tests/chassis_motor_mapping_test.exe
```

Expected: compilation fails because `chassis_motor_mapping.h` does not exist.

- [ ] **Step 3: Add the minimal mapping helper**

```c
#ifndef CHASSIS_MOTOR_MAPPING_H_
#define CHASSIS_MOTOR_MAPPING_H_

#include <stdint.h>

#define CHASSIS_PHYSICAL_MOTOR_COUNT (4U)

typedef struct {
    int16_t motor[CHASSIS_PHYSICAL_MOTOR_COUNT];
} ChassisMotor_PhysicalCommand;

static inline int16_t ChassisMotor_negateSaturated(int16_t value)
{
    return (value == INT16_MIN) ? INT16_MAX : (int16_t) -value;
}

static inline void ChassisMotor_mapSideSpeeds(int16_t left, int16_t right,
    ChassisMotor_PhysicalCommand *command)
{
    command->motor[0] = left;
    command->motor[1] = ChassisMotor_negateSaturated(right);
    command->motor[2] = ChassisMotor_negateSaturated(right);
    command->motor[3] = ChassisMotor_negateSaturated(left);
}

static inline int16_t ChassisMotor_averageWrappedEncoder(
    int16_t first, int16_t second)
{
    uint16_t difference = (uint16_t) second - (uint16_t) first;
    int32_t signedDifference = (difference <= INT16_MAX) ?
        (int32_t) difference : (int32_t) difference - 65536;
    int32_t midpoint = (int32_t) (uint16_t) first + signedDifference / 2;
    if (midpoint < 0) {
        midpoint += 65536;
    } else if (midpoint > UINT16_MAX) {
        midpoint -= 65536;
    }
    return (midpoint <= INT16_MAX) ? (int16_t) midpoint :
        (int16_t) (midpoint - 65536);
}

#endif
```

- [ ] **Step 4: Run the host test to verify GREEN**

Run the Step 2 command, then:

```powershell
.\tests\chassis_motor_mapping_test.exe
```

Expected: `chassis motor mapping tests passed`.

### Task 2: Modbus Integration And Status

**Files:**
- Modify: `chassis_motor.h`
- Modify: `chassis_motor.c`

**Interfaces:**
- Consumes: mapping interfaces from Task 1 and the existing 16-byte feedback payload.
- Produces: four-motor command frames, four physical command diagnostics, four physical encoder diagnostics, and compatible logical left/right encoder fields.

- [ ] **Step 1: Extend status without changing existing fields**

Add these arrays to `ChassisMotor_Status`:

```c
int16_t motorCommand[CHASSIS_PHYSICAL_MOTOR_COUNT];
int16_t motorEncoderCount[CHASSIS_PHYSICAL_MOTOR_COUNT];
```

Include `chassis_motor_mapping.h` from `chassis_motor.h` so the shared count and mapping type have one definition.

- [ ] **Step 2: Map all four speed registers**

In `sendSpeeds()`, call:

```c
ChassisMotor_PhysicalCommand command;
ChassisMotor_mapSideSpeeds(left, right, &command);
for (i = 0U; i < CHASSIS_PHYSICAL_MOTOR_COUNT; i++) {
    values[i] = (uint16_t) command.motor[i];
    gStatus.motorCommand[i] = command.motor[i];
}
```

Remove the old per-side inversion macros and the two zero-filled motor registers.

- [ ] **Step 3: Configure all four encoder directions**

Use exact polarity values `[0U, 1U, 1U, 1U]`. Write them in order to addresses `0x0009U + i`, preserving a 50 ms delay after each write except the last.

- [ ] **Step 4: Parse and aggregate four encoders**

Only update four-wheel odometry when at least four registers are present:

```c
for (i = 0U; i < CHASSIS_PHYSICAL_MOTOR_COUNT; i++) {
    gStatus.motorEncoderCount[i] = gStatus.feedback[i];
}
gStatus.leftEncoderCount = ChassisMotor_averageWrappedEncoder(
    gStatus.motorEncoderCount[0], gStatus.motorEncoderCount[3]);
gStatus.rightEncoderCount = ChassisMotor_averageWrappedEncoder(
    gStatus.motorEncoderCount[1], gStatus.motorEncoderCount[2]);
```

Initialize all new arrays to zero. When the watchdog replaces logical commands with zero, immediately clear all four physical command diagnostics before transmitting the zero frame.

- [ ] **Step 5: Build firmware**

Run:

```powershell
$env:LOCALAPPDATA='C:\Users\Administrator\workspace_ccstheia\driver\.sysconfig-local'
& 'E:\ccs\utils\bin\gmake.exe' -C Debug -j4 all
```

Expected: build completes and produces `Debug/driver.out` without compiler or linker errors.

### Task 3: Chassis Documentation And Final Verification

**Files:**
- Modify: `README.md`

**Interfaces:**
- Consumes: final four-wheel behavior from Tasks 1 and 2.
- Produces: wiring-independent direction and diagnostics guidance for bench testing.

- [ ] **Step 1: Replace stale two-wheel chassis descriptions**

Document the motor layout, physical mapping `[left, -right, -right, -left]`, polarity registers `[0, 1, 1, 1]`, side encoder averages `(motor0,motor3)` and `(motor1,motor2)`, and that the watchdog stops all four motors. Remove the statement that C/D are fixed to zero and replace stale `CHASSIS_*_INVERTED` tuning advice with wheels-raised verification guidance.

- [ ] **Step 2: Re-run automated checks**

Run the host test, the CCS build, and:

```powershell
git diff --check
```

Expected: host test passes, CCS build succeeds, and `git diff --check` reports no whitespace errors.

- [ ] **Step 3: Review scope and hardware caveat**

Confirm no gimbal source or generated SysConfig file was edited. Report that physical direction and all-positive forward encoder behavior still require a wheels-raised vehicle test because host and firmware builds cannot validate wiring or motor installation.
