# Four-Wheel Chassis Mapping Design

## Scope

Convert the existing two-wheel logical chassis interface to drive four physical motors through the existing Modbus RTU motor board. Gimbal code is out of scope and remains untouched.

The upper layers continue to use two logical commands:

- `leftSpeed`: requested forward-positive speed for the left side.
- `rightSpeed`: requested forward-positive speed for the right side.

This preserves the interfaces used by line following and square-path navigation.

## Physical Motor Mapping

The motor board speed registers are ordered as motor 0 through motor 3. The vehicle layout and observed positive-command directions are:

| Board motor | Vehicle position | Positive command moves vehicle wheel |
| --- | --- | --- |
| 0 | Left rear | Forward |
| 1 | Right rear | Backward |
| 2 | Right front | Backward |
| 3 | Left front | Backward |

The four transmitted speed values therefore are:

```text
motor0 =  leftSpeed
motor1 = -rightSpeed
motor2 = -rightSpeed
motor3 = -leftSpeed
```

Negation must handle `INT16_MIN` without signed overflow.

## Encoder Direction

The motor board encoder-polarity registers `0x0009` through `0x000C` will be configured as:

```text
motor0: 0
motor1: 1
motor2: 1
motor3: 1
```

The intended invariant is that rolling the complete vehicle forward makes all four configured encoder values increase. This keeps command and odometry signs consistent.

## Feedback Aggregation

The first four registers in each 16-byte feedback payload are treated as the cumulative encoders for motors 0 through 3. The public status will retain all four physical encoder values for diagnostics.

The existing logical odometry fields remain available:

```text
leftEncoderCount  = average(motor0, motor3)
rightEncoderCount = average(motor1, motor2)
```

The average must avoid arithmetic overflow, preserve negative values, and remain continuous when one 16-bit encoder has just wrapped from `32767` to `-32768` before the other. It will use the signed modular delta between the pair rather than a plain integer sum. These logical counts keep the existing navigation code compatible while using both motors on each side.

## Commands And Status

`ChassisMotor_setWheelSpeeds(left, right, nowMs)` remains the primary logical interface. The status continues to expose `leftCommand` and `rightCommand` as forward-positive logical side commands, not raw motor-register values.

Four physical motor command and encoder fields will be added to `ChassisMotor_Status` for debugger and diagnostic inspection. Existing callers that only use logical left/right fields remain source-compatible.

## Safety And Error Handling

- Zero logical speed sends zero to all four motors.
- The existing 250 ms command watchdog stops all four motors.
- Entering closed loop remains unchanged and occurs only on the first nonzero command.
- All four encoder-polarity writes occur while the driver is still outside closed loop, with the existing delay between writes.
- Feedback frames with fewer than four registers are not used to update four-wheel odometry.
- CRC, frame-length, UART-error, and timeout handling remain unchanged.

## Alternatives Considered

1. **Recommended: configure board encoder polarity and average by side.** This makes raw feedback forward-positive and keeps navigation simple.
2. Normalize encoder signs only in MSPM0 software. This avoids board configuration writes but makes raw diagnostics inconsistent with vehicle direction.
3. Use only one encoder per side. This is the smallest change but wastes the front encoders and hides a stalled or slipping motor.

The approved design uses option 1.

## Validation

1. Build with the existing CCS generated makefile without editing generated SysConfig files.
2. Verify a logical forward command produces raw values `[+, -, -, -]` in the transmitted register order.
3. Verify logical left/right turning still maps through `forward - turn` and `forward + turn` before four-motor expansion.
4. With wheels raised, command a small positive forward speed and confirm all four wheels move the vehicle-forward direction.
5. Manually roll the vehicle forward and confirm all four physical encoder fields increase and both logical side averages increase.
6. Stop refreshing commands and confirm the watchdog sends zero to all four motors.
