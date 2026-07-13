# MSPM0G3507 二自由度激光云台视觉跟踪

本工程运行在 TI LP-MSPM0G3507 上，通过 UART 接收摄像头开发板发送的激光点
位置误差，并分别控制 Pitch、Yaw 两个步进电机，使激光点持续趋近矩形中心。

- MCLK：80 MHz（SYSOSC + SYSPLL）
- 电机 1：Pitch 轴
- 电机 2：Yaw 轴
- 两轴使用独立硬件 PWM，可以分别启停、分别调速
- 串口：UART0，230400 bit/s，8 数据位，无校验，1 停止位，16 倍过采样
- 控制方式：带死区的比例速度控制
- 通信中断超过 200 ms 没有收到有效帧时停止运动并保持电机力矩
- 12 路数字巡线输入、128×64 OLED、IMU601 和 3 个脱机按键已经接入

## 接线

### 步进电机驱动器

| 信号 | MSPM0G3507 引脚 | 说明 |
| --- | --- | --- |
| Pitch STEP | PA12 / TIMG0_CCP0 | 电机 1，5 us 高电平脉冲 |
| Pitch DIR | PA16 | 电机 1 方向 |
| Pitch ENABLE/SLEEP | PA18 | 高电平使能，低电平休眠 |
| Yaw STEP | PB6 / TIMG6_CCP0 | 电机 2，5 us 高电平脉冲 |
| Yaw DIR | PA17 | 电机 2 方向 |
| Yaw ENABLE/SLEEP | PA15 | 高电平使能，低电平休眠 |
| Driver GND | LaunchPad GND | 必须与控制板共地 |

STEP/DIR/ENABLE 只能连接 A4988、DRV8825、TMC、TB6600 等外部驱动器的逻辑
输入，不能把电机线圈直接连接到 MSPM0G3507 GPIO。还需要根据电机额定电流设置
驱动器限流，并确认其数字输入兼容 3.3 V。

### 摄像头开发板串口

| 摄像头开发板 | MSPM0G3507 | 说明 |
| --- | --- | --- |
| TX | PA31 / UART0_RX | 摄像头数据发送到 MSPM0 |
| RX | PA28 / UART0_TX | 下位机测试信息或后续双向通信 |
| GND | GND | 两块开发板必须共地 |

两块开发板之间交叉连接 TX/RX。不要把 RS-232 的正负电压直接接到 MCU；本工程
使用的是 3.3 V TTL UART。

本工程 UART0 使用 PA28/PA31，PA12 仅作为 Pitch STEP 的定时器输出。

### 12 路巡线模块

巡线模块的数字输出接线如下。所有输入均未启用内部上下拉，要求模块输出稳定的
3.3 V 逻辑电平；不要向 MSPM0 GPIO 输入 5 V。

| 巡线通道 | MSPM0G3507 | `LineSensor_readRaw()` 位 |
| --- | --- | --- |
| LINE0 | PB0 | bit 0 |
| LINE1 | PB1 | bit 1 |
| LINE2 | PB4 | bit 2 |
| LINE3 | PB5 | bit 3 |
| LINE4 | PB7 | bit 4 |
| LINE5 | PB8 | bit 5 |
| LINE6 | PB9 | bit 6 |
| LINE7 | PB10 | bit 7 |
| LINE8 | PB11 | bit 8 |
| LINE9 | PB12 | bit 9 |
| LINE10 | PB13 | bit 10 |
| LINE11 | PB14 | bit 11 |

黑线输出高电平，对应位为 `1`；白色背景输出低电平，对应位为 `0`。例如返回
`0x801` 表示 LINE11 和 LINE0 检测到黑线。`line_sensor.c` 还提供单路读取和黑线
通道计数函数。

### 128×64 OLED

| OLED | MSPM0G3507 | 配置 |
| --- | --- | --- |
| SCL | PB2 / I2C1_SCL | 400 kHz |
| SDA | PB3 / I2C1_SDA | 400 kHz |
| VCC | 按模块规格接 3.3 V 或 5 V | I²C 电平必须为 3.3 V |
| GND | GND | 必须共地 |

驱动按 SSD1306、7 位地址 `0x3C` 实现。SysConfig 已启用内部上拉，但实际接线建议
保留模块自带或外接的 4.7 kΩ 左右上拉。驱动带传输错误和超时退出：OLED 未接时
不会永久卡死程序。状态页每 250 ms 刷新一次。

### 汇电籽 IMU601

| IMU601 | MSPM0G3507 | 说明 |
| --- | --- | --- |
| T / TX | PA25 / UART3_RX | IMU 发给 MSPM0 |
| R / RX | PA26 / UART3_TX | MSPM0 发给 IMU |
| V | 5 V（按参考模块要求） | 先确认具体模块供电规格 |
| G | GND | 与控制板共地 |

UART3 参数为 115200 bit/s、8N1、16 倍过采样。初始化时依次发送参考工程中的复位
命令和校准命令。接收帧固定 12 字节，以 `AA 55` 开头，校验为第 2～10 字节的
8 位累加和，第 11 字节为校验值。Yaw、Pitch、Roll 从第 5 字节开始按小端解析，
单位为 0.01°；Yaw 为无符号值，Pitch/Roll 为有符号值。新的接收状态机支持帧头
重同步并严格限制数组索引。

### 脱机按键

| 按键 | MSPM0G3507 | 电气与功能 |
| --- | --- | --- |
| MODE | PA3 | 接地按下；切换跟踪 RUN/HOLD |
| UP | PA4 | 接地按下；切换到下一 OLED 页面 |
| DOWN | PA5 | 接地按下；切换到上一 OLED 页面 |

三个输入均启用内部上拉并使用 20 ms 软件消抖。HOLD 模式停止两轴 STEP，但保持
驱动器 ENABLE 和锁定力矩；再次按 MODE 恢复接收摄像头误差并跟踪。

另外预留 5 个不参与当前程序逻辑的调试按键输入：

| 名称 | MSPM0G3507 | 配置 |
| --- | --- | --- |
| KEY1 | PA6 | 内部上拉，接地按下 |
| KEY2 | PA7 | 内部上拉，接地按下 |
| KEY3 | PA10 | 内部上拉，接地按下 |
| KEY4 | PA11 | 内部上拉，接地按下 |
| KEY5 | PA13 | 内部上拉，接地按下 |

### 两轮差速底盘电机板

底盘电机板使用独立 UART1，不占用摄像头 UART0 和 IMU601 UART3：

| 电机驱动板 | MSPM0G3507 | 配置 |
| --- | --- | --- |
| RX | PA8 / UART1_TX | MSPM0 向驱动板发送命令 |
| TX | PA9 / UART1_RX | 驱动板向 MSPM0 返回数据 |
| GND | GND | 必须共地 |

串口参数为 115200 bit/s、8N1、16 倍过采样，协议为参考例程使用的 Modbus RTU，
从站地址 `0x0A`，CRC16 低字节先发送。启动时执行：

1. 向寄存器 `0x0008` 写 `1`，进入速度闭环；
2. 配置 A、B 两路编码器方向寄存器 `0x0009`、`0x000A`；
3. 发送左右轮零速，C、D 两路始终写 `0`。

当前映射是电机 A=左轮、电机 B=右轮。差速接口计算公式为：

```text
left  = forward - turn
right = forward + turn
```

因此 `turn > 0` 时右轮更快，实际向左还是向右取决于电机安装、接线和正方向定义。
首次悬空测试若前进命令使某一轮反转，可修改 `chassis_motor.c` 顶部的
`CHASSIS_LEFT_MOTOR_INVERTED` 或 `CHASSIS_RIGHT_MOTOR_INVERTED`。

上层必须至少每 250 ms 刷新一次非零速度命令。超过时间后驱动自动把 A/B 写为 0，
并在 OLED 第 3 页显示 `MOTOR:TIMEOUT`。这项保护只能防止 MSPM0 上层逻辑停止更新，
不能替代驱动板自身的通信失联保护和硬件急停。

## 坐标定义

摄像头识别到的矩形中心是坐标原点：

```text
                 -Y（上）
                    |
                    |
       -X（左） ----+---- +X（右）
                    |
                    |
                 +Y（下）
```

`errX`、`errY` 表示激光点相对于矩形中心的位置：

- `errX > 0`：激光点在中心右侧；
- `errX < 0`：激光点在中心左侧；
- `errY > 0`：激光点在中心下方；
- `errY < 0`：激光点在中心上方；
- `(0, 0)`：激光点位于矩形中心。

## 串口帧协议

一帧固定为 8 字节：

| 字节序号 | 内容 | 示例 |
| --- | --- | --- |
| 0 | 帧头 1 | `0xFC` |
| 1 | 帧头 2 | `0xCF` |
| 2 | `errX` 高字节 | `X[15:8]` |
| 3 | `errX` 低字节 | `X[7:0]` |
| 4 | `errY` 高字节 | `Y[15:8]` |
| 5 | `errY` 低字节 | `Y[7:0]` |
| 6 | 帧尾 1 | `0xCF` |
| 7 | 帧尾 2 | `0xFC` |

也就是：

```text
FC CF X_H X_L Y_H Y_L CF FC
```

虽然线上字段是两个 `uint16_t`，程序会把它们按 `int16_t` 二进制补码解释，以便
表达负坐标。当前协议采用大端字节序（高字节先发送）。摄像头端可使用：

```c
uint8_t frame[8];
int16_t errX;
int16_t errY;

frame[0] = 0xFC;
frame[1] = 0xCF;
frame[2] = (uint8_t) ((uint16_t) errX >> 8);
frame[3] = (uint8_t) errX;
frame[4] = (uint8_t) ((uint16_t) errY >> 8);
frame[5] = (uint8_t) errY;
frame[6] = 0xCF;
frame[7] = 0xFC;
```

示例：`errX = 100`、`errY = -20` 时，数据帧为：

```text
FC CF 00 64 FF EC CF FC
```

如果上位机目前发送低字节在前，需要同时修改
`tracking_controller.c` 中 `rawX`、`rawY` 的组合顺序。

## 控制逻辑

每收到一个合法数据帧，程序分别计算两轴命令：

- `errY` 控制电机 1（Pitch）；
- `errX` 控制电机 2（Yaw）；
- 误差方向决定 DIR；
- 误差绝对值决定 STEP 频率；
- 误差越大，电机转得越快；
- 进入中心死区后停止 STEP，但 ENABLE 保持为高，不丢失云台保持力矩。

默认速度计算为：

```text
|error| <= 3 px：停止该轴 STEP，保持力矩
|error| >  3 px：rate = 200 + (|error| - 3) × 15 steps/s
最大速度限制：4000 steps/s
```

该控制器是比例速度控制，不需要预先知道目标的绝对角度。摄像头持续发送误差，
MSPM0 持续修正，直至激光点进入中心死区。

如果超过 200 ms 没有收到合法帧，两个电机都会停止 STEP，但仍保持使能，防止
Pitch 轴因重力下垂。若希望通信丢失时释放电机，可把
`TrackingController_process()` 超时分支里的 `StepperMotor_hold()` 改成
`StepperMotor_stop()`。

## 首次上电方向校准

电机线序、减速机构和驱动器 DIR 定义不同，无法仅由软件提前确定实际正反方向。
第一次测试时应降低驱动电流，并保证云台不会撞到机械限位。

1. 将激光点放在矩形中心右侧，即让摄像头发送 `errX > 0`；
2. 观察 Yaw 轴动作后误差绝对值是否减小；
3. 如果 `|errX|` 增大，把 `tracking_controller.c` 中
   `YAW_DIRECTION_INVERTED` 从 `0` 改为 `1`；
4. 将激光点放在矩形中心下方，即发送 `errY > 0`；
5. 如果 Pitch 动作后 `|errY|` 增大，把 `PITCH_DIRECTION_INVERTED` 从 `0`
   改为 `1`。

方向错误时必须立即停止测试，否则闭环会持续朝错误方向加速。

## 控制参数调节

控制参数位于 `tracking_controller.c` 文件顶部：

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `TRACKING_DEAD_BAND` | 3 px | 中心附近不动作的范围，过小容易抖动 |
| `TRACKING_MIN_RATE` | 200 steps/s | 克服电机静摩擦的最低运行速度 |
| `TRACKING_MAX_RATE` | 4000 steps/s | 限制最高速度，防止失步和撞限位 |
| `TRACKING_KP_STEPS_PER_PX` | 15 | 每增加一个像素误差，增加多少 steps/s |
| `TRACKING_TIMEOUT_MS` | 200 ms | 串口数据丢失保护时间 |

推荐调试顺序：

1. 先确认两个轴的方向正确；
2. 把最大速度临时降到 500～1000 steps/s；
3. 从较小的比例系数开始逐渐增加；
4. 如果中心附近来回振荡，增大死区或减小比例系数；
5. 如果大误差时跟踪太慢，增加最大速度；
6. 如果电机啸叫、堵转或丢步，降低最大速度并检查驱动电流和机械负载。

当前工程没有机械限位输入。正式运行前建议为 Pitch/Yaw 增加限位开关或软件角度
边界，否则识别错误或方向配置错误可能使云台撞击机械结构。

## 程序入口

`empty.c` 在初始化双步进电机和视觉控制器之后，还会初始化巡线、按键、IMU601
和 OLED。主循环持续处理视觉误差与按键，并按固定周期刷新状态页：

```c
int main(void)
{
    SYSCFG_DL_init();
    StepperMotor_init();
    TrackingController_init();
    LineSensor_init();
    Buttons_init();
    IMU601_init();
    OLED_Init();

    while (1) {
        TrackingController_process();
        Buttons_process(TrackingController_getMilliseconds());
        /* 处理按键事件，并每 250 ms 更新一次 OLED。 */
        __WFE();
    }
}
```

UART0 中断逐字节解析数据帧，SysTick 提供 1 ms 通信超时计时。帧头、帧尾或 UART
错误不会直接驱动电机，只有完整且帧尾正确的 8 字节数据才会更新控制目标。UART3
中断则独立接收 IMU601 数据，不会与摄像头 UART0 共用解析状态。

### UART0 发送测试

`empty.c` 中保留 UART 自检代码，但当前默认将 `UART_TX_TEST_ENABLE` 设为 `0U`，
避免调试字符串干扰摄像头。临时改为 `1U` 后，初始化完成会立即从 PA28
（UART0_TX）发送 `UART0 TX START`，之后主循环每 500 ms 发送一次：

```text
UART0 TX OK\r\n
```

测试时将 USB-TTL 的 RX 接 PA28、GND 接开发板 GND，串口工具设置为
230400、8N1，并使用文本接收模式。能稳定看到该字符串，说明 MSPM0 的 UART0 TX、
波特率和基本接线正常。测试完成后应恢复为 `0U`，避免调试字符串发送给摄像头：

```c
#define UART_TX_TEST_ENABLE (0U)
```

如果 UART0 收到并成功解析了一帧正确的 `FC CF ... CF FC` 数据，还会返回：

```text
UART0 RX FRAME OK\r\n
```

因此：只看到 `UART0 TX OK` 表示下位机发送正常，但尚未收到合法帧；同时看到
`UART0 RX FRAME OK` 才表示 PA31 接收、串口参数和帧格式也都正确。

## 新增外设 API

### 两轮底盘

- `ChassisMotor_init()`：开启电机板闭环、配置 A/B 编码器方向并发送零速；
- `ChassisMotor_process(nowMs)`：每 20 ms 刷新速度，并执行 250 ms 命令看门狗；
- `ChassisMotor_setWheelSpeeds(left, right, nowMs)`：直接设置左右轮有符号速度；
- `ChassisMotor_setDifferential(forward, turn, nowMs)`：用前进量和转向量进行两轮差速混控；
- `ChassisMotor_stop(nowMs)`：立即把目标速度设为零；
- `ChassisMotor_setPid(...)`：按参考协议写入 A/B 两路 PID，参数乘 1000 后发送；
- `ChassisMotor_getStatus()`：读取命令、8 个反馈寄存器和通信错误计数。

电机板参考工程只给出了 `0x03` 反馈解析，没有给出主动读取命令或 8 个反馈寄存器
的地址/含义。本工程因此接收并保存驱动板主动返回的 `0x03` 数据，但没有猜测未知
寄存器地址去轮询。拿到驱动板完整寄存器表后，应补充主动反馈查询和字段命名。

### 巡线

- `LineSensor_init()`：保留统一初始化入口，实际 GPIO 模式由 SysConfig 配置；
- `LineSensor_readRaw()`：返回 12 位原始值，bit 0～bit 11 对应 LINE0～LINE11；
- `LineSensor_read(index)`：读取指定一路，越界时返回 0；
- `LineSensor_countBlack()`：统计当前高电平（黑线）通道数。

### IMU601

- `IMU601_init()`：复位模块、发送校准命令并开启 UART3 中断；
- `IMU601_getAttitude()`：原子复制最新姿态、有效帧计数和校验错误计数；返回值表示
  是否至少收到过一个有效帧。

为了避免工具链的浮点格式化开销，姿态结构体保存的是百分之一度整数：
`yawCentiDegrees`、`pitchCentiDegrees`、`rollCentiDegrees`。

### 按键

- `Buttons_process(nowMs)`：应在主循环高频调用，执行 20 ms 消抖；
- `Buttons_wasPressed(id)`：读取并清除一次“刚按下”事件；
- `Buttons_isDown(id)`：读取稳定的持续按下状态。

### OLED

- `OLED_Init()`：初始化 SSD1306，并返回 I²C 通信是否成功；
- `OLED_Clear()`：只清空 RAM 显存；
- `OLED_ShowString(x, y, text)`：向显存写入 5×7 ASCII 状态文字；
- `OLED_Refresh()`：把显存发到屏幕；
- `OLED_IsPresent()`：判断初始化/最近一次刷新是否成功。

## 步进电机 API

### 电机选择

| 参数 | 对象 |
| --- | --- |
| `STEPPER_MOTOR_1` | Pitch 电机 |
| `STEPPER_MOTOR_2` | Yaw 电机 |
| `STEPPER_MOTOR_BOTH` | 两台电机 |

### 主要函数

- `StepperMotor_init()`：初始化两路电机，默认关闭 STEP 和使能；
- `StepperMotor_start(motor)`：使能指定驱动器，等待 1 ms，再开始 STEP；
- `StepperMotor_hold(motor)`：停止 STEP，但保持驱动器使能和锁定力矩；
- `StepperMotor_stop(motor)`：停止 STEP，并拉低 ENABLE 进入休眠；
- `StepperMotor_setDirection(motor, direction)`：安全修改方向；
- `StepperMotor_toggleDirection(motor)`：翻转当前方向；
- `StepperMotor_setMotorStepRate(motor, rate)`：设置单轴独立速度；
- `StepperMotor_getMotorStepRate(motor)`：读取单轴实际速度；
- `StepperMotor_setStepRate(rate)`：兼容接口，把两轴设置为同一速度；
- `StepperMotor_getStepRate()`：兼容接口，返回 Pitch 轴速度。

单轴速度有效范围为 100～20000 steps/s。由于定时器使用整数分频，实际速度可能
与请求值有很小的取整误差。

## 转速换算

如果电机是 200 整步/圈，驱动器设置为 16 细分，则一圈需要：

```text
200 × 16 = 3200 STEP/圈
```

当 STEP 频率为 1600 steps/s 时：

```text
1600 / 3200 = 0.5 圈/秒 = 30 RPM
```

## 工程审查：漏洞与待完善项

### 本轮已经修复或规避

- 底盘命令增加 250 ms 软件看门狗，防止控制循环卡住后持续保持非零速度；
- Modbus 接收限制帧长和寄存器数量，并校验 CRC，避免参考解析器失步后数组越界；
- C/D 两路未安装电机，所有速度帧都固定写 0，避免无关输出误动作；
- OLED I²C 有错误和超时退出，屏幕未接时不会永久阻塞在等待状态；
- 摄像头、IMU601、底盘分别使用 UART0、UART3、UART1，ISR 和解析缓冲区互相独立；
- UART0 测试字符串默认关闭，避免调试输出混入摄像头通信；
- 电机失联、IMU 校验错误、摄像头非法帧均有可读取的错误计数。

### 上车前必须确认

1. 悬空标定左右轮正方向和编码器方向。方向错误会让闭环速度异常，修改
   `CHASSIS_*_INVERTED`/`CHASSIS_*_ENCODER_REVERSED` 后再落地测试；
2. 确认驱动板串口是 3.3 V TTL。如果接口实际为 RS-485 或 RS-232，必须增加对应
   收发器，不能直接连接 MSPM0；
3. 确认速度寄存器单位、允许范围、PID 合法范围以及 8 个反馈寄存器的真实含义；
4. 确认电机板自身也有通信超时停车。MSPM0 软件看门狗无法处理 MCU 死机、断电或
   TX 线断开的情况；正式车辆建议增加硬件急停；
5. 云台仍没有机械限位，必须增加 Pitch/Yaw 限位开关或可靠的软件角度边界；
6. 巡线模块、UART 和按键只能输入 3.3 V 逻辑，所有模块必须共地。

### 后续功能缺口

- 尚未获得电机板完整寄存器表，因此没有主动轮速/累计编码器查询和反馈超时判断；
- 尚未填写轮径、轮距、减速比、编码器线数，不能计算 m/s、里程和航向里程计；
- 尚未实现巡线控制器、IMU 航向融合或底盘速度闭环的上层状态机；
- MODE 当前只切换云台视觉跟踪，尚未定义它是否同时停止底盘；
- IMU601 每次启动都会执行参考例程的复位和校准命令。若模块安装后不允许每次上电
  自动校准，应按协议改为按键触发或仅首次标定；
- OLED 刷新和 UART 发帧仍是阻塞式操作。当前数据量可以工作，但后续控制周期提高
  时应改用 FIFO 中断或 DMA；
- OLED 初始化失败后本次运行不会自动热插拔重试，可增加低频重新初始化；
- 当前控制器没有电池电压、电机过流、堵转、倾覆检测和故障状态总线；
- 缺少在真实硬件上的协议抓包、轮子悬空方向测试、急停测试和长时间稳定性测试。
