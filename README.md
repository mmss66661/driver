# MSPM0G3507 二自由度激光云台视觉跟踪

本工程运行在 TI LP-MSPM0G3507 上，通过 UART 接收摄像头开发板发送的激光点
位置误差，并分别控制 Pitch、Yaw 两个步进电机，使激光点持续趋近矩形中心。

- MCLK：80 MHz（SYSOSC + SYSPLL）
- 电机 1：Pitch 轴
- 电机 2：Yaw 轴
- 两轴使用独立硬件 PWM，可以分别启停、分别调速
- 串口：UART2，230400 bit/s，8 数据位，无校验，1 停止位，16 倍过采样
- 控制方式：带死区的比例速度控制
- 通信中断超过 200 ms 没有收到有效帧时停止运动并保持电机力矩

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
| TX | PA24 / UART2_RX | 摄像头数据发送到 MSPM0 |
| RX | PA23 / UART2_TX | 当前程序未发送数据，但已保留双向通信能力 |
| GND | GND | 两块开发板必须共地 |

两块开发板之间交叉连接 TX/RX。不要把 RS-232 的正负电压直接接到 MCU；本工程
使用的是 3.3 V TTL UART。

PA12 同时可能连接到 LaunchPad 的 XDS110 UART 回传通道。本工程没有启用 UART0；
如果板载 XDS110 对 PA12 造成影响，需要断开 LaunchPad 对应的隔离跳线。

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

`empty.c` 的主循环只负责处理串口解析后的最新误差：

```c
int main(void)
{
    SYSCFG_DL_init();
    StepperMotor_init();
    TrackingController_init();

    while (1) {
        TrackingController_process();
        __WFE();
    }
}
```

UART2 中断逐字节解析数据帧，SysTick 提供 1 ms 通信超时计时。帧头、帧尾或 UART
错误不会直接驱动电机，只有完整且帧尾正确的 8 字节数据才会更新控制目标。

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

## 异常诊断

工程提供 `fault_diagnostics.c`，用于区分 HardFault、NMI 和意外外设中断。如果程序
因异常停止，CCS 会优先停在 `__BKPT(0)`；在 **Expressions** 窗口添加
`gFaultInfo` 和 `gStartupStage`：

| 字段 | 含义 |
| --- | --- |
| `gFaultInfo.magic` | 等于 `0x4641554C` 表示已经记录到有效故障 |
| `exceptionNumber` | ARM 当前异常号；2=NMI，3=HardFault，外设中断从 16 开始 |
| `source` | 软件识别到的故障来源，通常与异常号相同 |
| `startupStage` | 1=刚进入 main，2=SysConfig 完成，3=电机初始化完成，4=UART/SysTick 完成，5=正常主循环 |
| `stackedPC` | HardFault 发生时正在执行的指令地址 |
| `stackedLR` | HardFault 发生时的返回地址 |
| `icsr` | SCB 中断控制状态寄存器原值 |

将这些数值和 `Debug/driver.map` 一起查看，即可把 `stackedPC` 定位到具体函数。正常
运行时 `gFaultInfo.magic` 为 0。
