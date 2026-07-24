# 循迹与 PID 提取说明

提取来源：`C:\Users\l2025\workspace_ccstheia\2024H`

目标工程：`E:\DIANSAI resources\26DIANSAI\DIANSAI_26`

## 已放入目标工程的文件

- `pid.c` / `pid.h`：通用 PID，包含位置式和增量式 PID。
- `sensor.c` / `sensor.h`：8 路灰度/红外传感器读取，3 位地址选择 + 1 路 OUT 读取。
- `motor.c` / `motor.h`：TB6612 类双路电机方向控制 + PWM 输出。
- `encoder.c` / `encoder.h`：左右轮编码器 GPIO 中断计数，10ms 速度与里程计算。
- `line_follow.c` / `line_follow.h`：最小可用循迹控制，把传感器误差、循迹 PID、速度 PID、电机输出串起来。
- `empty.c`：已改成最小启动入口，初始化后自动开始循迹，并在 10ms 定时器中调用 `LineTrack_Loop_10ms()`。
- `empty.syscfg`：已加入上述代码依赖的传感器、电机、编码器、PWM、10ms 定时器配置。
- `original_2024H_reference_not_compiled/`：原 2024H 相关文件备份，全部改成 `.txt` 后缀，不参与 CCS 编译。

## 最小调用流程

```c
SYSCFG_DL_init();
LineTrack_Init();

NVIC_SetPriority(GPIOB_INT_IRQn, 0);
NVIC_SetPriority(TIMER_0_INST_INT_IRQN, 2);
__enable_irq();

LineTrack_Start(12.0f);
DL_Timer_startCounter(TIMER_0_INST);
NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
```

10ms 定时中断中调用：

```c
void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            LineTrack_Loop_10ms();
            break;
        default:
            break;
    }
}
```

## 调参入口

主要改 `line_follow.c` 的 `LineTrack_Init()`：

```c
PID_Init(&pid_line, 0.7f, 0.05f, 0.5f, 10.0f, -10.0f, 8.0f);
PID_Init(&pid_speed_L, 80.0f, 5.0f, 0.5f, 2000.0f, 0.0f, 1000.0f);
PID_Init(&pid_speed_R, 80.0f, 5.0f, 0.5f, 2000.0f, 0.0f, 1000.0f);
```

`LineTrack_Start(12.0f)` 里的 `12.0f` 是基础速度，单位沿用原工程的“编码器脉冲/10ms”速度量纲，不是 cm/s。

## 当前引脚映射

| 功能 | 引脚 |
| --- | --- |
| 传感器 AD0/AD1/AD2 | PA25 / PA26 / PA27 |
| 传感器 OUT | PB20 |
| 左电机方向 AIN1/AIN2 | PA29 / PB26 |
| 右电机方向 BIN1/BIN2 | PB23 / PB27 |
| PWM 左/右 | PA13 / PA12 |
| 左编码器 A/B | PB7 / PB9 |
| 右编码器 A/B | PB6 / PB8 |
| 控制周期定时器 | TIMER_0，10ms |

## 和原工程相比的取舍

原 `control.c` 里混合了 2024H 赛题任务状态机、MPU6050、OLED、蓝牙、蜂鸣器和按键逻辑。这里没有直接复制它，而是提取出独立循迹模块，避免 empty 工程被不必要依赖卡住。

保留的控制链路：

```text
Sensor_Read_All -> Sensor_Get_Error -> pid_line -> 左右轮目标速度
编码器中断计数 -> Encoder_UpdateData_10ms -> pid_speed_L/R -> 电机 PWM
```

## 需要现场确认的点

- 灰度模块输出逻辑：代码默认 `1` 表示检测到黑线。如果你的模块黑线输出低电平，改 `sensor.c` 里的 `LINE_DETECTED` / `LINE_UNDETECTED`。
- 传感器左右方向：当前权重为 `{30,20,10,5,-5,-10,-20,-30}`。如果车偏线后修正方向反了，把权重整体取反，或交换左右电机差速符号。
- 编码器方向：如果前进时某侧速度为负，改 `encoder.c` 对应轮的 `left_pulse_count++/--` 或 `right_pulse_count++/--`。
- 电机方向：如果某个轮反转，交换 `motor.c` 中该轮的方向引脚置位逻辑，或调整电机接线。
