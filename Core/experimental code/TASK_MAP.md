/**
 * @file    TASK_MAP.md
 * @brief   实验任务映射表
 * @date    2024-09-22
 */

# 实验任务映射表

本文档说明 `experimental code/` 目录中的代码文件与实验指导书中任务的对应关系。

---

## 📚 实验1：GPIO输入输出与中断

**指导书**: `1-实验指导书_GPIO输入输出与中断 (1).docx`

| 任务编号 | 任务名称 | 文件名 | 函数名 | 状态 |
|---------|---------|--------|--------|------|
| 任务1.1 | CubeMX + VSCode 工具链，LED 闪烁 | task11_led_blink.c/h | `Task11_LED_BlinkTest()` | ✅ |
| 任务1.2 | GPIO 输入（按键检测 + 消抖） | task11_led_blink.c/h | `Task12_Button_ToggleLED()` | ✅ |
| 任务1.3 | LF04 4路循迹模块驱动 | task13_track_sensor.c/h | `Task13_Track_SensorTest()` | ✅ |
| 任务1.4 | EXTI 外部中断读循迹 | task14_exti_track.c/h | `Task14_EXTI_TrackTest()` | 🚧 待实现 |

**扩展实验**:
- PA2 电平控制 LED：`task11_led_blink.c` 中的 `Task1X_PA2_ControlLED()`

---

## 📚 实验2：串口USART

**指导书**: `2-实验指导书_串口USART.docx`

| 任务编号 | 任务名称 | 文件名 | 函数名 | 状态 |
|---------|---------|--------|--------|------|
| 任务2.1 | 串口回环 Echo（基础，必做） | task21_uart_echo.c/h | `Task21_UART_EchoTest()` | ✅ |
| 任务2.2 | printf 重定向打印编码器值（必做） | task22_encoder_printf.c/h | `Task22_PeriodicTask()` | ✅ 已完成 |
| 任务2.3 | DMA + 空闲中断 + VOFA+ 波形（重点） | task23_vofa_dma.c/h | `Task23_VOFA_Init()` + `Task23_VOFA_PeriodicTask()` | ✅ 已完成 |
| 任务2.4 | 综合：OpenMV 串口通信（H 题应用，选做） | task24_openmv_uart.c/h | `Task24_OpenMV_CommTest()` | 🚧 待实现 |

---

## 📚 实验3：定时器PWM与编码器

**指导书**: `3-实验指导书_定时器PWM与编码器.docx`

| 任务编号 | 任务名称 | 文件名 | 函数名 | 状态 |
|---------|---------|--------|--------|------|
| 任务3.1 | CubeMX 配置 TIM2 输出 PWM | task31_tim2_pwm.c/h | `Task31_Tim2PWM_Test()` | ✅ 已完成（PA0, 18 kHz, VOFA+ FireWater） |
| 任务3.2 | PWM 驱动电机 + 编码器 4 倍频测速 | task32_motor_encoder.c/h | `Task32_Motor_EncoderTest()` | ✅ |
| 任务3.3 | TIM6 10 ms 中断采样编码器并串口上报转速 | task33_encoder_periodic.c/h | `Task33_Encoder_PeriodicInit()` + `Task33_Encoder_PeriodicTask()` | ✅ FireWater 波形帧（I0=ΔCNT，I1=RPM；1560 计数/轮圈） |
| 任务3.4 | 电机闭环测速 + VOFA+ 曲线（综合） | task34_pid_vofa.c/h | `Task34_PID_VOFA_Init()` + `Task34_PID_VOFA_PeriodicTask()` | ✅ TIM6 每10 ms增量式PID；I0/I1为目标/实际，I2为调试PWM |

---

## 📚 综合实验

| 实验名称 | 文件名 | 函数名 | 状态 |
|---------|--------|--------|------|
| MPU6050 陀螺仪姿态测试（完整版） | task_mpu6050_attitude.c/h | `Task_MPU6050_AttitudeTest()` | ✅ |

**说明**: 
- MPU6050 实验集成了 TRACK、OLED、MPU6050 DMP、按键中断、BadApple 等多个模块
- 属于综合实验，未分配具体任务编号

---

## 🔧 使用方法

### 方式1：在 main.c 主循环中调用

```c
#include "experimental code/task22_encoder_printf.h"

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  
  chapter_main();  // 初始化所有模块
  
  // 调用任务2.2：printf 打印编码器
  Task22_Init();
  MOTOR_B_SetSpeed(500);
  
  while (1) {
    Task22_PeriodicTask();
    HAL_Delay(1);
  }
}
```

### 方式2：在 chapter.c 中调用

编辑 `chapter_main()` 函数，取消注释需要的实验。

---

## 📋 任务进度统计

| 实验 | 已完成 | 待实现 | 总计 |
|------|--------|--------|------|
| 实验1 | 3 | 1 | 4 |
| 实验2 | 3 | 1 | 4 |
| 实验3 | 2 | 2 | 4 |
| 综合 | 1 | - | 1 |
| **总计** | **9** | **4** | **13** |

**完成率**: 69.2%

---

## 📝 命名规范

### 文件命名
- 格式：`task<实验编号><任务编号>_<功能描述>.c/h`
- 示例：
  - `task11_led_blink.c` - 实验1 任务1：LED闪烁
  - `task22_encoder_printf.c` - 实验2 任务2：编码器printf
  - `task32_motor_encoder.c` - 实验3 任务2：电机编码器

### 函数命名
- 格式：`Task<实验编号><任务编号>_<功能描述>()`
- 示例：
  - `Task11_LED_BlinkTest()` - 任务1.1
  - `Task22_PeriodicTask()` - 任务2.2
  - `Task32_Motor_EncoderTest()` - 任务3.2

### 宏定义命名
- 格式：`__TASK<实验编号><任务编号>_<功能>_H`
- 示例：`__TASK22_ENCODER_PRINTF_H`

---

## 🔗 相关文档

- [实验代码使用说明](README.md)
- 项目介绍: `C:\Users\hshsgxlj\Desktop\2024电赛小车\stm32\介绍.txt`
- 实验指导书: `C:\Users\hshsgxlj\Desktop\2024电赛小车\实验指导\`

---

**最后更新**: 2024-09-22  
**维护者**: Claude Code
