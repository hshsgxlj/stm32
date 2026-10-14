/**
 * @file    README.md
 * @brief   实验代码模块说明文档
 * @date    2024
 */

# 实验代码模块说明

本目录包含所有从 `chapter.c` 重构出来的实验代码，按功能模块分类组织。

## 📁 文件结构

```
experimental code/
├── encoder_printf.c/h      # 任务2: printf 重定向打印编码器值
├── exp_led_test.c/h        # LED 闪烁和按键控制实验
├── exp_uart_test.c/h       # 实验21: UART 串口发送测试
├── exp_motor_test.c/h      # 实验22: 电机速度测试
├── exp_track_test.c/h      # 巡线传感器测试
├── exp_mpu6050_test.c/h    # MPU6050 陀螺仪姿态测试
└── README.md               # 本文档
```

---

## 🔧 模块功能说明

### 1. encoder_printf（任务2：必做）
**文件**: `encoder_printf.c/h`

**功能**:
- 重写 `fputc()`，实现 printf 重定向到 USART1
- 每 100 ms 读取 B 电机编码器增量
- 通过串口打印实际速度与目标值

**使用方法**:
```c
#include "experimental code/encoder_printf.h"

// 方式1: 主循环调用
EncoderPrintf_Init();
MOTOR_B_SetSpeed(500);
while(1) {
    EncoderPrintf_PeriodicTask();
    HAL_Delay(1);
}

// 方式2: 独立测试
EncoderPrintf_Test(); // 自动启动电机并持续打印
```

**串口输出示例**:
```
=== Encoder Printf Test Start ===
Target Speed: 500
[1023] speed=120, target=500
[1123] speed=125, target=500
```

---

### 2. exp_led_test（LED 实验）
**文件**: `exp_led_test.c/h`

**包含实验**:
- **实验1.2**: LED 闪烁测试
- 按键控制 LED 翻转
- PA2 电平控制 LED1

**使用方法**:
```c
#include "experimental code/exp_led_test.h"

// LED 闪烁
EXP_LED_BlinkTest();

// 按键控制 LED
EXP_LED_ButtonToggleTest();

// PA2 控制 LED
EXP_LED_PA2_ControlTest();
```

---

### 3. exp_uart_test（实验21）
**文件**: `exp_uart_test.c/h`

**功能**:
- 通过 USART1 每秒发送 "Hello, STM32!"
- 测试串口通信是否正常

**使用方法**:
```c
#include "experimental code/exp_uart_test.h"
EXP21_UART_SendTest();
```

**串口助手配置**:
- COM口: COM8（板载 CH9102F）
- 波特率: 115200
- 数据位: 8
- 校验位: None
- 停止位: 1

---

### 4. exp_motor_test（实验22）
**文件**: `exp_motor_test.c/h`

**功能**:
- B 电机 20% 正转测试（CN2 接口）
- 验证电机驱动和编码器连接

**使用方法**:
```c
#include "experimental code/exp_motor_test.h"
EXP22_Motor_SpeedTest();
```

**注意事项**:
- ⚠️ 测试时车轮必须悬空
- ⚠️ 必须接 6-16V 电池供电（VIN）
- ⚠️ 电源开关和电机使能开关需打开

---

### 5. exp_track_test（巡线传感器）
**文件**: `exp_track_test.c/h`

**功能**:
- 读取 4 路巡线传感器（PA8, PA3, PA2, PA15）
- 在 OLED 上实时显示传感器状态
- 黑色=0，白色=1

**使用方法**:
```c
#include "experimental code/exp_track_test.h"
EXP_Track_SensorTest();
```

---

### 6. exp_mpu6050_test（陀螺仪姿态测试）
**文件**: `exp_mpu6050_test.c/h`

**功能**:
- MPU6050 DMP 姿态解算
- OLED 显示 Pitch、Roll、Yaw 角度
- 集成巡线传感器、按键中断、BadApple

**使用方法**:
```c
#include "experimental code/exp_mpu6050_test.h"
EXP_MPU6050_AttitudeTest();
```

**显示内容**:
```
Track: 0 1 1 0        (4路巡线)
PITCH: +12.3°
ROLL:  -3.5°
YAW:   +45.6°
I: 2                  (MPU状态: 0=未初始化, 1=已初始化, 2=有数据)
```

---

## 🚀 如何使用

### 在 main.c 中调用实验
编辑 `Core/Src/main.c`:

```c
/* USER CODE BEGIN Includes */
#include "chapter.c"
#include "experimental code/encoder_printf.h"  // 包含需要的实验头文件
/* USER CODE END Includes */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  chapter_main();  // 初始化所有模块
  
  // 调用具体实验
  EncoderPrintf_Init();
  MOTOR_B_SetSpeed(500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    EncoderPrintf_PeriodicTask();  // 周期打印编码器
    HAL_Delay(1);
    /* USER CODE END WHILE */
  }
}
```

### 在 chapter.c 中调用实验
编辑 `Core/Src/chapter.c` 的 `chapter_main()` 函数：

```c
void chapter_main(void)
{
  LED_Init();
  MOTOR_Init();
  OLED_Init();

  // 取消注释需要的实验
  #include "experimental code/exp_uart_test.h"
  EXP21_UART_SendTest();
}
```

---

## 📋 实验清单

| 实验编号 | 实验名称 | 文件 | 状态 |
|---------|---------|------|------|
| 实验1.2 | LED 闪烁 | exp_led_test.c | ✅ 已迁移 |
| 实验21 | UART 串口发送 | exp_uart_test.c | ✅ 已迁移 |
| 实验22 | 电机速度测试 | exp_motor_test.c | ✅ 已迁移 |
| 任务2 | printf 打印编码器 | encoder_printf.c | ✅ 已完成 |
| - | 按键控制 LED | exp_led_test.c | ✅ 已迁移 |
| - | PA2 控制 LED | exp_led_test.c | ✅ 已迁移 |
| - | 巡线传感器测试 | exp_track_test.c | ✅ 已迁移 |
| - | MPU6050 姿态测试 | exp_mpu6050_test.c | ✅ 已迁移 |

---

## ⚙️ chapter.c 重构说明

重构后的 `chapter.c` 只保留：
- ✅ `__io_putchar()` - printf 重定向钩子
- ✅ `outputs_requested_on` - 全局输出使能标志
- ✅ `Chapter_ToggleOutputsFromISR()` - 按键中断回调
- ✅ `chapter_main()` - 模块初始化入口

所有实验代码已迁移到 `experimental code/` 目录，通过包含头文件调用。

---

## 📝 注意事项

1. **电机测试**: 必须接电池供电，车轮悬空
2. **串口通信**: 使用 COM8（板载 CH9102F），波特率 115200
3. **编码器测试**: 当前只接了 B 电机（CN2 接口），使用 `MOTOR_B_*` 函数
4. **按键中断**: PA0 已配置为 EXTI0，不能再初始化为普通输入
5. **MPU6050**: DMP 初始化需要时间，初次运行可能需要重试

---

## 🔗 相关文档

- 项目介绍: `C:\Users\hshsgxlj\Desktop\2024电赛小车\stm32\介绍.txt`
- 主控板: 轮趣 C10B (STM32F103RCT6)
- 开发环境: VS Code + CMake + GNU Arm Toolchain

---

**最后更新**: 2024-09-22
**维护者**: Claude Code
