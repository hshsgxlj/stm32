# 2024 电赛 H 题自动行驶小车

基于 WHEELTEC C10B 主板和 STM32F103RCT6 的自动行驶小车项目。工程包含电机驱动、编码器测速、PID 闭环、四路循迹、MPU6050 姿态检测、OLED 显示、蜂鸣器节点提示以及串口/VOFA+ 调试功能。

当前正式应用入口是 `Core/Src/chapter.c` 和 `Core/Src/h_course.c`。`Core/experimental code/` 保留课程实验和独立验证代码，不作为 H 题主流程的唯一入口。

## 1. 项目能力

- 任务 1：A-B 直线行驶，检测到四路传感器任意一路黑线后停车并提示。
- 任务 2：A-B-C-D-A 路线，直线段使用航向保持/左右轮同步，弯道使用四路循迹，节点和终点进行蜂鸣器提示。
- 任务 3：A-C 斜线、C-B 弧线、B-D 斜线、D-A 弧线。斜线使用 MPU6050 航向控制和编码器距离窗口，弧线使用循迹控制。
- 任务 4：任务 3 路线的多圈运行版本，包含出弯车头摆正、斜线超程救援和圈数管理。
- OLED 实时显示四路传感器、左右轮速度、任务相对航向、航向修正量以及运行/故障状态。
- USER 按键通过 EXTI0 切换任务；一次按键只产生一次切换，松开并稳定后才允许下一次按键。
- USART1 可输出调试信息和 FireWater 波形数据，便于串口助手或 VOFA+ 观察。

## 2. 硬件信息与接线

### 主控与供电

| 项目 | 配置 |
| --- | --- |
| 主控 | STM32F103RCT6（C10B 主板） |
| 电机电源 | 电机必须使用独立电池/电机电源，不能由 ST-Link 或 USB 供电 |
| 调试下载 | ST-Link 接 SWDIO、SWCLK、GND；3.3 V 按实际供电方案连接 |
| 串口 | 板载 CH9102F，USART1：PA9-TX、PA10-RX |
| 串口参数 | 115200 baud，8 数据位，无校验，1 停止位（8N1） |

### 电机、编码器和循迹

当前 `motor.h` 中的正速度已经按物理方向定义为小车前进。

| 功能 | 引脚/接口 |
| --- | --- |
| 电机 A（物理右轮）PWM/方向 | PA6 / PA7，TIM3 通道 |
| 电机 B（物理左轮）PWM/方向 | PB0 / PB1，TIM3 通道 |
| 电机 A 编码器 | PB6、PB7，TIM4 |
| 电机 B 编码器 | PC6、PC7，TIM8 |
| P1（最左） | PC8 |
| P2 | PC4 |
| P3 | PC9 |
| P4（最右） | PB8 |
| 循迹输入逻辑 | 黑线为 `0`，白底为 `1` |

循迹传感器的顺序按实际新接线定义为 P1、P2、P3、P4。若左右修正方向相反，优先检查传感器顺序和电机方向宏，不要直接大幅修改 PID。

### OLED、MPU6050、LED 和蜂鸣器

| 模块 | 引脚 |
| --- | --- |
| OLED SCL/SCK | PC14 |
| OLED SDA/MOSI | PB5 |
| OLED RES | PB4 |
| OLED DC | PB3 |
| MPU6050 软件 I2C SCL | PB14 |
| MPU6050 软件 I2C SDA | PB15 |
| MPU6050 INT | PB9 |
| LED1 / LED2 | PC2 / PC3，低电平点亮 |
| 蜂鸣器 | PA15，高电平有效 |
| USER 按键 | PA0，EXTI0；当前工程由 CubeMX 配置为上升沿中断 |

OLED 和 MPU6050 当前使用工程内的软件时序驱动，实际引脚以 `Core/Inc/oled.h` 和 `Core/Inc/mpu6050.h` 为准。不要仅根据实验指导书中的 SPI2/I2C1 示例修改正式应用的引脚。

## 3. 工程目录

```text
stm32/
├── Core/
│   ├── Inc/
│   │   ├── h_course.h       # H 题路线控制参数和接口
│   │   ├── motor.h          # 电机、编码器接口
│   │   ├── track.h          # 四路循迹接口
│   │   ├── mpu6050*.h       # MPU6050/DMP 接口
│   │   ├── oled.h           # OLED 接口
│   │   └── chapter.h        # 主任务入口和按键接口
│   ├── Src/
│   │   ├── main.c           # CubeMX 初始化和 chapter_main() 入口
│   │   ├── chapter.c        # 按键任务状态机
│   │   ├── h_course.c       # H 题四任务核心控制
│   │   ├── motor.c          # PWM、方向和编码器
│   │   ├── track.c           # 四路传感器读取
│   │   ├── mpu6050.c        # 软件 I2C、原始数据和滤波
│   │   ├── mpu6050_dmp.c    # DMP 初始化和姿态角
│   │   └── oled.c            # OLED 软件串行驱动
│   ├── experimental code/   # 课程任务 1.x~5.x 独立实验
│   └── badapple/            # 独立的 Bad Apple 扩展功能
├── Drivers/                 # STM32F1 HAL 和 CMSIS
├── cmake/                   # CubeMX/CMake 工具链配置
├── build/Debug/             # CMake 构建输出
├── stm32.ioc               # CubeMX 工程配置
├── CMakeLists.txt
└── README.md
```

## 4. 编译与下载

### 编译环境

- STM32CubeMX 生成的 CMake 工程
- CMake 3.22 或更高版本
- Ninja 或其他 CMake 支持的构建工具
- `arm-none-eabi-gcc` 工具链
- VS Code STM32 扩展、STM32CubeIDE 或命令行环境均可

确认编译器在 PATH 中：

```powershell
arm-none-eabi-gcc --version
cmake --version
```

### 构建

在 `stm32/stm32` 目录执行：

```powershell
cmake --build "build\Debug"
```

清理后完整构建：

```powershell
cmake --build "build\Debug" --clean-first
```

成功后至少应生成：

```text
build/Debug/stm32.elf
```

如果 VS Code 报 `arm-none-eabi-gcc was not found in PATH`，先安装 GNU Arm Embedded Toolchain，或在 CMake 工具链配置中填写 `arm-none-eabi-gcc.exe` 的完整路径。`EBUSY` 通常表示构建目录中的日志文件被 VS Code/CMake 进程占用，关闭正在运行的构建任务后再重新配置。

### 下载

1. 小车断开电机电池，先只连接 ST-Link 和开发板 GND。
2. ST-Link 的 SWDIO 接 PA13，SWCLK 接 PA14，GND 共地。
3. 使用 STM32CubeProgrammer、STM32CubeIDE 或 VS Code 的烧录功能下载 `stm32.elf`/转换后的 HEX 文件。
4. 下载完成后先断开或关闭电机电源，确认程序能启动，再连接电池进行架空测试。

USB Type-C 在本项目中主要用于板载 CH9102F 串口和供电，不等同于 STM32 的 USB DFU 下载接口。正常下载仍建议使用 ST-Link SWD。

## 5. 主程序使用方法

`main.c` 完成 HAL、时钟、GPIO、USART1、TIM2 等底层初始化后调用：

```c
chapter_main();
```

`chapter_main()` 会初始化 OLED、H 题控制器和 USER 按键中断，然后进入统一任务循环。按键任务顺序如下：

```text
上电等待
  -> 第一次按键：任务1
  -> 第二次按键：停止任务1
  -> 第三次按键：任务2
  -> 第四次按键：停止任务2
  -> 第五次按键：任务3
  -> 第六次按键：停止任务3
  -> 第七次按键：任务4
  -> 第八次按键：停止任务4
  -> 第九次按键：距离标定测试
  -> 下一次按键：回到任务1
```

任务自然完成后会自动进入对应停止状态，但不会自动跳到下一个任务；需要再次按 USER 键确认进入下一项。

## 6. H 题控制逻辑

### 任务 1

任务 1 使用直线速度基准和航向保持。控制器读取四路传感器，只要任意一路检测到黑线，就停止两轮、关闭电机使能并蜂鸣器提示。

### 任务 2

任务 2 将路线分为：

```text
AB 直线 -> BC 弯道 -> CD 直线 -> DA 弯道 -> 结束
```

直线段使用 MPU6050 航向保持；MPU6050 读数暂时无效时退回编码器左右轮同步。弯道使用四路循迹，采用全白持续确认、黑线数量确认和节点冷却时间，减少传感器间隙造成的误判。B、C、D、A 等阶段切换会按配置进行蜂鸣器提示。

### 任务 3

任务 3 的阶段为：

```text
起步转向 AC
  -> AC 编码器距离窗口 + 航向保持
  -> C 点制动与转向
  -> CB 弧线循迹
  -> B 点出弯车头摆正
  -> BD 编码器距离窗口 + 航向保持
  -> D 点制动与转向
  -> DA 弧线循迹
  -> 终点制动
```

转弯完成后会进入短暂稳定确认，避免车轮惯性和陀螺仪瞬时抖动直接推进下一阶段。若距离窗口、航向或转向超时，会进入故障状态并在 OLED 上显示故障代码。

### 任务 4

任务 4 复用任务 3 的斜线和弧线控制，连续运行设定圈数。每次出弯先将车头调整到全局目标航向，再进入下一段转向或直线。斜线超过预计距离后会进入救援摆正逻辑；救援仍无法在允许时间内捕捉线路时停车报错，避免小车无控制地继续运行。

## 7. 主要参数调节

所有 H 题现场参数集中在 `Core/Inc/h_course.h`，修改后重新构建即可。

| 参数类别 | 常用宏 | 作用 |
| --- | --- | --- |
| 直行速度 | `H_COURSE_STRAIGHT_PWM_LEFT/RIGHT` | 直线段基础 PWM |
| 循迹速度 | `H_COURSE_MAX_SPEED`、路线 PWM 宏 | 限制循迹和转向最高输出 |
| 循迹 PID | `H_COURSE_LINE_KP/KI/KD` | 传感器误差到差速修正 |
| 航向 PID | `H_COURSE_YAW_KP/KI/KD` | MPU6050 航向误差到差速修正 |
| 左右轮同步 | `H_COURSE_SYNC_KP/KI/KD` | 直线段编码器转速同步 |
| 轮径/计距 | `H_COURSE_TASK3_WHEEL_CIRC_MM` | 编码器距离换算 |
| 编码器分辨率 | `H_COURSE_TASK1_COUNTS_PER_REV` | 计数到转速/距离换算 |
| 斜线方向 | `H_COURSE_TASK3_DIAG_ANGLE_X10` | AC/BD 斜线目标角度 |
| 车头摆正 | `H_COURSE_TASK34_B_EXIT_TARGET_X10` | 任务 3/4 B 点出弯目标角度 |
| 方向微调 | `H_COURSE_TASK3_AC_TRIM_X10`、`H_COURSE_TASK3_BD_TRIM_X10` | 现场修正斜线方向 |
| 丢线恢复 | `H_COURSE_LINE_RECOVER_*` | 全白、外侧传感器压线时的恢复策略 |
| 故障超时 | `H_COURSE_TASK3_*_TIMEOUT_MS` | 保护电机，避免阶段永久卡住 |

推荐调参顺序：先确认电机正方向和传感器顺序，再调基础速度；然后调 `Kp`，再调 `Kd`，最后只在确有持续偏差时增加少量 `Ki`。调参时优先架空测试，再低速接地测试。

## 8. OLED 和串口调试

OLED 的常规显示包括：

- 第一行：P1、P2、P3、P4 传感器状态；
- 第二行：左右轮实际速度；
- 第三行：相对任务启动航向 `Y` 和当前航向修正量 `C`；
- 第四行：任务阶段、运行状态或错误代码。

调试固件默认开启 USART1 遥测，115200、8N1，每 100 ms 输出一行 CSV。字段包括运行时间、任务与状态、节点计数、P1–P4、左右轮编码器转速、轮速估算、相对任务起点的 `Y` 偏航角、目标航向误差和 IMU 有效标志。转速和角度以 0.1 为单位输出；车速估算单位为 mm/s。循迹传感器值 `0` 表示黑线，`1` 表示白底。

```text
time_ms,task,state,node,P1,P2,P3,P4,left_rpm_x10,right_rpm_x10,speed_mm_s_est,Y_deg_x10,Yerr_deg_x10,imu_ok
1500,1,1,0,1,0,1,1,120,118,36,12,0,1
```

`state` 数值依次表示待机、循迹运行、直线航向保持、完成、停止、故障。`tools/capture_telemetry.ps1` 可把串口数据持续保存到 `telemetry_logs` 目录：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\capture_telemetry.ps1 -PortName COM9
```

USART1 信号为 PA9-TX、PA10-RX。可通过主板板载 CH9102F 的 USB 串口采集；若使用调试器的 COM 口作为串口桥，需要将 PA9-TX 接到桥接器 RX、共地，并使用对应 COM 口。CMSIS-DAP 的 SWD 下载接口本身是有线调试，不提供无线遥测；无线采集需另接 Wi-Fi 或蓝牙串口模块。

## 9. 安全测试顺序

1. 不接电机电池，只连接 ST-Link，确认程序能编译和下载。
2. 连接 USB 串口，确认 USART1 能输出启动信息。
3. 检查 OLED、传感器和 MPU6050 的供电电压，确认没有 5 V 直接进入 3.3 V GPIO。
4. 小车架空，确认 `MOTOR_SetSpeeds(+x, +x)` 两个轮子都代表前进。
5. 架空测试低 PWM、编码器方向、四路传感器状态和蜂鸣器。
6. 先运行任务 1，再逐项运行任务 2、任务 3、任务 4。
7. 最后才接地运行，并准备随时断开电机电源。

不要在电机电源开启时反复插拔电机接口、编码器接口或 ST-Link 线。ST-Link 只负责调试下载，不能替代电机电源；电机堵转、接线短路或电源反接都可能损坏驱动器、开发板或 ST-Link。

## 10. 常见问题

### 按一次 USER 键切换两次

当前按键中断使用锁存和释放确认：一次上升沿只产生一次请求，PA0 稳定释放 100 ms 后才解锁。若仍出现异常，检查 PA0 的 CubeMX 触发边沿是否与实际按键电平一致，确认没有同时在其他文件中实现第二个 `HAL_GPIO_EXTI_Callback()`。

### 电机不转或转得很慢

确认电机电池已接通、驱动器 VM 和 GND 正常、`MOTOR_SetEnabled(1U)` 已执行，并先用低风险固定 PWM 架空测试。USB 或 ST-Link 不能提供电机所需电流。

### 任务显示 E2、E3、E5 等错误

错误通常表示 MPU6050 读数失效、航向持续超限、距离窗口超出或转向阶段超时。先看 OLED 状态和串口调试信息，再检查 MPU6050 供电、PB14/PB15 接线、传感器顺序、电机方向和 `h_course.h` 的距离/角度参数。

### 编译器找不到

检查 `arm-none-eabi-gcc` 是否加入 PATH；不要把 Python 环境或普通系统 GCC 当作 ARM 交叉编译器。关闭被占用的 VS Code/CMake 构建任务后，可以删除并重新生成 `build/Debug` 配置目录。

## 11. 课程实验代码

`Core/experimental code/` 中保留以下独立实验：

- GPIO、LED、按键和循迹传感器；
- USART 回环、`printf` 重定向、DMA/空闲中断和 VOFA+；
- PWM、电机、编码器和闭环速度 PID；
- MPU6050 原始数据、DMP 姿态和 OLED 显示；
- 位置式 PID、增量式 PID。

这些文件用于课程验收和模块验证。正式 H 题运行时只通过 `chapter_main()` 进入 `h_course.c`，避免同时启动多个任务抢占同一组电机、定时器或串口资源。

## 12. 维护说明

- 修改引脚时同步检查 `stm32.ioc`、`main.c` 和对应模块头文件。
- 修改电机方向时只调整 `motor.h` 中的方向符号，不要在多个任务中分别取反。
- 修改 MPU6050 或 OLED 接线时，先更新头文件宏，再检查 `motor.c`、`oled.c`、`mpu6050.c` 的 GPIO 初始化是否冲突。
- 新增源文件后必须加入 `CMakeLists.txt` 的 `target_sources()`，否则代码不会参与构建。
- 现场调试结束后关闭高频串口调试输出，减少控制循环被阻塞的风险。

## 13. 版本状态

当前工程已完成：

- CMake 全量构建通过；
- C10B 电机、编码器、四路循迹、OLED、MPU6050、蜂鸣器和串口模块已接入；
- H 题任务 1~4 的主流程和故障保护已实现；
- USER 按键消抖和任务切换状态机已实现。

实车最终成绩仍取决于电池电压、轮胎摩擦、传感器安装高度、编码器标定和现场 PID 参数。首次运行必须遵守“架空、低速、可断电”的测试顺序。
