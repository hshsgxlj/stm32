# STM32F103 LED控制项目

基于STM32F103RCT6的LED控制程序，适用于2024电赛小车项目。

## 📋 项目信息

- **芯片型号**: STM32F103RCT6
- **开发板**: WHEELTEC C10B主板
- **开发环境**: STM32CubeIDE / CMake + Ninja
- **HAL库版本**: STM32Cube FW_F1 V1.8.7
- **编译工具链**: GNU ARM Embedded Toolchain 14.3.1

## 🔌 硬件连接

根据原理图配置：

| LED | GPIO引脚 | 连接方式 | 点亮逻辑 |
|-----|----------|----------|----------|
| LED1 | PC2 | 共阳极（3.3V → LED → 限流电阻 → PC2） | 低电平点亮 |
| LED2 | PC3 | 共阳极（3.3V → LED → 限流电阻 → PC3） | 低电平点亮 |

**共阳极说明**：
- 输出 `0`（低电平）→ LED **点亮** ✅
- 输出 `1`（高电平）→ LED **熄灭** ❌

## 📁 项目结构

```
stm32/
├── Core/
│   ├── Inc/
│   │   ├── main.h              # 主程序头文件
│   │   └── led.h               # LED控制头文件（引脚定义、函数声明）
│   └── Src/
│       ├── main.c              # 主程序
│       ├── chapter.c           # LED测试示例代码
│       └── led.c               # LED功能实现
├── Drivers/                    # STM32 HAL库驱动
├── build/                      # 编译输出目录
├── CMakeLists.txt              # CMake构建配置
├── stm32.ioc                   # STM32CubeMX配置文件
└── README.md                   # 本文档
```

## 🎯 功能特性

### 基础控制函数

| 函数 | 功能说明 | 示例 |
|------|----------|------|
| `LED_Init()` | 初始化LED GPIO | `LED_Init();` |
| `LED_On(led_num)` | 打开指定LED | `LED_On(1);` |
| `LED_Off(led_num)` | 关闭指定LED | `LED_Off(2);` |
| `LED_Toggle(led_num)` | 翻转LED状态 | `LED_Toggle(1);` |
| `LED_All_On()` | 打开所有LED | `LED_All_On();` |
| `LED_All_Off()` | 关闭所有LED | `LED_All_Off();` |

### 高级效果函数

| 函数 | 功能说明 | 参数说明 |
|------|----------|----------|
| `LED_Blink(led_num, times, delay_ms)` | LED闪烁 | `led_num`: 0=全部, 1/2=单个<br>`times`: 闪烁次数<br>`delay_ms`: 间隔时间 |
| `LED_RunningLight(delay_ms, cycles)` | 流水灯效果 | `delay_ms`: LED间隔<br>`cycles`: 循环次数 |
| `LED_Breath(led_num, cycles)` | 呼吸灯效果 | `led_num`: 1或2<br>`cycles`: 呼吸周期 |
| `LED_SOS()` | SOS摩尔斯码 | 无参数，发送求救信号 |

## 🚀 快速开始

### 1. 编译项目

使用CMake构建：

```bash
# 进入构建目录
cd build/Debug

# 编译
ninja
```

或使用CMake命令：

```bash
cmake --build build/Debug
```

编译成功后会生成：
- `stm32.elf` - ELF可执行文件
- `stm32.hex` - HEX烧录文件
- `stm32.bin` - BIN烧录文件

### 2. 烧录程序

#### 方式A: 使用STM32CubeProgrammer（推荐）

1. 连接ST-Link到开发板的SWD接口
   ```
   ST-Link    →    开发板
   GND        →    GND
   SWDIO      →    SWDIO (PA13)
   SWCLK      →    SWCLK (PA14)
   3.3V       →    3.3V (可选)
   ```

2. 打开STM32CubeProgrammer
3. 连接方式选择 "ST-LINK"
4. 点击 "Connect" 连接芯片
5. 点击 "Open file" 选择 `stm32.hex`
6. 点击 "Download" 开始烧录

#### 方式B: 使用命令行（OpenOCD）

```bash
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program stm32.hex verify reset exit"
```

### 3. 运行效果

烧录完成后，程序自动运行，你会看到：

1. **LED1闪烁** 5次（间隔500ms）
2. **所有LED闪烁** 3次（间隔300ms）
3. **流水灯效果** - LED1→LED2循环3次
4. **呼吸灯效果** - LED1逐渐变亮变暗2个周期
5. **SOS信号** - 三短三长三短
6. **同步闪烁** - 两个LED同时翻转10次

## 💻 代码示例

### 示例1: 简单控制

```c
#include "led.h"

void chapter_main(void)
{
    LED_Init();           // 初始化LED
    
    LED_On(1);            // 点亮LED1
    HAL_Delay(1000);      // 延时1秒
    
    LED_Off(1);           // 熄灭LED1
    LED_On(2);            // 点亮LED2
    HAL_Delay(1000);
    
    LED_All_Off();        // 全部熄灭
}
```

### 示例2: LED闪烁

```c
LED_Init();
LED_Blink(1, 5, 500);     // LED1闪烁5次，每次间隔500ms
LED_Blink(0, 3, 200);     // 所有LED闪烁3次，间隔200ms
```

### 示例3: 流水灯

```c
LED_Init();
LED_RunningLight(200, 10);  // 流水灯，间隔200ms，循环10次
```

### 示例4: 呼吸灯

```c
LED_Init();
LED_Breath(1, 3);          // LED1呼吸灯效果，3个周期
```

### 示例5: SOS求救信号

```c
LED_Init();
LED_SOS();                 // 发送SOS摩尔斯码
```

## 🔧 自定义配置

### 修改LED引脚

编辑 `Core/Inc/led.h` 文件：

```c
/* LED引脚定义 */
#define LED1_PIN                    GPIO_PIN_2      // 修改引脚号
#define LED1_GPIO_PORT              GPIOC           // 修改端口
#define LED1_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()
```

### 修改LED极性

如果你的LED是**共阴极**（高电平点亮），修改 `led.h` 中的宏定义：

```c
/* 共阴极连接: 高电平点亮，低电平熄灭 */
#define LED1_ON()       HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_SET)
#define LED1_OFF()      HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_RESET)
```

### 修改测试程序

编辑 `Core/Src/chapter.c` 文件，注释/取消注释不同的示例函数：

```c
void chapter_main(void)
{
    LED_Init();
    
    // 取消注释想要的效果
    // LED_Blink(1, 5, 500);
    // LED_RunningLight(200, 3);
    // LED_Breath(1, 2);
    // LED_SOS();
}
```

## 📖 GPIO配置详解

### GPIO_MODE_OUTPUT_PP（推挽输出）

**你选中的这个配置模式非常重要！**

```c
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // 推挽输出模式
```

**推挽输出（Push-Pull）的工作原理**：

```
         VDD (3.3V)
            |
         [P-MOS]  ← 上管
            |
         引脚 ----→ 输出
            |
         [N-MOS]  ← 下管
            |
          GND
```

- **输出高电平**：上管导通，下管关闭 → 引脚输出3.3V
- **输出低电平**：上管关闭，下管导通 → 引脚输出0V
- **驱动能力强**：可提供/吸收最大20mA电流

**适用场景**：
- ✅ 驱动LED（我们的应用）
- ✅ 驱动蜂鸣器
- ✅ 驱动继电器
- ✅ 普通数字信号输出

**其他GPIO模式对比**：

| 模式 | 说明 | 高电平 | 低电平 | 应用 |
|------|------|--------|--------|------|
| `GPIO_MODE_OUTPUT_PP` | 推挽输出 | 强3.3V | 强0V | LED、蜂鸣器 |
| `GPIO_MODE_OUTPUT_OD` | 开漏输出 | 高阻态 | 强0V | I2C总线 |
| `GPIO_MODE_INPUT` | 输入模式 | - | - | 按键、传感器 |
| `GPIO_MODE_AF_PP` | 复用推挽 | 强3.3V | 强0V | UART、SPI |

## 🛠️ 故障排查

### LED不亮？

1. **检查硬件连接**
   - 确认LED连接到PC2和PC3
   - 检查LED极性是否正确
   - 测量引脚电压（应该能在0V和3.3V之间切换）

2. **检查代码配置**
   - 确认 `led.h` 中引脚定义正确
   - 确认LED极性宏定义正确（共阳/共阴）
   - 确认调用了 `LED_Init()`

3. **检查编译**
   - 确认 `led.c` 已添加到 `CMakeLists.txt`
   - 重新清理编译：`ninja clean && ninja`

4. **检查烧录**
   - 确认烧录成功
   - 按下复位键重启MCU
   - 检查芯片型号是否匹配

### 编译错误？

```
undefined reference to `LED_Init'
```

**解决方法**：检查 `CMakeLists.txt` 是否包含了 `led.c`：

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/led.c
)
```

## 📚 技术文档

- [STM32F103数据手册](https://www.st.com/resource/en/datasheet/stm32f103rc.pdf)
- [STM32F1 HAL库用户手册](https://www.st.com/resource/en/user_manual/um1850-description-of-stm32f1-hal-and-lowlayer-drivers-stmicroelectronics.pdf)
- [GPIO详解](https://www.st.com/resource/en/application_note/an4899-stm32-gpio-configuration-for-hardware-settings-and-lowpower-consumption-stmicroelectronics.pdf)

## 📝 版本历史

### v1.0 (2026-09-16)
- ✅ 初始版本
- ✅ 支持PC2、PC3双LED控制
- ✅ 实现基础控制函数
- ✅ 实现高级效果（闪烁、流水灯、呼吸灯、SOS）
- ✅ 完整的HAL库GPIO配置

## 👥 作者

- 开发者：电赛小车项目组
- 芯片：STM32F103RCT6
- 主板：WHEELTEC C10B

## 📄 许可证

本项目基于 STMicroelectronics 软件许可协议。

---

**祝你在2024电赛中取得好成绩！** 🏆

如有问题，请检查硬件连接和代码配置。
