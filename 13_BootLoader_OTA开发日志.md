# 13_BootLoader_OTA 开发日志

> 整理日期：2026-09-20  
> 产品：LFC-WATCH；目标芯片：STM32F411CEUx  
> 工程：独立裸机 Bootloader + 从 12_LowPower 继承的 FreeRTOS/LVGL Application  
> 通信：KT6368A 蓝牙串口透传、USART1 115200/8N1、YMODEM 接收  
> 本文依据当前源码、Keil 工程及 Scatter、此前学习对话中的截图和用户反馈编写。本次仅写文档，没有重新烧录或实施故障注入。

## 项目目标与完成范围

项目 10 建立了传感器中心，项目 11 建立蓝牙应用协议，项目 12 完成可恢复的低功耗流程。到这一阶段，手表已经可以运行实际业务，但更新程序仍依赖 ST-Link。项目 13 要把“运行手表”和“接收新手表程序”分开，让应用固件能经过蓝牙串口更新。

本项目形成的主要链路是：APP 收到 `LFC+OTA` → 先回复应答 → ControlTask 保持供电并进入 Bootloader → 电脑重新连接蓝牙串口 → YMODEM 传送 APP 二进制 → 擦写和校验 → 提交有效记录 → 自动进入 APP。

目前已实现独立 Bootloader、APP 链接地址迁移、按键/软件请求留驻、SOH/STX 收包、包 CRC、重复包处理、超时和取消、单 APP 区写入、文件读回 CRC、有效记录、双向软件交接。用户已反馈完整固件传输后进入手表界面，以及无需手动开机即可由 APP 返回 Bootloader 并持续收到 C。

本阶段没有实现 A/B 固件、自动回滚、断点续传、签名认证、加密或防版本回退。成功完成一次升级，也不等于所有电压、断电时刻、无线干扰和重复升级场景均已验证。

## 开始本项目前的系统基础

Application 保留项目 12 的 FreeRTOS、LVGL、传感器、RTC、EEPROM 亮度存储和电源管理。与 OTA 直接有关的职责如下。

| 模块/任务 | 在本项目中的作用 |
|---|---|
| BLETask | 从 UART 字节队列组合 LFC 帧，识别升级命令并发送应答 |
| ControlTask | 接收升级事件，延时后执行外设清理及软件交接 |
| PowerTask/POWER_EN | 维持手表电源自锁，解释了为什么普通复位可能造成掉电 |
| WatchdogTask/TPS3823 | APP 正常运行时喂狗，交接前必须处理无人喂狗窗口 |
| RTC 备份寄存器 | BKP1R 传递一次性升级请求；BKP0R仍服务日历初始化 |
| UiTask/SensorTask | 保留各自 LVGL/I2C 所有权，不负责接收和写入固件 |
| 项目 11 的 LFC 协议 | 承载“请求升级”命令，不承载整份固件正文 |

Bootloader 不启动这些任务，也不初始化完整手表界面。它使用 GPIO、USART1、SysTick 和少量 TIM3 PWM，减少与业务代码的依赖。

## Bootloader、启动和 C 运行环境

### 复位后不是直接从 main 开始

本板按主 Flash 启动时，启动映射让内核从向量表读取初始栈指针和复位入口；主 Flash 的物理起点是 `0x08000000`。因此“上电从 0x08000000 开始”只能作为简写：该地址先放向量表，不是普通 C 函数的第一条指令。其他启动引脚/启动模式不在本项目验证范围内。

两份 `MDK-ARM/startup_stm32f411xe.s` 都以向量表开头，前两项分别为 `__initial_sp` 和 `Reset_Handler`。复位处理随后调用 `SystemInit`，再进入 ARM C 库的 `__main`，完成运行环境初始化后才进入用户 `main()`。

```text
向量表第0项 → 初始MSP
向量表第1项 → Reset_Handler
                  → SystemInit
                  → __main：初始化RW/ZI等运行环境
                  → main
```

有初值的全局变量通常需要把初值从 Flash 装载到 RAM；零初始化区需要清零。假设 APP 中有 `int a = 5;` 和 `static int b;`，直接跳到 APP 的 `main()` 无法保证它们分别为 5 和 0。RAM 可能保留 Bootloader 使用后的内容，不能把“声明时写了初值”理解为 CPU 会自动完成赋值。

### 为什么拆成两个程序

Bootloader 是固定启动入口和升级接收者；APP 是被升级的手表业务。两者分别链接、分别拥有启动代码和 RAM 布局。RAM 地址可以相同，因为正常设计是交接后重新建立目标程序的运行环境，而不是让两个程序同时执行。

IAP 表示程序在运行中改写内部 Flash。OTA 说明固件通过无线链路到达设备。本项目是“蓝牙 SPP 串口透传上的 YMODEM IAP”，不是 STM32 自建 BLE GATT OTA 服务。UART 是 MCU 侧接口，YMODEM 是文件传输协议，三者不是互相替代的名称。

## 工程结构、产物和构建

项目根目录为 `D:\Study\Qian\32\Ov-Watch Learn\13_BootLoader_OTA`。

```text
13_BootLoader_OTA
├─ BootLoader
│  ├─ Core/Src/main.c                 裸机启动、收包、Flash、记录与跳转
│  ├─ Core/Src/system_stm32f4xx.c     早期供电保持、系统初始化
│  └─ MDK-ARM/BootLoader.uvprojx      Bootloader工程
└─ Application
   ├─ Core/Src/freertos.c             OTA命令执行与任务交接
   ├─ Core/Src/ble_command.c          LFC+OTA识别
   ├─ Core/Src/ble_protocol.c         LFC帧解析与生成
   └─ MDK-ARM/07_LVGL_FreeRTOS.uvprojx APP工程
```

Application 的目标名仍为 `07_LVGL_FreeRTOS`，是连续复制留下的构建名称。当前 `app_version.h` 也仍为 `0.12.0`，不能因为项目目录是 13 就在日志中声称固件版本已变成 0.13.0。

两个工程分别引用各自 `Core` 和驱动文件。它们沿用同一 HAL/CMSIS 体系，但不是共享一个正在运行的 HAL 句柄或 RTOS，也没有把 APP 的 LFC 解析器链接进 Bootloader。

| 文件类型 | 用途 |
|---|---|
| `.axf` | 带链接地址和调试信息，供 Keil 下载、调试及 fromelf 转换 |
| `.hex` | 带地址的烧录数据，可用于烧录工具 |
| `.bin` | 纯字节固件；没有目标地址，接收端必须按约定写到 APP_BASE_ADDR |
| `.sct` | 链接器的装载区、执行区和 RAM 布局 |

分别打开两个 `.uvprojx` 编译。首次安装 Bootloader 使用 ST-Link；YMODEM 发送的是 **Application 的 BIN**，不能发送 Bootloader AXF、HEX，也不能只把 AXF 改后缀。

当前工具路径为 `D:\studaysoftware\Keil5MDK\ARM\ARMCC\bin\fromelf.exe`。重新生成 APP BIN 的命令可使用：

```powershell
$otaRoot = 'D:\Study\Qian\32\Ov-Watch Learn'
$otaAxF = Join-Path $otaRoot '13_BootLoader_OTA\Application\MDK-ARM\07_LVGL_FreeRTOS\07_LVGL_FreeRTOS.axf'
$otaBin = Join-Path $otaRoot '素材\13_BootLoader_OTA\watch_app.bin'
& 'D:\studaysoftware\Keil5MDK\ARM\ARMCC\bin\fromelf.exe' --bin --output $otaBin $otaAxF
Get-Item -LiteralPath $otaBin | Select-Object FullName, Length
```

先成功编译 APP 再转换，否则可能发送上一次的产物。素材保存于 D 盘；日志本身保存在正式项目根目录。本次核对 `watch_app.bin` 为 358060 字节，历史截图中的 357408 字节属于较早构建，不能混作同一固件的长度。

## Flash 分区与 SRAM

当前芯片配置为 STM32F411CEUx，Flash 512 KiB、SRAM 128 KiB。扇区组织用 ST 的 [RM0383 Flash module organization](https://www.st.com/resource/en/reference_manual/dm00119316.pdf) 与工程配置交叉核对。

| 区域 | 起始地址 | 最后一个地址（包含） | 大小 | 扇区 |
|---|---|---|---:|---|
| Bootloader | 0x08000000 | 0x08007FFF | 32 KiB | 0、1 |
| APP有效记录 | 0x08008000 | 0x0800BFFF | 16 KiB | 2 |
| APP | 0x0800C000 | 0x0807FFFF | 464 KiB / 475136字节 | 3～7 |
| SRAM | 0x20000000 | 0x2001FFFF | 128 KiB | 非Flash |

扇区 0～3 各 16 KiB，扇区 4 为 64 KiB，扇区 5～7 各 128 KiB。元数据只用 20 字节，但擦除粒度仍是整个扇区 2；不能以“只改五个字”为理由忽略其他地址的擦除影响。

`APP_FLASH_END = 0x08080000U` 是末端的下一地址，不是可写地址。`0x08080000 - 0x0800C000 = 0x74000 = 475136`。`SRAM_END = 0x20020000U` 同样是 RAM 末端下一地址，但允许作为向下增长栈的初始栈顶。

Bootloader Scatter 的装载/执行区是 `0x08000000, 0x8000`；APP 是 `0x0800C000, 0x74000`。因此 APP 不能进入记录区，Bootloader 也不能增长到扇区 2。升级代码固定擦扇区 3～7，记录操作单独擦扇区 2，软件路径不触碰扇区 0、1。这是范围约束，不能等同于已设置芯片 Option Bytes 写保护。

## Bootloader 启动与留驻流程

实际顺序以 `BootLoader/Core/Src/main.c` 和 `SystemInit()` 为准：

```text
Reset_Handler
 → SystemInit尽早把PA3/POWER_EN保持为高
 → __main建立C运行环境
 → main：SystemCoreClockUpdate → HAL_Init → SystemClock_Config
 → GPIO、USART1初始化
 → 读取BKP1R，一次性消费升级请求
 → 检查Flash记录和APP正文CRC
 → 等待500ms，KEY1低电平经20ms再次采样确认
 → 无留驻请求且记录有效：Boot_JumpToAPP
 → 其他情况：初始化暗光PWM，重启蓝牙模块，进入接收循环
```

APP 软件交接保留原系统时钟，所以 Bootloader 先更新 `SystemCoreClock` 再调用 HAL 初始化时基，随后切换到自己使用的 HSI。不能假定每次进入 Bootloader 都是完整硬件复位后的时钟状态。

核心判断：

```c
if((boot_stay_requested == 0U) && (boot_app_record_valid != 0U))
{
  Boot_JumpToAPP();
}
```

`boot_stay_requested` 表示“这次启动要求留驻”，不是 Flash 固件有效性。软件请求或 KEY1 都能将它置 1；记录无效时，即使它为 0 也不会尝试正常启动 APP。跳转函数若发现向量不合理会返回，后续仍进入接收循环。

留驻状态每秒发一个 `0x43`（C）。Bootloader 不显示手表界面；PB0 背光每 500ms 在熄灭与 0.5% 占空比之间切换，视觉上很微弱。TIM3 分频 15、周期 999，在 16MHz 定时器时钟下得到 1kHz PWM。阻塞收包/擦写期间闪烁可能延后，不能用不均匀闪烁单独判断死机。

按键只有电源键和 KEY1。学习过程中提到的 RST 是 Keil 调试器的复位操作，不是板上额外存在一颗 RST 键。超时或取消会话后恢复等待升级，不会自动选择旧 APP；只有明确的正常启动条件或升级完成自动交接条件会尝试跳转。

## APP 的有效性不是单一判断

当前检查分为两层。

| 层次 | 函数 | 检查 |
|---|---|---|
| 记录及正文 | `Boot_CheckAppRecord()` | magic、长度/CRC反码、8～475136字节、CRC字段16位范围、Flash正文CRC |
| 可交接入口 | `Boot_JumpToAPP()` | 初始MSP在SRAM范围且8字节对齐、Reset向量Thumb位、入口在APP分区 |

记录结构依次保存 `magic`、`file_size`、`file_crc`、`file_size_inv`、`file_crc_inv`。`file_crc` 虽使用 32 位字段存储，但有效 CRC 是 16 位；反码对整个 32 位字段取反。

magic 为 `0x41505031`，只在最后写入。擦除态 `0xFFFFFFFF` 不满足 magic，因此仅仅有一些文件数据已经写入并不能通过启动检查。

反码字段检查元数据的一致性，正文 CRC 检查对应长度内的数据。即使它们均正确，也不能证明这份文件是本硬件的可信程序。当前没有产品型号、签名、版本策略，也未校验入口一定落在“声明的文件长度”内部，只检查它处于 APP 分区。

特别注意：`boot_app_record_valid == 1` 不等于一定成功进入业务界面；向量检查和 APP 自身初始化仍可能失败。

## 从 Bootloader 跳转 APP：代码与执行环境

`Boot_JumpToAPP()` 先取得向量表两项：

```c
uint32_t app_sp = *(volatile uint32_t *)APP_BASE_ADDR;
uint32_t app_reset = *(volatile uint32_t *)(APP_BASE_ADDR + 4U);
uint32_t app_entry = app_reset & ~1U;
```

`APP_BASE_ADDR` 是整数地址；`(volatile uint32_t *)` 把它解释为指向 32 位对象的地址；最外层 `*` 才真正读该地址的内容。`volatile` 要求产生相应访问，但不提供合法性验证、互斥或安全认证。加 4 是移动到下一个 32 位向量项。

`app_reset` 最低位为 Thumb 标志，范围比较时清除此位得到实际指令地址，真正交接仍使用带 Thumb 位的原向量。

检查通过后的顺序为：停止已经启用的 TIM3 PWM并复位/关时钟 → PB0切回低电平普通输出 → 反初始化UART → 屏蔽普通中断 → 停SysTick → 清NVIC使能与挂起 → 清SysTick/PendSV挂起 → 设置VTOR → 屏障 → 汇编切栈跳转。

正常开机立即进入 APP 时，PWM 尚未初始化，因此用 `boot_backlight_tim.Instance == TIM3` 判断是否需要停止，不能假定两条入口路径状态相同。

```c
SCB->VTOR = APP_BASE_ADDR;
__DSB();
__ISB();
Boot_EnterApp(app_sp, app_reset);
```

VTOR 是向量表偏移寄存器，决定异常入口从哪里查表。它不修改当前 PC，也不替代 MSP。DSB/ISB 用于使内存操作及后续指令观察到更新后的配置。

当前工程实际使用 ARMCC 汇编，而不是 C 函数指针调用：

```c
__asm void Boot_EnterApp(uint32_t app_sp, uint32_t app_reset)
{
    MSR MSP, r0
    CPSIE I
    BX r1
}
```

参数通过 r0/r1 传入，`MSR MSP` 完成通常由 `__set_MSP()` 表达的栈设置，`BX r1` 转到复位入口。这里没有调用 `__set_MSP()`，也没有 C 的 `AppEntry_t` 类型，不应把通用教程示例写成项目真实代码。切栈后不继续执行依赖旧栈的 C 语句，也不期望返回 Bootloader。

## APP 链接地址和 VTOR 必须配套

实际配置：

```text
Application/MDK-ARM/07_LVGL_FreeRTOS/07_LVGL_FreeRTOS.sct
LR_IROM1 / ER_IROM1：0x0800C000，大小0x00074000
RW_IRAM1：0x20000000，大小0x00020000

Application工程的C预定义宏：
USER_VECT_TAB_ADDRESS
VECT_TAB_OFFSET=0x0000C000U
```

Scatter 中 `*.o (RESET, +First)` 将向量表放在执行区开头。APP `system_stm32f4xx.c` 的 `SystemInit()` 使用上述宏设置 `SCB->VTOR`，Bootloader 交接前也设置一次。只看源文件默认的 `VECT_TAB_OFFSET=0` 会误判，因为该默认值受 `#if !defined` 保护，项目预定义值优先。

链接决定指令、常量和向量中的地址值。把一个按 `0x08000000` 链接的 BIN 搬到 `0x0800C000`，不会自动把文件内部绝对地址全部加上偏移。正确做法是按目标地址重新链接。

VTOR 决定中断查表位置。链接正确却 VTOR 错误，主线程可能暂时工作，但 SysTick、DMA 或其他中断进入旧表中的入口，可能异常或失去调度。因此“看到 main 跑起来”不是中断重定位验收的全部证据。

## 固件怎样从电脑到达 Flash

实际传输链为：

```text
APP AXF → fromelf生成watch_app.bin
 → 电脑YMODEM发送端分包/等待应答/重发
 → Windows蓝牙串口SPP → KT6368A
 → USART1字节流 → Bootloader整包缓存
 → 包CRC和包号检查 → APP区写入
 → 文件读回CRC → 有效记录提交
```

KT6368A 负责无线连接和 UART 透传，不替 STM32 验证 YMODEM 文件或写入内部 Flash。发送端负责组织协议包及按应答重试；Bootloader 负责检查、应答、存储和结束判定。无线分段不等于 YMODEM 包边界，接收端仍按预期长度收齐包体。

手动测试曾使用 Android Serial Bluetooth Terminal，随后使用网页串口终端 `https://serial.baud-dance.com/#/`。Windows 已配对 SPP 时使用网页的“串口”入口连接对应蓝牙 COM；手机应先断开，其他占用该串口的软件也应关闭。端口号随电脑变化，日志不固定写某个 COM 编号。

最终文件传输工具按学习对话记为 **YMODEM Sender**；未取得可核对的版本/发布地址，因此不虚构菜单名称、版本或下载地址。打开实际工具后选择蓝牙串口、115200/8N1和APP BIN，等待接收端C后启动文件传输。普通终端“发送文件”未必实现 YMODEM，不能默认替代。

APP 发起请求的可复制测试帧如下。HEX 模式下只发送这些字节，已经包含 CRLF，不再附加换行：

```text
40 30 37 3A 4C 46 43 2B 4F 54 41 2A 33 38 0D 0A
```

对应 ASCII 为 `@07:LFC+OTA*38\r\n`；回复 `@06:OK:OTA*64\r\n`。`06` 是 ASCII 长度字段，不是 YMODEM 的单字节 ACK `0x06`。两阶段协议不能混淆。

## 实际 YMODEM 接收状态机

### 包结构与三个长度

| 类型 | 首字节 | 包号/反码 | 正文容量 | CRC | 整包字节数 |
|---|---:|---:|---:|---:|---:|
| SOH | 01 | 2字节 | 128 | 2 | 133 |
| STX | 02 | 2字节 | 1024 | 2 | 1029 |

包CRC只覆盖数据区，包含尾部填充，不覆盖首字节和包号。`Boot_CheckPacket()` 检查整包长度、包号反码和 CRC；是否是当前需要的包号由外层处理。

`boot_packet_length` 是整包长度；`data_length` 是128或1024；`valid_size` 是本包属于文件的字节数。这三个数不是同一个含义。

150字节文件使用128字节包时：第一包有效128，第二包有效22，剩余106填充不写Flash。第二包地址为 `0x0800C000 + 0x80 = 0x0800C080`；结束后累计长度150，不是256，也不是133。第二包前22字节属于文件的后22字节，不是“补进第一包之后还要写106字节”。

### 状态与响应

| 当前状态/输入 | 行为 | 应答与后续 |
|---|---|---|
| 未接受文件信息 | 每秒请求CRC模式 | C |
| 第0包：文件名、长度合法 | 先擦记录，再擦APP，expected=1 | ACK |
| 包格式/CRC错 | 不推进文件进度 | NAK |
| 期待的新数据包 | 写有效正文、读回、更新累计CRC/长度/包号 | ACK |
| 上一包重传 | 不重复写Flash，不推进长度/CRC | ACK；首次数据前的第0包重传再补C |
| 非预期包号 | 不接受本包 | NAK，计包序错误 |
| 文件未收齐收到EOT | 不允许结束 | NAK |
| 文件收齐后第一次EOT | eot_seen=1 | NAK |
| 第二次EOT | wait_end_packet=1 | ACK、C |
| 最终空文件名第0包 | 读回CRC、提交记录、复查记录 | ACK |
| 完成后重复最终空包 | 只补应答 | ACK，不重复写记录 |
| 连续两个包外CAN | 清会话状态 | 回到周期C |
| 会话无活动15秒 | 取消并重置 | CAN CAN，随后周期C |
| Flash链路失败 | 记录诊断、取消并重置 | CAN CAN |

数据包号是8位，会由255回绕到0；进入数据阶段后不能把每一个0号包都当成新文件信息。源码先判断结束阶段，再判断是否尚未收文件信息，再判断期望/重复包号，就是为了解决同一包号在不同状态的含义。

`Boot_ParseFileInfo()` 有文件名长度上限63字符，扫描必须停在数据区内。文件长度是十进制ASCII，逐位转换时先检查界限再乘10，避免越界和溢出。可接受长度字符串后空格分隔的附加信息，但当前不据此实施版本或权限策略。

### 写成功以后才推进状态

`Boot_ReceivePacketBody()` 的核心关系：

```c
remaining_size = boot_file_size - boot_file_received_size;
valid_size = data_length;
if(valid_size > remaining_size)
{
  valid_size = remaining_size;
}
boot_flash_status = Boot_FlashWrite(APP_BASE_ADDR + boot_file_received_size, &boot_packet_buffer[3], valid_size);
```

成功路径随后才更新 `boot_file_crc`、`boot_file_received_size` 和 `boot_expected_packet`。ACK 丢失时，发送端重发上一包；因为 expected 已推进，重传进入上一包分支，只补 ACK。这就是为什么“接收到了两次”不能等于“文件长度加两次”。

主循环先收首字节，包体单独阻塞接收最多1000ms。未收齐记 `boot_packet_incomplete_count` 并发NAK；CRC失败记 `boot_packet_bad_count`。这两个计数分别代表未完整接收和内容检查失败，不能混作一个原因。

## Flash 擦除、写入与上锁

新文件信息接受后先调用 `Boot_EraseAppRecord()`，再调用 `Boot_FlashEraseApp()`。前者擦扇区2并读回五个字是否全1，后者固定擦扇区3～7。**当前不是按文件长度计算最少擦除扇区**；即使只传150字节，也会擦整个APP区。

Flash不能像RAM一样任意覆盖。擦除恢复为全1，编程再改变位值；重复使用一块区域前必须按擦除粒度处理。HAL解锁允许发起擦写，上锁用于收尾防止无意操作，并不构成固件认证。

正文使用 `FLASH_TYPEPROGRAM_BYTE` 逐字节编程，立即读回比较，所以不足4字节的文件尾部不需要自行拼成字写。记录字段使用 `FLASH_TYPEPROGRAM_WORD`，地址依次为记录基址加0、4、8、12、16，满足32位字段对齐。`FLASH_VOLTAGE_RANGE_3` 对应MCU供电范围选择，不能直接以锂电池端电压代替芯片供电判断。

越界检查先验证 `address` 属于APP区，再比较 `length > APP_FLASH_END - address`。采用剩余空间相减而不是直接判断 `address + length`，避免加法溢出掩盖越界。

每个成功解锁后的擦写流程都经过重新上锁。循环中编程失败或读回不符会 `break`；它只退出最近的循环，后面的 `HAL_FLASH_Lock()` 仍会执行。`return` 才是离开函数，所以不能在这里把两者混为一谈。失败返回后上层发送CAN CAN、清协议状态，而不是带着半包继续ACK。

## 包CRC、文件CRC和有效记录

CRC16使用多项式 `0x1021`、初值0；`Boot_CRC16Update()` 可以沿上一段的结果继续计算。包CRC每包从0开始，文件CRC只累计新接受的有效正文，重复包不重复累计。

最终空包触发三步：读取APP Flash有效长度计算CRC，与接收累计值比较 → 写记录 → 再按开机相同规则读记录和正文检查。文件CRC是接收内容自身的完整性记录，不是从可信发布方取得的独立签名。

提交记录先写四个信息字段，全部编程/读回成功后才写magic：

```c
if(status == HAL_OK)
{
  status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, APP_RECORD_ADDR, APP_RECORD_MAGIC);
  /* 随后检查返回状态和magic读回值。 */
}
```

此处为节选，省略分支不代表真实函数只有这几行。若前面的字段写失败，status已不是HAL_OK，跳过magic，但仍执行后面的上锁。先写magic会让中途断电时的记录更容易看起来“已经完成”，因此顺序是机制的一部分。

CRC能够发现许多传输/存储错误，但存在碰撞，也不证明作者可信。攻击者可以自行重算CRC。当前没有数字签名、公钥校验或固件加密，不能写成安全启动或量产级防篡改。

## 取消、超时、断电与恢复边界

`Boot_ResetReception()` 清文件信息、长度、累计CRC、包号、EOT/结束标志及自动跳转请求；保留Flash内容和累计诊断计数。它不复位MCU，也不恢复被擦掉的旧文件。

15秒超时只在已接受文件信息且未完成时生效，依据 `boot_last_activity_tick` 判断。它是串口活动间隔，不是整份文件必须15秒传完，也不是“有效包进度”超时：其他收到的字节也会刷新时间。包体接收返回后再次更新时间，避免把一次阻塞接收期间全部算为闲置。

最终ACK发送成功才置 `boot_auto_jump_pending`，串口安静10秒后清此标志并尝试进入APP。新字节会延后交接。若入口无效，跳转函数返回；当前没有在这个分支自动重新发起一个新的升级会话，诊断时需要区分“协议完成”和“APP启动成功”。

| 中断时刻 | 数据可能怎样 | 下次启动的判断 |
|---|---|---|
| 尚未接受新文件信息 | 旧记录和旧APP通常仍在 | 以实际记录/CRC为准 |
| 已擦记录、尚未完成写入 | 旧APP可能已擦除或被部分覆盖 | 无有效记录，留驻等待重新升级 |
| 正文已收齐但magic未提交 | 数据可能完整，仍未正式提交 | 不因正文看似存在而启动 |
| 记录提交且全链检查通过 | 存在候选有效APP | 仍需启动时检查正文及跳转时检查向量 |

上表是代码设计推导，不是所有断电时刻均已实测。记录擦除/编程过程的电源跌落、Bootloader自身供电稳定性及恢复连接仍需专项测试。本项目的恢复能力是保留升级入口、可重新传完整文件，不是恢复旧版本或从中断字节续传。

## APP 请求进入 Bootloader：最终采用软件交接

APP处理链是 `BLE_Protocol_Parse → BLE_Command_GetType → BLETask → ControlEventQueue → ControlTask → APP_JumpToBootloader`。

BLETask先组 `OK:OTA` 并阻塞发送，HAL_OK后才投递 `CONTROL_EVENT_ENTER_OTA`。HAL_OK仅说明MCU侧UART发送完成，不保证电脑已收到。入队失败则在已发OK之后补 `ERR:OTA_QUEUE`；组帧/发送失败不入队。

ControlTask延时 `osKernelGetTickFreq()/2`，约500ms，给模块转发应答。随后检查当前是特权线程上下文、Bootloader栈顶合法、复位入口Thumb位和Boot区范围合法。通过后才保存PRIMASK并屏蔽普通中断。

```c
RTC->BKP1R = APP_BOOT_REQUEST_MAGIC;
__DSB();
if(RTC->BKP1R != APP_BOOT_REQUEST_MAGIC)
{
  HAL_PWR_DisableBkUpAccess();
  __set_PRIMASK(saved_primask);
  return;
}
```

BKP1R请求值为 `0x4F544131`。先写后读回，失败时尚未拆除APP运行环境，可以恢复中断并返回。随后保持POWER_EN、禁用外部看门狗、停SysTick、复位DMA1/2以及USART1/SPI1/ADC/TIM2/TIM3，清EXTI和NVIC及系统异常挂起，设置Bootloader VTOR。

最后由 `APP_EnterBootloader()` 清BASEPRI、设置MSP、把CONTROL归零，并跳向Bootloader Reset_Handler。FreeRTOS任务通常使用PSP，Bootloader裸机需要MSP；只换PC却不交接栈和线程控制状态不能替代这一步。

Bootloader启动时读取匹配的BKP1R，清零并置 `boot_stay_requested=1`，即“一次性消费”。之后再正常启动且不按KEY1时，这个已清除请求不会永远强迫留驻。普通全局变量会在目标程序C初始化中被重置，不能替代跨交接请求；RTC备份域也不是永不丢失，备份电源丢失或备份域复位仍可能清除内容。

历史方案中的整机复位曾使电源保持丢失，表现为必须手动按电源才能继续。最终源码不调用整机复位来完成OTA切换，而是保留GPIO电平的软件交接，并在Bootloader SystemInit早期接管PA3。因此“发命令后无需手动开机、重连后持续收到C”才是这条链的关键实板结果。

## 中断、DMA与两个运行环境

| 项目 | Application | Bootloader |
|---|---|---|
| 执行模型 | FreeRTOS任务 | 裸机main循环 |
| USART接收 | DMA+空闲事件，回调入字节队列 | HAL阻塞接收首字节/包体 |
| 协议 | 行结束的LFC命令 | 二进制YMODEM |
| 时间基准 | HAL TIM2与RTOS SysTick | HAL SysTick |
| 固件Flash操作 | 不负责 | main上下文执行 |

APP回调只搬字节和恢复接收，BLETask才解释命令。不能把Flash擦写、完整协议状态机放进UART中断，也不能直接从中断跳向另一个程序：异常返回现场和任务栈不会因此自动消失。

交接前复位DMA是为了避免旧传输在Bootloader重新初始化RAM时仍写入内存。停止节拍和清挂起则防止新向量表与旧任务调度状态交叉使用。NVIC的外设IRQ与SysTick/PendSV分属不同控制位置，清一组并不等于清掉全部。

Bootloader没有RAM驻留Flash写入算法；擦写内部Flash期间取指可能停顿，不能要求背光/轮询持续准时。协议安排在接收完整包后写入，写完再ACK，让正常发送端等应答后继续；不能由此推断在任意连续灌入或噪声条件下都不会丢字节。

## 关键文件、函数和变量速查

以下路径以项目根目录为基准，函数名可直接搜索。

| 文件 | 入口/状态 | 位置与复习重点 |
|---|---|---|
| BootLoader/Core/Src/main.c | `main` | 消费请求、有效检查、留驻与主循环 |
| 同上 | `Boot_CheckPacket` / `Boot_ParseFileInfo` | 格式CRC与业务文件信息分层 |
| 同上 | `Boot_ReceivePacketBody` | 状态分支顺序、新包/重复包/结束空包 |
| 同上 | `Boot_HandleEOT` | 文件收齐才允许两次EOT握手 |
| 同上 | `Boot_FlashWrite` | 字节编程、边界、读回、失败上锁 |
| 同上 | `Boot_FlashEraseApp` / `Boot_EraseAppRecord` | 先使旧记录失效再擦APP |
| 同上 | `Boot_CRC16Update` / `Boot_CRC16` | 分段文件CRC与独立包CRC |
| 同上 | `Boot_WriteAppRecord` / `Boot_CheckAppRecord` | magic最后写与启动时检查 |
| 同上 | `Boot_ResetReception` / `Boot_AbortFlashTransfer` | 状态清理不等于回滚Flash |
| 同上 | `Boot_CheckReceiveTimeout` | 15秒活动超时 |
| 同上 | `Boot_JumpToAPP` / `Boot_EnterApp` | 向量验证、环境清理和汇编交接 |
| BootLoader/Core/Src/system_stm32f4xx.c | `SystemInit` | PA3尽早保持供电；重新生成须核对自定义段 |
| Application/Core/Src/freertos.c | `StartBLETask` / `StartControlTask` | 应答与交接分属两个任务 |
| 同上 | `APP_JumpToBootloader` / `APP_EnterBootloader` | BKP1R、保持供电、DMA和任务栈清理 |
| Application/Core/Src/ble_command.c | `BLE_Command_GetType` | 精确7字节LFC+OTA识别 |
| Application/Core/Src/ble_protocol.c | `BLE_Protocol_Parse/Build` | LFC文本帧，不是YMODEM解析器 |
| Application/Core/Src/system_stm32f4xx.c、MDK-ARM工程 | `VECT_TAB_OFFSET` | 项目宏与源码条件共同决定VTOR |
| 两份MDK-ARM输出目录下.sct | `LR_IROM1/ER_IROM1` | 编译地址与Flash分区一致 |

调试时优先观察下列变量，而不是仅观察屏幕。

| 变量 | 含义/生命周期 |
|---|---|
| boot_loop_count | 主循环累计次数 |
| boot_rx_count | 主循环接收到的首字节/控制字节；不含另一HAL调用收到的包体 |
| boot_packet_ok_count/bad_count/incomplete_count | 新包接受、检查失败、包体不完整；会话取消不清历史计数 |
| boot_expected_packet | 期待的新包号；8位回绕 |
| boot_packet_duplicate_count | 上一包重传累计次数 |
| boot_file_size/boot_file_received_size | 声明长度/有效正文进度 |
| boot_last_data_size | 最近新数据包有效字节数 |
| boot_file_crc/boot_flash_read_crc | 接收累计/Flash整文件读回CRC |
| boot_eot_seen/boot_wait_end_packet | 两阶段结束握手状态 |
| boot_transfer_complete | 记录与正文已提交并复查，不保证APP界面运行 |
| boot_auto_jump_pending | 最终ACK后等待安静窗口 |
| boot_flash_status/boot_flash_write_with_error_addr | 最近Flash结果及失败地址 |
| boot_app_record_valid/boot_stay_requested | 固件有效性与本次启动策略，含义独立 |

## 真实开发问题与排查过程

### 长HEX文本发送没有回应

现象：`?`测试有响应，整包没有，网页一度显示ASCII发送398字节。关键证据是发送模式和长度，而不是“蓝牙已连接”。133字节二进制包用两位HEX及空格写成文本时会膨胀，ASCII模式发送的是字符，不是原始包。

修正操作是切到HEX，取消额外行尾并核对实际发送字节数。后续截图明确出现133字节HEX包及ACK/NAK。手机微信的视觉自动折行不一定等于插入换行；截图中的控制字符和早期ASCII模式只能证明需核查实际发送内容，不能断言微信一定修改了每一包。最终转到电脑发送减少人工复制的干扰。

### 复制文件信息包却收到NAK

截图显示文件信息包只有132字节，而SOH应为133，尾部为D9 12。这直接说明发送长度不足，不能首先怀疑蓝牙或CRC算法。补齐正确包并重新测试后用户确认正常。教训是先核对模式和长度，再检查CRC与接收状态，避免拿一份少字节的包反复调固件。

### 改CRC后bad_count增加

有效包末尾D0 53被改为D0 52，截图回复0x15，用户确认bad_count增加而其他进度不变。该证据证明包CRC错误会拒绝并保留进度，不等于已经验证恶意固件、整文件Flash损坏或数字签名。

### 调试器停在初始化行或HAL UART内部

学习中出现“画面还在HAL_Init”“Watch没变”和停在ORE判断附近。源码窗口显示位置不能单独证明CPU仍暂停；一帧UART源码截图也不能单独证明已经发生溢出。应确认运行/暂停状态，观察主循环断点、接收计数和HAL返回值。

最后能命中 `boot_loop_count++`，继续运行后恢复周期C，说明Bootloader已到主循环。不要仅因黑屏或上一暂停行没有刷新而重写启动逻辑。

### 有效记录为0，记录地址看起来是指令

曾在0x08008000看到类似指令内容，`record_valid`也是0。用户随后确认修改没有重新编译到下载产物，重新处理后恢复预期。最终记录区与链接区已分开，但不能把这一张历史截图直接归因为最终版本的分区覆盖。先核对当前工程、保存、编译和实际下载文件是一条重要调试顺序。

### 取消后黑屏、无法继续传输

当时Keil输出反复显示Programming Done、Verify OK、Application running，同时屏幕黑。最后通过Bootloader主循环断点与周期C确认程序在运行，背光只是设置得极暗。这里的Application running是下载工具启动目标的提示，不代表已经进入手表Application。

学习期间还出现电源线断开并重新焊接；这是实际硬件事件，但没有电源波形证明它是所有黑屏现象的共同根因。日志不将独立现象强行合并。

### 超时后又开始发C

用户确认等待15秒收到18 18，随后恢复循环43。这与 `Boot_CheckReceiveTimeout → Boot_ResetReception → Boot_RequestFile`一致，是取消会话并重新等待，不是重启循环故障。CAN取消计数应按一次有效的双CAN序列和累计历史理解，不能看到数值2就直接断言一次取消执行了两次。

### BKP1R无法手动改、断点处留驻标志仍为0

曾收到资源访问错误。随后用程序写入请求、断点跟踪分支，用户确认 `boot_stay_requested`最终为1。停在赋值行代表这条语句尚待执行，不能用执行前的0判断赋值失败。

最终代码明确开启PWR时钟、允许备份域访问、写后读回并关闭访问权限。此前调试器报错的单一硬件原因没有足够证据，不能仅据错误文本断言RTC损坏。

### APP发命令后要手动开机

截图应答解码为OK:OTA，用户最初以为是把请求原样发回。请求与响应正文不同，实际命令已识别。真正后续问题是交接时失去供电保持，必须按电源键才能继续。

最终改为 `APP_JumpToBootloader` 软件交接，保留GPIO、PA3保持、停看门狗与DMA、清理任务运行环境，并在Bootloader早期接管电源。用户最终明确反馈“无需手动操作，重连之后可以持续收到C了”。这是最终设计采用软件跳转而不是整机复位的依据。

### 传完文件没有自动进入APP

用户曾明确反馈传完没有自动跳转，后续确认已可进入APP。最终代码包含最终ACK发送成功标志和10秒串口安静窗口。当前可确认最终实现和成功反馈，但中间每一次修改的完整差异未保留在本日志证据中，因此不把单一原因写成已确定根因。

## 测试记录与证据层级

“通过”只对应所列操作和证据，不扩展为所有边界条件通过。未保存每次测试完整操作的笼统“可以了”，不用于补造精确次数或故障注入结果。

| 项目 | 操作与预期 | 实际证据 | 层级 | 结论 |
|---|---|---|---|---|
| Bootloader留驻 | 命中主循环，继续后周期C | 用户断点截图及恢复C反馈 | 调试器+实板 | 通过 |
| 暗光提示 | 留驻时背光很微弱地闪烁 | 用户明确确认 | 实板 | 通过 |
| 正确SOH包 | HEX完整包后ACK | 截图0x06、后续确认 | 协议实板 | 通过对应包 |
| 错误包CRC | 改D0 53为D0 52，拒绝推进 | 截图NAK、bad_count增加 | 故障注入 | 通过 |
| 132字节信息包 | 不完整包拒绝 | 截图132字节和NAK | 协议异常 | 已观察 |
| 文件名/长度解析 | 观察watch.bin、12345 | Watch截图 | 调试器 | 通过对应样例 |
| 2050字节样例 | 收到ota_test_2050.bin及2050进度 | Watch截图和用户确认 | 调试器/传输 | 通过；不作为APP启动证据 |
| 重复包 | 只ACK不重复累计正文 | 代码明确；对话存在相关预期确认，但缺完整计数截图 | 代码/历史反馈 | 建议保留专项复测，非独立截图验收 |
| 双CAN取消 | 发送18 18回等待状态 | 发送截图及用户确认 | 协议实板 | 通过基本取消 |
| 15秒无活动 | CAN CAN后恢复C | 用户明确描述该顺序 | 超时实板 | 通过 |
| 完整APP升级 | 发送真实APP BIN，随后进入手表 | 多次明确“回到APP/符合预期”反馈 | 完整升级+跳转 | 通过基本链路 |
| APP软件请求 | OK:OTA后无需按电源，重连持续C | 最终用户明确确认 | 实板交接 | 通过 |
| 一次性请求消费 | 执行后留驻标志1 | 断点截图及确认 | 调试器 | 通过置位；完整跨复位专项记录不齐 |
| 非法MSP/入口 | 向量检查应拒绝 | 当前代码可确认，缺专项注入结果 | 代码审阅 | 实板未独立验证 |
| 超大/零长度固件 | 解析应拒绝 | 代码检查存在，缺明确发送记录 | 代码审阅 | 未独立验证 |
| Flash编程失败 | CAN CAN并保留错误地址 | 代码路径存在 | 代码审阅 | 未故障注入 |
| 整文件读回CRC失败 | 不提交有效记录 | 代码路径存在 | 代码审阅 | 未独立故障注入 |
| APP全部中断/任务 | 升级后界面工作 | 手表界面反馈，非各IRQ/任务全覆盖 | 实板 | 基本运行通过，专项覆盖未做 |
| 第二次/第三次连续升级 | 每轮均可请求、发送、运行 | 有重复操作反馈，没有完整逐轮记录 | 历史反馈 | 不声称固定轮数/压力测试通过 |
| 升级中掉电 | 重启拒绝残缺APP并可重传 | 无受控断电时刻测试记录 | 设计分析 | 未测试 |
| Bootloader硬件写保护 | Option Bytes读回验证 | 未取得配置读回证据 | 无 | 未验证启用 |

### 编译与产物证据

注释整理阶段的完整重编译记录：Bootloader为0错误、1条既有 `boot_file_name` 未使用警告；APP全量为0错误、286条既有第三方/生成代码警告。最近增量编译APP为0错误0警告，只重新编译受影响文件，不能替代全量告警数。

当时比较注释整理前后输出，固件逐字节一致：

```text
Bootloader BIN：10180字节
SHA256：2A0001D8844B1446AE1367D70970F0DCF4A59E9666AAECA4B061ABA7591D3E4D
Application BIN：358060字节
SHA256：E799A8F1D5D160689BA24B3059B2BB6BEAC34801D7D06A13AE3C8B2B4F8E8FAB
```

编译日志与比较产物保存在上一级 `素材/13_BootLoader_OTA/注释整理_20260919`。这些散列用于定位本次构建，不是设备端的签名校验，也不证明用户之前每张实板截图都使用了这个完全相同的二进制。本次日志编写未重新实施硬件验证。

## 三条完整控制与数据链

### 正常启动

```text
主Flash启动 → Boot Reset_Handler → 供电保持与C初始化
 → 消费一次性请求 → Flash记录/正文检查 → KEY1留驻判断
 → 无请求且记录有效 → 向量检查 → 清理UART/PWM/中断
 → VTOR/MSP/Reset_Handler交接 → APP SystemInit/__main/main → RTOS手表
```

### 升级

```text
APP LFC+OTA → OK:OTA → ControlEventQueue → ControlTask延时
 → BKP1R写请求 → 保电与软件交接 → Boot消费请求并留驻
 → 重启蓝牙、电脑重连 → C → 信息包 → 记录失效 → 擦APP
 → 数据包CRC/包号 → 写有效字节/读回 → 更新长度/CRC → ACK
 → EOT/NAK/EOT/ACK+C → 最终空包
 → Flash整文件CRC → 信息字段 → magic → 再检查 → ACK
 → 10秒串口安静 → 尝试进入APP
```

### 错误处理

```text
格式/包CRC/包号错误 → NAK，保留已接受进度，等待重传
双CAN / 15秒超时 → 清会话状态 → 周期C，等待重新开始
Flash或整文件校验失败 → CAN CAN → 清会话状态 → 周期C
复位后记录无效 → 留驻；不自动回滚、不自动补齐文件
```

这三条链说明：普通坏包不是立即放弃整个升级；Flash失败才走取消链。不能把它们画成同一种“任何错误都复位并启动旧APP”。

## 已知限制、潜在问题与不一致

1. 单APP分区，没有旧固件备份、A/B切换、自动回滚或断点续传。新信息包接受后就可能失去旧APP。
2. 包CRC、读回CRC和magic都是教学级完整性机制，没有固件发布者认证、加密、防降级、目标硬件型号验证。
3. 接收器软件限定Bootloader擦写范围，但没有取得硬件写保护已设置的证据。
4. `Boot_ParseFileInfo()`允许1～7字节的非零文件信息通过，而 `Boot_WriteAppRecord()`和启动检查要求至少8字节。这样的文件可能先触发擦除，最终才被拒绝提交。这是代码确认的边界不一致，本次只记录，未修改。
5. `Boot_CheckAppRecord()`与跳转向量检查是分开的。合法CRC文件若入口不合理，可以完成记录提交但不能成功交接；最终自动跳转标志已清，当前没有该失败情形的专门自动恢复协议。应安排受控非法向量测试再评估行为。
6. `OK:OTA`先发后入队，入队失败时会出现OK后跟ERR:OTA_QUEUE。应答含义是已识别命令，不是已经成功交接；当前没有完整的上位机事务确认或并发OTA请求去重机制。
7. 15秒看的是串口活动，不是有效文件进度；持续无效输入可能延后超时。没有针对恶意输入的强健性测试。
8. 背光和轮询可能被Flash擦写/阻塞收包延后，没有长时间无线压力、各类丢包组合和电源波动测试。
9. 软件交接专用于当前已清理的外设与上下文；未来APP新增DMA、定时器、中断或供电控制后必须重新检查清理清单。它不是可随意复用到任意APP的通用跳转器。
10. Bootloader的早期PA3修改位于无USER CODE保护的system文件，需要在CubeMX再生成后核对。工程名、版本宏仍有继承旧名，本文按真实配置记录，没有顺手改名。
11. 受控掉电测试没有完成。偶然断电、接线故障与受控覆盖记录擦除/正文编程/magic提交各窗口不是同一个验证层级。
12. 发送工具的确切版本和下载来源未确认；历史对话部分只有“符合预期”而无完整操作/数值，不能反向补写成有精确次数的验收表。

## 项目总结与复习路线

这个项目把两类知识连了起来：一类是链接地址、向量表、栈和启动代码；另一类是文件传输、Flash粒度、状态提交和错误恢复。前者决定收到的固件能否执行，后者决定什么时候允许执行。

建议复习顺序：先对照两份Scatter画出Flash地图；然后从Bootloader main追踪留驻条件；读记录检查和跳转；再用150字节例子追踪收包、写入和重复包；最后沿APP的LFC+OTA命令追到软件交接。

以后自查时只需围绕这些核心问题，不必再做一次冗长项目考核：

- 链接地址、VTOR、MSP和Reset_Handler分别解决什么问题？为什么其中一个不能代替另外几个？
- 为什么133字节的包只可能提供128字节正文？150字节文件第二包实际写多少、从哪里写？
- ACK丢失后为什么只补ACK，不再增加received_size？
- 为什么EOT不等于记录已经提交，为什么magic最后写？
- break与return对Flash重新上锁有什么不同影响？
- 为什么APP返回Bootloader要清DMA、BASEPRI和CONTROL，却保留GPIO供电状态？
- 取消或中途掉电之后，哪些数据仍在，哪些旧数据不能恢复？
- CRC正确能证明什么，不能证明什么？

### 资料与追溯入口

- 风格参考：项目10 Sensor Hub、项目11 BLE Communication、项目12 Low Power的开发日志；以它们“结合机制、关键代码、真实问题和边界”的写法组织本文。
- 实现依据：本项目Bootloader main/system/GPIO/UART/启动文件；Application freertos、ble_command、ble_protocol、rtc、system、启动文件；两份uvprojx和sct。
- 硬件扇区核对：ST RM0383链接见第5节。
- 实板依据：本任务已有学习对话中的截图、用户明确测试反馈；未补造日期、轮次、工具版本和缺失的故障注入结果。
- 本次操作边界：仅创建开发日志，不修改Bootloader或APP代码，不调整分区，不自动修复第21节问题，不开始项目考核。
