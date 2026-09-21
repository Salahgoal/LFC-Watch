/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  *
  * 本文件是LFC-WATCH 13_BootLoader_OTA工程的APP任务与跨任务通信中心。
  * 保留12_LowPower的UI、传感器、蓝牙、分级休眠和外部看门狗职责，
  * 新增LFC+OTA命令链路：BLETask应答 -> ControlEventQueue -> ControlTask
  * -> RTC备份寄存器请求标记 -> 保持供电并软件跳转到Bootloader。
  * YMODEM收包和Flash提交由独立Bootloader完成，不在APP任务中执行。
  *
  * 任务分工：
  * 1. DefaultTask周期读取RTC和电池数据，再通过UiEventQueue更新界面。
  * 2. PowerTask维持POWER_EN电源自锁，正常关机时由ControlTask终止。
  * 3. UiTask独占LVGL，负责界面更新、空闲时间和显示电源状态转换。
  * 4. KeyScanTask识别WAKE长按并向ControlTask发送关机请求。
  * 5. ControlTask管理背光PWM、STOP与唤醒恢复、关机及OTA软件交接。
  * 6. SensorTask独占背板软件I2C，管理传感器模式、抬腕采样和心率测量。
  * 7. BLETask独占USART1蓝牙业务，处理模块电源、DMA、LFC协议和数据传输。
  * 8. WatchdogTask在正常运行时周期翻转WDI，STOP期间由系统暂停外部看门狗。
  *
  * 低功耗主链：
  * LVGL空闲时间 -> UiTask在5秒暂停页面传感器并请求暗屏
  *     -> 15秒时让LCD、触摸休眠 -> ControlEventQueue -> ControlTask
  *     -> ControlTask关闭背光并进入STM32 STOP
  *     -> KEY1直接唤醒 / RTC周期唤醒后请求SensorTask检查抬腕
  *     -> 恢复系统时钟、HAL时基、FreeRTOS调度、LCD、触摸和传感器状态。
  *
  * 蓝牙开启时不进入STOP，只关闭背光并保持USART1、DMA和BLETask运行。
  * 心率测量期间UiTask保持显示活跃，SensorTask在得到有效BPM或20秒超时后
  * 关闭EM7028，并通过UiEventQueue发布测量结果或状态。
  *
  * 蓝牙命令接收链路：
  * 手机 -> KT6368A -> USART1 RX DMA -> HAL_UARTEx_RxEventCallback()
  *      -> BLERXQueue -> BLETask行缓冲 -> BLE_Protocol_Parse()
  *      -> BLE_Command_GetType() -> 命令执行 -> BLE_Protocol_Build()
  *      -> USART1 -> KT6368A -> 手机
  *
  * 传感器蓝牙数据链路：
  * SensorTask -> BLESensorQueue -> BLETask最新值缓存
  *      -> LFC+SEND快照回复 / LFC+ENV=ON连续推送 -> USART1 -> 手机
  *
  * 蓝牙电源与状态链路：
  * SquareLine开关 -> APP_BLEPowerChanged() -> BLETask线程标志
  *      -> BLE_EN与USART1 DMA启停 -> UiEventQueue -> UiTask -> OFF / READY / ACTIVE
  *
  * 关键约束：
  * 1. UART DMA回调只快速搬运字节并重启接收，不在回调中组帧或执行命令。
  * 2. BLETask只使用SensorTask发来的数据副本，不跨越任务直接访问传感器或I2C。
  * 3. SensorTask通过env_ui_requested与env_ble_requested合并UI和蓝牙对AHT21的需求。
  * 4. ACTIVE只表示已成功收到过合法LFC帧，不等同于KT6368A的实时物理连接状态。
  * 5. 只有UiTask可以调用LVGL API；其他任务通过队列或线程标志传递数据与请求。
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>

/* 系统硬件与基础外设。 */
#include "lcd.h" // ST7789底层显示驱动
#include "usart.h" // USART1蓝牙业务通信接口
#include "tim.h" // TIM3 PWM句柄，用于控制LCD背光
#include "adc.h" // ADC1句柄，用于读取电池分压
#include "rtc.h" // RTC句柄，用于读取日期和时间

/* LVGL、触摸输入与SquareLine界面。 */
#include "lvgl.h" // LVGL图形库核心接口
#include "touch.h" // CST816触摸芯片驱动
#include "lv_port_disp.h" // LVGL显示端口
#include "lv_port_indev.h" // LVGL触摸输入端口
#include "ui.h" // SquareLine生成的界面声明
#include "app_ui_bridge.h" // SquareLine界面层调用的应用接口

/* RTC、设置持久化与系统数据。 */
#include "rtc_service.h" // RTC日期时间读取服务
#include "bl24c02.h" // BL24C02 EEPROM底层驱动
#include "settings_service.h" // 亮度设置读取与保存服务
#include "system_data.h" // 系统日期、时间、电池和心率数据结构

/* 背板传感器驱动与数据处理服务。 */
#include "sensor_i2c.h" // 背板传感器软件I2C总线
#include "mpu6050.h" // MPU6050陀螺仪/加速度计驱动
#include "mpu6050_service.h" // MPU6050单位换算、计步和抬腕服务
#include "aht21.h" // AHT21温湿度传感器驱动
#include "aht21_service.h" // AHT21原始数据的物理单位换算
#include "spl06.h" // SPL06气压传感器驱动
#include "spl06_service.h" // SPL06原始数据的物理单位换算
#include "lsm303.h" // LSM303加速度计和磁力计驱动
#include "lsm303_service.h" // LSM303磁场校准与指南针计算服务
#include "em7028.h" // EM7028心率传感器驱动
#include "em7028_service.h" // EM7028心率计算服务

/* LFC-WATCH版本、蓝牙协议与命令层。 */
#include "app_version.h" // 软件版本号定义
#include "ble_protocol.h" // 蓝牙协议解析与封装服务
#include "ble_command.h" // 蓝牙应用层命令解析服务

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define BLE_UART_DMA_RX_SIZE 64U // USART1 DMA单次接收缓冲区容量

/*
 * ControlTask能够处理的控制事件。
 *
 * 发送者将事件编号和参数封装进ControlMessage_t，再放入ControlEventQueue。
 * ControlTask根据event字段执行背光调节、配置保存、暗屏、唤醒、关机或OTA交接。
 */
typedef enum
{
  CONTROL_EVENT_WAKE_LONG = 1U, // WAKE按键长按：执行关机
  CONTROL_EVENT_BRIGHTNESS_CHANGED = 2U, // 亮度变化：更新正常背光PWM
  CONTROL_EVENT_SCREEN_DIM = 3U, // 长时间无操作：降低背光
  CONTROL_EVENT_SCREEN_WAKE = 4U, // 暗屏后触摸：恢复正常背光
  CONTROL_EVENT_BRIGHTNESS_SAVE = 5U, // 滑块释放：保存最终亮度到EEPROM
  CONTROL_EVENT_SCREEN_OFF = 6U, // 二级超时15秒：关闭背光并请求进入STOP
  CONTROL_EVENT_SCREEN_OFF_NONSTOP = 7U, // 蓝牙开启时只关闭背光
  CONTROL_EVENT_ENTER_OTA = 8U, // 写入升级请求并保持供电，软件跳转到Bootloader
} ControlEventCode_t;

/*
 * ControlTask维护的系统级电源模式。
 * 它描述CPU和整机运行状态，不等同于UiTask内部的屏幕显示状态。
 */
typedef enum
{
  SYSTEM_POWER_ACTIVE = 0U, // 正常亮屏，CPU和任务正常运行
  SYSTEM_POWER_DIMMED, // 背光变暗，CPU和任务仍正常运行
  SYSTEM_POWER_SCREEN_OFF_AWAKE, // 背光关闭，但因BLE开启而不进入STOP
  SYSTEM_POWER_STOP, // MCU已经准备进入或正在STOP
  SYSTEM_POWER_SHUTDOWN // 正在释放POWER_EN执行整机关机
} SystemPowerMode_t;

/*
 * STOP唤醒原因。
 * ControlTask根据原因决定完整唤醒，还是完成短暂后台工作后重新休眠。
 */
typedef enum
{
  SYSTEM_WAKE_NONE = 0U,
  SYSTEM_WAKE_KEY1,
  SYSTEM_WAKE_RTC,
  SYSTEM_WAKE_WRIST
} SystemWakeReason_t;

/*
 * UiTask维护的屏幕电源状态。
 * 这里只记录显示策略，不直接代表STM32已经进入STOP。
 */
typedef enum
{
  SCREEN_POWER_ACTIVE = 0U, // 正常亮屏
  SCREEN_POWER_DIMMED,      // 背光降低到1%
  SCREEN_POWER_OFF,         // 显示端已关闭，CPU状态由ControlTask决定
} ScreenPowerState_t;

/*
 * UiTask能够处理的界面数据事件。
 *
 * 事件只表示需要更新的数据类型，不携带LVGL对象指针。
 * UiTask收到消息后，通过SquareLine导出的控件指针完成实际界面更新。
 */
typedef enum
{
  UI_EVENT_HEART_RATE_UPDATE = 1U, // 更新主页心率
  UI_EVENT_TIME_UPDATE = 2U, // 更新主页时间
  UI_EVENT_BATTERY_UPDATE = 3U, // 更新主页电池百分比
  UI_EVENT_DATE_UPDATE = 4U, // 更新主页日期
  UI_EVENT_MPU6050_UPDATE = 5U, // 更新MPU6050传感器数据
  UI_EVENT_AHT21_UPDATE = 6U, // 更新AHT21温湿度数据
  UI_EVENT_SPL06_UPDATE = 7U, // 更新SPL06气压数据
  UI_EVENT_LSM303_UPDATE = 8U, // 更新LSM303磁力计数据
  UI_EVENT_BLE_STATUS_UPDATE = 9U, // 更新蓝牙模块及应用会话状态
  UI_EVENT_HEART_RATE_STATE_UPDATE = 10U, // 更新心率测量过程状态
} UIEventCode_t;

/* 一次心率测量会话的运行状态。 */
typedef enum
{
  HEART_RATE_MEASUREMENT_IDLE = 0U,
  HEART_RATE_MEASUREMENT_RUNNING,
  HEART_RATE_MEASUREMENT_TIMEOUT,
  HEART_RATE_MEASUREMENT_UNAVAILABLE,
} HeartRateMeasurementState_t;

/* ControlTask消息只需要一个事件编号和一个整数参数。 */
typedef struct
{
  uint16_t event; // 控制事件编号
  int32_t value; // 亮度百分比或其他控制参数
} ControlMessage_t;

/*
 * UI消息的载荷使用联合体。
 *
 * RTC、日期、电池和心率使用单个value；MPU6050和AHT21分别使用自己的
 * 数据结构。联合体在同一时刻只保存其中一种数据，由event字段决定
 * UiTask应该按照哪一种格式解释。
 *
 * 这样一次队列消息就能携带同一采样时刻的全部三轴数据，避免将七项
 * 数据拆成七条消息后出现新旧数据混合或占满队列的问题。
*/
typedef union
{
  int32_t value; // 时间、日期、电量和心率等单值数据
  MPU6050_Data_t mpu6050; // MPU6050传感器数据
  AHT21_Data_t aht21; // AHT21温湿度数据
  SPL06_Data_t spl06; // SPL06气压和温度数据
  LSM303_CompassData_t lsm303; // LSM303磁力计数据
} UIMessagePayload_t;

typedef struct
{
  uint16_t event; // UI事件编号
  UIMessagePayload_t payload; // 事件参数
} UIMessage_t;

/* SensorTask发送给BLETask的传感器数据类型。 */
typedef enum
{
  BLE_SENSOR_EVENT_AHT21_UPDATE = 1U, // 最近一次温湿度
  BLE_SENSOR_EVENT_STEP_UPDATE = 2U, // 当前累计步数
  BLE_SENSOR_EVENT_HEART_RATE_UPDATE = 3U, // 最近一次有效心率
} BLE_SensorEventCode_t;

/* event决定payload当前保存哪一种传感器数据。 */
typedef union
{
  AHT21_Data_t aht21; // AHT21温湿度数据
  uint32_t step_count; // MPU6050累计步数
  uint16_t heart_rate_bpm; // EM7028心率数据
} BLE_SensorPayload_t;

typedef struct
{
  uint16_t event; // 表示payload中保存的传感器数据类型
  BLE_SensorPayload_t payload; // 队列传递的传感器数据副本
} BLE_SensorMessage_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEY_SCAN_PERIOD_MS 20U // 按键扫描周期
#define KEY_LONG_PRESS_MS 1500U // WAKE长按关机判定时间

#define SCREEN_IDLE_TIMEOUT_MS 5000U // 无操作达到5秒后进入暗屏
#define SCREEN_OFF_TIMEOUT_MS 15000U // 总空闲15秒后完全关闭背光
#define SCREEN_DIM_PERCENT 1U         // 暗屏时的背光百分比
#define HEART_LONG_PRESS_TIME_MS 2000U // 爱心持续按住2秒后启动一次心率测量
#define HEART_MEASUREMENT_TIMEOUT_S 20U // 单次心率测量最多持续20秒

/* 电池ADC采样与换算参数。 */
#define BATTERY_SAMPLE_COUNT 8U // 每轮电池测量的ADC采样次数
#define BATTERY_UPDATE_PERIOD_S 5U // 电池数据更新周期
#define BATTERY_ADC_REFERENCE_MV 3300UL // ADC参考电压，单位mV
#define BATTERY_DIVIDER_SCALE 2UL // 1:2电阻分压的电压还原倍数

/*
 * ADC换算结果与万用表实测值存在少量系统误差，因此使用1037/1000进行校准。
 * 使用整数分子和分母可以避免在电压换算中引入不必要的浮点运算。
 */
#define BATTERY_CALIBRATION_NUMERATOR 1037UL
#define BATTERY_CALIBRATION_DENOMINATOR 1000UL

#define COMPASS_HEADING_OFFSET_DEGREES 0U // 传感器零度到屏幕顶部方向的固定修正量

/* 页面回调通过线程标志请求SensorTask启停对应传感器。 */
#define SENSOR_FLAG_HEART_RATE_START (1UL << 0) // 请求启动EM7028心率采集
#define SENSOR_FLAG_HEART_RATE_STOP  (1UL << 1) // 请求停止EM7028心率采集
#define SENSOR_FLAG_HEART_RATE_MASK  (SENSOR_FLAG_HEART_RATE_START | SENSOR_FLAG_HEART_RATE_STOP)

#define SENSOR_FLAG_COMPASS_START (1UL << 2) // 请求SensorTask启动LSM303
#define SENSOR_FLAG_COMPASS_STOP  (1UL << 3) // 请求SensorTask停止LSM303
#define SENSOR_FLAG_COMPASS_MASK  (SENSOR_FLAG_COMPASS_START | SENSOR_FLAG_COMPASS_STOP)

#define SENSOR_FLAG_ENV_START (1UL << 4) // 请求启动AHT21和SPL06
#define SENSOR_FLAG_ENV_STOP  (1UL << 5) // 请求停止AHT21和SPL06
#define SENSOR_FLAG_ENV_MASK  (SENSOR_FLAG_ENV_START | SENSOR_FLAG_ENV_STOP)

#define SENSOR_FLAG_BLE_ENV_START (1UL << 6) // BLE请求启动AHT21温湿度采集
#define SENSOR_FLAG_BLE_ENV_STOP  (1UL << 7) // BLE取消AHT21温湿度采集
#define SENSOR_FLAG_BLE_ENV_MASK  (SENSOR_FLAG_BLE_ENV_START | SENSOR_FLAG_BLE_ENV_STOP)

/*
 * BLETask控制请求。
 *
 * 线程标志属于目标任务，每一位表示一种待处理请求。
 * 它只传递“要做什么”，不传递大块数据。
 */
#define BLE_FLAG_POWER_ON   (1UL << 0)
#define BLE_FLAG_POWER_OFF  (1UL << 1)
#define BLE_FLAG_POWER_MASK (BLE_FLAG_POWER_ON | BLE_FLAG_POWER_OFF)

/* SensorTask的MPU6050模式切换与STOP采样请求。 */
#define SENSOR_FLAG_MPU_LOW_POWER (1UL << 8) // 请求MPU6050进入低功耗循环采样
#define SENSOR_FLAG_MPU_NORMAL    (1UL << 9) // 请求MPU6050恢复六轴正常工作
#define SENSOR_FLAG_MPU_POWER_MASK (SENSOR_FLAG_MPU_LOW_POWER | SENSOR_FLAG_MPU_NORMAL)

#define SENSOR_FLAG_MPU_STOP_PROBE (1UL << 10) // RTC唤醒后请求SensorTask执行一次MPU6050采样窗口

#define MPU_STOP_PROBE_SAMPLE_COUNT 3U // RTC唤醒后连续读取3组加速度，约覆盖100ms
#define MPU_STOP_PROBE_UP_CONFIRM_SAMPLES 3U // 3个样本全部为UP才确认看表姿态
#define MPU_STOP_PROBE_REARM_MAX_UP_SAMPLES 0U // 3个样本全部为非UP时才重新允许唤醒
#define MPU_STOP_PROBE_TIMEOUT_TICKS 500U // ControlTask最多等待SensorTask约500ms
#define MPU_STOP_REARM_STABLE_SAMPLES 10U // 低功耗20Hz下连续约500ms非UP才允许抬腕唤醒

/*
 * ControlTask的线程标志。
 * SensorTask完成STOP期间的MPU6050采样窗口后，通过该标志回复ControlTask。
 */
#define CONTROL_FLAG_MPU_STOP_PROBE_DONE (1UL << 0)

/* ControlTask通知UiTask完成STOP后的显示与传感器恢复。 */
#define UI_FLAG_SYSTEM_WAKE (1U << 0) // 通知UiTask完成STOP唤醒后的状态恢复

/*
 * 外部TPS3823看门狗：
 * 0=保持禁用，适合长时间断点调试；
 * 1=正式启用，WatchdogTask必须持续喂狗。
 */
#define EXTERNAL_WATCHDOG_ENABLED 1U
#define WATCHDOG_FEED_PERIOD_TICKS 100U // 当前1kHz系统节拍下约为100ms

/* BKP1R供APP与Bootloader交接请求；BKP0R仍由RTC日历初始化使用。 */
#define APP_BOOT_REQUEST_MAGIC 0x4F544131U // 必须与Bootloader的BOOT_REQUEST_MAGIC一致

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* 主页爱心长按识别状态，只在UiTask的LVGL回调中访问。 */
static uint32_t heart_press_start_tick = 0U; // 本次按下爱心时的LVGL时刻
static uint8_t heart_press_tracking = 0U; // 1表示正在判断本次按压是否达到2秒

/* USART1 RX DMA缓冲区与字节队列诊断计数。 */
static uint8_t ble_rx_dma_buffer[BLE_UART_DMA_RX_SIZE] = {0}; // USART1 RX DMA单次接收缓冲区
static volatile uint32_t ble_rx_count = 0U; // 成功放入BLERXQueue的字节数
static volatile uint32_t ble_rx_drop_count = 0U; // BLERXQueue无效或已满时丢弃的字节数

/* BLETask行组帧、命令与应用会话诊断计数。 */
static volatile uint32_t ble_line_count = 0U; // 成功接收的完整数据行数
static volatile uint32_t ble_unknown_cmd_count = 0U; // 收到未知LFC命令的次数
static volatile uint32_t ble_line_overflow_count = 0U; // 接收行超过行缓冲区容量的次数
static volatile uint8_t ble_app_session_active = 0U; // 1表示已成功收到过合法LFC帧
static volatile uint32_t ble_connect_event_count = 0U; // 应用会话进入ACTIVE的次数
static volatile uint32_t ble_disconnect_event_count = 0U; // 模块错误触发应用会话清除的次数

/* USART1 DMA启动、回调与错误恢复诊断数据，供Keil Watch观察。 */
volatile HAL_StatusTypeDef ble_dma_start_status = HAL_ERROR; // BLETask最近一次启动DMA的结果
volatile HAL_StatusTypeDef ble_dma_restart_status = HAL_ERROR; // 回调最近一次重启DMA的结果
volatile HAL_StatusTypeDef ble_uart_deinit_status = HAL_ERROR; // 最近一次反初始化USART1的结果
volatile uint32_t ble_uart_init_count = 0U; // BLE重新开启时初始化USART1的次数
volatile uint32_t ble_dma_callback_count = 0U; // RX空闲或满缓冲接收回调次数
volatile uint16_t ble_dma_last_size = 0U; // 最近一次回调中的有效字节数
volatile uint32_t ble_uart_error_count = 0U; // USART1接收错误回调次数
volatile uint32_t ble_uart_last_error = HAL_UART_ERROR_NONE; // 最近一次USART1错误码

/* 设置页蓝牙开关请求诊断计数。 */
volatile uint32_t ble_power_on_request_count = 0U; // BLETask处理的开启请求次数
volatile uint32_t ble_power_off_request_count = 0U; // BLETask处理的关闭请求次数

/* STOP、触摸恢复和蓝牙阻止STOP的观测状态。 */
static volatile uint8_t key1_wake_pending = 0U; // KEY1中断已经唤醒CPU
volatile uint8_t touch_sleep_write_ok = 0U; // 最近一次休眠命令：1=写入成功，0=失败
volatile uint8_t touch_wake_chip_id = 0U; // 最近一次复位唤醒后读到的芯片ID
volatile uint32_t ble_stop_block_count = 0U; // 蓝牙开启而跳过STOP的次数

/*
 * 蓝牙模块的目标电源状态。
 *
 * BLETask和UART中断回调都会读取它，因此使用volatile。
 * 1=模块应该保持开启，0=模块正在关闭或已经关闭。
 */
static volatile uint8_t ble_module_enabled = 0U;

/*
 * 只有ControlTask能够修改系统电源模式。
 * 使用volatile是为了让Keil Watch能够随时读取最新值。
 */
volatile SystemPowerMode_t system_power_mode = SYSTEM_POWER_ACTIVE;
volatile uint32_t stop_unexpected_wake_count = 0U; // 无法归因于KEY1或RTC的STOP返回次数

/* RTC周期唤醒状态与HAL调用结果。 */
volatile uint8_t rtc_wake_pending = 0U; // RTC WakeUp中断已经发生
volatile uint32_t rtc_wakeup_irq_count = 0U; // RTC WakeUp中断总次数
volatile uint32_t rtc_stop_wake_count = 0U; // STOP期间由RTC唤醒的次数
volatile HAL_StatusTypeDef rtc_wakeup_start_status = HAL_ERROR; // 最近一次启动RTC WakeUp定时器的结果
volatile HAL_StatusTypeDef rtc_wakeup_stop_status = HAL_ERROR; // 最近一次停止RTC WakeUp定时器的结果

/* STOP期间由RTC唤醒后执行MPU6050采样窗口的观测数据。 */
volatile uint32_t mpu_stop_probe_complete_count = 0U; // SensorTask完成RTC后台读取的次数
volatile uint32_t mpu_stop_probe_timeout_count = 0U; // ControlTask等待SensorTask超时的次数
volatile uint8_t mpu_stop_probe_read_ok = 0U; // 最近一次后台读取是否成功
volatile int32_t mpu_stop_probe_accel_x_mg = 0; // 最近一次后台读取的X轴加速度
volatile int32_t mpu_stop_probe_accel_y_mg = 0; // 最近一次后台读取的Y轴加速度
volatile int32_t mpu_stop_probe_accel_z_mg = 0; // 最近一次后台读取的Z轴加速度

volatile uint8_t mpu_stop_probe_successful_samples = 0U; // 最近一个窗口成功读取的样本数
volatile uint8_t mpu_stop_probe_wrist_event = 0U; // 最近一个窗口是否识别到新抬腕
volatile uint32_t mpu_stop_wrist_event_count = 0U; // STOP后台累计识别到的抬腕次数
volatile uint8_t mpu_stop_probe_up_samples = 0U; // 最近一个采样窗口中的UP样本数
volatile uint8_t mpu_stop_wrist_armed = 0U; // 1表示已经离开看表姿态，允许下一次UP唤醒
volatile SystemWakeReason_t system_last_wake_reason = SYSTEM_WAKE_NONE; // 最近一次有效的STOP唤醒原因

/* 外部看门狗喂狗诊断计数。 */
volatile uint32_t watchdog_feed_count = 0U; // WatchdogTask完成喂狗电平翻转的次数

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PowerTask */
osThreadId_t PowerTaskHandle;
const osThreadAttr_t PowerTask_attributes = {
  .name = "PowerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UiTask */
osThreadId_t UiTaskHandle;
const osThreadAttr_t UiTask_attributes = {
  .name = "UiTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for KeyScanTask */
osThreadId_t KeyScanTaskHandle;
const osThreadAttr_t KeyScanTask_attributes = {
  .name = "KeyScanTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BLETask */
osThreadId_t BLETaskHandle;
const osThreadAttr_t BLETask_attributes = {
  .name = "BLETask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for WatchdogTask */
osThreadId_t WatchdogTaskHandle;
const osThreadAttr_t WatchdogTask_attributes = {
  .name = "WatchdogTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UiEventQueue */
osMessageQueueId_t UiEventQueueHandle;
const osMessageQueueAttr_t UiEventQueue_attributes = {
  .name = "UiEventQueue"
};
/* Definitions for ControlEventQueue */
osMessageQueueId_t ControlEventQueueHandle;
const osMessageQueueAttr_t ControlEventQueue_attributes = {
  .name = "ControlEventQueue"
};
/* Definitions for BLERXQueue */
osMessageQueueId_t BLERXQueueHandle;
const osMessageQueueAttr_t BLERXQueue_attributes = {
  .name = "BLERXQueue"
};
/* Definitions for BLESensorQueue */
osMessageQueueId_t BLESensorQueueHandle;
const osMessageQueueAttr_t BLESensorQueue_attributes = {
  .name = "BLESensorQueue"
};
/* Definitions for LcdMutex */
osMutexId_t LcdMutexHandle;
const osMutexAttr_t LcdMutex_attributes = {
  .name = "LcdMutex"
};
/* Definitions for RtcMutex */
osMutexId_t RtcMutexHandle;
const osMutexAttr_t RtcMutex_attributes = {
  .name = "RtcMutex"
};
/* Definitions for LcdDmaSem */
osSemaphoreId_t LcdDmaSemHandle;
const osSemaphoreAttr_t LcdDmaSem_attributes = {
  .name = "LcdDmaSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/*
 * ARMCC 5 汇编交接函数。
 * r0：Bootloader向量表第0项，初始MSP。
 * r1：Bootloader向量表第1项，Reset_Handler入口。
 *
 * 只能在特权任务上下文中调用，调用前必须完成外设、中断和向量表清理。
 * 清BASEPRI并将CONTROL归零，使Bootloader在线程模式、特权态下使用MSP。
 * 切换栈后直接进入Reset_Handler，由启动代码重建Bootloader的C运行环境；
 * 不再执行依赖APP任务PSP栈的C代码，也不返回原来的FreeRTOS任务。
 */
__asm void APP_EnterBootloader(uint32_t boot_sp, uint32_t boot_reset)
{
  CPSID I
  MOVS r2, #0
  MSR BASEPRI, r2
  MSR MSP, r0
  MSR CONTROL, r2
  ISB
  DSB
  CPSIE I
  BX r1
}

/*
 * BLETask完成OTA应答并投递事件后，由ControlTask延时约500ms再调用。
 * 保留GPIO输出和RTC备份域，不执行整机复位，避免POWER_EN失去保持导致掉电。
 * 先验证Bootloader向量并写入/读回BKP1R请求；失败时还未拆除APP运行环境，
 * 可以恢复中断并返回。成功后停用外部看门狗、节拍、DMA及相关外设，
 * 清理中断和VTOR，再用汇编交接栈及入口；开始清理后不再恢复原任务。
 * 当前系统时钟保留给Bootloader，由其main()更新SystemCoreClock并配置HSI。
 */
static void APP_JumpToBootloader(void)
{
  uint32_t boot_sp = *(volatile const uint32_t *)0x08000000U; // Bootloader向量表第0项，初始MSP
  uint32_t boot_reset = *(volatile const uint32_t *)0x08000004U; // Bootloader向量表第1项，Reset_Handler入口
  uint32_t boot_entry = boot_reset & ~1U; // 去掉Thumb标志用于地址范围检查，实际跳转仍用boot_reset

  /* 交接失败时恢复原中断屏蔽状态；交接成功则清理NVIC并离开APP。 */
  uint32_t saved_primask;
  uint32_t i;

  /*只允许从特权线程上下文调用，不能从中断里直接跳走*/
  if((__get_IPSR() != 0U) || ((__get_CONTROL() & 1U) != 0U))
  {
    return;
  }

  /*检查Bootloader初始栈顶和复位入口*/
  if((boot_sp <= 0x20000000U) || (boot_sp > 0x20020000U) || ((boot_sp & 7U) != 0U))
  {
    return;
  }

  if(((boot_reset & 1U) == 0U) || (boot_entry < 0x08000000U) || (boot_entry >= 0x08008000U))
  {
    return;
  }

  saved_primask = __get_PRIMASK();
  __disable_irq(); //从此处开始，禁止任务切换和普通中断打断交接

  /*先写入升级请求：失败时还未拆除APP运行环境，可以恢复中断并返回*/
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  RTC->BKP1R = APP_BOOT_REQUEST_MAGIC;
  __DSB();

  if(RTC->BKP1R != APP_BOOT_REQUEST_MAGIC)
  {
    HAL_PWR_DisableBkUpAccess();
    __set_PRIMASK(saved_primask);
    return;
  }

  HAL_PWR_DisableBkUpAccess();

  /*保持电源自锁，并停用外部看门狗，避免交接时期无人喂狗*/
  HAL_GPIO_WritePin(POWER_EN_GPIO_Port, POWER_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(WDOG_EN_GPIO_Port, WDOG_EN_Pin, GPIO_PIN_SET);

  /*停止FreeRTOS调度节拍*/
  SysTick->CTRL = 0U;
  SysTick->LOAD = 0U;
  SysTick->VAL = 0U;

  /*停止DMA，避免Bootloader初始化RAM时，旧DMA仍在读写内存*/
  __HAL_RCC_DMA1_FORCE_RESET();
  __HAL_RCC_DMA2_FORCE_RESET();
  __DSB();
  __HAL_RCC_DMA1_RELEASE_RESET();
  __HAL_RCC_DMA2_RELEASE_RESET();

  /*复位APP使用的通信、采样和定时器外设，不复位GPIO或RTC备份域*/
  __HAL_RCC_USART1_FORCE_RESET();
  __HAL_RCC_SPI1_FORCE_RESET();
  __HAL_RCC_ADC_FORCE_RESET();
  __HAL_RCC_TIM2_FORCE_RESET();
  __HAL_RCC_TIM3_FORCE_RESET();
  __DSB();
  __HAL_RCC_USART1_RELEASE_RESET();
  __HAL_RCC_SPI1_RELEASE_RESET();
  __HAL_RCC_ADC_RELEASE_RESET();
  __HAL_RCC_TIM2_RELEASE_RESET();
  __HAL_RCC_TIM3_RELEASE_RESET();

  /*
   * 清除APP留下的EXTI请求，随后关闭NVIC外设中断并清除挂起位。
   * NVIC清理不包含SysTick和PendSV，所以后面还要单独清除这两个系统异常。
   */
  EXTI->IMR = 0U;
  EXTI->EMR = 0U;
  EXTI->PR = 0xFFFFFFFFU;

  for(i = 0U; i < 3U; i++)
  {
    NVIC->ICER[i] = 0xFFFFFFFFU;
    NVIC->ICPR[i] = 0xFFFFFFFFU;
  }

  SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;

  /*
   * VTOR只决定异常入口到哪里查表，不会改变当前执行位置或栈。
   * 屏障确保向量表切换生效，再由汇编设置MSP并跳到Reset_Handler。
   */
  SCB->VTOR = 0x08000000U;
  __DSB();
  __ISB();

  APP_EnterBootloader(boot_sp, boot_reset);
}

/* 电池采样与换算。 */
static uint8_t Battery_ReadVoltageMv(uint16_t *voltage_mv);
static uint8_t Battery_VoltageToPercent(uint16_t voltage_mv);

/* SquareLine事件转发与页面生命周期。 */
static void APP_BrightnessSliderReleased(lv_event_t *e);
static void APP_HeartIconEvent(lv_event_t *e);
static void APP_HomeScreenLifecycleEvent(lv_event_t *e);
static void APP_CompassScreenLifecycleEvent(lv_event_t *e);
static void APP_EnvScreenLifecycleEvent(lv_event_t *e);
static void APP_SetActivePageSensorsEnabled(uint8_t enabled);

/* UiTask文本格式化工具。 */
static void UI_SetGyroLabel(lv_obj_t *label, const char *axis, int32_t value_mdps);
static void UI_SetTemperatureLabel(lv_obj_t *label, int32_t temperature_centi_c);
static void UI_SetHumidityLabel(lv_obj_t *label, int32_t humidity_centi_rh);
static void UI_SetPressureLabel(lv_obj_t *label, int32_t pressure_pa);
static void UI_SetAltitudeLabel(lv_obj_t *label, int32_t altitude_cm);

static const char *UI_GetCompassDirection(uint16_t heading_degrees);

/* BLETask向UiTask发布当前模块与应用会话状态。 */
static void BLE_PublishUiStatus(void);

/* main.c中的系统时钟配置函数，STOP唤醒后需要重新执行。 */
extern void SystemClock_Config(void);

/* 进入STOP，并在KEY1或RTC唤醒后恢复系统时钟和任务调度。 */
static SystemWakeReason_t System_EnterStopMode(void);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartPowerTask(void *argument);
void StartUiTask(void *argument);
void StartKeyScanTask(void *argument);
void StartControlTask(void *argument);
void StartSensorTask(void *argument);
void StartBLETask(void *argument);
void StartWatchdogTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of LcdMutex */
  LcdMutexHandle = osMutexNew(&LcdMutex_attributes);

  /* creation of RtcMutex */
  RtcMutexHandle = osMutexNew(&RtcMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of LcdDmaSem */
  LcdDmaSemHandle = osSemaphoreNew(1, 0, &LcdDmaSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of UiEventQueue */
  UiEventQueueHandle = osMessageQueueNew (8, sizeof(UIMessage_t), &UiEventQueue_attributes);

  /* creation of ControlEventQueue */
  ControlEventQueueHandle = osMessageQueueNew (8, sizeof(ControlMessage_t), &ControlEventQueue_attributes);

  /* creation of BLERXQueue */
  BLERXQueueHandle = osMessageQueueNew (64, sizeof(uint8_t), &BLERXQueue_attributes);

  /* creation of BLESensorQueue */
  BLESensorQueueHandle = osMessageQueueNew (4, sizeof(BLE_SensorMessage_t), &BLESensorQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of PowerTask */
  PowerTaskHandle = osThreadNew(StartPowerTask, NULL, &PowerTask_attributes);

  /* creation of UiTask */
  UiTaskHandle = osThreadNew(StartUiTask, NULL, &UiTask_attributes);

  /* creation of KeyScanTask */
  KeyScanTaskHandle = osThreadNew(StartKeyScanTask, NULL, &KeyScanTask_attributes);

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  /* creation of BLETask */
  BLETaskHandle = osThreadNew(StartBLETask, NULL, &BLETask_attributes);

  /* creation of WatchdogTask */
  WatchdogTaskHandle = osThreadNew(StartWatchdogTask, NULL, &WatchdogTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief 周期数据采集与生产任务
  *
  * 本任务以一秒为基本周期，负责：
  * 1. 读取RTC真实时间，将时间转换为当天累计秒数，将日期组合为YYYYMMDD。
  * 2. 每5秒采样一次电池电压，经过平均、校准和低通滤波后估算电量。
  *
  * 本任务不直接调用LVGL。所有界面数据都通过UiEventQueue发送给UiTask，
  * 由UiTask集中更新SquareLine控件，避免不同任务同时访问LVGL。
  *
  * @param argument 创建任务时传入的参数，本工程未使用
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  SystemData_t system_data = {0}; // 当前系统日期、时间、电池和心率数据
  UIMessage_t ui_message; // 循环复用的UI消息
  uint32_t seconds_of_day = 0U; // RTC时间对应的当天累计秒数
  uint8_t rtc_read_ok = 0U; // RTC读取成功标志

  /*
   * next_wake_tick保存下一次绝对唤醒时刻，ticks_per_second表示一秒包含的RTOS tick。
   * 配合osDelayUntil()可以获得稳定的一秒周期，避免普通osDelay()产生累计漂移。
   */
  uint32_t next_wake_tick = osKernelGetTickCount();
  uint32_t ticks_per_second = osKernelGetTickFreq();

  uint8_t battery_update_count = 0U; // 电池采样周期计数器
  uint32_t filtered_battery_voltage_mv = 0U; // 低通滤波后的电池电压

  /* 日期去重状态。 */
  uint32_t date_value = 0U; // 当前日期，格式为YYYYMMDD
  uint32_t last_date_value = 0U; // 上一次发送的日期，用于避免重复刷新

  for(;;)
  {
    /*
     * RTC_Service_Read()需要连续读取时间和日期，RtcMutex防止BLETask校时
     * 与本次读取交叉。读取成功后，时间按当天累计秒数、日期按YYYYMMDD
     * 分别封装到UI消息中，再由UiTask拆分并格式化显示。
     */
    rtc_read_ok = 0U;
    if(osMutexAcquire(RtcMutexHandle, osWaitForever) == osOK)
    {
      rtc_read_ok = RTC_Service_Read(&system_data.date_time);
      osMutexRelease(RtcMutexHandle);
    }

    if(rtc_read_ok != 0U)
    {
      seconds_of_day = (uint32_t)system_data.date_time.hour * 3600U;
      seconds_of_day += (uint32_t)system_data.date_time.minute * 60U;
      seconds_of_day += system_data.date_time.second;

      ui_message.event = UI_EVENT_TIME_UPDATE;
      ui_message.payload.value = (int32_t)seconds_of_day;
      osMessageQueuePut(UiEventQueueHandle, &ui_message, 0U, 0U);

      date_value = (uint32_t)system_data.date_time.year * 10000U;
      date_value += (uint32_t)system_data.date_time.month * 100U;
      date_value += system_data.date_time.date;

      /* 日期通常一天只变化一次，因此仅在日期改变时发送更新消息。 */
      if(date_value != last_date_value)
      {
        last_date_value = date_value;

        ui_message.event = UI_EVENT_DATE_UPDATE;
        ui_message.payload.value = (int32_t)date_value;
        osMessageQueuePut(UiEventQueueHandle, &ui_message, 0U, 0U);
      }
    }

    next_wake_tick += ticks_per_second; // 计算下一次绝对唤醒时刻

    /* 每5个一秒周期采集一次电池，降低ADC采样和界面刷新的频率。 */
    if(battery_update_count == 0U)
    {
      if(Battery_ReadVoltageMv(&system_data.battery_voltage_mv) != 0U)
      {
        if(filtered_battery_voltage_mv == 0U)
        {
          filtered_battery_voltage_mv = system_data.battery_voltage_mv; // 首次采样用于初始化滤波器
        }
        else
        {
          /*
           * 一阶低通滤波：新结果=75%历史值+25%本次测量值。
           * 这样可以抑制ADC噪声和负载变化造成的电池百分比跳动。
           */
          filtered_battery_voltage_mv = (filtered_battery_voltage_mv * 3U + system_data.battery_voltage_mv) / 4U;
        }

        system_data.battery_percent = Battery_VoltageToPercent((uint16_t)filtered_battery_voltage_mv);

        ui_message.event = UI_EVENT_BATTERY_UPDATE;
        ui_message.payload.value = system_data.battery_percent;
        osMessageQueuePut(UiEventQueueHandle, &ui_message, 0U, 0U);
      }
    }

    battery_update_count++;
    if(battery_update_count >= BATTERY_UPDATE_PERIOD_S) battery_update_count = 0U;

    osDelayUntil(next_wake_tick); // 阻塞到预定的一秒周期点
  }

  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartPowerTask */
/**
 * @brief 电源自锁维持任务
 *
 * 按下WAKE键只能让稳压电源暂时启动。MCU启动后必须持续拉高POWER_EN，
 * 才能维持整块板子的供电。本任务每秒重新写高一次POWER_EN。
 *
 * 执行关机时，ControlTask必须先终止本任务，再拉低POWER_EN；
 * 否则本任务可能再次把POWER_EN写高，导致关机失败。
 *
 * @param argument 未使用
 * @retval None
 */
/* USER CODE END Header_StartPowerTask */
void StartPowerTask(void *argument)
{
  /* USER CODE BEGIN StartPowerTask */

  for(;;)
  {
    HAL_GPIO_WritePin(POWER_EN_GPIO_Port, POWER_EN_Pin, GPIO_PIN_SET); // 维持电源自锁
    osDelay(1000U); // 每秒确认一次即可
  }

  /* USER CODE END StartPowerTask */
}

/* USER CODE BEGIN Header_StartUiTask */
/**
 * @brief LVGL初始化、界面更新和显示状态管理任务
 *
 * UiTask是本工程唯一允许直接调用LVGL API的任务，负责：
 * 1. 初始化LCD、LVGL显示端口、CST816触摸端口和SquareLine页面。
 * 2. 读取EEPROM中的亮度，并同步PWM、滑块位置和百分比标签。
 * 3. 从UiEventQueue取出系统数据和传感器消息，更新对应页面控件。
 * 4. 周期调用lv_timer_handler()处理绘制、动画、触摸和控件事件。
 * 5. 根据LVGL空闲时间依次请求暗屏和熄屏，STOP前让LCD与触摸进入休眠。
 * 6. 收到ControlTask的系统唤醒标志后，恢复LCD、触摸、页面传感器和LVGL状态。
 *
 * SquareLine Studio 1.6.1没有提供Released事件选项，因此页面创建完成后，
 * 本任务会直接为亮度滑块注册LVGL原生的LV_EVENT_RELEASED回调。
 *
 * @param argument 未使用
 * @retval None
 */
/* USER CODE END Header_StartUiTask */
void StartUiTask(void *argument)
{
  /* USER CODE BEGIN StartUiTask */

  /* 任务间消息、LVGL调度时间和显示电源状态。 */
  UIMessage_t ui_message; // 从UiEventQueue接收的界面数据
  ControlMessage_t control_message; // 发给ControlTask的暗屏、熄屏或唤醒命令
  uint32_t wait_time = 0U; // LVGL建议的下一次处理等待时间
  uint32_t inactive_time = 0U; // 距离最近一次用户输入的时间
  ScreenPowerState_t screen_power_state = SCREEN_POWER_ACTIVE; // 当前屏幕电源状态
  uint8_t touch_sleep_pending = 0U; // 已尝试让触摸休眠，恢复前暂停轮询
  uint8_t lcd_sleep_pending = 0U; // ST7789已经进入Sleep，唤醒前禁止LVGL刷新
  uint8_t heart_measurement_in_progress = 0U; // 1表示当前需要保持亮屏等待心率结果

  /* 界面初始化值与消息格式化的临时数据。 */
  uint8_t initial_brightness = SETTINGS_DEFAULT_BRIGHTNESS; // 开机亮度
  uint8_t battery_percent = 0U; // 消息中的电池百分比
  uint16_t compass_heading = 0U; // 经过屏幕方向修正后的最终角度
  const char *compass_direction = "--"; // 当前角度对应的八方向文字

  uint32_t seconds_of_day;
  uint32_t hours;
  uint32_t minutes;

  uint32_t date_value;
  uint32_t year;
  uint32_t month;
  uint32_t date;

  /*
   * 按照显示硬件、LVGL核心、显示端口、触摸端口和应用页面的顺序初始化。
   * LCD先填充为黑色，可以在LVGL第一次完整刷新前减少上电花屏。
   */
  LCD_Init();
  LCD_FillColor(LCD_COLOR_BLACK);
  lv_init();
  lv_tick_set_cb(HAL_GetTick);
  LVGL_PortDisplay_Init();
  Touch_Init();
  LVGL_PortInput_Init();

  /*
   * SettingsService将亮度业务与BL24C02底层读写分开。
   * EEPROM没有有效数据时使用默认亮度，并将默认值写入作为后续启动基准。
   */
  SettingsService_Init();

  if(SettingsService_LoadBrightness(&initial_brightness) == 0U)
  {
    initial_brightness = SETTINGS_DEFAULT_BRIGHTNESS;
    SettingsService_SaveBrightness(initial_brightness);
  }

  ui_init(); // 创建全部SquareLine页面，并加载HomeScreen

  /* 页面生命周期回调只发送线程标志，实际传感器I2C操作仍由SensorTask完成。 */
  if(ui_HomeScreen != NULL) lv_obj_add_event_cb(ui_HomeScreen, APP_HomeScreenLifecycleEvent, LV_EVENT_ALL, NULL);
  if(ui_CompassScreen != NULL) lv_obj_add_event_cb(ui_CompassScreen, APP_CompassScreenLifecycleEvent, LV_EVENT_ALL, NULL);
  if(ui_EnvScreen != NULL) lv_obj_add_event_cb(ui_EnvScreen, APP_EnvScreenLifecycleEvent, LV_EVENT_ALL, NULL);
  if(ui_StatusIcon != NULL)
  {
    lv_obj_set_style_translate_y(ui_StatusIcon, 2, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_add_event_cb(ui_StatusIcon, APP_HeartIconEvent, LV_EVENT_ALL, NULL);
  }

  /*
   * SquareLine页面创建完成后滑块对象才有效。
   * 这里补充注册LV_EVENT_RELEASED，并用EEPROM亮度覆盖生成代码中的默认值。
   */
  if(ui_BrightnessSlider != NULL)
  {
    lv_obj_add_event_cb(ui_BrightnessSlider, APP_BrightnessSliderReleased, LV_EVENT_RELEASED, NULL);
    lv_slider_set_value(ui_BrightnessSlider, initial_brightness, LV_ANIM_OFF);
  }

  if(ui_BrightnessValueLabel != NULL)
  {
    lv_label_set_text_fmt(ui_BrightnessValueLabel, "%u%%", (unsigned int)initial_brightness);
  }

  APP_BrightnessChanged(initial_brightness); // 请求ControlTask应用开机亮度
  lv_display_trigger_activity(NULL); // 从初始化完成时开始统计空闲时间

  for(;;)
  {
    /*
     * ControlTask已经恢复系统时钟和背光后才设置该标志。UiTask随后按显示端
     * 的拥有关系恢复LCD、触摸和LVGL，再通知SensorTask恢复页面传感器与
     * MPU6050；没有唤醒标志时直接继续处理普通UI消息。
     */
    if((osThreadFlagsGet() & UI_FLAG_SYSTEM_WAKE) != 0U)
    {
      osThreadFlagsClear(UI_FLAG_SYSTEM_WAKE);

      if(lcd_sleep_pending != 0U)
      {
        LCD_Wake();
        lcd_sleep_pending = 0U;
      }

      /* 系统时钟和HAL时基已经恢复，现在可以执行带延时的触摸复位。 */
      if(touch_sleep_pending != 0U)
      {
        Touch_Wake();
        touch_wake_chip_id = Touch_GetChipID();
        touch_sleep_pending = 0U;
      }

      screen_power_state = SCREEN_POWER_ACTIVE;
      APP_SetActivePageSensorsEnabled(1U);
      LVGL_PortInput_SetWakeOnly(0U); // 取消触摸唤醒模式
      lv_display_trigger_activity(NULL); // 重新开始统计空闲时间
      if(SensorTaskHandle != NULL) osThreadFlagsSet(SensorTaskHandle, SENSOR_FLAG_MPU_NORMAL);
    }

    /*
     * 使用非阻塞while循环取出当前积压的全部UI消息，避免只处理一条消息时，
     * 心率、RTC或电池数据长时间滞留在队列中。
     */
    while(osMessageQueueGet(UiEventQueueHandle, &ui_message, NULL, 0U) == osOK)
    {
      if(ui_message.event == UI_EVENT_HEART_RATE_UPDATE)
      {
        heart_measurement_in_progress = 0U;

        if(ui_HeartRateLabel != NULL)
        {
          if(ui_message.payload.value > 0)
          {
            lv_label_set_text_fmt(ui_HeartRateLabel, "%ld BPM", (long)ui_message.payload.value);
          }
          else
          {
            lv_label_set_text(ui_HeartRateLabel, "-- BPM");
          }
        }
      }
      else if(ui_message.event == UI_EVENT_HEART_RATE_STATE_UPDATE)
      {
        /*
         * RUNNING期间保持显示活跃并显示测量提示；TIMEOUT和UNAVAILABLE结束
         * 本次会话并显示原因；其余状态只在测量尚未结束时恢复默认占位文字。
         */
        if(ui_message.payload.value == HEART_RATE_MEASUREMENT_RUNNING)
        {
          heart_measurement_in_progress = 1U;
          if(ui_HeartRateLabel != NULL) lv_label_set_text(ui_HeartRateLabel, "Measuring...");
        }
        else if(ui_message.payload.value == HEART_RATE_MEASUREMENT_TIMEOUT)
        {
          heart_measurement_in_progress = 0U;
          if(ui_HeartRateLabel != NULL) lv_label_set_text(ui_HeartRateLabel, "Try again");
        }
        else if(ui_message.payload.value == HEART_RATE_MEASUREMENT_UNAVAILABLE)
        {
          heart_measurement_in_progress = 0U;
          if(ui_HeartRateLabel != NULL) lv_label_set_text(ui_HeartRateLabel, "Unavailable");
        }
        else
        {
          if((heart_measurement_in_progress != 0U) && (ui_HeartRateLabel != NULL)) lv_label_set_text(ui_HeartRateLabel, "-- BPM");
          heart_measurement_in_progress = 0U;
        }
      }
      else if(ui_message.event == UI_EVENT_TIME_UPDATE)
      {
        /*
         * 时间消息使用当天累计秒数表示。取模86400可以把异常输入限制在一天内，
         * 然后分别计算小时和分钟供首页时间标签显示。
         */
        seconds_of_day = (uint32_t)ui_message.payload.value % 86400U;
        hours = seconds_of_day / 3600U;
        minutes = (seconds_of_day % 3600U) / 60U;

        if(ui_TimeLabel != NULL)
        {
          lv_label_set_text_fmt(ui_TimeLabel, "%02lu:%02lu", (unsigned long)hours, (unsigned long)minutes);
        }
      }
      else if(ui_message.event == UI_EVENT_BATTERY_UPDATE)
      {
        /* 将异常电池数据限制在控件能够显示的0~100范围内。 */
        if(ui_message.payload.value < 0)
        {
          ui_message.payload.value = 0;
        }
        else if(ui_message.payload.value > 100)
        {
          ui_message.payload.value = 100;
        }

        battery_percent = (uint8_t)ui_message.payload.value;

        if(ui_BatteryLabel != NULL)
        {
          lv_label_set_text_fmt(ui_BatteryLabel, "%u%%", (unsigned int)battery_percent);

          /* 使用红、黄、绿三种颜色提示低电量、中等电量和正常电量。 */
          if(battery_percent <= 20U)
          {
            lv_obj_set_style_text_color(ui_BatteryLabel, lv_color_hex(0xEF5350), LV_PART_MAIN);
          }
          else if(battery_percent <= 50U)
          {
            lv_obj_set_style_text_color(ui_BatteryLabel, lv_color_hex(0xFFCA28), LV_PART_MAIN);
          }
          else
          {
            lv_obj_set_style_text_color(ui_BatteryLabel, lv_color_hex(0x81C784), LV_PART_MAIN);
          }
        }
      }
      else if(ui_message.event == UI_EVENT_DATE_UPDATE)
      {
        date_value = (uint32_t)ui_message.payload.value;

        /* 将YYYYMMDD重新拆分为年、月、日。 */
        year = date_value / 10000U;
        month = (date_value / 100U) % 100U;
        date = date_value % 100U;

        if(ui_DateLabel != NULL)
        {
          lv_label_set_text_fmt(ui_DateLabel, "%04lu-%02lu-%02lu", (unsigned long)year, (unsigned long)month, (unsigned long)date);
        }
      }
      else if(ui_message.event == UI_EVENT_MPU6050_UPDATE)
      {
        /*
         * MPU6050消息携带同一次采样换算后的完整数据。
         * UiTask集中更新所有SportScreen标签，SensorTask不持有LVGL对象，
         * 从而避免传感器采集和界面绘制并发访问LVGL。
         */
        if(ui_AccelXLabel != NULL) lv_label_set_text_fmt(ui_AccelXLabel, "X: %ld mg", (long)ui_message.payload.mpu6050.accel_x_mg);
        if(ui_AccelYLabel != NULL) lv_label_set_text_fmt(ui_AccelYLabel, "Y: %ld mg", (long)ui_message.payload.mpu6050.accel_y_mg);
        if(ui_AccelZLabel != NULL) lv_label_set_text_fmt(ui_AccelZLabel, "Z: %ld mg", (long)ui_message.payload.mpu6050.accel_z_mg);

        UI_SetGyroLabel(ui_GyroXLabel, "X", ui_message.payload.mpu6050.gyro_x_mdps);
        UI_SetGyroLabel(ui_GyroYLabel, "Y", ui_message.payload.mpu6050.gyro_y_mdps);
        UI_SetGyroLabel(ui_GyroZLabel, "Z", ui_message.payload.mpu6050.gyro_z_mdps);

        UI_SetTemperatureLabel(ui_MPUTemperatureLabel, ui_message.payload.mpu6050.temperature_centi_c);
        if(ui_MPUStepCountLabel != NULL) lv_label_set_text_fmt(ui_MPUStepCountLabel, "STEPS: %lu", (unsigned long)ui_message.payload.mpu6050.step_count);
        if(ui_MPUWristLabel != NULL) lv_label_set_text_fmt(ui_MPUWristLabel, "WRIST: %s  RAISE: %lu", ui_message.payload.mpu6050.wrist_is_up != 0U ? "UP" : "DOWN", (unsigned long)ui_message.payload.mpu6050.wrist_raise_count);
      }
      else if(ui_message.event == UI_EVENT_AHT21_UPDATE)
      {
        /* AHT21数据只由UiTask写入EnvScreen，SensorTask不直接访问LVGL。 */
        UI_SetTemperatureLabel(ui_AHTTempLabel, ui_message.payload.aht21.temperature_centi_c);
        UI_SetHumidityLabel(ui_AHTHumiLabel, ui_message.payload.aht21.humidity_centi_rh);
      }
      else if(ui_message.event == UI_EVENT_SPL06_UPDATE)
      {
        UI_SetPressureLabel(ui_SPLPressureLabel, ui_message.payload.spl06.pressure_pa);
        UI_SetAltitudeLabel(ui_SPLAltitudeLabel, ui_message.payload.spl06.altitude_cm);
      }
      else if(ui_message.event == UI_EVENT_LSM303_UPDATE)
      {
        /*
         * 方向角先叠加PCB安装方向修正，再限制到0°～359°。
         * 当前偏移为0，保留该常量便于以后根据实物轴向统一修正。
         */
        compass_heading = (uint16_t)((ui_message.payload.lsm303.tilt_heading_degrees + COMPASS_HEADING_OFFSET_DEGREES) % 360U);
        compass_direction = UI_GetCompassDirection(compass_heading);

        if(ui_CompassStatusLabel  != NULL)
        {
          lv_label_set_text(ui_CompassStatusLabel, "READY");
          lv_obj_set_style_text_color(ui_CompassStatusLabel, lv_color_hex(0x2E7D32), LV_PART_MAIN);
        }

        if(ui_CompassDirectionLabel != NULL) lv_label_set_text(ui_CompassDirectionLabel, compass_direction);
        if(ui_CompassHeadingLabel != NULL) lv_label_set_text_fmt(ui_CompassHeadingLabel, "%u\xC2\xB0", (unsigned int)compass_heading);
        if(ui_CompassPitchLabel != NULL) lv_label_set_text_fmt(ui_CompassPitchLabel, "PITCH: %d\xC2\xB0", (int)ui_message.payload.lsm303.pitch_degrees);
        if(ui_CompassRollLabel != NULL) lv_label_set_text_fmt(ui_CompassRollLabel, "ROLL: %d\xC2\xB0", (int)ui_message.payload.lsm303.roll_degrees);
      }
      else if(ui_message.event == UI_EVENT_BLE_STATUS_UPDATE)
      {
        BLEStatusLabelUpdate((uint8_t)ui_message.payload.value);
      }
    }

    /*
     * lv_timer_handler()负责处理LVGL输入设备、控件事件、动画和界面刷新，
     * 返回值表示LVGL建议等待多久后再次调用。
     */
    if((touch_sleep_pending == 0U) && (lcd_sleep_pending == 0U))
    {
      wait_time = lv_timer_handler();
    }
    else
    {
      wait_time = 20U;
    }

    if(heart_measurement_in_progress != 0U) lv_display_trigger_activity(NULL);
    inactive_time = lv_display_get_inactive_time(NULL);

    /*
     * UiTask只维护显示策略并向ControlTask发送硬件控制消息，不直接修改TIM3。
     * 空闲状态按ACTIVE -> DIMMED -> OFF逐级下降；DIMMED和未进入STOP的OFF
     * 状态下，第一次触摸只负责唤醒，不继续传递给当前页面的按钮。
     */
    if(screen_power_state == SCREEN_POWER_ACTIVE)
    {
      if(inactive_time >= SCREEN_IDLE_TIMEOUT_MS)
      {
        control_message.event = CONTROL_EVENT_SCREEN_DIM;
        control_message.value = SCREEN_DIM_PERCENT;

        /* 只有消息成功入队后才更新本地状态，保证软件状态与硬件动作一致。 */
        if(osMessageQueuePut(ControlEventQueueHandle, &control_message, 0U, 0U) == osOK)
        {
          screen_power_state = SCREEN_POWER_DIMMED;

          APP_SetActivePageSensorsEnabled(0U); // 暗屏后暂停传感器采样，降低功耗

          if(SensorTaskHandle != NULL)
          {
            osThreadFlagsSet(SensorTaskHandle, SENSOR_FLAG_MPU_LOW_POWER);
          }

          /*
           * 暗屏后的首次触摸只产生唤醒请求，不继续传递给LVGL。
           * 这样第一次触摸只负责亮屏，不会误触屏幕上的按钮。
           */
          LVGL_PortInput_SetWakeOnly(1U);
        }
      }
    }
    else
    {
      /*
       * 触摸判断放在熄屏判断前面
       * 如果触摸恰好发生在15秒临界点，优先执行唤醒而不是继续熄屏
       */
      if(LVGL_PortInput_TakeWakeRequest() != 0U)
      {
        control_message.event = CONTROL_EVENT_SCREEN_WAKE;
        control_message.value = 0;

        if(osMessageQueuePut(ControlEventQueueHandle, &control_message, 0U, 0U) == osOK)
        {
          screen_power_state = SCREEN_POWER_ACTIVE;
          APP_SetActivePageSensorsEnabled(1U); // 唤醒后恢复传感器采样

          if(SensorTaskHandle != NULL)
          {
            osThreadFlagsSet(SensorTaskHandle, SENSOR_FLAG_MPU_NORMAL);
          }

          LVGL_PortInput_SetWakeOnly(0U); // 恢复普通触摸输入
          lv_display_trigger_activity(NULL); // 从本次唤醒重新统计空闲时间
        }
      }
      else if((screen_power_state == SCREEN_POWER_DIMMED) && (inactive_time >= SCREEN_OFF_TIMEOUT_MS))
      {
        if(APP_BLEPowerIsEnabled() != 0U)
        {
          control_message.event = CONTROL_EVENT_SCREEN_OFF_NONSTOP;
        }
        else
        {
          #if STOP_MODE_TEST_ENABLED

            LCD_EnterSleep();
            lcd_sleep_pending = 1U; // 暂停LVGL刷新，等待下一次唤醒

            touch_wake_chip_id = 0U; // 清除上一次结果，方便观察本轮恢复
            touch_sleep_write_ok = Touch_EnterSleep();
            touch_sleep_pending = 1U; // 暂停轮询，等待下一次唤醒
          #endif

          control_message.event = CONTROL_EVENT_SCREEN_OFF;
        }

        control_message.value = 0;

        if(osMessageQueuePut(ControlEventQueueHandle, &control_message, 0U, 0U) == osOK)
        {
          screen_power_state = SCREEN_POWER_OFF;
          if(control_message.event == CONTROL_EVENT_SCREEN_OFF_NONSTOP) ble_stop_block_count++;
        }
        else
        {
          /*
           * 熄屏消息未能进入ControlEventQueue时，ControlTask不会进入STOP。
           * 因此UiTask立即恢复已经休眠的LCD和触摸，避免系统停在半休眠状态。
           */
          if(lcd_sleep_pending != 0U)
          {
            LCD_Wake();
            lcd_sleep_pending = 0U;
          }

          if(touch_sleep_pending != 0U)
          {
            Touch_Wake();
            touch_wake_chip_id = Touch_GetChipID();
            touch_sleep_pending = 0U;
          }
        }
      }
    }

    /*
     * 将等待时间限制在5~20ms：
     * - 小于5ms会使UiTask运行过于频繁；
     * - 大于20ms可能降低触摸和动画的响应速度。
     */
    if(wait_time < 5U)
    {
      wait_time = 5U;
    }

    if(wait_time > 20U)
    {
      wait_time = 20U;
    }

    osDelay(wait_time);
  }

  /* USER CODE END StartUiTask */
}

/* USER CODE BEGIN Header_StartKeyScanTask */
/**
 * @brief WAKE按键扫描和长按识别任务
 *
 * WAKE按键按下时PA4为高电平，松开时为低电平。本任务每20ms读取一次电平，
 * 通过上一次状态和当前状态检测新的按下动作，再利用RTOS tick计算按住时间。
 * 持续按下达到1500ms后只发送一次关机消息。
 *
 * MCU通常由按下WAKE键启动，因此任务启动时会先等待本次开机按键松开，
 * 避免把开机动作误判为长按关机。
 *
 * @param argument 未使用
 * @retval None
 */
/* USER CODE END Header_StartKeyScanTask */
void StartKeyScanTask(void *argument)
{
  /* USER CODE BEGIN StartKeyScanTask */

  uint8_t wake_last = 0U; // 上一次扫描状态：0=松开，1=按下
  uint8_t wake_now; // 本次扫描状态
  uint8_t wake_long_sent = 0U; // 防止一次长按重复发送关机消息
  uint32_t wake_press_tick = 0U; // 本次按下时的RTOS tick
  ControlMessage_t control_message; // 发给ControlTask的按键控制消息

  /* 等待用户松开用于启动设备的这次按键。 */
  while(HAL_GPIO_ReadPin(KEY_WAKE_GPIO_Port, KEY_WAKE_Pin) == GPIO_PIN_SET)
  {
    osDelay(KEY_SCAN_PERIOD_MS);
  }

  for(;;)
  {
    wake_now = (HAL_GPIO_ReadPin(KEY_WAKE_GPIO_Port, KEY_WAKE_Pin) == GPIO_PIN_SET) ? 1U : 0U;

    /* 上次松开、本次按下，说明出现了一次新的按下沿。 */
    if(wake_last == 0U && wake_now == 1U)
    {
      wake_press_tick = osKernelGetTickCount();
      wake_long_sent = 0U;
    }

    /* 按键保持按下且尚未上报长按事件时，检查持续时间。 */
    if(wake_now == 1U && wake_long_sent == 0U)
    {
      if(osKernelGetTickCount() - wake_press_tick >= KEY_LONG_PRESS_MS)
      {
        control_message.event = CONTROL_EVENT_WAKE_LONG;
        control_message.value = 0U;

        /*
         * 关机命令不能丢失，因此使用osWaitForever等待队列空间。
         * 发送成功后置位wake_long_sent，按住不放也不会重复发送。
         */
        if(osMessageQueuePut(ControlEventQueueHandle, &control_message, 0U, osWaitForever) == osOK)
        {
          wake_long_sent = 1U;
        }
      }
    }

    wake_last = wake_now; // 保存状态供下一轮进行边沿判断
    osDelay(KEY_SCAN_PERIOD_MS); // 固定扫描周期并让出CPU
  }

  /* USER CODE END StartKeyScanTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
 * @brief 背光、设置保存和电源控制任务
 *
 * ControlTask阻塞等待ControlEventQueue，没有控制消息时不占用CPU。
 * 它负责TIM3背光PWM及POWER_EN关机/交接控制（PowerTask平时保持高电平），并调用SettingsService
 * 保存最终亮度，避免UiTask或SquareLine回调直接操作硬件和EEPROM。
 *
 * 主要事件与状态变化：
 * 1. 亮度变化：更新正常亮度，亮屏时立即应用到PWM。
 * 2. 亮度保存：用户松开滑块后，将最终亮度写入EEPROM。
 * 3. 自动暗屏：临时降低PWM，同时保留正常亮度。
 * 4. 蓝牙开启时熄屏：停止背光PWM，但CPU和蓝牙任务继续运行。
 * 5. 蓝牙关闭时熄屏：启动RTC唤醒并进入STOP，RTC只短暂检查抬腕。
 * 6. 触摸、KEY1或抬腕唤醒：恢复背光；STOP唤醒还会通知UiTask恢复外设。
 * 7. WAKE长按：终止电源维持任务、关闭背光并释放电源自锁。
 * 8. OTA请求：给模块留出应答转发时间，再保持供电并交接到Bootloader。
 *
 * @param argument 未使用
 * @retval None
 */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */

  /* 控制消息与TIM3背光PWM状态。 */
  ControlMessage_t message; // 当前控制命令
  uint32_t timer_period; // TIM3自动重装载值
  uint32_t compare_value; // 临时PWM比较值
  uint32_t normal_compare_value = 200U; // 用户正常亮度对应的PWM比较值
  uint8_t screen_is_dimmed = 0U; // 0=正常亮屏，1=暗屏或完全熄屏

  /* STOP唤醒原因与SensorTask采样窗口应答。 */
  SystemWakeReason_t wake_reason = SYSTEM_WAKE_NONE; // 本次STOP返回的唤醒原因
  uint32_t probe_wait_result = 0U; // 等待SensorTask后台读取完成的结果

  for(;;)
  {
    /* 没有控制消息时一直阻塞，避免无意义轮询。 */
    if(osMessageQueueGet(ControlEventQueueHandle, &message, NULL, osWaitForever) == osOK)
    {
      if(message.event == CONTROL_EVENT_BRIGHTNESS_CHANGED)
      {
        /* 防止异常百分比产生超过PWM周期的比较值。 */
        if(message.value < 0) message.value = 0;
        if(message.value > 100) message.value = 100;

        timer_period = __HAL_TIM_GET_AUTORELOAD(&htim3);
        normal_compare_value = ((uint32_t)message.value * timer_period) / 100U;

        /*
         * 暗屏期间只记录新的正常亮度，不立即点亮屏幕；
         * 正常亮屏时则直接应用新的PWM比较值。
         */
        if(screen_is_dimmed == 0U)
        {
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, normal_compare_value);
        }
      }
      else if(message.event == CONTROL_EVENT_BRIGHTNESS_SAVE)
      {
        /*
         * 滑动期间只实时修改PWM，用户松手后才保存一次最终亮度。
         * 这样可以减少EEPROM写入次数，并避免写周期影响滑块响应。
         */
        if(message.value < 0) message.value = 0;
        if(message.value > 100) message.value = 100;

        SettingsService_SaveBrightness((uint8_t)message.value);
      }
      else if(message.event == CONTROL_EVENT_SCREEN_DIM)
      {
        timer_period = __HAL_TIM_GET_AUTORELOAD(&htim3);
        compare_value = ((uint32_t)message.value * timer_period) / 100U;

        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, compare_value);
        screen_is_dimmed = 1U;
        system_power_mode = SYSTEM_POWER_DIMMED;
      }
      else if(message.event == CONTROL_EVENT_SCREEN_OFF_NONSTOP)
      {
        /*
         * 蓝牙正在运行时只关闭背光。
         * CPU继续运行，因此USART1、DMA、BLETask和触摸轮询都保持工作。
         */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3); // 停止PWM通道和TIM3计数，避免无用定时器活动
        screen_is_dimmed = 1U;
        system_power_mode = SYSTEM_POWER_SCREEN_OFF_AWAKE;
      }
      else if(message.event == CONTROL_EVENT_SCREEN_OFF)
      {
        /*
         * 先关闭背光，再进入STOP。
         * MPU6050已经在5秒暗屏时进入20Hz低功耗模式，
         * 因此这里不用重复配置MPU6050。
         */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U); // 先将占空比清零，确保背光输出关闭
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3); // 背光完全关闭后停止TIM3计数
        screen_is_dimmed = 1U;

        #if STOP_MODE_TEST_ENABLED
          system_power_mode = SYSTEM_POWER_STOP;

          key1_wake_pending = 0U; // 第一次进入STOP前丢弃以前残留的KEY1事件

          /*
           * RTC WakeUp只在整机准备进入STOP时开启。
           * 先停止可能残留的旧定时器，再重新开始一个完整周期。
           */
          rtc_wakeup_stop_status = HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
          rtc_wake_pending = 0U;
          rtc_wakeup_start_status = HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0U, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);

          do
          {
            wake_reason = System_EnterStopMode();

            /*
             * RTC只负责周期性唤醒CPU，不直接决定亮屏。ControlTask请求SensorTask
             * 读取一个MPU6050窗口：超时或未识别到抬腕时，循环再次进入STOP；
             * 识别成功时将临时RTC唤醒升级为整机抬腕唤醒。
             */
            if(wake_reason == SYSTEM_WAKE_RTC)
            {
              if(SensorTaskHandle != NULL)
              {
                osThreadFlagsClear(CONTROL_FLAG_MPU_STOP_PROBE_DONE);
                osThreadFlagsSet(SensorTaskHandle, SENSOR_FLAG_MPU_STOP_PROBE);
                probe_wait_result = osThreadFlagsWait(CONTROL_FLAG_MPU_STOP_PROBE_DONE, osFlagsWaitAny, MPU_STOP_PROBE_TIMEOUT_TICKS);

                if((probe_wait_result & osFlagsError) != 0U)
                {
                  mpu_stop_probe_timeout_count++;
                }
                else if(mpu_stop_probe_wrist_event != 0U)
                {
                  /* SensorTask已确认完整抬腕，将RTC临时唤醒升级为整机唤醒。 */
                  wake_reason = SYSTEM_WAKE_WRIST;
                  system_last_wake_reason = SYSTEM_WAKE_WRIST;
                }
              }
              else
              {
                mpu_stop_probe_timeout_count++;
              }
            }
          } while(wake_reason == SYSTEM_WAKE_RTC);

          /*
           * 循环结束表示KEY1或抬腕要求完整恢复系统。
           * 关闭RTC WakeUp定时器，避免正常亮屏期间继续产生周期中断。
           */
          rtc_wakeup_stop_status = HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
          rtc_wake_pending = 0U;

          /*
           * System_EnterStopMode()已经恢复系统时钟、HAL时基和任务调度。
           * 对KEY1或抬腕事件，ControlTask还要恢复PWM与电源模式，再通知UiTask
           * 恢复LCD、触摸、LVGL活动时间和传感器状态。
           */
          if((wake_reason == SYSTEM_WAKE_KEY1) || (wake_reason == SYSTEM_WAKE_WRIST))
          {
            key1_wake_pending = 0U;
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, normal_compare_value); // 恢复用户设置的正常亮度
            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
            screen_is_dimmed = 0U;
            system_power_mode = SYSTEM_POWER_ACTIVE;
            if(UiTaskHandle != NULL) osThreadFlagsSet(UiTaskHandle, UI_FLAG_SYSTEM_WAKE);
          }
        #endif
      }
      else if(message.event == CONTROL_EVENT_SCREEN_WAKE)
      {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, normal_compare_value); // 恢复用户设置的正常亮度
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
        screen_is_dimmed = 0U;
        system_power_mode = SYSTEM_POWER_ACTIVE;
      }
      else if(message.event == CONTROL_EVENT_ENTER_OTA)
      {
        /*
         * BLETask已先发送OK:OTA；留出约500ms让蓝牙模块转发后才清理USART1。
         * osDelay只阻塞ControlTask，期间其他任务仍可运行；它不确认对端已收包。
         */
        osDelay(osKernelGetTickFreq() / 2U);

        APP_JumpToBootloader(); // 保持供电，清理APP运行状态并软件跳转到Bootloader
      }
      else if(message.event == CONTROL_EVENT_WAKE_LONG)
      {
        /*
         * 关机顺序：
         * 1. 终止PowerTask，防止POWER_EN再次被写高。
         * 2. 将PWM比较值清零并停止TIM3，关闭LCD背光。
         * 3. 拉低POWER_EN，释放硬件电源自锁。
         */
        system_power_mode = SYSTEM_POWER_SHUTDOWN;
        osThreadTerminate(PowerTaskHandle);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
        HAL_GPIO_WritePin(POWER_EN_GPIO_Port, POWER_EN_Pin, GPIO_PIN_RESET);

        /*
         * 正常情况下整板会在POWER_EN拉低后断电。
         * 如果调试器仍给MCU供电，则停留在此处，不再执行其他控制逻辑。
         */
        for(;;)
        {
          osDelay(1000U);
        }
      }
    }
  }

  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
  * @brief 背板传感器采集、算法处理和页面生命周期控制任务
  *
  * SensorTask是背板传感器软件I2C总线的唯一拥有者，负责初始化并读取
  * MPU6050、AHT21、SPL06、LSM303和EM7028。其他任务和LVGL页面回调
  * 不能直接访问这些传感器，只能通过线程标志请求启停。
  *
  * MPU6050亮屏时约50Hz采样；暗屏后切换为仅加速度计工作的约20Hz模式，
  * STOP期间只在RTC唤醒窗口中读取。EM7028由主页爱心长按启动，并在得到
  * 有效BPM、20秒超时、离开主页或暗屏时停止；环境与指南针传感器则按页面
  * 生命周期和蓝牙订阅需求启停。
  *
  * 传感器数据经过Service层换算或算法处理后封装成UIMessage_t，通过
  * UiEventQueue交给UiTask。SensorTask不持有LVGL对象，也不直接更新页面。
  *
  * @param argument 创建任务时传入的参数，本工程未使用
  * @retval None
  */
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */

  /* 跨任务消息与SensorTask控制标志。 */
  UIMessage_t ui_message = {0}; // 发送给UiTask的传感器消息
  BLE_SensorMessage_t ble_sensor_message = {0}; // 发送给BLETask的传感器消息
  uint32_t sensor_flags = 0U; // SensorTask当前收到的控制标志

  /* MPU6050采样、计步与抬腕状态。 */
  uint8_t mpu6050_device_id = 0U; // MPU6050设备ID
  uint8_t mpu6050_initialized = 0U; // 1表示MPU6050已经初始化成功
  MPU6050_RawData_t mpu6050_raw_data = {0}; // MPU6050原始数据
  MPU6050_Data_t mpu6050_data = {0}; // 换算后的MPU6050物理单位数据
  MPU6050_StepCounterState_t mpu6050_step_counter = {0}; // MPU6050基础计步状态
  MPU6050_WristRaiseState_t mpu6050_wrist_raise = {0}; // MPU6050抬腕检测状态
  uint8_t ui_update_divider = 0U; // MPU6050界面刷新分频计数器
  uint32_t last_ble_step_count = 0U; // 上一次成功放入蓝牙队列的步数
  uint8_t ble_step_published = 0U; // 1表示已经向BLETask发送过初始步数
  uint8_t mpu6050_low_power = 0U; // 1表示当前处于低功耗循环采样模式
  uint32_t mpu6050_next_read_tick = 0U; // 下一次读取MPU6050的时间
  uint32_t mpu6050_active_period_ticks; // 正常模式约50Hz读取周期
  uint32_t mpu6050_low_power_period_ticks; // 低功耗模式约20Hz读取周期

  uint8_t stop_probe_sample_index = 0U; // STOP后台采样窗口的循环下标
  uint8_t stop_probe_successful_samples = 0U; // 当前窗口成功读取的样本数
  uint8_t stop_probe_wrist_event = 0U; // 当前窗口是否产生新抬腕事件
  uint8_t stop_probe_up_samples = 0U; // 当前窗口满足看表姿态的样本数
  uint8_t stop_rearm_stable_samples = 0U; // 进入STOP前连续满足非UP姿态的样本数

  /* UI页面与蓝牙订阅对环境传感器的独立需求状态。 */
  uint8_t env_ui_requested = 0U; // 1表示环境页面需要AHT21和SPL06数据
  uint8_t env_ble_requested = 0U; // 1表示蓝牙连续推送需要AHT21
  uint8_t env_request_changed = 0U; // 本轮是否收到环境需求变化
  uint8_t aht21_should_run = 0U; // 综合两个使用者后得出的AHT21目标状态

  /* AHT21初始化、非阻塞触发测量与周期计时。 */
  uint8_t aht21_status = 0U; // AHT21返回的状态字节
  uint8_t aht21_initialized = 0U; // 1表示AHT21已经可以正常测量
  uint8_t aht21_measurement_enabled = 0U; // 1表示UI或蓝牙当前需要AHT21数据
  uint8_t aht21_measurement_pending = 0U; // 1表示已经触发测量，正在等待结果
  AHT21_RawData_t aht21_raw_data = {0}; // AHT21的20位温湿度原始数据
  AHT21_Data_t aht21_data = {0}; // 换算后的温湿度数据
  uint32_t aht21_measurement_start_tick = 0U; // 本轮测量命令发出的时刻
  uint32_t aht21_next_measurement_tick = 0U; // 下一轮测量开始时刻
  uint32_t current_tick = 0U; // 当前FreeRTOS系统节拍
  uint32_t ticks_per_second = osKernelGetTickFreq(); // 每秒包含的系统节拍数
  uint32_t aht21_wait_ticks; // 命令发出后等待结果的节拍数
  uint32_t aht21_period_ticks; // 两轮AHT21测量之间的节拍数

  /* SPL06初始化、连续测量与周期读取。 */
  uint8_t spl06_device_id = 0U; // 本次读取到的SPL06设备ID
  uint8_t spl06_initialized = 0U; // 1表示SPL06身份、校准系数和配置均有效
  uint8_t spl06_measurement_active = 0U; // 1表示SPL06处于连续测量模式
  SPL06_Calibration_t spl06_calibration = {0}; // SPL06的出厂校准系数
  SPL06_RawData_t spl06_raw_data = {0}; // SPL06气压和温度原始数据
  SPL06_Data_t spl06_data = {0}; // 补偿后的温度、气压和估算海拔
  uint32_t spl06_next_read_tick = 0U; // 下一次读取SPL06的系统节拍

  /* LSM303启停、磁场校准与指南针数据。 */
  uint8_t lsm303_initialized = 0U; // 1表示LSM303身份验证和配置成功
  uint8_t lsm303_measurement_active = 0U; // 1表示LSM303正在连续测量
  LSM303_RawData_t lsm303_raw_data = {0}; // 两组三轴原始数据
  LSM303_MagCalibration_t lsm303_calibration = {0}; // 磁场Min-Max校准状态
  LSM303_CompassData_t lsm303_compass_data = {0}; // 方向角、俯仰角和横滚角
  uint32_t lsm303_next_read_tick = 0U; // 下一次读取LSM303的系统节拍
  uint32_t lsm303_period_ticks = 0U; // LSM303读取周期

  /* EM7028光电采样与心率计算状态。 */
  uint8_t em7028_initialized = 0U; // 1表示EM7028身份验证和配置成功
  uint8_t em7028_measurement_active = 0U; // 1表示EM7028正在进行光电采样
  uint16_t em7028_raw_data = 0U; // 最近一次PPG原始值
  uint32_t em7028_next_read_tick = 0U; // 下一次读取EM7028的系统节拍
  uint32_t em7028_period_ticks = 0U; // EM7028读取周期
  uint32_t em7028_measurement_deadline_tick = 0U; // 本次测量的20秒截止时间
  EM7028_HeartRateState_t em7028_heart_rate_state = {0}; // EM7028心率计算状态
  uint16_t em7028_bpm = 0U; // EM7028计算出的心率

  /* 将以秒或Hz表示的周期统一换算成FreeRTOS系统节拍。 */
  if(ticks_per_second == 0U) ticks_per_second = 1000U;
  aht21_wait_ticks = ticks_per_second / 10U; // 约100ms
  aht21_period_ticks = ticks_per_second * 2U; // 每2秒测量一次
  if(aht21_wait_ticks == 0U) aht21_wait_ticks = 1U;

  lsm303_period_ticks = ticks_per_second / 10U; // 约100ms
  if(lsm303_period_ticks == 0U) lsm303_period_ticks = 1U;

  em7028_period_ticks = ticks_per_second / 50U; // 约20ms，即50Hz采样
  if(em7028_period_ticks == 0U) em7028_period_ticks = 1U;

  /* MPU6050正常与低功耗模式的读取周期。 */
  mpu6050_active_period_ticks = ticks_per_second / 50U; // 约20ms
  mpu6050_low_power_period_ticks = ticks_per_second / 20U; // 约50ms

  if(mpu6050_active_period_ticks == 0U) mpu6050_active_period_ticks = 1U;
  if(mpu6050_low_power_period_ticks == 0U) mpu6050_low_power_period_ticks = 1U;

  (void)argument;

  /* 软件I2C总线只在本任务初始化一次，随后等待所有传感器上电稳定。 */
  SensorI2C_Init();
  osDelay(100U); // 等待传感器上电稳定

  /*
   * 先确认设备能够应答，再读取WHO_AM_I身份寄存器。
   * 只有设备ID为0x68时才配置MPU6050，避免总线上其他设备的应答
   * 被错误识别为MPU6050。
   */
  if((MPU6050_IsReady() != 0U) && (MPU6050_ReadDeviceID(&mpu6050_device_id) != 0U) && (mpu6050_device_id == MPU6050_WHO_AM_I_VALUE))
  {
    mpu6050_initialized = MPU6050_Init();
    if(mpu6050_initialized != 0U)
    {
      osDelay(50U); // 等待MPU6050内部配置稳定
      mpu6050_next_read_tick = osKernelGetTickCount();
    }
  }

  /*
   * AHT21上电后先读取校准状态。若状态位3尚未置位，则发送BE 08 00
   * 初始化命令并等待至少10ms，再次读取状态确认芯片已经可以测量。
   */
  if((AHT21_IsReady() != 0U) && (AHT21_ReadStatus(&aht21_status) != 0U))
  {
    if(AHT21_IsCalibrated(aht21_status) != 0U) aht21_initialized = 1U;
    else if(AHT21_SendInitCommand() != 0U)
    {
      osDelay(10U);
      if((AHT21_ReadStatus(&aht21_status) != 0U) && (AHT21_IsCalibrated(aht21_status) != 0U)) aht21_initialized = 1U;
    }
  }

  /*
   * SPL06初始化需要依次确认I2C应答、产品ID和出厂校准系数。
   * StartContinuousMeasurement()负责写入测量参数；由于开机时不在
   * EnvScreen，配置完成后立即停止转换，等待页面事件重新启动。
   */
  if((SPL06_IsReady() != 0U) && (SPL06_ReadDeviceID(&spl06_device_id) != 0U) && (spl06_device_id == SPL06_EXPECTED_ID))
  {
    if(SPL06_ReadCalibration(&spl06_calibration) != 0U)
    {
      spl06_initialized = 1U;
      if(SPL06_StartContinuousMeasurement() != 0U)
      {
        spl06_measurement_active = 1U;
        if(SPL06_StopMeasurement() != 0U) spl06_measurement_active = 0U; // 停止测量，等待EnvScreen事件触发启动
      }
    }
  }

  /*
   * LSM303_Init()会分别检查0x19加速度计和0x1E磁力计，并验证磁力计
   * “H43”身份。校准结构保存运行期间采集的Min-Max范围，不是出厂系数。
   */
  LSM303_Service_InitMagCalibration(&lsm303_calibration);
  lsm303_initialized = LSM303_Init();

  if(lsm303_initialized != 0U)
  {
    /* 初始化结束时处于连续测量；开机不显示指南针，因此立即休眠。 */
    lsm303_measurement_active = 1U;

    if(LSM303_StopMeasurement() != 0U) lsm303_measurement_active = 0U; // 等待CompassScreen触发启动
  }

  /* 开机只初始化EM7028并保持停止，等待用户长按主页爱心。 */
  EM7028_Service_Init(&em7028_heart_rate_state);

  if(EM7028_Init() != 0U)
  {
    em7028_initialized = 1U;
    em7028_measurement_active = 0U; // 等待HomeScreen触发启动
  }

  MPU6050_Service_InitStepCounter(&mpu6050_step_counter);
  MPU6050_Service_InitWristRaise(&mpu6050_wrist_raise);

  for(;;)
  {
    sensor_flags = osThreadFlagsGet();
    env_request_changed = 0U;

    /*
     * UiTask只发送模式请求，实际寄存器操作仍由SensorTask完成。
     * 如果两个请求同时存在，让NORMAL优先，避免用户已经唤醒后
     * MPU6050却仍停留在低功耗模式。
     */
    if((sensor_flags & SENSOR_FLAG_MPU_POWER_MASK) != 0U)
    {
      osThreadFlagsClear(SENSOR_FLAG_MPU_POWER_MASK);

      if((sensor_flags & SENSOR_FLAG_MPU_NORMAL) != 0U)
      {
        if((mpu6050_initialized != 0U) && (MPU6050_EnterNormalMode()) != 0U)
        {
          osDelay(mpu6050_low_power_period_ticks); // 等待MPU6050内部配置稳定
          mpu6050_low_power = 0U;
          mpu6050_next_read_tick = osKernelGetTickCount();
        }
      }
      else if((sensor_flags & SENSOR_FLAG_MPU_LOW_POWER) != 0U)
      {
        if((mpu6050_initialized != 0U) && (MPU6050_EnterLowPowerMode() != 0U))
        {
          mpu_stop_wrist_armed = 0U;
          stop_rearm_stable_samples = 0U;
          mpu6050_low_power = 1U;
          mpu6050_next_read_tick = osKernelGetTickCount();
        }
      }
    }

    /*
     * RTC后台唤醒后连续读取3组低功耗加速度数据，样本间隔约50ms，
     * 整个窗口约覆盖100ms。MPU6050仍由SensorTask独占，ControlTask
     * 只负责发出请求并等待结果。
     */
    if((sensor_flags & SENSOR_FLAG_MPU_STOP_PROBE) != 0U)
    {
      (void)osThreadFlagsClear(SENSOR_FLAG_MPU_STOP_PROBE);

      stop_probe_successful_samples = 0U;
      stop_probe_up_samples = 0U;
      stop_probe_wrist_event = 0U;
      mpu_stop_probe_read_ok = 0U;
      mpu_stop_probe_wrist_event = 0U;

      for(stop_probe_sample_index = 0U; stop_probe_sample_index < MPU_STOP_PROBE_SAMPLE_COUNT; stop_probe_sample_index++)
      {
        if((mpu6050_initialized != 0U) && (MPU6050_ReadRawData(&mpu6050_raw_data) != 0U) && (MPU6050_Service_ConvertRawData(&mpu6050_raw_data, &mpu6050_data) != 0U))
        {
          stop_probe_successful_samples++;

          mpu_stop_probe_accel_x_mg = mpu6050_data.accel_x_mg;
          mpu_stop_probe_accel_y_mg = mpu6050_data.accel_y_mg;
          mpu_stop_probe_accel_z_mg = mpu6050_data.accel_z_mg;

          MPU6050_Service_ProcessWristRaise(&mpu6050_data, &mpu6050_wrist_raise);

          if(mpu6050_wrist_raise.wrist_is_up != 0U) stop_probe_up_samples++;

        }

        if((stop_probe_sample_index + 1U) < MPU_STOP_PROBE_SAMPLE_COUNT) osDelay(mpu6050_low_power_period_ticks);
      }

      /*
       * 只有窗口中的全部样本都有效时才判断姿态。满足UP且armed已置位时
       * 产生一次抬腕事件；全部为非UP时重新置位armed，防止保持看表姿态
       * 时被RTC周期唤醒反复点亮屏幕。
       */
      if(stop_probe_successful_samples == MPU_STOP_PROBE_SAMPLE_COUNT)
      {
        if(stop_probe_up_samples >= MPU_STOP_PROBE_UP_CONFIRM_SAMPLES)
        {
          if(mpu_stop_wrist_armed != 0U)
          {
            stop_probe_wrist_event = 1U;
            mpu_stop_wrist_armed = 0U;
          }
        }
        else if(stop_probe_up_samples <= MPU_STOP_PROBE_REARM_MAX_UP_SAMPLES)
        {
          mpu_stop_wrist_armed = 1U;
        }
      }

      mpu6050_data.wrist_raise_count = mpu6050_wrist_raise.raise_count;
      mpu6050_data.wrist_is_up = mpu6050_wrist_raise.wrist_is_up;

      mpu_stop_probe_successful_samples = stop_probe_successful_samples;
      mpu_stop_probe_up_samples = stop_probe_up_samples;
      mpu_stop_probe_read_ok = (stop_probe_successful_samples == MPU_STOP_PROBE_SAMPLE_COUNT) ? 1U : 0U;
      mpu_stop_probe_wrist_event = stop_probe_wrist_event;

      if(stop_probe_wrist_event != 0U) mpu_stop_wrist_event_count++;

      /* 当前窗口已经取得最新数据，推迟普通周期采样以避免立刻重复读取。 */
      mpu6050_next_read_tick = osKernelGetTickCount() + mpu6050_low_power_period_ticks;
      mpu_stop_probe_complete_count++;

      if(ControlTaskHandle != NULL) (void)osThreadFlagsSet(ControlTaskHandle, CONTROL_FLAG_MPU_STOP_PROBE_DONE);
    }

    /*
     * 页面可能在相邻时刻连续发出START和STOP。每组标志先统一清除，
     * 再让STOP拥有更高优先级，避免页面已经卸载后传感器仍被重新启动。
     */
    if((sensor_flags & SENSOR_FLAG_HEART_RATE_MASK) != 0U)
    {
      osThreadFlagsClear(SENSOR_FLAG_HEART_RATE_MASK);

      if((sensor_flags & SENSOR_FLAG_HEART_RATE_STOP) != 0U)
      {
        if((em7028_measurement_active != 0U) && (EM7028_StopMeasurement() != 0U))
        {
          em7028_measurement_active = 0U;
          em7028_measurement_deadline_tick = 0U;
          em7028_bpm = 0U;
          EM7028_Service_Init(&em7028_heart_rate_state); // 停止测量时重置心率计算状态

          ui_message.event = UI_EVENT_HEART_RATE_STATE_UPDATE;
          ui_message.payload.value = HEART_RATE_MEASUREMENT_IDLE;
          osMessageQueuePut(UiEventQueueHandle, &ui_message, 0U, 0U);
        }
      }
      else if((sensor_flags & SENSOR_FLAG_HEART_RATE_START) != 0U)
      {
        if(em7028_initialized == 0U)
        {
          ui_message.event = UI_EVENT_HEART_RATE_STATE_UPDATE;
          ui_message.payload.value = HEART_RATE_MEASUREMENT_UNAVAILABLE;
          osMessageQueuePut(UiEventQueueHandle, &ui_message, 0U, 0U);
        }
        else if(em7028_measurement_active == 0U)
        {
          if(EM7028_StartMeasurement() != 0U)
          {
            em7028_measurement_active = 1U;
            em7028_bpm = 0U;
            EM7028_Service_Init(&em7028_heart_rate_state);
            em7028_next_read_tick = osKernelGetTickCount() + em7028_period_ticks; // 约20ms后读取EM7028数据
            em7028_measurement_deadline_tick = osKernelGetTickCount()  + ticks_per_second * HEART_MEASUREMENT_TIMEOUT_S; // 20秒后超时

            ui_message.event = UI_EVENT_HEART_RATE_STATE_UPDATE;
            ui_message.payload.value = HEART_RATE_MEASUREMENT_RUNNING;
            osMessageQueuePut(UiEventQueueHandle, &ui_message, 0U, 0U);
          }
        }
      }
    }

    /* CompassScreen只控制LSM303，加速度计与磁力计始终同步启停。 */
    if((sensor_flags & SENSOR_FLAG_COMPASS_MASK) != 0U)
    {
      (void)osThreadFlagsClear(SENSOR_FLAG_COMPASS_MASK);

      if((sensor_flags & SENSOR_FLAG_COMPASS_STOP) != 0U)
      {
        if((lsm303_measurement_active != 0U) && (LSM303_StopMeasurement() != 0U)) lsm303_measurement_active = 0U;
      }
      else if((sensor_flags & SENSOR_FLAG_COMPASS_START) != 0U)
      {
        if((lsm303_initialized != 0U) && (lsm303_measurement_active == 0U) && (LSM303_StartMeasurement() != 0U))
        {
          lsm303_measurement_active = 1U;
          lsm303_next_read_tick = osKernelGetTickCount() + lsm303_period_ticks; // 约100ms后读取LSM303数据
        }
      }
    }

    /*
     * EnvScreen事件修改UI对环境传感器的需求；START和STOP同时到达时
     * STOP优先。AHT21的最终开关还要与蓝牙需求合并，SPL06只跟随UI需求。
     */
    if((sensor_flags & SENSOR_FLAG_ENV_MASK) != 0U)
    {
      (void)osThreadFlagsClear(SENSOR_FLAG_ENV_MASK);

      if((sensor_flags & SENSOR_FLAG_ENV_STOP) != 0U)
      {
        env_ui_requested = 0U;
      }
      else if((sensor_flags & SENSOR_FLAG_ENV_START) != 0U)
      {
        env_ui_requested = 1U;
      }

      env_request_changed = 1U;
    }
    /*
     * 收到BLE_ENV_STOP时只清除env_ble_requested，收到BLE_ENV_START时只置位该状态；
     * 两个标志同时到达时STOP分支优先，本轮不再执行START。
     * 此处只记录蓝牙需求，后面再与UI需求合并，不直接启停AHT21或SPL06。
     */
    if((sensor_flags & SENSOR_FLAG_BLE_ENV_MASK) != 0U)
    {
      (void)osThreadFlagsClear(SENSOR_FLAG_BLE_ENV_MASK);

      if((sensor_flags & SENSOR_FLAG_BLE_ENV_STOP) != 0U)
      {
        env_ble_requested = 0U;
      }
      else if((sensor_flags & SENSOR_FLAG_BLE_ENV_START) != 0U)
      {
        env_ble_requested = 1U;
      }

      env_request_changed = 1U;
    }

    if(env_request_changed != 0U)
    {
      /*
       * UI或蓝牙需求发生变化后重新计算AHT21目标状态。任意一方仍有需求时，
       * 启用AHT21并安排立即测量；两方都无需求时，停止后续触发并清除等待中状态。
       */
      aht21_should_run = env_ui_requested || env_ble_requested;

      if(aht21_should_run != 0U)
      {
        if((aht21_initialized != 0U) && (aht21_measurement_enabled == 0U))
        {
          aht21_measurement_enabled = 1U;
          aht21_measurement_pending = 0U;
          aht21_next_measurement_tick = osKernelGetTickCount();
        }
      }
      else
      {
        aht21_measurement_enabled = 0U;
        aht21_measurement_pending = 0U;
      }

      /*
       * SPL06当前只服务EnvScreen：UI需求存在时启动连续测量，UI需求清除时停止。
       * 蓝牙订阅只使用AHT21，因此env_ble_requested不会单独启动SPL06。
       */
      if(env_ui_requested != 0U)
      {
        if((spl06_initialized != 0U) && (spl06_measurement_active == 0U) && (SPL06_StartContinuousMeasurement() != 0U))
        {
          spl06_measurement_active = 1U;
          spl06_next_read_tick = osKernelGetTickCount() + ticks_per_second; // 约1秒后读取SPL06数据
        }
      }
      else
      {
        if((spl06_measurement_active != 0U) && (SPL06_StopMeasurement() != 0U))
        {
          spl06_measurement_active = 0U;
        }
      }
    }

    current_tick = osKernelGetTickCount();

    if((mpu6050_initialized != 0U) && (int32_t)(current_tick - mpu6050_next_read_tick) >= 0)
    {
      /*
       * 使用“上一次计划时间+周期”，可以在SensorTask每20ms运行一次的情况下，
       * 让50ms周期在40ms和60ms之间交替，长期平均仍接近20Hz。
       */
      if(mpu6050_low_power != 0U)
      {
        mpu6050_next_read_tick += mpu6050_low_power_period_ticks;
      }
      else
      {
        mpu6050_next_read_tick += mpu6050_active_period_ticks;
      }

      if((mpu6050_initialized != 0U) && (MPU6050_ReadRawData(&mpu6050_raw_data) != 0U) && (MPU6050_Service_ConvertRawData(&mpu6050_raw_data, &mpu6050_data) != 0U))
      {
        /* 正常模式约50Hz、低功耗模式约20Hz；每10次成功采样向UI发布一次。 */
        (void)MPU6050_Service_ProcessStep(&mpu6050_data, &mpu6050_step_counter);
        mpu6050_data.step_count = mpu6050_step_counter.step_count; // 将基础计步结果写入MPU6050数据结构，供UI显示

        /* 首次采样或步数发生变化时才向BLETask发布；步数未变时跳过队列写入。 */
        if(ble_step_published == 0U || mpu6050_data.step_count != last_ble_step_count)
        {
          ble_sensor_message.event = BLE_SENSOR_EVENT_STEP_UPDATE;
          ble_sensor_message.payload.step_count = mpu6050_data.step_count;

          if(osMessageQueuePut(BLESensorQueueHandle, &ble_sensor_message, 0U, 0U) == osOK)
          {
            last_ble_step_count = mpu6050_data.step_count;
            ble_step_published = 1U;
          }
        }

        (void)MPU6050_Service_ProcessWristRaise(&mpu6050_data, &mpu6050_wrist_raise);
        mpu6050_data.wrist_raise_count = mpu6050_wrist_raise.raise_count; // 将抬腕检测结果写入MPU6050数据结构，供UI显示
        mpu6050_data.wrist_is_up = mpu6050_wrist_raise.wrist_is_up; // 将抬腕状态写入MPU6050数据结构，供UI显示

        /*
         * 只在MPU6050已经进入低功耗模式后准备抬腕唤醒。
         * 连续约500ms都是非UP才置位armed，单次抖动或阈值噪声不会触发。
         */
        if(mpu6050_low_power != 0U)
        {
          if(mpu6050_wrist_raise.wrist_is_up == 0U)
          {
            if(stop_rearm_stable_samples < MPU_STOP_REARM_STABLE_SAMPLES) stop_rearm_stable_samples++;

            if(stop_rearm_stable_samples >= MPU_STOP_REARM_STABLE_SAMPLES) mpu_stop_wrist_armed = 1U;
          }
          else
          {
            stop_rearm_stable_samples = 0U;
          }
        }
        ui_update_divider++;

        if(ui_update_divider >= 10U)
        {
          ui_update_divider = 0U;
          ui_message.event = UI_EVENT_MPU6050_UPDATE;
          ui_message.payload.mpu6050 = mpu6050_data;
          (void)osMessageQueuePut(UiEventQueueHandle, &ui_message, 0U, 0U);
        }
      }
    }

    /*
     * LSM303连续测量期间每约100ms读取一次。每组数据先更新Min-Max校准，
     * range_ready后再计算基础角度和倾斜补偿角度，最后发送给CompassScreen。
     */
    if((lsm303_initialized != 0U) && (lsm303_measurement_active != 0U) && ((int32_t)(current_tick - lsm303_next_read_tick) >= 0))
    {
      if(LSM303_ReadRawData(&lsm303_raw_data) != 0U)
      {
        if(LSM303_Service_UpdateMagCalibration(&lsm303_calibration, &lsm303_raw_data) != 0U)
        {
          if(LSM303_Service_CalculateHorizontalHeading(&lsm303_raw_data, &lsm303_calibration, &lsm303_compass_data) != 0U)
          {
            if(LSM303_Service_CalculateTiltHeading(&lsm303_raw_data, &lsm303_calibration, &lsm303_compass_data) != 0U)
            {
              ui_message.event = UI_EVENT_LSM303_UPDATE;
              ui_message.payload.lsm303 = lsm303_compass_data;
              (void)osMessageQueuePut(UiEventQueueHandle, &ui_message, 0U, 0U);
            }
          }
        }
      }
      lsm303_next_read_tick = current_tick + lsm303_period_ticks; // 约100ms后再次读取LSM303数据
    }

    /*
     * SPL06连续测量时每秒读取一次。数据尚未就绪时100ms后重试，
     * 这属于正常测量状态，不当作传感器初始化失败。
     */
    if((spl06_initialized != 0U) && (spl06_measurement_active != 0U) && ((int32_t)(current_tick - spl06_next_read_tick) >= 0))
    {
      if(SPL06_ReadRawData(&spl06_raw_data) != 0U)
      {
        if(SPL06_Service_ConvertRawData(&spl06_raw_data, &spl06_calibration, &spl06_data) != 0U)
        {
          ui_message.event = UI_EVENT_SPL06_UPDATE;
          ui_message.payload.spl06 = spl06_data;
          (void)osMessageQueuePut(UiEventQueueHandle, &ui_message, 0U, 0U);
        }

        spl06_next_read_tick = current_tick + ticks_per_second; // 读取成功时1秒后再读取
      }
      else
      {
        spl06_next_read_tick = current_tick + ticks_per_second / 10U; // 读取失败时1/10秒后再次尝试
      }
    }

    /* 到达测量周期后只发送触发命令，不在这里阻塞等待结果。 */
    if((aht21_initialized != 0U) && (aht21_measurement_pending == 0U) && (aht21_measurement_enabled != 0U) &&
       ((int32_t)(current_tick - aht21_next_measurement_tick) >= 0))
    {
      if(AHT21_SendMeasureCommand() != 0U)
      {
        aht21_measurement_pending = 1U;
        aht21_measurement_start_tick = current_tick;
      }
      else
      {
        aht21_next_measurement_tick = current_tick + aht21_period_ticks;
      }
    }

    /* 测量等待时间到达后读取6字节结果，并换算成0.01℃和0.01%RH。 */
    if((aht21_measurement_enabled != 0U) && (aht21_measurement_pending != 0U) && ((current_tick - aht21_measurement_start_tick) >= aht21_wait_ticks))
    {
      aht21_measurement_pending = 0U;

      if(AHT21_ReadMeasurement(&aht21_raw_data, &aht21_status) != 0U)
      {
        if(AHT21_Service_ConvertRawData(&aht21_raw_data, &aht21_data) != 0U)
        {
          /*
           * UI队列和蓝牙队列各保存一份结构体副本。
           * UiTask和BLETask分别消费自己的队列，互不争抢消息。
           */
          ui_message.event = UI_EVENT_AHT21_UPDATE;
          ui_message.payload.aht21 = aht21_data;
          (void)osMessageQueuePut(UiEventQueueHandle, &ui_message, 0U, 0U);

          ble_sensor_message.event = BLE_SENSOR_EVENT_AHT21_UPDATE;
          ble_sensor_message.payload.aht21 = aht21_data;
          (void)osMessageQueuePut(BLESensorQueueHandle, &ble_sensor_message, 0U, 0U);
        }
      }
      aht21_next_measurement_tick = current_tick + aht21_period_ticks;
    }

    /*
     * SensorTask以约50Hz读取EM7028原始PPG数据，并交给心率服务处理。
     * ProcessSample()只有在产生新的稳定BPM时才返回1；有效结果会同时
     * 发布到UI与蓝牙队列。SensorTask不直接访问任何LVGL对象。
     *
     * 数据流：EM7028 -> SensorTask -> UiEventQueue/BLESensorQueue
     */
    if((em7028_initialized != 0U) && (em7028_measurement_active != 0U) && ((int32_t)(current_tick - em7028_next_read_tick) >= 0))
    {
      if(EM7028_ReadRawData(&em7028_raw_data) != 0U)
      {
        if(EM7028_Service_ProcessSample(&em7028_heart_rate_state, em7028_raw_data, current_tick, ticks_per_second, &em7028_bpm) != 0U && em7028_bpm >= 40U && em7028_bpm <= 200U)
        {
          /*
           * 先确认EM7028已经停止，再发布最终BPM。
           * 因此界面一旦显示结果，就代表本次测量已经真正结束。
           */
          if(EM7028_StopMeasurement() != 0U)
          {
            em7028_measurement_active = 0U;
            em7028_measurement_deadline_tick = 0U;
            EM7028_Service_Init(&em7028_heart_rate_state);

            ui_message.event = UI_EVENT_HEART_RATE_UPDATE;
            ui_message.payload.value = em7028_bpm;
            (void)osMessageQueuePut(UiEventQueueHandle, &ui_message, 0U, 0U);

            ble_sensor_message.event = BLE_SENSOR_EVENT_HEART_RATE_UPDATE;
            ble_sensor_message.payload.heart_rate_bpm = em7028_bpm;
            (void)osMessageQueuePut(BLESensorQueueHandle, &ble_sensor_message, 0U, 0U);
          }
        }
      }

      if(em7028_measurement_active != 0U) em7028_next_read_tick = current_tick + em7028_period_ticks;
    }

    /*
     * 20秒内没有得到有效BPM时关闭传感器。
     * 使用有符号差值比较，可在FreeRTOS Tick计数回绕时继续正确判断截止时间。
     */
    if((em7028_measurement_active != 0U) && ((int32_t)(current_tick - em7028_measurement_deadline_tick) >= 0))
    {
      if(EM7028_StopMeasurement() != 0U)
      {
        em7028_measurement_active = 0U;
        em7028_measurement_deadline_tick = 0U;
        em7028_bpm = 0U;
        EM7028_Service_Init(&em7028_heart_rate_state);

        ui_message.event = UI_EVENT_HEART_RATE_STATE_UPDATE;
        ui_message.payload.value = HEART_RATE_MEASUREMENT_TIMEOUT;
        (void)osMessageQueuePut(UiEventQueueHandle, &ui_message, 0U, 0U);
      }
    }

    osDelay(20U);
  }

  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartBLETask */
/**
 * @brief 蓝牙串口接收、协议解析与响应任务
 *
 * DMA配合USART1空闲线事件接收数据，回调将字节副本放入BLERXQueue。
 * BLETask持续取出字节，以换行符为边界组合完整数据行。
 *
 * ER+X、AT+VER等内容属于KT6368A模块反馈，不是LFC协议帧。
 * 只有以@开头的数据才交给BLE_Protocol_Parse()检查。
 *
 * BLETask是USART1业务收发的唯一拥有者。协议模块只处理内存中的
 * 字节，不直接访问UART、FreeRTOS或其他硬件。
 * LFC+OTA只负责应答并向ControlTask投递事件，当前任务不接收固件、不跳转。
 *
 * @param argument 创建任务时传入的参数，本工程未使用
 * @retval None
 */
/* USER CODE END Header_StartBLETask */
void StartBLETask(void *argument)
{
  /* USER CODE BEGIN StartBLETask */

  /* BLETask只发出控制请求，外设清理和跳转由ControlTask完成。 */
  ControlMessage_t ota_message = {0}; // OTA命令请求消息

  /* USART1字节接收、行组帧与协议解析。 */
  uint8_t rx_byte; // 从BLERXQueue取出的单字节数据
  char line_buffer[BLE_MAX_ENCODED_LENGTH]; // 累积接收字节，直到遇到换行符
  uint32_t line_length = 0U; // 当前数据行已经累积的字节数
  uint8_t line_overflow = 0U; // 1表示当前数据行已经超过缓冲区容量
  BLE_Frame_t rx_frame; // 保存BLE_Protocol_Parse()解析得到的负载

  /* LFC响应帧构建与命令分发。 */
  uint8_t tx_buffer[BLE_MAX_ENCODED_LENGTH]; // 保存准备通过USART1发送的完整LFC帧
  uint16_t tx_length = 0U; // BLE_Protocol_Build()返回的完整帧长度
  BLE_CommandType_t command_type; // 协议帧解析成功后识别出的命令类型

  /* SensorTask输入与BLETask保存的最新传感器快照。 */
  BLE_SensorMessage_t sensor_message = {0}; // 从BLESensorQueue取出的传感器消息副本
  AHT21_Data_t latest_aht21_data = {0}; // 最近一次温湿度数据
  uint8_t latest_aht21_valid = 0U; // 1表示latest_aht21_data已经收到有效值
  uint32_t latest_step_count = 0U; // 最近一次累计步数
  uint8_t latest_step_valid = 0U; // 1表示已经收到过步数消息
  uint16_t latest_heart_rate_bpm = 0U; // 最近一次有效心率
  uint8_t latest_heart_rate_valid = 0U; // 1表示已经收到过有效心率

  /* 传感器、活动数据与版本响应的文本负载。 */
  uint8_t sensor_payload[BLE_MAX_PAYLOAD_LENGTH + 1U] = {0}; // DATA或ENV传感器负载
  int32_t formatted_length = 0; // snprintf()返回的传感器负载字符数
  uint8_t activity_payload[BLE_MAX_PAYLOAD_LENGTH + 1U] = {0}; // ACT活动数据负载
  int activity_length = 0; // snprintf()返回的活动负载字符数
  uint8_t version_payload[BLE_MAX_PAYLOAD_LENGTH + 1U] = {0}; // VERSION版本负载
  int version_length = 0; // snprintf()返回的版本负载字符数

  /* RTC校时请求与LFC+SEND时间快照。 */
  RTC_DateTime_t requested_date_time; // TIME命令解析后的目标日期时间
  uint8_t rtc_set_ok = 0U; // 1表示RTC日期时间设置成功
  RTC_DateTime_t snapshot_date_time = {0}; // LFC+SEND读取到的当前RTC时间
  uint8_t rtc_read_ok = 0U; // 1表示RTC时间快照读取成功

  /* BLETask控制请求与手机环境订阅状态。 */
  uint32_t ble_control_flags = 0U; // 本轮收到的蓝牙电源控制请求
  uint8_t env_stream_enabled = 0U; // 1表示手机已经开启温湿度连续推送

  /* 命令处理使用的固定响应负载。 */
  static const uint8_t pong_payload[] = "PONG"; // PING命令成功响应
  static const uint8_t ota_payload[] = "OK:OTA"; // OTA命令已识别的应答，不代表固件传输完成
  static const uint8_t unknown_payload[] = "ERR:UNKNOWN"; // 未识别的LFC命令
  static const uint8_t frame_error_payload[] = "ERR:FRAME"; // LFC帧格式、长度或校验错误
  static const uint8_t overflow_payload[] = "ERR:OVERFLOW"; // 接收行超过缓冲区容量
  static const uint8_t time_ok_payload[] = "OK:TIME"; // RTC日期时间设置成功
  static const uint8_t time_error_payload[] = "ERR:TIME"; // TIME命令格式或RTC日期时间无效
  static const uint8_t version_error_payload[] = "ERR:VERSION"; // 版本负载格式化失败
  static const uint8_t no_env_payload[] = "ERR:NO_ENV"; // 尚未收到有效AHT21测量结果
  static const uint8_t rtc_error_payload[] = "ERR:RTC"; // LFC+SEND读取RTC快照失败
  static const uint8_t data_error_payload[] = "ERR:DATA"; // DATA或ACT负载格式化失败
  static const uint8_t env_on_payload[] = "OK:ENV=ON"; // 温湿度连续推送已开启
  static const uint8_t env_off_payload[] = "OK:ENV=OFF"; // 温湿度连续推送已关闭

  /*
   * 如果创建BLETask时模块已经开启，立即启动USART1空闲接收DMA。
   * 启动成功后关闭半传中断，只在空闲或满缓冲事件中处理有效字节。
   * 模块未开启时跳过此处，后续由POWER_ON分支给模块上电并启动DMA。
   */
  if(ble_module_enabled != 0U)
  {
    ble_dma_start_status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, ble_rx_dma_buffer, sizeof(ble_rx_dma_buffer));
    if(ble_dma_start_status == HAL_OK)
    {
      __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
  }
  else
  {
    ble_uart_deinit_status = HAL_UART_DeInit(&huart1);
  }

  for(;;)
  {
    /* 非阻塞读取开关请求；没有标志时继续处理传感器和UART队列。 */
    ble_control_flags = osThreadFlagsGet();

    if((ble_control_flags & BLE_FLAG_POWER_MASK) != 0U)
    {
      /* 先清除ON和OFF两位，保证本轮读到的开关请求只执行一次。 */
      osThreadFlagsClear(BLE_FLAG_POWER_MASK);

      /* ON和OFF同时到达时先进入OFF分支，不再执行本轮ON请求。 */
      if((ble_control_flags & BLE_FLAG_POWER_OFF) != 0U)
      {
        ble_power_off_request_count++;

        if(ble_module_enabled != 0U)
        {
          /*
           * 模块当前已开启时，先清除应用会话和环境订阅，并通知SensorTask
           * 取消蓝牙对AHT21的需求；随后终止DMA、清空字节队列和半帧，最后断电。
           * 模块已经关闭时跳过整套操作，避免重复终止USART1接收。
           */
          ble_module_enabled = 0U;

          ble_app_session_active = 0U;
          env_stream_enabled = 0U;

          if(SensorTaskHandle != NULL)
          {
            osThreadFlagsSet(SensorTaskHandle, SENSOR_FLAG_BLE_ENV_STOP);
          }

          /* 返回时DMA、IDLE中断和USART DMA请求均已停止。 */
          HAL_UART_AbortReceive(&huart1);

          /* 丢弃断电前未处理的字节，防止下次上电时新数据与旧半帧拼接。 */
          if(BLERXQueueHandle != NULL)
          {
            osMessageQueueReset(BLERXQueueHandle);
          }

          line_length = 0U;
          line_overflow = 0U;

          HAL_GPIO_WritePin(BLE_EN_GPIO_Port, BLE_EN_Pin, GPIO_PIN_RESET); // 关闭蓝牙模块电源
          ble_uart_deinit_status = HAL_UART_DeInit(&huart1); // 关闭USART1时钟、GPIO复用、DMA和中断
          BLE_PublishUiStatus();
        }
      }
      else if((ble_control_flags & BLE_FLAG_POWER_ON) != 0U)
      {
        ble_power_on_request_count++;

        if(ble_module_enabled == 0U)
        {
          /*
           * 模块当前关闭时，先记录目标状态再拉高BLE_EN；等待上电稳定后
           * 清除旧IDLE标志并启动DMA，最后向UiTask发布READY状态。
           * 模块已经开启时跳过本分支，不重复上电或重启接收。
           */
          ble_module_enabled = 1U;
          HAL_GPIO_WritePin(BLE_EN_GPIO_Port, BLE_EN_Pin, GPIO_PIN_SET); // 打开蓝牙模块电源

          osDelay(200U); // 等待KT6368A上电稳定

          MX_USART1_UART_Init(); // 重新初始化USART1，恢复DMA和中断
          ble_uart_init_count++;

          __HAL_UART_CLEAR_IDLEFLAG(&huart1); // 清除空闲线标志，避免上电瞬间触发中断

          ble_dma_start_status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, ble_rx_dma_buffer, sizeof(ble_rx_dma_buffer));
          BLE_PublishUiStatus();

          if(ble_dma_start_status == HAL_OK)
          {
            __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT); // 禁止半传中断，避免干扰空闲线回调
          }
        }
      }
    }

    /*
     * 先非阻塞取完BLESensorQueue中已积压的消息，将各类数据更新到BLETask本地缓存。
     * 队列取空后离开while，继续处理手机发来的UART字节；SensorTask不直接访问huart1。
     */
    while(osMessageQueueGet(BLESensorQueueHandle, &sensor_message, NULL, 0U) == osOK)
    {
      if(sensor_message.event == BLE_SENSOR_EVENT_AHT21_UPDATE)
      {
        latest_aht21_data = sensor_message.payload.aht21;
        latest_aht21_valid = 1U;

        /*
         * 应用会话有效且环境订阅已开启时，将本次温湿度缓存格式化为ENV负载，
         * 再封装成LFC协议帧，通过USART1交给KT6368A发送至手机。
         * 任意一个条件不满足时跳过本次推送，但前面更新的最新值缓存仍然保留。
         */
        if(ble_app_session_active != 0U && env_stream_enabled != 0U)
        {
          formatted_length = snprintf((char *)sensor_payload, sizeof(sensor_payload), "ENV:T=%ld,H=%ld", (long)latest_aht21_data.temperature_centi_c, (long)latest_aht21_data.humidity_centi_rh);

          /* 格式化成功且未截断时继续组帧；否则跳过本次发送。 */
          if(formatted_length > 0 && (uint32_t)formatted_length < sizeof(sensor_payload))
          {
            tx_length = BLE_Protocol_Build(sensor_payload, (uint16_t)formatted_length, tx_buffer, sizeof(tx_buffer));

            if(tx_length > 0U) HAL_UART_Transmit(&huart1, tx_buffer, tx_length, 50U);
          }
        }

      }
      else if(sensor_message.event == BLE_SENSOR_EVENT_STEP_UPDATE)
      {
        latest_step_count = sensor_message.payload.step_count;
        latest_step_valid = 1U;
      }
      else if(sensor_message.event == BLE_SENSOR_EVENT_HEART_RATE_UPDATE)
      {
        latest_heart_rate_bpm = sensor_message.payload.heart_rate_bpm;
        latest_heart_rate_valid = 1U;
      }
    }

    /*
     * 最多等待20个系统节拍接收一个UART字节。超时时回到循环顶部，
     * 使BLETask在手机没有发数据时仍能处理传感器缓存和开关请求。
     */
    if(osMessageQueueGet(BLERXQueueHandle, &rx_byte, NULL, 20U) != osOK) continue;

    if(rx_byte == '\r') continue; // 忽略回车，命令以换行符结束

    if(rx_byte == '\n')
    {
      if(line_overflow != 0U)
      {
        /*
         * 当前行已超出缓冲区：收到换行后只回复OVERFLOW，不解析被截断的内容。
         * 回复后清空行状态，下一个字节从新帧开始累积。
         */
        tx_length = BLE_Protocol_Build(overflow_payload, sizeof(overflow_payload) - 1U, tx_buffer, sizeof(tx_buffer));

        if(tx_length > 0U) HAL_UART_Transmit(&huart1, tx_buffer, tx_length, 50U);

        line_length = 0U;
        line_overflow = 0U;
        continue;
      }

      if(line_length == 0U) continue; // 空行没有可解析内容，直接等待下一行

      line_buffer[line_length] = '\0'; // 确保字符串以'\0'结尾
      ble_line_count++;
      tx_length = 0U;

      /*
       * ER+X由KT6368A产生，不是手机发来的LFC命令，因此不进入协议解析。
       * 应用会话已经ACTIVE时，清除会话和订阅并通知SensorTask停止蓝牙需求；
       * 尚未ACTIVE时只忽略此模块反馈，不向KT6368A回复LFC错误帧。
       */
      if(strncmp(line_buffer, "ER+", 3U) == 0)
      {
        if(ble_app_session_active != 0U)
        {
          ble_app_session_active = 0U;
          env_stream_enabled = 0U;
          ble_disconnect_event_count++;

          BLE_PublishUiStatus();

          if(SensorTaskHandle != NULL)
          {
            osThreadFlagsSet(SensorTaskHandle, SENSOR_FLAG_BLE_ENV_STOP);
          }
        }
      }
      /*
       * 非@开头的AT+VER、TM+、TN+、TD+、T4+等内容属于KT6368A信息，直接忽略。
       * 如果把这些内容当成LFC坏帧并回复ERR:FRAME，模块可能再返回ER+X，形成错误反馈链。
       */
      else if(line_buffer[0] != '@')
      {
        /* 忽略模块上电信息和其他非LFC数据。 */
      }
      else if(BLE_Protocol_Parse((const uint8_t *)line_buffer, (uint16_t)line_length, &rx_frame) == 0U)
      {
        /* @开头但帧结构、负载长度或异或校验失败，回复ERR:FRAME后重置当前行。 */
        tx_length = BLE_Protocol_Build(frame_error_payload, sizeof(frame_error_payload) - 1U, tx_buffer, sizeof(tx_buffer));
      }
      else
      {
        /*
         * 协议校验成功后，将本次会话标记为ACTIVE并发布给UiTask；
         * 已经ACTIVE时不重复发布状态，直接进入命令识别与执行。
         */
        if(ble_app_session_active == 0U)
        {
          ble_app_session_active = 1U;
          ble_connect_event_count++;
          BLE_PublishUiStatus();
        }

        command_type = BLE_Command_GetType(rx_frame.payload, rx_frame.payload_length);

        switch(command_type)
        {
          case BLE_COMMAND_PING:
            tx_length = BLE_Protocol_Build(pong_payload, sizeof(pong_payload) - 1U, tx_buffer, sizeof(tx_buffer));
            break;

          case BLE_COMMAND_ENTER_OTA:
            tx_length = BLE_Protocol_Build(ota_payload, sizeof(ota_payload) - 1U, tx_buffer, sizeof(tx_buffer));

            /*
             * 先完成MCU串口发送，再投递交接请求，避免清理USART1时截断应答。
             * HAL_OK不代表对端已经收到；ControlTask会再延时约500ms供模块转发。
             * 发送失败不入队；入队失败时补ERR:OTA_QUEUE并继续留在APP。
             */
            if((tx_length > 0U) && (HAL_UART_Transmit(&huart1, tx_buffer, tx_length, 50U) == HAL_OK))
            {
              tx_length = 0U; // 本分支已发送，后面的统一发送位置不再重复发送

              ota_message.event = CONTROL_EVENT_ENTER_OTA;
              ota_message.value = 0;

              /*
               * 非阻塞投递：成功后ControlTask异步处理，BLETask不在此等待跳转。
               * 失败时前面的OK:OTA已经发出，再由统一发送位置补ERR:OTA_QUEUE。
               */
              if(osMessageQueuePut(ControlEventQueueHandle, &ota_message, 0U, 0U) != osOK)
              {
                tx_length = BLE_Protocol_Build((const uint8_t *)"ERR:OTA_QUEUE", sizeof("ERR:OTA_QUEUE") - 1U, tx_buffer, sizeof(tx_buffer));
              }
            }
            else
            {
              tx_length = 0U;
            }
            break;

          case BLE_COMMAND_GET_VERSION:
            /* 版本号格式化成功时回复VERSION帧；缓冲区不足或格式化失败时回复ERR:VERSION。 */
            version_length = snprintf((char *)version_payload, sizeof(version_payload), "VERSION=V%u.%u.%u", APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_VERSION_PATCH);

            if((version_length > 0) && ((uint32_t)version_length < sizeof(version_payload)))
            {
              tx_length = BLE_Protocol_Build(version_payload, (uint16_t)version_length, tx_buffer, sizeof(tx_buffer));
            }
            else
            {
              tx_length = BLE_Protocol_Build(version_error_payload, sizeof(version_error_payload) - 1U, tx_buffer, sizeof(tx_buffer));
            }
            break;

          case BLE_COMMAND_SEND_DATA:
            /*
             * LFC+SEND先检查温湿度缓存；没有有效数据时立即回复ERR:NO_ENV，
             * 不再继续读RTC或构建DATA、ACT两个响应帧。
             */
            if(latest_aht21_valid == 0U)
            {
              tx_length = BLE_Protocol_Build(no_env_payload, sizeof(no_env_payload) - 1U, tx_buffer, sizeof(tx_buffer));
              break;
            }

            rtc_read_ok = 0U;

            /* 获取互斥锁后完整读取RTC时间快照；获锁或读取失败时回复ERR:RTC。 */
            if(osMutexAcquire(RtcMutexHandle, osWaitForever) == osOK)
            {
              rtc_read_ok = RTC_Service_Read(&snapshot_date_time);
              osMutexRelease(RtcMutexHandle);
            }

            if(rtc_read_ok == 0U)
            {
              tx_length = BLE_Protocol_Build(rtc_error_payload, sizeof(rtc_error_payload) - 1U, tx_buffer, sizeof(tx_buffer));
              break;
            }

            formatted_length = snprintf((char *)sensor_payload, sizeof(sensor_payload), "DATA:%04u-%02u-%02u,%02u:%02u:%02u,T=%ld,H=%ld",
              (unsigned int)snapshot_date_time.year, (unsigned int)snapshot_date_time.month, (unsigned int)snapshot_date_time.date,
              (unsigned int)snapshot_date_time.hour, (unsigned int)snapshot_date_time.minute, (unsigned int)snapshot_date_time.second,
              (long)latest_aht21_data.temperature_centi_c, (long)latest_aht21_data.humidity_centi_rh);

            if(formatted_length > 0 && (uint32_t)formatted_length < sizeof(sensor_payload))
            {
              tx_length = BLE_Protocol_Build(sensor_payload, (uint16_t)formatted_length, tx_buffer, sizeof(tx_buffer));

              /*
              * LFC+SEND需要返回两个帧。先发送DATA帧，阻塞式发送完成后，
              * tx_buffer才可以安全地重新用于构建ACT帧。
              */
              if(tx_length > 0U)
              {
                HAL_UART_Transmit(&huart1, tx_buffer, tx_length, 50U);
              }
            }
            else
            {
              tx_length = BLE_Protocol_Build(data_error_payload, sizeof(data_error_payload) - 1U, tx_buffer, sizeof(tx_buffer));
              break;
            }

            /*
             * 心率和步数都有效时填入实际值；任何一项尚无有效缓存时，
             * ACT帧对应字段写入NA，不因单个传感器数据缺失放弃整个活动快照。
             */
            if(latest_heart_rate_valid != 0U && latest_step_valid != 0U)
            {
              activity_length = snprintf((char *)activity_payload, sizeof(activity_payload), "ACT:HR=%u,STEP=%lu", (unsigned int)latest_heart_rate_bpm, (unsigned long)latest_step_count);
            }
            else if(latest_heart_rate_valid  != 0U)
            {
              activity_length = snprintf((char *)activity_payload, sizeof(activity_payload), "ACT:HR=%u,STEP=NA",(unsigned int)latest_heart_rate_bpm);
            }
            else if(latest_step_valid != 0U)
            {
              activity_length = snprintf((char *)activity_payload, sizeof(activity_payload),"ACT:HR=NA,STEP=%lu", (unsigned long)latest_step_count);
            }
            else
            {
              activity_length = snprintf((char *)activity_payload, sizeof(activity_payload),"ACT:HR=NA,STEP=NA");
            }

            if(activity_length > 0 && (uint32_t)activity_length < sizeof(activity_payload))
            {
              tx_length = BLE_Protocol_Build(activity_payload, (uint16_t)activity_length,tx_buffer, sizeof(tx_buffer));
            }
            else
            {
              tx_length = BLE_Protocol_Build(data_error_payload, sizeof(data_error_payload) - 1U, tx_buffer, sizeof(tx_buffer));
            }
            break;

          case BLE_COMMAND_SET_TIME:
            rtc_set_ok = 0U;

            /*
             * TIME负载格式合法时获取RtcMutex，将完整日期时间一次写入RTC。
             * 文本解析、获锁或RTC参数校验任意一步失败时，rtc_set_ok保持0并回复ERR:TIME。
             */
            if(BLE_Command_ParseTime(rx_frame.payload, rx_frame.payload_length, &requested_date_time) != 0U)
            {
              if(osMutexAcquire(RtcMutexHandle, osWaitForever) == osOK)
              {
                rtc_set_ok = RTC_Service_SetDateTime(&requested_date_time);
                osMutexRelease(RtcMutexHandle);
              }
            }

            if(rtc_set_ok != 0U)
            {
              tx_length = BLE_Protocol_Build(time_ok_payload, sizeof(time_ok_payload) - 1U, tx_buffer, sizeof(tx_buffer));
            }
            else
            {
              tx_length = BLE_Protocol_Build(time_error_payload, sizeof(time_error_payload) - 1U, tx_buffer, sizeof(tx_buffer));
            }
            break;

          case BLE_COMMAND_ENV_STREAM_ON:
            /* 记录订阅状态并通知SensorTask开启蓝牙对AHT21的需求，后续新温湿度会自动推送。 */
            env_stream_enabled = 1U;

            if(SensorTaskHandle != NULL) osThreadFlagsSet(SensorTaskHandle, SENSOR_FLAG_BLE_ENV_START);

            tx_length = BLE_Protocol_Build(env_on_payload, sizeof(env_on_payload) - 1U, tx_buffer, sizeof(tx_buffer));
            break;

          case BLE_COMMAND_ENV_STREAM_OFF:
            /* 清除订阅并通知SensorTask取消蓝牙需求；只停止主动推送，不清除BLETask最新值缓存。 */
            env_stream_enabled = 0U;
            if(SensorTaskHandle != NULL) osThreadFlagsSet(SensorTaskHandle, SENSOR_FLAG_BLE_ENV_STOP);
            tx_length = BLE_Protocol_Build(env_off_payload, sizeof(env_off_payload) - 1U, tx_buffer, sizeof(tx_buffer));
            break;

          case BLE_COMMAND_UNKNOWN:
          default:
            ble_unknown_cmd_count++;
            tx_length = BLE_Protocol_Build(unknown_payload, sizeof(unknown_payload) - 1U, tx_buffer, sizeof(tx_buffer));
            break;
        }
      }

      /*
       * SEND的DATA帧和OTA的OK:OTA已在各自分支发送；其余响应、SEND的ACT帧
       * 以及OTA入队失败的错误帧在此统一发送。tx_length为0也可能表示已经发送，
       * 此时跳过重复发送，但仍重置行缓冲并继续接收下一帧。
       */
      if(tx_length > 0U) HAL_UART_Transmit(&huart1, tx_buffer, tx_length, 50U);

      line_length = 0U; // 重置行缓冲区长度，准备接收下一行
      continue;
    }

    if(line_overflow == 0U)
    {
      /* 缓冲区仍有空间时保存当前字节；否则标记溢出，丢弃后续字节直到收到换行符。 */
      if(line_length < sizeof(line_buffer) - 1U)
      {
        line_buffer[line_length++] = (char)rx_byte; // 将接收到的字节存入行缓冲区
      }
      else
      {
        line_overflow = 1U; // 标记当前命令过长
        ble_line_overflow_count++;
      }
    }
  }
  /* USER CODE END StartBLETask */
}

/* USER CODE BEGIN Header_StartWatchdogTask */
/**
 * @brief 外部TPS3823看门狗喂狗任务
 *
 * 任务周期翻转WDI并累计喂狗次数。EXTERNAL_WATCHDOG_ENABLED为1时，
 * WDOG_EN保持低电平使能看门狗；为0时保持高电平，便于长时间断点调试。
 * 进入STOP前由System_EnterStopMode()暂时禁用看门狗，唤醒后重新使能
 * 并立即补一次WDI边沿，避免恢复任务调度前发生复位。
 *
 * @param argument 未使用
 * @retval None
 */
/* USER CODE END Header_StartWatchdogTask */
void StartWatchdogTask(void *argument)
{
  /* USER CODE BEGIN StartWatchdogTask */
  for(;;)
  {
    HAL_GPIO_TogglePin(WDOG_WDI_GPIO_Port, WDOG_WDI_Pin);
    watchdog_feed_count++;

    #if EXTERNAL_WATCHDOG_ENABLED
      HAL_GPIO_WritePin(WDOG_EN_GPIO_Port, WDOG_EN_Pin, GPIO_PIN_RESET);
    #else
      HAL_GPIO_WritePin(WDOG_EN_GPIO_Port, WDOG_EN_Pin, GPIO_PIN_SET);
    #endif

    osDelay(WATCHDOG_FEED_PERIOD_TICKS);
  }
  /* USER CODE END StartWatchdogTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief 接收SquareLine亮度滑块产生的实时亮度值
 *
 * SquareLine在滑块数值变化时调用本函数。本函数不直接修改TIM3，
 * 而是将亮度百分比发送给ControlTask，由ControlTask统一更新PWM。
 *
 * 调用链路：
 * BrightnessSliderChanged()
 *     -> APP_BrightnessChanged()
 *     -> ControlEventQueue
 *     -> ControlTask
 *     -> TIM3 PWM
 *
 * @param percent 当前亮度百分比，范围为0~100
 */
void APP_BrightnessChanged(uint8_t percent)
{
  ControlMessage_t message;

  message.event = CONTROL_EVENT_BRIGHTNESS_CHANGED;
  message.value = percent;

  /* 非阻塞发送，避免LVGL事件回调因控制队列暂满而长时间停顿。 */
  osMessageQueuePut(ControlEventQueueHandle, &message, 0U, 0U);
}

/**
 * @brief 接收设置页面提交的蓝牙电源状态
 *
 * enabled非0时向BLETask发送POWER_ON，为0时发送POWER_OFF。
 * BLETask句柄尚未创建时直接返回；有效时只发送请求，不在LVGL回调中
 * 直接操作BLE_EN、USART1或DMA。
 *
 * @param enabled 1=请求开启蓝牙，0=请求关闭蓝牙
 */
void APP_BLEPowerChanged(uint8_t enabled)
{
  if(BLETaskHandle == NULL) return;

  if(enabled != 0U)
  {
    osThreadFlagsSet(BLETaskHandle, BLE_FLAG_POWER_ON);
  }
  else
  {
    osThreadFlagsSet(BLETaskHandle, BLE_FLAG_POWER_OFF);
  }
}

/**
 * @brief 查询蓝牙模块的软件目标电源状态
 * @retval 1U=模块已开启，0U=模块已关闭
 */
uint8_t APP_BLEPowerIsEnabled(void)
{
  if(ble_module_enabled != 0U) return 1U;

  return 0U;
}

/**
 * @brief 根据模块电源与LFC应用会话生成UI状态
 *
 * 模块关闭时返回OFF；模块开启且已收到合法LFC帧时返回ACTIVE；
 * 其余已上电但应用通信尚未验证的情况返回READY。
 */
APP_BLEStatus_t APP_BLEGetStatus(void)
{
  if(ble_module_enabled == 0U) return APP_BLE_STATUS_OFF;

  if(ble_app_session_active != 0U) return APP_BLE_STATUS_ACTIVE;

  return APP_BLE_STATUS_READY;
}

/*
 * 将APP_BLEGetStatus()得到的OFF、READY或ACTIVE封装成UIMessage_t。
 * UiEventQueue未创建时跳过发送；队列有效时将状态交给UiTask，
 * 由UiTask调用BLEStatusLabelUpdate()，BLETask不直接访问LVGL对象。
 */
static void BLE_PublishUiStatus(void)
{
  UIMessage_t ui_message = {0};

  if(UiEventQueueHandle == NULL) return;

  ui_message.event = UI_EVENT_BLE_STATUS_UPDATE;
  ui_message.payload.value = (int32_t)APP_BLEGetStatus();

  osMessageQueuePut(UiEventQueueHandle, &ui_message, 0U, 0U);
}

/**
 * @brief 请求保存用户最终选择的亮度
 *
 * 本函数不直接访问EEPROM，只将最终亮度发送给ControlTask。
 * EEPROM的软件I2C操作和内部写周期不会因此阻塞UiTask。
 *
 * 调用链路：
 * APP_BrightnessSliderReleased()
 *     -> APP_BrightnessSaveRequested()
 *     -> ControlEventQueue
 *     -> ControlTask
 *     -> SettingsService
 *     -> BL24C02 EEPROM
 *
 * @param percent 用户松开滑块时的最终亮度百分比
 */
void APP_BrightnessSaveRequested(uint8_t percent)
{
  ControlMessage_t message;

  message.event = CONTROL_EVENT_BRIGHTNESS_SAVE;
  message.value = percent;

  /* 非阻塞发送，避免LVGL事件回调等待控制任务。 */
  osMessageQueuePut(ControlEventQueueHandle, &message, 0U, 0U);
}

/**
 * @brief 处理亮度滑块释放事件
 *
 * SquareLine Studio 1.6.1没有提供Released事件选项，因此UiTask在ui_init()
 * 完成后直接给亮度滑块注册LVGL原生的LV_EVENT_RELEASED回调。
 *
 * 拖动过程中由LV_EVENT_VALUE_CHANGED实时更新PWM；用户松手后，本回调
 * 只读取一次最终亮度并发出保存请求，从而避免连续写入EEPROM。
 *
 * @param e LVGL滑块释放事件
 */
static void APP_BrightnessSliderReleased(lv_event_t *e)
{
  lv_obj_t *slider; // 产生释放事件的滑块
  int32_t brightness_percent; // 用户最终选择的亮度

  slider = lv_event_get_target(e);
  if(slider == NULL) return;

  brightness_percent = lv_slider_get_value(slider);

  /* 防止异常数据进入控制队列和EEPROM。 */
  if(brightness_percent < 0) brightness_percent = 0;
  if(brightness_percent > 100) brightness_percent = 100;

  APP_BrightnessSaveRequested((uint8_t)brightness_percent);
}

/**
 * @brief 多次采样ADC并换算成校准后的电池电压
 *
 * 运行流程：
 * 1. 启动ADC并尝试采样8次，只累计成功样本。
 * 2. 对成功样本求平均，降低随机噪声。
 * 3. 根据12位ADC、3.3V参考电压和1:2硬件分压还原电池电压。
 * 4. 使用万用表实测得到的1037/1000系数修正系统误差。
 *
 * @param voltage_mv 用于接收电池电压，单位mV
 * @retval 1U=读取成功，0U=参数错误或全部ADC采样失败
 */
static uint8_t Battery_ReadVoltageMv(uint16_t *voltage_mv)
{
  uint32_t adc_sum = 0U; // 成功样本的ADC码累加值
  uint32_t adc_average; // 成功样本的平均ADC码
  uint8_t sample_index; // 当前采样序号
  uint8_t successful_samples = 0U; // 成功采样次数
  uint32_t calculated_voltage_mv; // 电压换算中间值

  if(voltage_mv == NULL) return 0U;

  __HAL_RCC_ADC1_CLK_ENABLE(); // 电池采样开始前临时开启ADC1时钟

  for(sample_index = 0U; sample_index < BATTERY_SAMPLE_COUNT; sample_index++)
  {
    if(HAL_ADC_Start(&hadc1) != HAL_OK) continue;

    if(HAL_ADC_PollForConversion(&hadc1, 10U) == HAL_OK)
    {
      adc_sum += HAL_ADC_GetValue(&hadc1);
      successful_samples++;
    }

    HAL_ADC_Stop(&hadc1);
  }

  /* 防止除以0，也避免把全部采样失败误认为电池电压为0。 */
  if(successful_samples == 0U)
  {
    __HAL_RCC_ADC1_CLK_DISABLE(); // 采样失败后关闭ADC1时钟
    return 0U;
  }

  adc_average = adc_sum / successful_samples;

  /*
   * 12位ADC满量程为4095，PA1和电池电压换算关系为：
   * PA1_mV = adc_average × 3300 / 4095
   * BAT_mV = PA1_mV × 2
   *
   * 分子增加2047，利用整数运算实现接近四舍五入的效果。
   */
  calculated_voltage_mv = (adc_average * BATTERY_ADC_REFERENCE_MV * BATTERY_DIVIDER_SCALE + 2047UL) / 4095UL;

  /*
   * 根据万用表实测结果乘以约1.037进行校准。
   * 分子增加500，同样用于改善整数除法的舍入结果。
   */
  calculated_voltage_mv = (calculated_voltage_mv * BATTERY_CALIBRATION_NUMERATOR + 500UL) / BATTERY_CALIBRATION_DENOMINATOR;

  *voltage_mv = (uint16_t)calculated_voltage_mv;

  __HAL_RCC_ADC1_CLK_DISABLE(); // 采样结束后关闭ADC1时钟

  return 1U;
}

/**
 * @brief 根据单节锂电池电压粗略估算剩余电量
 *
 * 锂电池电压与容量不是线性关系，因此主要区间使用经验分段表。
 * 低电量区域使用线性计算，避免电压接近下限时百分比突然跳变。
 *
 * 该结果适合手表界面上的趋势提示，不等同于专业电量计芯片
 * 通过库仑计量得到的精确剩余容量。
 *
 * @param voltage_mv 经过平均、校准和低通滤波的电池电压，单位mV
 * @retval 估算电量百分比，范围0~100
 */
static uint8_t Battery_VoltageToPercent(uint16_t voltage_mv)
{
  /* 从高到低匹配经验电压区间。 */
  if(voltage_mv >= 4200U) return 100U;
  if(voltage_mv >= 4100U) return 90U;
  if(voltage_mv >= 4000U) return 80U;
  if(voltage_mv >= 3920U) return 70U;
  if(voltage_mv >= 3860U) return 60U;
  if(voltage_mv >= 3800U) return 50U;
  if(voltage_mv >= 3760U) return 40U;
  if(voltage_mv >= 3720U) return 30U;
  if(voltage_mv >= 3680U) return 20U;
  if(voltage_mv >= 3600U) return 10U;
  if(voltage_mv >= 3450U) return 5U;

  /*
   * 3300~3450mV进入这里，并使用3300~3600mV映射到0~10%的公式。
   * 这样可以让低电量区域逐步下降，而不是直接从5%跳到0%。
   */
  if(voltage_mv > 3300U)
  {
    return (uint8_t)((uint32_t)((voltage_mv - 3300U) * 10U) / 300U);
  }

  return 0U; // 低于或等于3300mV时视为电量耗尽
}

/**
 * @brief 将mdps格式的角速度显示为带三位小数的dps
 *
 * 例如-4175mdps显示为“-4.175”。先取得数值绝对值，再单独添加符号，
 * 可以避免C语言负数取余产生负小数部分。
 *
 * @param label 需要更新的LVGL标签
 * @param axis 坐标轴名称，例如"X"、"Y"或"Z"
 * @param value_mdps 角速度，单位mdps
*/
static void UI_SetGyroLabel(lv_obj_t *label, const char *axis, int32_t value_mdps)
{
  uint32_t magnitude;
  const char *sign;

  if(label == NULL || axis == NULL) return;

  if(value_mdps < 0)
  {
    sign = "-";
    magnitude = (uint32_t)(-(int64_t)value_mdps);
  }
  else
  {
    sign = "";
    magnitude = (uint32_t)value_mdps;
  }

  lv_label_set_text_fmt(label, "%s %s%lu.%03lu", axis, sign, (unsigned long)(magnitude / 1000U), (unsigned long)(magnitude % 1000U));
}

/**
 * @brief 将0.01℃格式的温度显示为带两位小数的摄氏温度
 * @param label 需要更新的LVGL标签
 * @param temperature_centi_c 温度，单位0.01℃
*/
static void UI_SetTemperatureLabel(lv_obj_t *label, int32_t temperature_centi_c)
{
  uint32_t magnitude;
  const char *sign;

  if(label == NULL) return;

  if(temperature_centi_c < 0)
  {
    sign = "-";
    magnitude = (uint32_t)(-(int64_t)temperature_centi_c);
  }
  else
  {
    sign = "";
    magnitude = (uint32_t)temperature_centi_c;
  }

  lv_label_set_text_fmt(label, "TEMP: %s%lu.%02lu C", sign, (unsigned long)(magnitude / 100U), (unsigned long)(magnitude % 100U));
}

/**
 * @brief 将0.01%RH格式的湿度显示为带两位小数的相对湿度
 * @param label 需要更新的LVGL标签
 * @param humidity_centi_rh 湿度，单位0.01%RH
 */
static void UI_SetHumidityLabel(lv_obj_t *label, int32_t humidity_centi_rh)
{
  uint32_t humidity;

  if(label == NULL) return;
  if(humidity_centi_rh < 0) humidity_centi_rh = 0;
  if(humidity_centi_rh > 10000) humidity_centi_rh = 10000;

  humidity = (uint32_t)humidity_centi_rh;
  lv_label_set_text_fmt(label, "HUM: %lu.%02lu %%RH", (unsigned long)(humidity / 100U), (unsigned long)(humidity % 100U));
}

/**
 * @brief 将Pa格式的气压显示为带两位小数的hPa
 * @param label 需要更新的LVGL标签
 * @param pressure_pa 气压，单位Pa
 */
static void UI_SetPressureLabel(lv_obj_t *label, int32_t pressure_pa)
{
  uint32_t pressure;

  if(label == NULL) return;
  if(pressure_pa < 0) pressure_pa = 0;

  pressure = (uint32_t)pressure_pa;
  lv_label_set_text_fmt(label, "PRESS: %lu.%02lu hPa", (unsigned long)(pressure / 100U), (unsigned long)(pressure % 100U));
}

/**
 * @brief 将厘米格式的海拔显示为带两位小数的米
 * @param label 需要更新的LVGL标签
 * @param altitude_cm 海拔，单位cm
 */
static void UI_SetAltitudeLabel(lv_obj_t *label, int32_t altitude_cm)
{
  uint32_t magnitude;
  const char *sign;

  if(label == NULL) return;

  if(altitude_cm < 0)
  {
    sign = "-";
    magnitude = (uint32_t)(-(int64_t)altitude_cm);
  }
  else
  {
    sign = "";
    magnitude = (uint32_t)altitude_cm;
  }

  lv_label_set_text_fmt(label, "ALT: %s%lu.%02lu m", sign, (unsigned long)(magnitude / 100U), (unsigned long)(magnitude % 100U));
}

/*
 * 将0°~359°方向角转换为八个常用方向。
 *
 * 每个方向覆盖45°，边界位于两个方向正中间：
 * N以0°为中心，因此同时覆盖接近360°和接近0°的区域。
 */
static const char *UI_GetCompassDirection(uint16_t heading_degrees)
{
  heading_degrees %= 360U;

  if(heading_degrees >= 338U || heading_degrees < 23U) return "N";
  if(heading_degrees < 68U) return "NE";
  if(heading_degrees < 113U) return "E";
  if(heading_degrees < 158U) return "SE";
  if(heading_degrees < 203U) return "S";
  if(heading_degrees < 248U) return "SW";
  if(heading_degrees < 293U) return "W";

  return "NW";
}

/**
 * @brief 按当前显示页面暂停或恢复页面专用传感器
 *
 * UiTask通过lv_screen_active()判断当前页面，只向SensorTask发送线程标志。
 * SensorTask仍然是背板软件I2C和传感器寄存器的唯一操作者。
 *
 * @param enabled 1U=恢复当前页面传感器，0U=暂停当前页面传感器
 */
static void APP_SetActivePageSensorsEnabled(uint8_t enabled)
{
  lv_obj_t *active_screen;
  uint32_t sensor_flag = 0U;

  if(SensorTaskHandle == NULL) return;

  active_screen = lv_screen_active();

  if((active_screen == ui_HomeScreen) && (enabled == 0U))
  {
    sensor_flag = SENSOR_FLAG_HEART_RATE_STOP;
  }
  else if(active_screen == ui_CompassScreen)
  {
    sensor_flag = (enabled != 0U) ? SENSOR_FLAG_COMPASS_START : SENSOR_FLAG_COMPASS_STOP;
  }
  else if(active_screen == ui_EnvScreen)
  {
    sensor_flag = (enabled != 0U) ? SENSOR_FLAG_ENV_START : SENSOR_FLAG_ENV_STOP;
  }

  if(sensor_flag != 0U) osThreadFlagsSet(SensorTaskHandle, sensor_flag);
}

/*
 * 识别主页爱心的局部2秒长按。
 *
 * PRESSED记录开始时刻；PRESSING持续检查时间；
 * RELEASED和PRESS_LOST负责取消未完成的长按。
 * 达到2秒后只向SensorTask发送一次启动标志，不直接访问EM7028。
 */
static void APP_HeartIconEvent(lv_event_t *e)
{
  lv_event_code_t event_code;

  if(e == NULL) return;

  event_code = lv_event_get_code(e);

  if(event_code == LV_EVENT_PRESSED)
  {
    heart_press_start_tick = lv_tick_get();
    heart_press_tracking = 1U;
  }
  else if(event_code == LV_EVENT_PRESSING)
  {
    if((heart_press_tracking != 0U) && (lv_tick_elaps(heart_press_start_tick) >= HEART_LONG_PRESS_TIME_MS))
    {
      heart_press_tracking = 0U;
      if(SensorTaskHandle != NULL) osThreadFlagsSet(SensorTaskHandle, SENSOR_FLAG_HEART_RATE_START);
    }
  }
  else if((event_code == LV_EVENT_RELEASED) || (event_code == LV_EVENT_PRESS_LOST))
  {
    heart_press_tracking = 0U;
  }
}

/*
 * 根据HomeScreen的生命周期控制EM7028测量状态。
 *
 * 本回调由LVGL在UiTask中执行，只向SensorTask设置线程标志，
 * 不直接访问EM7028。SensorTask收到标志后再执行I2C启停操作，
 * 从而继续保持背板传感器总线只有SensorTask访问。
 *
 * 控制链路：
 * HomeScreen加载或卸载
 *     -> APP_HomeScreenLifecycleEvent()
 *     -> SensorTask线程标志
 *     -> SensorTask
 *     -> EM7028_StartMeasurement()或EM7028_StopMeasurement()
 */
static void APP_HomeScreenLifecycleEvent(lv_event_t *e)
{
  lv_event_code_t event_code;

  if(e == NULL || SensorTaskHandle == NULL) return;

  event_code = lv_event_get_code(e);

  /*
   * 离开主页时必须取消可能正在进行的测量。
   * 加载主页时保持EM7028关闭，等待用户长按爱心。
   */
  if(event_code == LV_EVENT_SCREEN_UNLOADED)
  {
    osThreadFlagsSet(SensorTaskHandle, SENSOR_FLAG_HEART_RATE_STOP);
  }
}

/*
 * CompassScreen只负责发出指南针启停请求。
 *
 * LVGL回调在UiTask中执行，不直接访问LSM303或软件I2C；
 * 实际寄存器读写仍然由SensorTask完成。
 *
 * 数据流：
 * CompassScreen页面事件
 *     -> SensorTask线程标志
 *     -> SensorTask
 *     -> LSM303启动或休眠
 */
static void APP_CompassScreenLifecycleEvent(lv_event_t *e)
{
  lv_event_code_t event_code;

  if(e == NULL || SensorTaskHandle == NULL) return;

  event_code = lv_event_get_code(e);

  if(event_code == LV_EVENT_SCREEN_LOADED)
  {
    (void)osThreadFlagsSet(SensorTaskHandle, SENSOR_FLAG_COMPASS_START);
  }
  else if(event_code == LV_EVENT_SCREEN_UNLOADED)
  {
    (void)osThreadFlagsSet(SensorTaskHandle, SENSOR_FLAG_COMPASS_STOP);
  }
}

/*
 * EnvScreen通过线程标志控制AHT21和SPL06测量状态。
 *
 * 页面回调运行在UiTask中，只负责发送启停请求；AHT21和SPL06的
 * 实际I2C通信仍由SensorTask完成，避免多个任务同时访问传感器总线。
 *
 * 控制链路：
 * EnvScreen页面事件
 *     -> SensorTask线程标志
 *     -> SensorTask
 *     -> AHT21触发控制和SPL06工作模式控制
 */
static void APP_EnvScreenLifecycleEvent(lv_event_t *e)
{
  lv_event_code_t event_code;

  if(e == NULL || SensorTaskHandle == NULL) return;

  event_code = lv_event_get_code(e);

  if(event_code == LV_EVENT_SCREEN_LOADED)
  {
    (void)osThreadFlagsSet(SensorTaskHandle, SENSOR_FLAG_ENV_START);
  }
  else if(event_code == LV_EVENT_SCREEN_UNLOADED)
  {
    (void)osThreadFlagsSet(SensorTaskHandle, SENSOR_FLAG_ENV_STOP);
  }
}

/**
 * @brief USART1 DMA空闲接收事件回调
 *
 * DMA已经把本批数据搬入ble_rx_dma_buffer。本回调仍然遵循中断
 * 快进快出原则，只把有效字节复制进BLERXQueue，不解析协议、不发送回应。
 *
 * @param huart 产生接收事件的UART句柄
 * @param Size 本次DMA缓冲区中的有效字节数量
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint16_t index;

  /* 非USART1事件或模块已关闭时不处理缓冲区，也不重启蓝牙接收DMA。 */
  if(huart == NULL || huart->Instance != USART1) return;
  if(ble_module_enabled == 0U) return;

  ble_dma_callback_count++;
  ble_dma_last_size = Size;

  /* HAL报告的长度超出实际缓冲区时限制到数组大小，防止越界读取。 */
  if(Size > sizeof(ble_rx_dma_buffer))
  {
    Size = sizeof(ble_rx_dma_buffer);
  }

  /*
   * 逐字节非阻塞写入BLERXQueue。队列尚未创建或已满时记录丢字节数，
   * 其余字节仍继续尝试入队；回调内不组帧、不校验、不执行命令。
   */
  for(index = 0U; index < Size; index++)
  {
    if(BLERXQueueHandle == NULL)
    {
      ble_rx_drop_count++;
      continue;
    }

    if(osMessageQueuePut(BLERXQueueHandle, &ble_rx_dma_buffer[index], 0U, 0U) == osOK)
    {
      ble_rx_count++;
    }
    else
    {
      ble_rx_drop_count++;
    }
  }

  /* 本批字节入队后立即重启空闲接收DMA；成功时再次关闭半传中断。 */
  ble_dma_restart_status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, ble_rx_dma_buffer, sizeof(ble_rx_dma_buffer));

  if(ble_dma_restart_status == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
}

/**
 * @brief USART1接收错误回调
 *
 * DMA接收模式下，帧错误、噪声错误和溢出错误都会终止本轮接收。
 * 这里记录错误、清除错误标志，然后重新启动DMA接收。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* 只恢复当前已开启的USART1蓝牙接收；其他UART或关机期间的回调直接返回。 */
  if(huart == NULL || huart->Instance != USART1) return;
  if(ble_module_enabled == 0U) return;

  ble_uart_error_count++;
  ble_uart_last_error = huart->ErrorCode;

  /*
   * STM32F4通过依次读取SR和DR清除PE、FE、NE、ORE等接收错误标志。
   * __HAL_UART_CLEAR_FEFLAG()内部完成这个读取顺序。
   */
  __HAL_UART_CLEAR_FEFLAG(huart);

  ble_dma_restart_status = HAL_UARTEx_ReceiveToIdle_DMA(huart, ble_rx_dma_buffer, sizeof(ble_rx_dma_buffer));

  if(ble_dma_restart_status == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
}

/**
 * @brief 让STM32进入STOP，并在KEY1或RTC唤醒后恢复运行
 *
 * STOP期间：
 * 1. CPU和高速系统时钟停止；
 * 2. SRAM和任务变量保持；
 * 3. RTC继续由LSE运行；
 * 4. KEY1外部中断和RTC WakeUp定时器可以唤醒CPU。
 *
 * 本函数从ControlTask调用。每次有效唤醒后都会先恢复PLL、系统时钟、
 * SysTick、HAL时基和外部看门狗，再恢复任务调度并返回唤醒原因。
 * KEY1由ControlTask继续执行完整恢复；RTC则只触发一次SensorTask采样窗口，
 * 未识别到抬腕时ControlTask会再次调用本函数进入STOP。
 */
static SystemWakeReason_t System_EnterStopMode(void)
{
  uint32_t systick_control;
  SystemWakeReason_t wake_reason = SYSTEM_WAKE_NONE;

  rtc_wake_pending = 0U; // 丢弃进入STOP前遗留的RTC事件
  SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk; // 清除SysTick挂起标志，避免误触发
  __HAL_GPIO_EXTI_CLEAR_IT(KEY1_Pin); // 清除KEY1外部中断挂起标志

  /*
   * RTC后台处理期间可能已经按下KEY1。
   * 此时直接返回KEY1唤醒原因，不能再次进入STOP。
   */
  if(key1_wake_pending != 0U)
  {
    system_last_wake_reason = SYSTEM_WAKE_KEY1;
    return SYSTEM_WAKE_KEY1;
  }

  /*
   * 暂停任务调度，保证准备STOP和恢复时钟的过程不会切换到其他任务。
   * 中断仍然可以发生，因此KEY1的PA5外部中断能够唤醒CPU。
   */
  vTaskSuspendAll();

  #if EXTERNAL_WATCHDOG_ENABLED
    HAL_GPIO_WritePin(WDOG_EN_GPIO_Port, WDOG_EN_Pin, GPIO_PIN_SET); // 拉高低有效使能，STOP期间关闭外部看门狗
  #endif

  /* TIM2是HAL的1ms时基，进入STOP前暂停其更新中断，避免成为非预期唤醒源。 */
  HAL_SuspendTick();

  /* 保存并关闭FreeRTOS SysTick的计数器与中断，唤醒后按原值恢复。 */
  systick_control = SysTick->CTRL;
  SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);

  /* 清除遗留的SysTick和KEY1挂起状态，避免WFI把旧事件当成新的唤醒。 */
  SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
  __HAL_GPIO_EXTI_CLEAR_IT(KEY1_Pin);

  /*
   * FPDS只在STOP期间让Flash进入掉电状态。
   * 唤醒时硬件会自动恢复Flash，代价是增加少量唤醒等待时间。
   */
  HAL_PWREx_EnableFlashPowerDown();

  /*
   * WFI可能因未归类的中断提前返回，此时只累计诊断计数并再次进入STOP。
   * KEY1和RTC都是有效唤醒源：KEY1优先，RTC次之；识别后跳出循环，
   * 执行统一的时钟与任务恢复，再把原因交回ControlTask继续判断。
   */
  while(wake_reason == SYSTEM_WAKE_NONE)
  {
    __DSB();
    __ISB();
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

    if(key1_wake_pending != 0U)
    {
      rtc_wake_pending = 0U; // KEY1优先，丢弃可能同时到达的RTC事件
      wake_reason = SYSTEM_WAKE_KEY1;
    }
    else if(rtc_wake_pending != 0U)
    {
      rtc_wake_pending = 0U;
      rtc_stop_wake_count++;
      wake_reason = SYSTEM_WAKE_RTC;
    }
    else
    {
      stop_unexpected_wake_count++;
    }
  }
  /*
   * 程序执行到这里说明某个中断已经唤醒CPU。
   * STOP唤醒后先恢复PLL和总线时钟，再恢复两个系统节拍。
   */
  SystemClock_Config();

  SysTick->VAL = 0U; // 清除暂停前的剩余计数，让恢复后的节拍重新开始
  SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk; // 清除唤醒过程中可能遗留的SysTick挂起状态
  SysTick->CTRL = systick_control; // 恢复SysTick控制寄存器

  HAL_ResumeTick(); // 恢复HAL的1ms时基

  #if EXTERNAL_WATCHDOG_ENABLED
    HAL_GPIO_WritePin(WDOG_EN_GPIO_Port, WDOG_EN_Pin, GPIO_PIN_RESET); // 唤醒后重新启用外部看门狗
    HAL_GPIO_TogglePin(WDOG_WDI_GPIO_Port, WDOG_WDI_Pin); // 在任务恢复前立即产生一次喂狗边沿
  #endif

  xTaskResumeAll(); // 恢复任务调度

  system_last_wake_reason = wake_reason; // 记录本次唤醒原因，供ControlTask查询

  return wake_reason;
}

/**
 * @brief RTC WakeUp定时器中断回调
 *
 * 回调只记录唤醒事件，不恢复时钟、不读取传感器，也不操作LVGL。
 */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
  if(hrtc == NULL || hrtc->Instance != RTC) return;

  rtc_wake_pending = 1U;
  rtc_wakeup_irq_count++;
}

/**
 * @brief GPIO外部中断统一回调
 *
 * 中断中只记录KEY1事件，不操作PWM、LVGL或RTOS队列，
 * 避免在中断环境执行耗时或非中断安全的函数。
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == KEY1_Pin) key1_wake_pending = 1U;
}

/* USER CODE END Application */
