/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : LFC-WATCH Bootloader与YMODEM固件接收入口
  *
  * Flash布局：Bootloader占扇区0~1，固件有效记录占扇区2，APP占扇区3~7。
  * 启动时消费RTC备份域的升级请求，并检查有效记录；无留驻请求且记录有效
  * 才尝试进入APP，否则留在裸机主循环，通过USART1接收YMODEM固件。
  * 升级顺序：旧记录失效 -> 擦除APP -> 分包写入 -> 整文件读回CRC ->
  * 最后提交记录magic。最终ACK发送成功且串口安静10秒后尝试进入APP。
  * 本程序不启动FreeRTOS或LCD界面，低占空比背光闪烁用于提示留驻状态。
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
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* 保存在预留Flash中的固件有效记录，共5个32位字段。 */
typedef struct
{
  uint32_t magic; // 最后写入的完成标记，擦除态为0xFFFFFFFF
  uint32_t file_size; // 有效文件字节数，不含YMODEM包头和填充
  uint32_t file_crc; // 文件正文CRC16，保存在32位字段的低16位
  uint32_t file_size_inv; // file_size按位取反，用于检查记录一致性
  uint32_t file_crc_inv; // 整个32位file_crc字段的按位取反值
} BootAppRecord_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BOOT_BACKLIGHT_DIM 5U // 周期1000个计数，5对应0.5%占空比

/* Flash分区、固件提交标记与一次性升级请求。 */
#define APP_BASE_ADDR     0x0800C000U // APP向量表和文件正文起点
#define APP_RECORD_ADDR   0x08008000U // 扇区2中的固件有效记录
#define APP_RECORD_MAGIC  0x41505031U // Flash记录提交标记，不是升级请求
#define BOOT_REQUEST_MAGIC 0x4F544131U // RTC->BKP1R中的一次性APP升级请求
#define APP_FLASH_END     0x08080000U // APP分区末端的下一地址，不可写入

/* SRAM_END允许作为向下增长栈的初始栈顶，不是可读写数据地址。 */
#define SRAM_START        0x20000000U
#define SRAM_END          0x20020000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* 启动策略、主循环与串口诊断。裸机主循环负责以下接收状态。 */
static TIM_HandleTypeDef boot_backlight_tim; // 留驻后才初始化的TIM3背光PWM句柄

volatile uint32_t boot_loop_count = 0U; // 主循环运行计数
volatile uint8_t boot_stay_requested = 0U; // 本次启动是否要求留在BootLoader
volatile HAL_StatusTypeDef boot_uart_tx_status = HAL_OK; // 最近一次串口发送结果

uint8_t boot_rx_byte = 0U; // 当前接收到的字节
volatile uint32_t boot_rx_count = 0U; // 主循环收到的首字节/控制字节等数量，不含独立调用收到的包体
volatile HAL_StatusTypeDef boot_uart_rx_status = HAL_OK; // 最近一次串口接收结果

uint32_t boot_blink_tick = 0U; // 最近一次切换背光的时刻
uint32_t boot_status_tick = 0U; // 最近一次周期请求C的时刻

/* 整包缓存、校验与包号诊断；累计计数在取消会话时保留。 */
static uint8_t boot_packet_buffer[1029]; // 容纳最大的 1024 字节数据包及包头、CRC

volatile uint32_t boot_packet_length = 0U; // 当前完整包长：133或1029字节
volatile uint32_t boot_packet_ok_count = 0U; // 新文件信息包和新数据包的接受次数
volatile uint32_t boot_packet_bad_count = 0U; // 整包格式或CRC检查失败次数
volatile uint32_t boot_packet_incomplete_count = 0U; // 包体未完整收到的次数
volatile HAL_StatusTypeDef boot_packet_rx_status = HAL_OK; // 最近一次阻塞接收包体的结果

volatile uint8_t boot_expected_packet = 0U; // 当前期待的新包号
volatile uint8_t boot_has_last_packet = 0U; // 是否已经接受过一包
volatile uint32_t boot_packet_duplicate_count = 0U; // 上一包的重传次数
volatile uint32_t boot_packet_sequence_error_count = 0U; // 非预期包号次数

/* 本轮文件信息与有效正文进度，Boot_ResetReception()负责清空。 */
volatile uint8_t boot_file_info_received = 0U; // 是否已接受真实文件信息包
volatile uint8_t boot_data_started = 0U; // 是否已接受至少一个文件数据包
volatile uint32_t boot_file_info_error_count = 0U; // 文件信息解析失败次数

static char boot_file_name[64]; // 最多保存 63 个文件名字符，最后留给结束符
volatile uint32_t boot_file_size = 0U; // 解析出的文件长度，单位：字节
volatile uint8_t boot_file_info_result = 0U; // 1：解析成功，0：拒绝

volatile uint32_t boot_file_received_size = 0U; // 已接受的有效文件字节数
volatile uint32_t boot_last_data_size = 0U; // 最近一个新数据包的有效字节数
volatile uint16_t boot_file_crc = 0U; // 已接受的有效文件正文累计CRC，不含尾包填充

/* Flash结果与诊断；会话取消不清除这些历史结果。 */
volatile uint32_t boot_flash_sector_error = 0xFFFFFFFFU; // 擦除失败的扇区号
volatile HAL_StatusTypeDef boot_flash_status = HAL_OK; // 最近一次Flash操作结果
volatile uint32_t boot_flash_error_count = 0U; // 累计Flash操作或整文件读回失败次数
volatile uint16_t boot_flash_read_crc = 0U; // 从Flash读回整份文件后计算的CRC
volatile uint8_t boot_app_record_valid = 0U; // 记录和APP正文检查是否通过
volatile uint32_t boot_flash_write_with_error_addr = 0xFFFFFFFFU; // 编程或读回检查失败的地址
volatile uint32_t boot_packet_extra_count = 0U; // 文件收齐后仍发送新数据包的次数

/* 结束握手与自动交接；文件收齐不等于已经提交有效记录。 */
volatile uint8_t boot_eot_seen = 0U; // 收齐文件后，是否已收到第一次 EOT
volatile uint8_t boot_wait_end_packet = 0U; // EOT 握手完成，等待最终空文件名包

volatile uint8_t boot_transfer_complete = 0U; // 正文校验及有效记录提交通过，入口合理性仍由跳转函数检查
volatile uint8_t boot_auto_jump_pending = 0U; // 最终ACK已发出，等待自动进入APP

/* 包外取消序列与会话活动时间。 */
static uint8_t boot_can_pending = 0U; // 上一个包外字节是否为CAN
volatile uint32_t boot_cancel_count = 0U; // 累计取消次数

static uint32_t boot_last_activity_tick = 0U; // 最近一次串口活动的时间戳
volatile uint32_t boot_session_timeout_count = 0U; // 累计接收超时次数

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* 清空本轮协议状态，保留Flash内容与累计诊断计数。 */
static void Boot_ResetReception(void);
/* Flash链路失败时发出CAN CAN，并回到等待文件信息的状态。 */
static void Boot_AbortFlashTransfer(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * PB0切换为TIM3_CH3，由硬件输出背光PWM。
 * 当前Bootloader定时器时钟为16MHz，分频后1MHz，PWM频率1kHz。
 * 初始占空比为0，后续由主循环切换暗光和熄灭。
 */
static void Boot_BacklightInit(void)
{
  TIM_OC_InitTypeDef channel = {0};
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();

  /* 清除定时器旧状态，确保从已知配置开始 */
  __HAL_RCC_TIM3_FORCE_RESET();
  __HAL_RCC_TIM3_RELEASE_RESET();

  boot_backlight_tim.Instance = TIM3;
  boot_backlight_tim.Init.Prescaler = 15U; // 16MHz/16=1MHz
  boot_backlight_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
  boot_backlight_tim.Init.Period = 999U; // 1MHz/1000=1kHz
  boot_backlight_tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  boot_backlight_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if(HAL_TIM_PWM_Init(&boot_backlight_tim) != HAL_OK)
  {
    Error_Handler();
  }

  channel.OCMode = TIM_OCMODE_PWM1;
  channel.Pulse = 0U; // 初始占空比0
  channel.OCPolarity = TIM_OCPOLARITY_HIGH;
  channel.OCFastMode = TIM_OCFAST_DISABLE;

  if(HAL_TIM_PWM_ConfigChannel(&boot_backlight_tim, &channel, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  gpio.Pin = LCD_BLK_Pin;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(LCD_BLK_GPIO_Port, &gpio);

  if(HAL_TIM_PWM_Start(&boot_backlight_tim, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
 * 将 RAM 中的一段有效正文写入 APP 区，目标区域必须已擦除。
 * 采用逐字节编程，能够直接处理最后一包不足4字节的情况。
 * 每写一个字节就读回检查；中途失败可能已经写入部分数据。
 * 本函数不擦除Flash，不修改接收计数，也不发送ACK。
 */
static HAL_StatusTypeDef Boot_FlashWrite(uint32_t address, const uint8_t *data, uint32_t length)
{
  HAL_StatusTypeDef status;
  HAL_StatusTypeDef lock_status;
  uint32_t i;

  boot_flash_write_with_error_addr = 0xFFFFFFFFU;

  if(data == NULL || length == 0U)
  {
    return HAL_ERROR; // 数据指针为空或长度为0
  }

  /* 先确认起始地址合法，再用剩余容量检查长度。 */
  if((address < APP_BASE_ADDR) || (address >= APP_FLASH_END))
  {
    return HAL_ERROR; // 起始地址不在APP区
  }

  if(length > (APP_FLASH_END - address))
  {
    return HAL_ERROR; // 长度超出APP区剩余容量
  }

  status = HAL_FLASH_Unlock();
  if(status != HAL_OK)
  {
    return status;
  }

  for(i = 0U; i < length; i++)
  {
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address + i, data[i]);
    if(status != HAL_OK)
    {
      boot_flash_write_with_error_addr = address + i;
      break;
    }
    /* 从目标地址读回，检查实际保存的字节。 */
    if(*(volatile const uint8_t *)(address + i ) != data[i])
    {
      boot_flash_write_with_error_addr = address + i;
      status = HAL_ERROR;
      break;
    }
  }

  /* 失败时也经过这里，重新锁定Flash。 */
  lock_status = HAL_FLASH_Lock();

  if(status != HAL_OK)
  {
    return status;
  }

  return lock_status;

}

/*
 * 擦除整个 APP 区：扇区3～7，保留 Bootloader 和扇区2。
 * 调用后旧 APP 会被删除；只应在正式启动升级时执行一次。
 * VoltageRange 按 MCU 的3.3V供电选择，不是按电池电压选择。
 */
static HAL_StatusTypeDef Boot_FlashEraseApp(void)
{
  FLASH_EraseInitTypeDef erase = {0};
  HAL_StatusTypeDef status;
  HAL_StatusTypeDef lock_status;
  uint32_t sector_error = 0xFFFFFFFFU;

  boot_flash_sector_error = 0xFFFFFFFFU;

  status = HAL_FLASH_Unlock();
  if(status != HAL_OK)
  {
    return status;
  }

  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Sector = FLASH_SECTOR_3;
  erase.NbSectors = 5U; // 扇区3～7
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  status = HAL_FLASHEx_Erase(&erase, &sector_error);
  boot_flash_sector_error = sector_error;

  /* 擦除成功或失败，都重新锁定Flash控制寄存器 */
  lock_status = HAL_FLASH_Lock();

  if(status != HAL_OK)
  {
    return status;
  }

  return lock_status;
}

/*
 * 升级开始前，先擦除存放有效记录的扇区2。
 * 只有记录擦除成功，调用方才允许继续擦除APP。
 */
static HAL_StatusTypeDef Boot_EraseAppRecord(void)
{
  FLASH_EraseInitTypeDef erase = {0};
  HAL_StatusTypeDef status;
  HAL_StatusTypeDef lock_status;
  uint32_t sector_error = 0xFFFFFFFFU;
  uint32_t i;

  boot_app_record_valid = 0U;
  boot_flash_sector_error = 0xFFFFFFFFU;

  status = HAL_FLASH_Unlock();
  if(status != HAL_OK)
  {
    return status;
  }

  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Sector = FLASH_SECTOR_2;
  erase.NbSectors = 1U; // 扇区2
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  status = HAL_FLASHEx_Erase(&erase, &sector_error);
  boot_flash_sector_error = sector_error;

  if(status == HAL_OK)
  {
    /* 检查记录占用的20字节确实恢复成空白 */
    for(i = 0U; i < 5U; i++)
    {
      if(*(volatile const uint32_t *)(APP_RECORD_ADDR + i * 4U) != 0xFFFFFFFFU)
      {
        boot_flash_sector_error = FLASH_SECTOR_2;
        status = HAL_ERROR; // 记录擦除后读回仍非全1
        break;
      }
    }
  }

  lock_status = HAL_FLASH_Lock();

  if(status != HAL_OK)
  {
    return status;
  }

  return lock_status;
}

/*
 * 调用前必须已擦除记录，并完成APP正文写入和校验。
 * 先写4个信息字段，再最后写magic，作为本次提交的完成标记。
 * 不使用Boot_FlashWrite，因为该函数只允许写APP正文区。
 * 返回HAL_OK才表示本次提交和重新上锁均成功；失败由调用方取消会话。
 */
static HAL_StatusTypeDef Boot_WriteAppRecord(uint32_t file_size, uint16_t file_crc)
{
  HAL_StatusTypeDef status;
  HAL_StatusTypeDef lock_status;

  /* magic单独提交，这里只暂存其后的四个记录字段。 */
  uint32_t fields[4];
  uint32_t address;
  uint32_t i;

  boot_flash_write_with_error_addr = 0xFFFFFFFFU;

  if((file_size < 8U) || (file_size > (APP_FLASH_END - APP_BASE_ADDR)))
  {
    return HAL_ERROR; // 文件长度不合理
  }

  /* 必须在空白上提交，不能覆盖已有的记录 */
  for(i = 0U; i < 5U; i++)
  {
    address = APP_RECORD_ADDR + i * 4U;
    if(*(volatile const uint32_t *)address != 0xFFFFFFFFU)
    {
      boot_flash_write_with_error_addr = address;
      return HAL_ERROR; // 记录区域非空
    }
  }

  fields[0] = file_size;
  fields[1] = (uint32_t)file_crc;
  fields[2] = ~file_size;
  fields[3] = ~((uint32_t)file_crc);

  status = HAL_FLASH_Unlock();
  if(status != HAL_OK)
  {
    return status;
  }

  /* 跳过偏移0处的magic，先写偏移4~16处的信息字段 */
  for(i = 0U; i < 4U; i++)
  {
    address = APP_RECORD_ADDR + 4U + i * 4U;
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, fields[i]);

    if(status != HAL_OK)
    {
      boot_flash_write_with_error_addr = address;
      break;
    }

    if(*(volatile const uint32_t *)address != fields[i])
    {
      boot_flash_write_with_error_addr = address;
      status = HAL_ERROR;
      break;
    }
  }

  /*
   * 信息字段全部写好并读回正确后才提交magic。上面的break只离开for；
   * status失败时跳过提交，但仍执行后面的HAL_FLASH_Lock统一收尾。
   */
  if(status == HAL_OK)
  {
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, APP_RECORD_ADDR, APP_RECORD_MAGIC);

    if(status != HAL_OK)
    {
      boot_flash_write_with_error_addr = APP_RECORD_ADDR;
    }
    else if(*(volatile const uint32_t *)APP_RECORD_ADDR != APP_RECORD_MAGIC)
    {
      boot_flash_write_with_error_addr = APP_RECORD_ADDR;
      status = HAL_ERROR;
    }
  }

  lock_status = HAL_FLASH_Lock();

  if(status != HAL_OK)
  {
    return status;
  }

  return lock_status;

}

/*
 * 从传入的 crc 状态继续处理这一段数据。
 * 第一次传 0，后续传入上一次返回的结果，即可分段计算整份文件。
 */
static uint16_t Boot_CRC16Update(uint16_t crc, const uint8_t *data, uint32_t length)
{
  uint32_t i;
  uint8_t bit;

  for(i = 0U; i < length; i++)
  {
    crc ^= (uint16_t)((uint16_t)data[i] << 8);

    for(bit = 0U; bit < 8U; bit++)
    {
      if((crc & 0x8000U) != 0U)
      {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      }
      else
      {
        crc = (uint16_t)(crc << 1);
      }
    }
  }

  return crc;
}

/* CRC16/XMODEM初值为0；单包计算覆盖完整数据区，包括尾包填充。 */
static uint16_t Boot_CRC16(const uint8_t *data, uint32_t length)
{
  return Boot_CRC16Update(0U, data, length);
}

/*
 * 检查Flash中的固件记录，再检查记录所描述的APP正文。
 * 本函数只读Flash，不擦除、不写入，也不执行跳转。
 * 先检查长度边界，才能按该长度读取APP并计算CRC。
 */
static uint8_t Boot_CheckAppRecord(void)
{
  const volatile BootAppRecord_t *record = (const volatile BootAppRecord_t *)APP_RECORD_ADDR;
  uint32_t file_size;
  uint32_t file_crc;
  uint16_t actual_crc;

  /* magic未提交时直接拒绝，包括擦除后断电或升级中途取消的情况。 */
  if(record->magic != APP_RECORD_MAGIC)
  {
    return 0U; // 记录魔数不匹配
  }

  file_size = record->file_size;
  file_crc = record->file_crc;

  /* 反码检查记录字段是否一致；后面的正文CRC才检查Flash中的文件内容。 */
  if((file_size ^ record->file_size_inv) != 0xFFFFFFFFU)
  {
    return 0U; // 文件长度校验失败
  }

  if((file_crc ^ record->file_crc_inv) != 0xFFFFFFFFU)
  {
    return 0U; // 文件CRC校验失败
  }

  /* 至少包含初始MSP和复位入口，同时不能超出APP分区 */
  if((file_size < 8U) || (file_size > (APP_FLASH_END - APP_BASE_ADDR)))
  {
    return 0U; // 文件长度不合理
  }

  if(file_crc > 0xFFFFU)
  {
    return 0U; // CRC不可能大于16位
  }

  actual_crc = Boot_CRC16Update(0U, (const uint8_t *)APP_BASE_ADDR, file_size);

  if(actual_crc != (uint16_t)file_crc)
  {
    return 0U; // APP正文CRC不匹配
  }

  return 1U;
}

/*
 * 检查 RAM 中一包完整的 Ymodem 数据。
 * 返回 1：包格式和 CRC 通过；返回 0：检查失败。
 * 此处尚不判断是否为预期包号，也不解析文件信息或写入 Flash。
 */
static uint8_t Boot_CheckPacket(const uint8_t *packet, uint32_t packet_length)
{
  uint32_t data_length;
  uint16_t received_crc;
  uint16_t calculated_crc;

  if((packet == NULL) || (packet_length < 5U))
  {
    return 0U; // 包指针为空或长度不够
  }

  if(packet[0] == 0x01U)
  {
    data_length = 128U;

  }
  else if(packet[0] == 0x02U)
  {
    data_length = 1024U;
  }
  else
  {
    return 0U; // 包头不合法
  }

  /* 3字节包头+数据区+2字节CRC */
  if(packet_length != (data_length + 5U))
  {
    return 0U;
  }

  /* 序号与其按位取反值异或，结果应当为0xFF */
  if((uint8_t)(packet[1] ^ packet[2])!= 0xFFU)
  {
    return 0U;
  }

  /* YMODEM线上CRC高字节在前，按此顺序还原接收到的16位值。 */
  received_crc = (uint16_t)(((uint16_t)packet[data_length + 3U] << 8) | packet[data_length + 4U]);
  calculated_crc = Boot_CRC16(&packet[3], data_length);

  if(received_crc != calculated_crc)
  {
    return 0U; // CRC不匹配
  }

  return 1U; // 包检查通过
}

/*
 * 解析第 0 包的数据区：文件名\0十进制文件长度。
 * 调用前应由外层确认包号和 CRC；本函数只负责文件信息。
 * 拒绝空文件名、过长文件名、非法长度、零长度和超大固件。
 * 返回 1 表示成功，返回 0 时清空输出，避免误用上一次的结果。
 */
static uint8_t Boot_ParseFileInfo(const uint8_t *data, uint32_t length)
{
  /* 文件名边界与扫描位置。 */
  uint32_t name_length = 0U;
  uint32_t index;

  /* 将ASCII十进制长度转换为数值，并限制在APP分区容量内。 */
  uint32_t size = 0U;
  uint32_t digit;
  uint32_t digit_count = 0U;
  const uint32_t max_size = APP_FLASH_END - APP_BASE_ADDR;

  boot_file_name[0] = '\0';
  boot_file_size = 0U;

  if((data == NULL) || (length == 0U))
  {
    return 0U; // 数据指针为空或长度不够
  }

  /* 在数据区边界内寻找文件名结束符 */
  while((name_length < length) && (data[name_length] != '\0'))
  {
    name_length++;
  }

  if((name_length == 0U) || (name_length >= length) || (name_length >= sizeof(boot_file_name)))
  {
    return 0U; // 空文件名、过长文件名或未找到结束符
  }

  index = name_length + 1U; // 跳过文件名和结束符，指向文件长度字符串

  /* 文件长度后可接结束符，或空格分隔的可选信息 */
  while((index < length) && (data[index] != '\0') && (data[index] != ' '))
  {
    if((data[index] < '0') || (data[index] > '9'))
    {
      return 0U; // 非法字符
    }

    digit = (uint32_t)(data[index] - '0');

    /* 先判断再乘加，既限制固件大小，也避免整数溢出 */
    if(size > (max_size - digit) / 10U)
    {
      return 0U; // 超大固件
    }

    size = size * 10U + digit;
    digit_count++;
    index++;
  }

  if((index >= length) || (digit_count == 0U) || (size == 0U))
  {
    return 0U; // 非法长度或零长度
  }

  /* 所有检查通过后，才保存有效结果 */
  for(index = 0U; index < name_length; index++)
  {
    boot_file_name[index] = (char)data[index];
  }
  boot_file_name[name_length] = '\0';
  boot_file_size = size;

  return 1U; // 解析成功
}

/*
 * 处理包外的 EOT 控制字节。
 * 文件未收齐时拒绝结束；收齐后采用两次 EOT 握手。
 * 文件收齐时有效正文已写入Flash，整文件读回校验和记录提交仍要等最终空包。
 */
static void Boot_HandleEOT(void)
{
  uint8_t response;

  /* 已完成提交时再次收到EOT只补ACK，不能退回结束握手或重复写记录。 */
  if(boot_transfer_complete != 0U)
  {
    response = 0x06U;
    boot_uart_tx_status = HAL_UART_Transmit(&huart1, &response, 1U, 100U);
    return;
  }

  if((boot_file_info_received == 0U) || (boot_file_received_size != boot_file_size))
  {
    response = 0x15U; // NAK
    boot_uart_tx_status = HAL_UART_Transmit(&huart1, &response, 1U, 100U);
    return;
  }

  if(boot_eot_seen == 0U)
  {
    boot_eot_seen = 1U;
    response = 0x15U; // NAK，要求发送第二次 EOT
    boot_uart_tx_status = HAL_UART_Transmit(&huart1, &response, 1U, 100U);
    return;
  }

  /* 第二次EOT完成正文结束握手，仍需接收最终空文件名包。 */
  boot_wait_end_packet = 1U; // 等待最终空文件名包
  response = 0x06U; // ACK，确认结束
  boot_uart_tx_status = HAL_UART_Transmit(&huart1, &response, 1U, 100U);

  if(boot_uart_tx_status == HAL_OK)
  {
    response = 0x43U; // C，要求发送最终空文件名包
    boot_uart_tx_status = HAL_UART_Transmit(&huart1, &response, 1U, 100U);
  }
}

/*
 * 主循环收到SOH/STX后调用：阻塞接收余下包体，最多等待1000ms。
 * 先检查完整包的格式和CRC，再按结束空包、首次文件信息、新数据、上一包
 * 重传的顺序处理。结束包和文件信息包都使用包号0，必须先看会话状态。
 * 新文件信息通过后先擦记录再擦APP；新数据写入并逐字节读回成功后，才
 * 累加有效长度、更新文件CRC和期望包号并ACK。重复包只补应答，不重复写。
 * 最终空包触发整文件读回校验及记录提交；Flash链路失败发送CAN CAN，
 * 重置协议等待新会话。普通坏包NAK后保留当前进度，等待发送端重传。
 */
static void Boot_ReceivePacketBody(uint8_t first_byte)
{
  /* 当前包的接收长度与应答控制。 */
  uint16_t remaining_length; // 除去主循环已经收到的SOH/STX
  uint8_t response;
  uint8_t request_data = 0U; // 本次ACK后是否需要再发送C

  /* 数据区容量与本包实际需要保存的文件长度。 */
  uint32_t data_length; // 128或1024，包含最后一包的填充
  uint32_t remaining_size; // 文件还缺多少字节
  uint32_t valid_size; // min(data_length, remaining_size)，只保存这一段

  if(first_byte == 0x01U)
  {
    boot_packet_length = 133U; // 1+1+1+128+2
  }
  else if(first_byte == 0x02U)
  {
    boot_packet_length = 1029U; // 1+1+1+1024+2
  }
  else
  {
    return; // 非法包头，直接返回
  }

  boot_packet_buffer[0] = first_byte; // 保存包头
  remaining_length = (uint16_t)(boot_packet_length - 1U);

  boot_packet_rx_status = HAL_UART_Receive(&huart1, &boot_packet_buffer[1], remaining_length, 1000U);

  /* 包体未收齐时不解析缓存中的残留内容，也不推进文件进度。 */
  if(boot_packet_rx_status != HAL_OK)
  {
    boot_packet_incomplete_count++;

    response = 0x15U; // NAK
    boot_uart_tx_status = HAL_UART_Transmit(&huart1, &response, 1U, 100U);

    return;
  }

  if(Boot_CheckPacket(boot_packet_buffer, boot_packet_length) == 0U)
  {
    boot_packet_bad_count++;
    response = 0x15U; // NAK
  }
  else if((boot_wait_end_packet != 0U) || (boot_transfer_complete != 0U))
  {
    /*
     * 结束阶段优先处理包号0且文件名为空的包，不能误当作新文件开始。
     * 已提交时只补ACK，避免最终ACK丢失后重复写记录。
     */
    if((boot_packet_buffer[1] == 0U) && (boot_packet_buffer[3] == 0U))
    {
      if(boot_transfer_complete == 0U)
      {

        /* 只读取有效文件长度，不包含最后一包的填充 */
        boot_flash_read_crc = Boot_CRC16((const uint8_t *)APP_BASE_ADDR, boot_file_received_size);

        if(boot_flash_read_crc != boot_file_crc)
        {
          boot_flash_status = HAL_ERROR; // 读回CRC不匹配
          Boot_AbortFlashTransfer();
          return;
        }

        /* APP读回CRC已通过，现在提交有效记录 */
        boot_flash_status = Boot_WriteAppRecord(boot_file_received_size, boot_file_crc);

        if(boot_flash_status != HAL_OK)
        {
          Boot_AbortFlashTransfer();
          return;
        }

        /* 按启动时相同的规则，再检查一次记录与APP正文 */
        boot_app_record_valid = Boot_CheckAppRecord();

        if(boot_app_record_valid == 0U)
        {
          boot_flash_status = HAL_ERROR; // 记录或正文检查失败
          Boot_AbortFlashTransfer();
          return;
        }

        boot_transfer_complete = 1U;
        boot_wait_end_packet = 0U;
      }
      else
      {
        boot_packet_duplicate_count++;
      }

      response = 0x06U; // ACK
    }
    else
    {
      response = 0x15U; // NAK
    }
  }
  else if(boot_file_info_received == 0U)
  {
    /* 文件信息尚未接受时，只允许第0包进入解析 */
    if(boot_packet_buffer[1] != 0U)
    {
      boot_packet_sequence_error_count++;
      response = 0x15U; // NAK
    }
    else
    {
      boot_file_info_result = Boot_ParseFileInfo(&boot_packet_buffer[3], boot_packet_length - 5U);

      if(boot_file_info_result != 0U)
      {
        /* 仅接受新文件信息时擦除，重复第0包不会进入这里。 */
        boot_flash_read_crc = 0U;
        boot_flash_write_with_error_addr = 0xFFFFFFFFU;

        /* 先使旧记录失效，成功后才能破坏旧APP。 */
        boot_flash_status = Boot_EraseAppRecord();
        if(boot_flash_status != HAL_OK)
        {
          Boot_AbortFlashTransfer();
          return;
        }

        boot_flash_status = Boot_FlashEraseApp();

        if(boot_flash_status != HAL_OK)
        {
          Boot_AbortFlashTransfer();
          return;
        }

        boot_file_info_received = 1U;
        boot_transfer_complete = 0U; // 接收新文件信息时，清除上一轮完成的状态。
        boot_eot_seen = 0U; // 新文件开始，清除 EOT 标志
        boot_wait_end_packet = 0U; // 新文件开始，清除等待结束包标志
        boot_file_received_size = 0U;
        boot_last_data_size = 0U;
        boot_file_crc = 0U;
        boot_has_last_packet = 1U;
        boot_expected_packet = 1U; // 下一个包号应为1
        boot_packet_ok_count++;
        response = 0x06U; // ACK
        request_data = 1U; // ACK后再发送 C
      }
      else
      {
        boot_file_info_error_count++;
        response = 0x15U; // NAK
      }
    }
  }
  /* 已收到文件信息后按期望包号接收正文；8位包号回绕到0也仍是数据包。 */
  else if(boot_packet_buffer[1] == boot_expected_packet)
  {
    /* 声明长度已收满，接下来应等待EOT，不能把额外新包继续写到文件末尾。 */
    if(boot_file_received_size >= boot_file_size)
    {
      boot_packet_extra_count++;
      response = 0x15U;
    }
    else
    {
      data_length = boot_packet_length - 5U; // 数据区长度
      remaining_size = boot_file_size - boot_file_received_size;
      valid_size = data_length;

      if(valid_size > remaining_size)
      {
        valid_size = remaining_size; // 最后一包只取文件剩余的有效字节
      }

      /*
       * 使用更新前的累计长度定位写入地址。尾包超出valid_size的填充已经
       * 参与包CRC检查，但不会写入Flash，也不会计入文件长度或文件CRC。
       */
      boot_flash_status = Boot_FlashWrite(APP_BASE_ADDR + boot_file_received_size, &boot_packet_buffer[3], valid_size);

      if(boot_flash_status != HAL_OK)
      {
        Boot_AbortFlashTransfer();
        return;
      }

      /* 写入成功才提交本包进度；若ACK丢失，重传会进入重复包分支。 */
      boot_file_crc = Boot_CRC16Update(boot_file_crc, &boot_packet_buffer[3], valid_size);
      boot_last_data_size = valid_size;
      boot_file_received_size += valid_size;

      boot_packet_ok_count++;
      boot_data_started = 1U;
      boot_has_last_packet = 1U;
      boot_expected_packet = (uint8_t)(boot_expected_packet + 1U);
      response = 0x06U; // ACK
    }
  }
  else if((boot_has_last_packet != 0U) && (boot_packet_buffer[1] == (uint8_t)(boot_expected_packet - 1U)))
  {
    /* 上一包可能因ACK丢失而重传；只补ACK，不推进长度、CRC、包号或Flash地址。 */
    boot_packet_duplicate_count++;
    response = 0x06U; // ACK

    /* 第1个数据包尚未接受:第0包重传时，重新回复ACK和C */
    if(boot_data_started == 0U)
    {
      request_data = 1U; // ACK后再发送 C
    }
  }
  else
  {
    boot_packet_sequence_error_count++;
    response = 0x15U; // NAK
  }

  /* 业务处理成功后才ACK；数据包此时已写入，最终空包此时已提交记录。 */
  boot_uart_tx_status = HAL_UART_Transmit(&huart1, &response, 1U, 100U);

  if((boot_transfer_complete != 0U) && (response == 0x06U) && (boot_uart_tx_status == HAL_OK))
  {
    boot_auto_jump_pending = 1U; // ACK后等待自动进入APP
    boot_last_activity_tick = HAL_GetTick(); // 重新记录自动跳转等待的起点
  }

  if((boot_uart_tx_status == HAL_OK) && (request_data != 0U))
  {
    response = 0x43U; // C
    boot_uart_tx_status = HAL_UART_Transmit(&huart1, &response, 1U, 100U);
  }
}

/*
 * 放弃本轮接收，重新等待第 0 包。
 * 只复位接收状态，不复位单片机，也不操作 Flash。
 * 累计成功、错误和取消次数保留，便于观察调试历史。
 * 不恢复旧固件、不补写有效记录；启动检查仍以Flash中的真实记录为准。
 */
static void Boot_ResetReception(void)
{
  boot_file_info_received = 0U;
  boot_file_info_result = 0U;
  boot_file_name[0] = '\0';
  boot_file_size = 0U;

  boot_file_received_size = 0U;
  boot_last_data_size = 0U;
  boot_file_crc = 0U;
  boot_data_started = 0U;

  boot_expected_packet = 0U;
  boot_has_last_packet = 0U;
  boot_packet_length = 0U;

  boot_eot_seen = 0U;
  boot_wait_end_packet = 0U;
  boot_transfer_complete = 0U;
  boot_auto_jump_pending = 0U;

  boot_can_pending = 0U;
  boot_status_tick = HAL_GetTick(); // 约一秒后重新发送 C
}

/*
 * Flash操作失败后取消本轮传输。
 * 保留Flash错误诊断，清理协议状态，等待发送端重新开始。
 */
static void Boot_AbortFlashTransfer(void)
{
  uint8_t cancel_message[2] = {0x18U, 0x18U}; // 连续两个 CAN

  boot_flash_error_count++;
  boot_uart_tx_status = HAL_UART_Transmit(&huart1, cancel_message, sizeof(cancel_message), 100U);
  Boot_ResetReception(); // 复位本轮接收状态
}

/*
 * 文件信息已接受，但整轮传输尚未完成时，检查接收活动间隔。
 * 串口无活动达到15秒则发送CAN CAN并重置会话；这不是整文件传输限时。
 */
static void Boot_CheckReceiveTimeout(void)
{
  uint8_t cancel_message[2] = {0x18U, 0x18U}; // 连续两个 CAN

  if((boot_file_info_received == 0U) || (boot_transfer_complete != 0U))
  {
    return; // 未开始接收或已完成传输，不检查超时
  }

  if((uint32_t)(HAL_GetTick() - boot_last_activity_tick) < 15000U)
  {
    return;
  }

  boot_session_timeout_count++;

  boot_uart_tx_status = HAL_UART_Transmit(&huart1, cancel_message, sizeof(cancel_message), 100U);
  Boot_ResetReception(); // 复位本轮接收状态
}

/*
 * 等待文件信息时发送字符 C，请求发送端使用 CRC 方式传输。
 * 只发送一个字节 0x43，不附加换行。
 */
static void Boot_RequestFile(void)
{
  uint8_t request = 0x43U;

  boot_uart_tx_status = HAL_UART_Transmit(&huart1, &request, 1U, 100U);
}

/*
 * ARMCC 5 汇编函数：r0 接收 APP 栈顶，r1 接收 APP 复位入口。
 * 更换 MSP 后直接跳转，避免继续执行依赖 BootLoader 旧栈的 C 代码。
 */
__asm void Boot_EnterApp(uint32_t app_sp, uint32_t app_reset)
{
    MSR MSP, r0
    CPSIE I
    BX r1
}

/*
 * main()在线程模式下调用，调用方已确认Flash记录和正文CRC有效。
 * 正常启动和升级完成后的自动跳转共用此入口；向量值不合理时返回并留驻。
 * 停止已启用的PWM和UART，屏蔽并清理中断，保留供电GPIO，切换VTOR后
 * 由汇编设置APP初始MSP并进入Reset_Handler，继续SystemInit和C运行环境
 * 初始化。这里不是直接调用APP的main()，也不执行整机硬件复位。
 */
static void Boot_JumpToAPP(void)
{
  /* 向量表检查通过前不拆除Bootloader的运行环境。 */
  uint32_t app_sp = *(volatile uint32_t *)APP_BASE_ADDR; // APP 栈顶
  uint32_t app_reset = *(volatile uint32_t *)(APP_BASE_ADDR + 4U); // APP 复位入口
  uint32_t app_entry = app_reset & ~1U; // 清除 Thumb 位，得到实际入口地址

  /* 交接时清理NVIC，并把背光引脚恢复为普通输出。 */
  uint32_t i;
  GPIO_InitTypeDef gpio = {0};

  /* 栈顶必须位于SRAM的有效范围内，并满足8字节对齐要求 */
  if((app_sp <= SRAM_START) || (app_sp > SRAM_END) || (app_sp & 7U) != 0U)
  {
    return; // 栈顶不合理，返回
  }

  /* Cortex-M入口最低位必须为1表示Thumb状态，实际指令地址必须在APP分区内。 */
  if(((app_reset & 1U) == 0U) || (app_entry < APP_BASE_ADDR) || (app_entry >= APP_FLASH_END))
  {
    return; // 入口不合理，返回
  }

  /* 升级后跳转时PWM已启动；正常开机直接跳转时则未启动 */
  if(boot_backlight_tim.Instance == TIM3)
  {
    HAL_TIM_PWM_Stop(&boot_backlight_tim, TIM_CHANNEL_3);
    __HAL_RCC_TIM3_FORCE_RESET();
    __HAL_RCC_TIM3_RELEASE_RESET();
    __HAL_RCC_TIM3_CLK_DISABLE();
  }

  /* 先准备低电平，再把PB0从定时器复用切回普通输出 */
  HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, GPIO_PIN_RESET);

  gpio.Pin = LCD_BLK_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BLK_GPIO_Port, &gpio);

  /* 入口检查通过后，再释放BootLoader使用的串口 */
  HAL_UART_DeInit(&huart1);

  /* 清理期间先屏蔽普通中断，避免跳转被打断 */
  __disable_irq();

  /* 停止BootLoader的SysTick，清除其计数状态 */
  SysTick->CTRL = 0U;
  SysTick->LOAD = 0U;
  SysTick->VAL = 0U;

  /* STM32F411的外部中断编号覆盖三个32位寄存器组 */
  for(i = 0U; i < 3U; i++)
  {
    NVIC->ICER[i] = 0xFFFFFFFFU; // 屏蔽该组外部IRQ
    NVIC->ICPR[i] = 0xFFFFFFFFU; // 清除该组外部IRQ挂起标志
  }

  /* SysTick PendSV 属于内核异常，要另外清除挂起状态 */
  SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;

  /* 保持当前HSI时钟和供电GPIO，由APP接着初始化 */
  SCB->VTOR = APP_BASE_ADDR; // 设置中断向量表偏移地址
  __DSB(); // 确保VTOR写入完成
  __ISB(); // 让后续指令在更新后的系统配置下执行

  Boot_EnterApp(app_sp, app_reset); // 汇编函数，切换栈顶并跳转

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  
  /* USER CODE BEGIN 1 */

  /* APP软件交接会保留时钟，先按寄存器更新频率，再让HAL_Init配置正确时基。 */
  SystemCoreClockUpdate(); // 随后的SystemClock_Config再切换到Bootloader所用HSI

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* 启动时清空文件信息，等待真实的YMODEM文件信息包 */
  boot_file_name[0] = '\0';
  boot_file_size = 0U;
  boot_file_info_result = 0U;

  /*
   * BKP1R只传递一次性升级请求，与Flash里的APP有效记录无关。
   * 匹配后立即清零并置本次留驻标志；下次启动不会重复消费同一请求。
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();

  if(RTC->BKP1R == BOOT_REQUEST_MAGIC)
  {
    RTC->BKP1R = 0U;
    boot_stay_requested = 1U;
  }

  HAL_PWR_DisableBkUpAccess();

  boot_app_record_valid = Boot_CheckAppRecord(); // 检查Flash中的固件记录和正文

  HAL_Delay(500U);

  /* KEY1是手动留驻入口；二次采样消抖只会置位，不会覆盖已有的软件请求。 */
  if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
  {
    HAL_Delay(20U);

    if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
    {
      boot_stay_requested = 1U; // 按键按下，要求留在BootLoader
    }
  }

  /* 无软件/按键留驻请求且记录校验通过才尝试启动，否则进入接收主循环。 */
  if((boot_stay_requested == 0U) && (boot_app_record_valid != 0U))
  {
    Boot_JumpToAPP(); // 尝试跳转到APP
  }

  Boot_BacklightInit(); // 初始化背光闪烁

  /* 留驻后重启蓝牙模块，清除上一阶段连接状态；发送端需要重新连接串口。 */
  HAL_GPIO_WritePin(BLE_EN_GPIO_Port, BLE_EN_Pin, GPIO_PIN_RESET);
  HAL_Delay(1000U);

  HAL_GPIO_WritePin(BLE_EN_GPIO_Port, BLE_EN_Pin, GPIO_PIN_SET);
  HAL_Delay(1000U);

  boot_blink_tick = HAL_GetTick();
  boot_status_tick = boot_blink_tick;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    boot_loop_count++;

    /* 最多等待约1ms，尝试接收一个字节 */
    boot_uart_rx_status = HAL_UART_Receive(&huart1, &boot_rx_byte, 1U, 1U);

    if(boot_uart_rx_status == HAL_OK)
    {
      boot_rx_count++;

      boot_last_activity_tick = HAL_GetTick(); // 记录最近一次串口活动时间

      /* 只在包外识别连续两个CAN；包体中的0x18由整包接收函数当作数据处理。 */
      if(boot_rx_byte == 0x18U)
      {
        if(boot_can_pending != 0U)
        {
          boot_cancel_count++;
          Boot_ResetReception();
        }
        else
        {
          boot_can_pending = 1U; // 先记住第一次CAN
        }
      }
      else
      {
        boot_can_pending = 0U; // 中间出现其他字节，就不是连续两个CAN

        if((boot_rx_byte == 0x01U) || (boot_rx_byte == 0x02U))
        {
          Boot_ReceivePacketBody(boot_rx_byte);
          boot_last_activity_tick = HAL_GetTick(); // 记录最近一次串口活动时间
        }
        else if(boot_rx_byte == 0x04U)
        {
          Boot_HandleEOT();
        }
      }
    }

    /* 包外轮询期间每500ms切换背光；阻塞收包或擦写Flash时，切换可能延后。 */
    if((uint32_t)(HAL_GetTick() - boot_blink_tick) >= 500U)
    {
      boot_blink_tick = HAL_GetTick();
      if(__HAL_TIM_GET_COMPARE(&boot_backlight_tim, TIM_CHANNEL_3) == 0U)
      {
        __HAL_TIM_SET_COMPARE(&boot_backlight_tim, TIM_CHANNEL_3, BOOT_BACKLIGHT_DIM);
      }
      else
      {
        __HAL_TIM_SET_COMPARE(&boot_backlight_tim, TIM_CHANNEL_3, 0U);
      }
    }

    Boot_CheckReceiveTimeout(); // 检查接收超时，必要时取消本轮接收

    /* 最终ACK发出后，串口连续安静10秒，再尝试进入APP；新字节会推迟跳转。 */
    if((boot_auto_jump_pending != 0U) && (boot_transfer_complete != 0U) && (boot_app_record_valid != 0U))
    {
      if((uint32_t)(HAL_GetTick() - boot_last_activity_tick) >= 10000U)
      {
        boot_auto_jump_pending = 0U; // 避免重复跳转
        Boot_JumpToAPP();
      }
    }

    /* 未接受文件信息时每秒发送C；接受第0包后停止周期请求，取消后恢复。 */
    if((boot_file_info_received == 0U) && ((uint32_t)(HAL_GetTick() - boot_status_tick) >= 1000U))
    {
      boot_status_tick = HAL_GetTick();
      Boot_RequestFile();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
