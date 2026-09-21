/*
 * touch.c
 *
 * 文件作用：
 * 1. 使用 GPIO 模拟 I2C 时序，与 CST816 触摸芯片通信。
 * 2. 读取 CST816 的芯片 ID、手指数量、原始 X/Y 坐标。
 * 3. 把原始触摸坐标映射成LCD坐标，供touch_event.c和TouchTask使用。
 */

#include "touch.h"
#include "main.h"
#include "lcd.h"
#include "soft_i2c.h"

/*
 * CST816 的 7 位 I2C 地址。
 *
 * 注意：
 * I2C 总线上真正发送地址时，需要把 7 位地址左移 1 位，
 * 最低位再拼接读写位：
 *
 * 写操作：(CST816_ADDR << 1) | 0U
 * 读操作：(CST816_ADDR << 1) | 1U
 */
#define CST816_ADDR             0x15U

/* CST816 常用寄存器地址 */
#define CST816_REG_CHIP_ID      0xA7U  /* 芯片ID寄存器 */
#define CST816_REG_FINGER_NUM   0x02U  /* 当前触点数量寄存器 */
#define CST816_REG_XPOS_H       0x03U  /* X坐标高4位所在寄存器 */
#define CST816_REG_XPOS_L       0x04U  /* X坐标低8位寄存器 */
#define CST816_REG_YPOS_H       0x05U  /* Y坐标高4位所在寄存器 */
#define CST816_REG_YPOS_L       0x06U  /* Y坐标低8位寄存器 */

#define CST816_REG_SLEEP_MODE 0xE5U // 触摸芯片睡眠控制寄存器
#define CST816_SLEEP_ENABLE 0x03U // 写入后进入深度睡眠

/*
 * 触摸模块内部状态变量。
 *
 * touch_info 只在 touch.c 内部直接修改。
 * 外部文件如果想读取触摸状态，需要通过 Touch_GetX()、
 * Touch_GetY()、Touch_IsPressed() 这些 getter 函数获取。
 */
static Touch_Info touch_info = {0};  /* 驱动内部唯一的触摸状态快照 */

/*
 * CST816使用的软件I2C总线。
 * 这里仅描述总线连接到了哪些GPIO，具体I2C时序由soft_i2c.c负责。
 */
static const SoftI2C_Bus_t touch_i2c_bus =
{
  TP_SDA_GPIO_Port, TP_SDA_Pin,  /* SDA 引脚 */
  TP_SCL_GPIO_Port, TP_SCL_Pin   /* SCL 引脚 */
};

/* 内部函数声明，只给 touch.c 自己使用 */
static void CST816_Reset(void);
static uint8_t CST816_ReadRegister(uint8_t reg);
static uint8_t CST816_WriteRegister(uint8_t reg, uint8_t data);
static void Touch_MapToLcd(void);

/**
 * @brief 初始化 CST816 触摸芯片
 */
void Touch_Init(void)
{
  /*
   * I2C 空闲状态：
   * SCL = 1
   * SDA = 1
   *
   * 因为 TP_SCL 和 TP_SDA 是开漏输出，
   * 写 GPIO_PIN_SET 可以理解为“释放总线”，让上拉电阻把线拉高。
   */
  SoftI2C_Init(&touch_i2c_bus);

  /* 硬件复位 CST816，让触摸芯片进入确定的初始状态 */
  CST816_Reset();

  /*
   * 读取芯片 ID，用来确认 I2C 通信是否正常。
   * CST816_REG_CHIP_ID 本质上就是 0xA7U。
   */
  touch_info.chip_id = CST816_ReadRegister(CST816_REG_CHIP_ID);
}

/**
 * @brief 让CST816进入深度睡眠
 * @return 1U=休眠命令写入成功，0U=软件I2C通信失败
 *
 * CST816睡眠后不会继续扫描触摸。由于当前硬件没有将TP_INT连接到MCU，
 * 后续只能先由KEY1唤醒STM32，再通过复位引脚唤醒CST816。
 */
uint8_t Touch_EnterSleep(void)
{
  touch_info.pressed = 0U;
  touch_info.finger_num = 0U;
  return CST816_WriteRegister(CST816_REG_SLEEP_MODE, CST816_SLEEP_ENABLE);
}

/**
 * @brief 通过硬件复位唤醒CST816
 *
 * 深度睡眠中的CST816不能依靠普通I2C读取自动恢复，因此拉低再释放RST。
 * 复位完成后重新读取芯片ID，用来确认触摸芯片已经恢复通信。
 */
void Touch_Wake(void)
{
  CST816_Reset();
  touch_info.pressed = 0U;
  touch_info.finger_num = 0U;
  touch_info.chip_id = CST816_ReadRegister(CST816_REG_CHIP_ID);
}

/**
 * @brief 更新一次触摸状态
 *
 * 运行流程：
 * 1. 先读取手指数量。
 * 2. 如果通信失败，认为没有有效触摸并退出。
 * 3. 如果没有手指触摸，也退出。
 * 4. 如果有触摸，再读取 X/Y 坐标寄存器。
 * 5. 拼接 X/Y 坐标。
 * 6. 映射到 LCD 坐标。
 * 7. 标记 pressed = 1U，表示当前有有效触摸。
 */
void Touch_Update(void)
{
  uint8_t finger_num;  /* 手指数量寄存器的读取结果 */
  uint8_t x_high;      /* X坐标高位寄存器，本项目只使用低4位 */
  uint8_t x_low;       /* X坐标低8位 */
  uint8_t y_high;      /* Y坐标高位寄存器，本项目只使用低4位 */
  uint8_t y_low;       /* Y坐标低8位 */

  /*
   * 第一步：先读取手指数量。
   *
   * 如果没有触摸，就没有必要继续读取坐标。
   * 这样可以避免使用旧坐标或者无意义坐标。
   */
  finger_num = CST816_ReadRegister(CST816_REG_FINGER_NUM);

  /*
   * 如果 ack_error != 0U，说明刚才 I2C 通信失败。
   *
   * 条件成立：
   *   pressed 设为 0，表示没有有效触摸；
   *   return 直接退出 Touch_Update()。
   *
   * 条件不成立：
   *   说明通信正常，继续往下执行。
   */
  if (touch_info.ack_error != 0U)
  {
    touch_info.pressed = 0U;
    return;
  }

  /* 通信正常时，把读到的手指数量保存到 touch_info */
  touch_info.finger_num = finger_num;

  /*
   * finger_num == 0U：
   *   表示当前没有检测到手指触摸。
   *
   * finger_num == 0xFFU：
   *   常见于无效数据、读取异常或芯片暂时没有有效坐标。
   *
   * 两种情况都认为当前没有有效触摸。
   */
  if (finger_num == 0U || finger_num == 0xFFU)
  {
    touch_info.pressed = 0U;
    return;
  }

  /*
   * 能执行到这里，说明：
   * 1. I2C 通信正常；
   * 2. finger_num 不是 0；
   * 3. 当前可能有有效触摸。
   *
   * 接下来开始读取 X/Y 坐标。
   */

  /* 读取 X 坐标高位 */
  x_high = CST816_ReadRegister(CST816_REG_XPOS_H);
  if (touch_info.ack_error != 0U)
  {
    touch_info.pressed = 0U;
    return;
  }

  /* 读取 X 坐标低位 */
  x_low = CST816_ReadRegister(CST816_REG_XPOS_L);
  if (touch_info.ack_error != 0U)
  {
    touch_info.pressed = 0U;
    return;
  }

  /* 读取 Y 坐标高位 */
  y_high = CST816_ReadRegister(CST816_REG_YPOS_H);
  if (touch_info.ack_error != 0U)
  {
    touch_info.pressed = 0U;
    return;
  }

  /* 读取 Y 坐标低位 */
  y_low = CST816_ReadRegister(CST816_REG_YPOS_L);
  if (touch_info.ack_error != 0U)
  {
    touch_info.pressed = 0U;
    return;
  }

  /*
   * 拼接坐标。
   *
   * CST816 的坐标通常是 12 位：
   *
   * 高位寄存器低 4 位 + 低位寄存器 8 位
   *
   * x_high & 0x0F：
   *   只保留 x_high 的低 4 位。
   *
   * << 8：
   *   把这 4 位移动到坐标的高位位置。
   *
   * | x_low：
   *   把低 8 位拼接进来。
   *
   * 例子：
   *   x_high 低 4 位 = 0x01
   *   x_low = 0x23
   *
   *   ((0x01) << 8) | 0x23 = 0x0123
   */
  touch_info.x = ((uint16_t)(x_high & 0x0FU) << 8) | x_low;
  touch_info.y = ((uint16_t)(y_high & 0x0FU) << 8) | y_low;

  /* 把 CST816 原始坐标转换成 LCD 坐标 */
  Touch_MapToLcd();

  /* 走到这里，说明本次触摸数据有效 */
  touch_info.pressed = 1U;
}

/**
 * @brief 获取当前是否有有效触摸
 * @retval 1U 有触摸，0U 没有触摸
 */
uint8_t Touch_IsPressed(void)
{
  return touch_info.pressed;
}

/**
 * @brief 获取映射到 LCD 后的 X 坐标
 */
uint16_t Touch_GetX(void)
{
  return touch_info.draw_x;
}

/**
 * @brief 获取映射到 LCD 后的 Y 坐标
 */
uint16_t Touch_GetY(void)
{
  return touch_info.draw_y;
}

/**
 * @brief 获取 CST816 原始 X 坐标
 */
uint16_t Touch_GetRawX(void)
{
  return touch_info.x;
}

/**
 * @brief 获取 CST816 原始 Y 坐标
 */
uint16_t Touch_GetRawY(void)
{
  return touch_info.y;
}

/**
 * @brief 获取 CST816 返回的手指数量
 */
uint8_t Touch_GetFingerNum(void)
{
  return touch_info.finger_num;
}

/**
 * @brief 获取 CST816 芯片 ID
 */
uint8_t Touch_GetChipID(void)
{
  return touch_info.chip_id;
}

/**
 * @brief 获取最近一次 I2C ACK 错误状态
 * @retval 0U 正常，1U 通信失败
 */
uint8_t Touch_GetAckError(void)
{
  return touch_info.ack_error;
}

/**
 * @brief 获取完整触摸信息结构体指针
 *
 * 返回 const 指针，表示外部可以读取 touch_info，
 * 但不应该直接修改 touch_info。
 */
const Touch_Info *Touch_GetInfo(void)
{
  return &touch_info;
}

/**
 * @brief 硬件复位 CST816
 */
static void CST816_Reset(void)
{
  /* 拉低复位引脚，让 CST816 进入复位状态 */
  HAL_GPIO_WritePin(TP_RST_GPIO_Port, TP_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);

  /* 拉高复位引脚，让 CST816 重新启动 */
  HAL_GPIO_WritePin(TP_RST_GPIO_Port, TP_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
}

/**
 * @brief 读取 CST816 的一个寄存器
 * @param reg 要读取的寄存器地址
 * @retval 读取成功返回寄存器值，失败返回 0xFFU
 */
static uint8_t CST816_ReadRegister(uint8_t reg)
{
  uint8_t data;

  touch_info.ack_error = 0U;

  /*
   * 第一阶段：使用“写方向”访问 CST816。
   * 此时并不是向寄存器写入数据，而是告诉 CST816：
   * 接下来需要读取的是哪一个寄存器。
   */
  SoftI2C_Start(&touch_i2c_bus);
  SoftI2C_WriteByte(&touch_i2c_bus, (CST816_ADDR << 1U) | 0U);

  if(SoftI2C_WaitAck(&touch_i2c_bus) != 0U)
  {
    touch_info.ack_error = 1U;
    SoftI2C_Stop(&touch_i2c_bus);
    return 0xFFU;
  }

  SoftI2C_WriteByte(&touch_i2c_bus, reg);

  /* 寄存器地址也是一个完整字节，发送后同样必须等待 ACK。 */
  if(SoftI2C_WaitAck(&touch_i2c_bus) != 0U)
  {
    touch_info.ack_error = 1U;
    SoftI2C_Stop(&touch_i2c_bus);
    return 0xFFU;
  }

  /*
   * 第二阶段：发送重复起始信号，不释放总线，
   * 将本次访问从“指定寄存器”切换到“读取寄存器数据”。
   */
  SoftI2C_Start(&touch_i2c_bus);
  SoftI2C_WriteByte(&touch_i2c_bus, (CST816_ADDR << 1U) | 1U);

  if(SoftI2C_WaitAck(&touch_i2c_bus) != 0U)
  {
    touch_info.ack_error = 1U;
    SoftI2C_Stop(&touch_i2c_bus);
    return 0xFFU;
  }

  data = SoftI2C_ReadByte(&touch_i2c_bus);

  /* 只读取一个字节，因此主机发送 NACK，表示不再继续读取。 */
  SoftI2C_SendNack(&touch_i2c_bus);
  SoftI2C_Stop(&touch_i2c_bus);

  return data;
}

/**
 * @brief 写入CST816的一个寄存器
 * @param reg 目标寄存器地址
 * @param data 准备写入的数据
 * @retval 1U写入成功，0U通信失败
 */
static uint8_t CST816_WriteRegister(uint8_t reg, uint8_t data)
{
  touch_info.ack_error = 0U;

  SoftI2C_Start(&touch_i2c_bus);
  SoftI2C_WriteByte(&touch_i2c_bus, (CST816_ADDR << 1U) | 0U);

  if(SoftI2C_WaitAck(&touch_i2c_bus) != 0U)
  {
    touch_info.ack_error = 1U;
    SoftI2C_Stop(&touch_i2c_bus);
    return 0U;
  }

  SoftI2C_WriteByte(&touch_i2c_bus, reg);

  if(SoftI2C_WaitAck(&touch_i2c_bus) != 0U)
  {
    touch_info.ack_error = 1U;
    SoftI2C_Stop(&touch_i2c_bus);
    return 0U;
  }

  SoftI2C_WriteByte(&touch_i2c_bus, data);

  if(SoftI2C_WaitAck(&touch_i2c_bus) != 0U)
  {
    touch_info.ack_error = 1U;
    SoftI2C_Stop(&touch_i2c_bus);
    return 0U;
  }

  SoftI2C_Stop(&touch_i2c_bus);
  return 1U;
}

/**
 * @brief 把 CST816 原始坐标映射到 LCD 坐标
 */
static void Touch_MapToLcd(void)
{
  /*
   * 当前实测触摸方向和 LCD 显示方向一致，
   * 所以先直接映射。
   */
  touch_info.draw_x = touch_info.x;
  touch_info.draw_y = touch_info.y;

  /*
   * X 边界保护。
   * LCD_WIDTH 是屏幕宽度，合法 X 范围是 0 ~ LCD_WIDTH - 1。
   */
  if (touch_info.draw_x >= LCD_WIDTH)
  {
    touch_info.draw_x = LCD_WIDTH - 1U;
  }

  /*
   * Y 边界保护。
   * LCD_HEIGHT 是屏幕高度，合法 Y 范围是 0 ~ LCD_HEIGHT - 1。
   */
  if (touch_info.draw_y >= LCD_HEIGHT)
  {
    touch_info.draw_y = LCD_HEIGHT - 1U;
  }
}
