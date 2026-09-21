/*
 * lcd.c
 *
 * 文件作用：
 * 1. 通过SPI1驱动ST7789 LCD控制器。
 * 2. 提供像素、矩形、直线、圆、5x7文字和RGB565图片绘制接口。
 * 3. 较长数据使用SPI DMA发送，并通过FreeRTOS二值信号量等待完成。
 *
 * 并发约束：
 * 本文件负责“一次SPI传输何时完成”，但不负责“哪个任务可以访问LCD”。
 * 多任务绘图时，调用者仍需使用LcdMutex保护一组完整绘制操作。
 */

#include "lcd.h"
#include "main.h"
#include "spi.h"
#include "cmsis_os.h"

/*
 * 该信号量由CubeMX在freertos.c中创建，初始计数为0。
 * LCD任务启动DMA后等待它；DMA完成中断回调负责释放它。
 */
extern osSemaphoreId_t LcdDmaSemHandle;

/* 矩形填充缓冲区一次准备64个RGB565像素，即128字节。 */
#define LCD_FILL_BUFFER_PIXELS 64U

/* 数据少于32字节时，直接阻塞发送比配置DMA更简单。 */
#define LCD_DMA_THRESHOLD_BYTES 32U

/*
 * STM32 HAL的SPI DMA长度参数是uint16_t。
 * 65534既不超过上限，又保持RGB565的2字节像素边界。
 */
#define LCD_DMA_MAX_TRANSFER_BYTES 65534U

/* 一个5x7字符及其逐行位图。每行只使用低5位，bit4对应最左侧像素。 */
typedef struct
{
  char character;     /* 该字模对应的ASCII字符 */
  uint8_t bitmap[7];  /* 7行点阵数据，每行5个有效位 */
} FontGlyph5x7;

/* 当前学习项目使用的空格、连字符、冒号、数字和26个大写英文字母。 */
static const FontGlyph5x7 font_5x7[] =
{
  {' ', {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
  {'-', {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00}},
  {':', {0x00, 0x04, 0x04, 0x00, 0x04, 0x04, 0x00}},

  {'0', {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}},
  {'1', {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E}},
  {'2', {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}},
  {'3', {0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E}},
  {'4', {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}},
  {'5', {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E}},
  {'6', {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}},
  {'7', {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}},
  {'8', {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}},
  {'9', {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C}},

  {'A', {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}},
  {'B', {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E}},
  {'C', {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}},
  {'D', {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E}},
  {'E', {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}},
  {'F', {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10}},
  {'G', {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F}},
  {'H', {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}},
  {'I', {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E}},
  {'J', {0x07, 0x02, 0x02, 0x02, 0x12, 0x12, 0x0C}},
  {'K', {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11}},
  {'L', {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F}},
  {'M', {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11}},
  {'N', {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11}},
  {'O', {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}},
  {'P', {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10}},
  {'Q', {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D}},
  {'R', {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11}},
  {'S', {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E}},
  {'T', {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}},
  {'U', {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}},
  {'V', {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04}},
  {'W', {0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A}},
  {'X', {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11}},
  {'Y', {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04}},
  {'Z', {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F}},
};

static void LCD_Select(void);
static void LCD_Unselect(void);
static void LCD_HardwareReset(void);
static void LCD_WriteCommand(uint8_t command);
static void LCD_WriteData(const uint8_t *data, uint32_t size);
static void LCD_SetWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
static void LCD_DrawPixelSafe(int16_t x, int16_t y, uint16_t color);
static void LCD_DrawCirclePoints(int16_t center_x, int16_t center_y, int16_t x, int16_t y, uint16_t color);
static void LCD_DrawGlyph5x7(uint16_t x, uint16_t y, const uint8_t bitmap[7], uint16_t foreground, uint16_t background, uint8_t scale);
static const uint8_t *LCD_FindGlyph5x7(char character);
static void LCD_WaitDmaDone(void);

/**
 * @brief  选中LCD，使其接收后续SPI字节
 * @note   ST7789的CS低电平有效。
 */
static void LCD_Select(void)
{
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  释放LCD，结束当前SPI事务
 * @note   CS拉高后，LCD忽略SPI总线上的后续数据。
 */
static void LCD_Unselect(void)
{
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief  通过RST引脚对ST7789执行硬件复位
 * @note   时序为RST=1 -> RST=0 -> 等待 -> RST=1 -> 等待。
 */
static void LCD_HardwareReset(void)
{
  /* 正常状态先保持高电平。 */
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(10U);

  /* 拉低RST，触发硬件复位。 */
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(100U);

  /* 结束复位，并等待LCD内部电路稳定。 */
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(120U);
}

/**
 * @brief  向ST7789发送一个命令字节
 * @param  command ST7789命令码
 * @retval None
 *
 * @note   DC=0表示命令。CS由上层控制，使一条命令及其参数可以保持
 *         在同一个SPI事务中发送。
 */
static void LCD_WriteCommand(uint8_t command)
{
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
  (void)HAL_SPI_Transmit(&hspi1, &command, 1U, 100U);
}

/**
 * @brief  向ST7789发送一段参数或像素数据
 * @param  data 指向待发送字节的只读指针
 * @param  size 数据总字节数，可大于一次HAL DMA允许的65535字节
 * @retval None
 *
 * @note   DC=1表示数据。短数据使用阻塞发送；长数据分块使用DMA，
 *         每块完成后通过LcdDmaSem唤醒当前任务。分块可避免整屏
 *         RGB565图片的134400字节长度被uint16_t截断。
 */
static void LCD_WriteData(const uint8_t *data, uint32_t size)
{
  uint16_t chunk_size;  /* 本轮交给HAL发送的字节数，必须能装入uint16_t */

  if(data == NULL || size == 0U)
  {
    return;
  }

  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);

  while(size > 0U)
  {
    if(size > LCD_DMA_MAX_TRANSFER_BYTES)
    {
      chunk_size = (uint16_t)LCD_DMA_MAX_TRANSFER_BYTES;
    }
    else
    {
      chunk_size = (uint16_t)size;
    }

    if(chunk_size < LCD_DMA_THRESHOLD_BYTES)
    {
      /* 小数据直接等待SPI发送完成，避免DMA配置开销。 */
      (void)HAL_SPI_Transmit(&hspi1, (uint8_t *)data, chunk_size, HAL_MAX_DELAY);
    }
    else if(HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)data, chunk_size) == HAL_OK)
    {
      /*
       * DMA启动成功后等待完成信号量。
       * 等待期间当前任务进入Blocked，CPU可以运行其他就绪任务。
       */
      LCD_WaitDmaDone();
    }
    else
    {
      /* DMA未能启动时退回阻塞发送，保证当前绘图数据不会直接丢失。 */
      (void)HAL_SPI_Transmit(&hspi1, (uint8_t *)data, chunk_size, HAL_MAX_DELAY);
    }

    data += chunk_size;
    size -= chunk_size;
  }
}

/* 对ST7789执行本项目所需的最小初始化。 */
/* 初始化命令：0x01软件复位；0x11退出休眠；0x36设置扫描方向；0x3A设置RGB565；0x21开启颜色反转；0x13进入正常显示；0x29开启显示输出。 */
void LCD_Init(void)
{
  uint8_t data;  /* 发送给0x36或0x3A命令的单字节参数 */

  /* 第一步：通过RST引脚完成硬件复位。 */
  LCD_HardwareReset();

  /* 第二步：0x01软件复位，并等待控制器内部状态恢复。 */
  LCD_Select();
  LCD_WriteCommand(0x01U);
  LCD_Unselect();
  HAL_Delay(120U);

  /* 第三步：0x11退出休眠。 */
  LCD_Select();
  LCD_WriteCommand(0x11U);
  LCD_Unselect();
  HAL_Delay(120U);

  /* 第四步：0x36+LCD_MADCTL设置扫描方向。 */
  LCD_Select();
  LCD_WriteCommand(0x36U);
  data = LCD_MADCTL;
  LCD_WriteData(&data, 1U);
  LCD_Unselect();

  /* 第五步：0x3A的参数0x55表示每个像素使用16位RGB565。 */
  data = 0x55U;
  LCD_Select();
  LCD_WriteCommand(0x3AU);
  LCD_WriteData(&data, 1U);
  LCD_Unselect();

  /* 第六步：当前屏幕模组需要0x21颜色反转才能显示正确色彩。 */
  LCD_Select();
  LCD_WriteCommand(0x21U);
  LCD_Unselect();

  /* 第七步：0x13退出部分显示等特殊模式，进入正常显示。 */
  LCD_Select();
  LCD_WriteCommand(0x13U);
  LCD_Unselect();

  /* 第八步：0x29打开显示输出，并等待首帧稳定。 */
  LCD_Select();
  LCD_WriteCommand(0x29U);
  LCD_Unselect();
  HAL_Delay(20U);
}

/**
 * @brief 让ST7789进入低功耗睡眠模式
 *
 * 0x10会停止ST7789内部振荡器、面板扫描和部分电源电路，
 * 显存内容仍然保留，唤醒时不需要重新执行完整LCD_Init()。
 */
void LCD_EnterSleep(void)
{
  LCD_Select();
  LCD_WriteCommand(0x10U);
  LCD_Unselect();

  HAL_Delay(5U);
}

/**
 * @brief 让ST7789退出睡眠模式
 *
 * 0x11重新启动内部电源、振荡器和面板扫描。
 * 这里不执行硬件复位，因此方向、颜色格式和显存内容仍然保留。
 */
void LCD_Wake(void)
{
  LCD_Select();
  LCD_WriteCommand(0x11U);
  LCD_Unselect();

  HAL_Delay(120U); // 等待面板电源和扫描电路稳定
}

/**
 * @brief  设置下一次显存写入窗口
 * @param  x1 逻辑窗口左上角X坐标
 * @param  y1 逻辑窗口左上角Y坐标
 * @param  x2 逻辑窗口右下角X坐标，包含该像素
 * @param  y2 逻辑窗口右下角Y坐标，包含该像素
 * @retval None
 *
 * @note   本屏幕内部显存大于实际可见区域，因此要叠加X/Y偏移。
 *         0x2A设置列范围，0x2B设置行范围，0x2C开始写显存。
 */
static void LCD_SetWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  uint8_t data[4];  /* 起点和终点各占高、低两个字节 */

  /* 把应用层逻辑坐标转换成ST7789显存坐标。 */
  x1 += LCD_X_OFFSET;
  x2 += LCD_X_OFFSET;
  y1 += LCD_Y_OFFSET;
  y2 += LCD_Y_OFFSET;

  LCD_Select();

  /* 0x2A：发送X起点和X终点。ST7789要求高字节在前。 */
  LCD_WriteCommand(0x2AU);
  data[0] = (uint8_t)(x1 >> 8);
  data[1] = (uint8_t)(x1 & 0xFF);
  data[2] = (uint8_t)(x2 >> 8);
  data[3] = (uint8_t)(x2 & 0xFF);

  LCD_WriteData(data, 4U);

  /* 0x2B：发送Y起点和Y终点。 */
  LCD_WriteCommand(0x2BU);
  data[0] = (uint8_t)(y1 >> 8);
  data[1] = (uint8_t)(y1 & 0xFF);
  data[2] = (uint8_t)(y2 >> 8);
  data[3] = (uint8_t)(y2 & 0xFF);
  LCD_WriteData(data, 4U);

  /* 0x2C：告诉ST7789，后续数据均为该窗口内的像素流。 */
  LCD_WriteCommand(0x2CU);

  LCD_Unselect();
}

/**
 * @brief  使用指定RGB565颜色填满整个逻辑屏幕
 * @param  color RGB565颜色值
 * @retval None
 */
void LCD_FillColor(uint16_t color)
{
  LCD_FillRectangle(0U, 0U, LCD_WIDTH, LCD_HEIGHT, color);
}

/**
 * @brief  在指定坐标绘制一个RGB565像素
 * @param  x     X坐标
 * @param  y     Y坐标
 * @param  color RGB565颜色
 */
void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
  uint8_t pixel_data[2];  /* RGB565高字节和低字节 */

  /* 合法坐标范围为0～LCD_WIDTH-1和0～LCD_HEIGHT-1。 */
  if(x >= LCD_WIDTH || y >= LCD_HEIGHT)
  {
    return;
  }

  /* 把显存写入窗口缩小到目标像素。 */
  LCD_SetWindow(x, y, x, y);

  /* ST7789的RGB565像素流使用高字节在前的顺序。 */
  pixel_data[0] = (uint8_t)(color >> 8);
  pixel_data[1] = (uint8_t)(color & 0xFFU);

  LCD_Select();
  LCD_WriteData(pixel_data, 2U);
  LCD_Unselect();
}

/**
 * @brief  填充一个RGB565矩形
 * @param  x      左上角X坐标
 * @param  y      左上角Y坐标
 * @param  width  矩形宽度
 * @param  height 矩形高度
 * @param  color  RGB565颜色
 */
void LCD_FillRectangle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
  /*
   * 重复颜色缓冲区。使用static可避免每次调用都在任务栈上分配128字节。
   * 因为所有LCD调用由外层LcdMutex串行化，所以不会被两个任务同时改写。
   */
  static uint8_t fill_buffer[LCD_FILL_BUFFER_PIXELS * 2U];

  uint32_t total_pixels;    /* 矩形裁剪后还需要发送的像素总数 */
  uint32_t pixels_to_send;  /* 本轮从重复颜色缓冲区发送的像素数 */
  uint16_t index;           /* 初始化64个缓冲区像素时的循环下标 */

  /* 起点越界或矩形没有面积时，不访问LCD。 */
  if(x >= LCD_WIDTH || y >= LCD_HEIGHT || width == 0U || height == 0U)
  {
    return;
  }

  /*
   * 只裁掉超出右边和下边的部分。x、y本身已经经过起点检查，
   * 因此LCD_WIDTH-x和LCD_HEIGHT-y不会发生无符号下溢。
   */
  if(width > LCD_WIDTH - x)
  {
    width = LCD_WIDTH - x;
  }

  if(height > LCD_HEIGHT - y)
  {
    height = LCD_HEIGHT - y;
  }

  /* 预先生成64个相同颜色的RGB565像素，后面循环重复发送。 */
  for(index = 0; index < LCD_FILL_BUFFER_PIXELS; index++)
  {
    fill_buffer[2U * index] = (uint8_t)(color >> 8);
    fill_buffer[2U * index + 1U] = (uint8_t)(color & 0xFFU);
  }

  total_pixels = (uint32_t)width * (uint32_t)height;

  /* ST7789窗口终点坐标是包含关系，所以需要减1。 */
  LCD_SetWindow(x, y, x + width - 1U, y + height - 1U);

  LCD_Select();
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);

  while(total_pixels > 0U)
  {
    if(total_pixels > LCD_FILL_BUFFER_PIXELS)
    {
      pixels_to_send = LCD_FILL_BUFFER_PIXELS;
    }
    else
    {
      pixels_to_send = total_pixels;
    }

    /* 每个RGB565像素占2字节。 */
    LCD_WriteData(fill_buffer, pixels_to_send * 2U);
    total_pixels -= pixels_to_send;
  }

  LCD_Unselect();
}

/**
 * @brief  绘制指定长度的水平线
 * @param  x 起点X坐标
 * @param  y 起点Y坐标
 * @param  length 线段长度，单位为像素
 * @param  color RGB565颜色
 * @retval None
 */
void LCD_DrawHorizontalLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color)
{
  LCD_FillRectangle(x, y, length, 1U, color);
}

/**
 * @brief  绘制指定长度的垂直线
 * @param  x 起点X坐标
 * @param  y 起点Y坐标
 * @param  length 线段长度，单位为像素
 * @param  color RGB565颜色
 * @retval None
 */
void LCD_DrawVerticalLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color)
{
  LCD_FillRectangle(x, y, 1U, length, color);
}

/**
 * @brief  绘制一个空心矩形（只绘制四条边，不填充内部）
 * @param  x       矩形左上角的X坐标
 * @param  y       矩形左上角的Y坐标
 * @param  width   矩形宽度，单位：像素
 * @param  height  矩形高度，单位：像素
 * @param  color   矩形边框颜色，格式：RGB565
 */
void LCD_DrawRectangle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
  /* 宽度或高度为0时没有可绘制内容，直接退出 */
  if(width == 0U || height == 0U)
  {
    return;
  }

  LCD_DrawHorizontalLine(x, y, width, color);

  if(height > 1U)
  {
    LCD_DrawHorizontalLine(x, y + height - 1U, width, color);
  }

  if(height > 2U)
  {
    LCD_DrawVerticalLine(x, y + 1U, height - 2U, color);

    if(width > 1U)
    {
      LCD_DrawVerticalLine(x + width - 1U, y + 1U, height - 2U, color);
    }
  }
}

/**
 * @brief  使用Bresenham算法绘制任意直线
 * @param  x0 起点X坐标
 * @param  y0 起点Y坐标
 * @param  x1 终点X坐标
 * @param  y1 终点Y坐标
 * @param  color RGB565颜色
 * @retval None
 *
 * @note   Bresenham算法只使用整数加减和比较。error记录理想直线与
 *         当前像素路径的累计误差，每轮根据误差决定X、Y是否前进一步。
 */
void LCD_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
  int16_t dx;            /* 起点到终点的X方向绝对距离 */
  int16_t dy;            /* Y方向距离，后续取负以统一误差公式 */
  int16_t step_x;        /* X每次前进+1或-1 */
  int16_t step_y;        /* Y每次前进+1或-1 */
  int16_t error;         /* 当前累计误差 */
  int16_t error_double;  /* 2*error，避免使用浮点数 */

  dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
  dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);

  step_x = (x0 < x1) ? 1 : -1;
  step_y = (y0 < y1) ? 1 : -1;

  dy = -dy;
  error = dx + dy;

  while(1)
  {
    /* 当前点在屏幕范围内时才真正写入像素。 */
    if(x0 >= 0 && x0 < (int16_t)LCD_WIDTH && y0 >= 0 && y0 < (int16_t)LCD_HEIGHT)
    {
      LCD_DrawPixel((uint16_t)x0, (uint16_t)y0, color);
    }

    if(x0 == x1 && y0 == y1)
    {
      break;
    }

    error_double = 2 * error;

    if(error_double >= dy)
    {
      error += dy;
      x0 += step_x;
    }

    if(error_double <= dx)
    {
      error += dx;
      y0 += step_y;
    }
  }
}

/**
 * @brief  安全绘制像素点（包含越界检查）
 * @param  x 允许为负数的X坐标
 * @param  y 允许为负数的Y坐标
 * @param  color RGB565颜色
 * @retval None
 */
static void LCD_DrawPixelSafe(int16_t x, int16_t y, uint16_t color)
{
  if(x < 0 || x >= (int16_t)LCD_WIDTH || y < 0 || y >= (int16_t)LCD_HEIGHT)
  {
    return;
  }

  LCD_DrawPixel((uint16_t)x, (uint16_t)y, color);
}

/**
 * @brief  绘制圆八个方向的所有对称点
 * @param  center_x 圆心X坐标
 * @param  center_y 圆心Y坐标
 * @param  x 当前算法点相对圆心的X偏移
 * @param  y 当前算法点相对圆心的Y偏移
 * @param  color RGB565颜色
 * @retval None
 *
 * @note   圆关于X轴、Y轴和两条对角线对称，计算一个八分圆上的点，
 *         就能一次得到圆周上的8个对称像素。
 */
static void LCD_DrawCirclePoints(int16_t center_x, int16_t center_y, int16_t x, int16_t y, uint16_t color)
{
  LCD_DrawPixelSafe(center_x + x, center_y + y, color);
  LCD_DrawPixelSafe(center_x - x, center_y + y, color);
  LCD_DrawPixelSafe(center_x + x, center_y - y, color);
  LCD_DrawPixelSafe(center_x - x, center_y - y, color);

  LCD_DrawPixelSafe(center_x + y, center_y + x, color);
  LCD_DrawPixelSafe(center_x - y, center_y + x, color);
  LCD_DrawPixelSafe(center_x + y, center_y - x, color);
  LCD_DrawPixelSafe(center_x - y, center_y - x, color);
}

/**
 * @brief  使用中点圆算法绘制空心圆
 * @param  center_x 圆心X坐标
 * @param  center_y 圆心Y坐标
 * @param  radius 半径，负数无效
 * @param  color RGB565颜色
 * @retval None
 *
 * @note   decision决定下一个点沿X前进，还是同时让Y减1以贴近圆周。
 *         算法只计算一个八分圆，再调用LCD_DrawCirclePoints展开对称点。
 */
void LCD_DrawCircle(int16_t center_x, int16_t center_y, int16_t radius, uint16_t color)
{
  int16_t x;         /* 当前相对圆心的X偏移，从0开始增加 */
  int16_t y;         /* 当前相对圆心的Y偏移，从radius开始减小 */
  int16_t decision;  /* 中点判别量，决定下一步是否减少Y */

  if(radius < 0)
  {
    return;
  }

  x = 0;
  y = radius;
  decision = 1 - radius;

  while(x <= y)
  {
    LCD_DrawCirclePoints(center_x, center_y, x, y, color);
    x++;

    if(decision < 0)
    {
      decision += 2 * x + 1;
    }
    else
    {
      y--;
      decision += 2 * (x - y) + 1;
    }
  }
}

/**
 * @brief  绘制一个5x7点阵字符
 * @param  x 字符左上角X坐标
 * @param  y 字符左上角Y坐标
 * @param  bitmap 7行字模，每行低5位代表5个像素
 * @param  foreground 字模位为1时使用的前景色
 * @param  background 字模位为0时使用的背景色
 * @param  scale 像素放大倍数，0无效
 * @retval None
 */
static void LCD_DrawGlyph5x7(uint16_t x, uint16_t y, const uint8_t bitmap[7], uint16_t foreground, uint16_t background, uint8_t scale)
{
  uint8_t row;      /* 字模行下标，范围0～6 */
  uint8_t column;   /* 字模列下标，范围0～4 */
  uint16_t color;   /* 当前点阵位对应的前景色或背景色 */

  if(bitmap == NULL || scale == 0U)
  {
    return;
  }

  for(row = 0U; row < 7U; row++)
  {
    for(column = 0U; column < 5U; column++)
    {
      /*
       * bit4代表最左列，bit0代表最右列。
       * 每一个点阵像素用scale×scale的小矩形放大。
       */
      color = (bitmap[row] & (1U << (4U - column))) ? foreground : background;
      LCD_FillRectangle(x + column * scale, y + row * scale, scale, scale, color);
    }
  }
}

/**
 * @brief  根据字符查找对应的5x7字模
 * @param  character 待查找的ASCII字符
 * @retval 找到时返回7字节字模首地址，未收录时返回NULL
 */
static const uint8_t *LCD_FindGlyph5x7(char character)
{
  uint16_t index;        /* 遍历字库时的数组下标 */
  uint16_t glyph_count;  /* font_5x7中实际收录的字符数量 */

  glyph_count = sizeof(font_5x7) / sizeof(font_5x7[0]);

  for(index = 0U; index < glyph_count; index++)
  {
    if(font_5x7[index].character == character)
    {
      return font_5x7[index].bitmap;
    }
  }

  return NULL;
}

/**
 * @brief  显示一个5x7字符
 * @param  x 字符左上角X坐标
 * @param  y 字符左上角Y坐标
 * @param  character 待显示字符
 * @param  foreground 前景色
 * @param  background 背景色
 * @param  scale 放大倍数
 * @retval None
 */
void LCD_DrawChar5x7(uint16_t x, uint16_t y, char character, uint16_t foreground, uint16_t background, uint8_t scale)
{
  const uint8_t *bitmap;  /* 在font_5x7中找到的7行字模地址 */

  bitmap = LCD_FindGlyph5x7(character);

  if(bitmap == NULL)
  {
    return;
  }

  LCD_DrawGlyph5x7(x, y, bitmap, foreground, background, scale);
}

/**
 * @brief  显示一个5x7字符串
 * @param  x 首行文字起点X坐标
 * @param  y 首行文字起点Y坐标
 * @param  text 以'\0'结尾的字符串
 * @param  foreground 前景色
 * @param  background 背景色
 * @param  scale 放大倍数
 * @retval None
 *
 * @note   每个字符占5列点阵，再留1列间距；遇到'\n'或右边界时换行，
 *         下一行基线向下移动8*scale像素。
 */
void LCD_DrawString5x7(uint16_t x, uint16_t y, const char *text, uint16_t foreground, uint16_t background, uint8_t scale)
{
  uint16_t cursor_x;  /* 当前字符左上角X坐标 */
  uint16_t cursor_y;  /* 当前字符左上角Y坐标 */

  if(text == NULL || scale == 0U)
  {
    return;
  }

  cursor_x = x;
  cursor_y = y;

  while (*text != '\0')
  {
    if(*text == '\n')
    {
      cursor_x = x;
      cursor_y += 8U * scale;
      text++;
      continue;
    }

    if(cursor_x + 5U * scale > LCD_WIDTH)
    {
      cursor_x = x;
      cursor_y += 8U * scale;
    }

    if(cursor_y + 7U * scale > LCD_HEIGHT)
    {
      break;
    }

    LCD_DrawChar5x7(cursor_x, cursor_y, *text, foreground, background, scale);
    cursor_x += 6U * scale;
    text++;
  }
}

/**
 * @brief  等待当前SPI1 DMA传输完成
 * @retval None
 *
 * @note   LcdDmaSem初始计数为0。没有完成信号时，调用任务进入Blocked；
 *         DMA完成中断释放信号量后，任务重新进入Ready并继续执行。
 */
static void LCD_WaitDmaDone(void)
{
  (void)osSemaphoreAcquire(LcdDmaSemHandle, osWaitForever);
}

/**
 * @brief  LCD驱动层的SPI发送完成回调
 * @param  hspi 发生发送完成事件的SPI句柄
 * @retval None
 *
 * @note   只有SPI1完成才属于LCD。CMSIS-RTOS2允许在中断上下文释放信号量，
 *         但不能在这里执行会阻塞的LCD绘图操作。
 */
void LCD_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if(hspi == &hspi1)
  {
    /* DMA完成：让等待任务从Blocked恢复为Ready。 */
    if(LcdDmaSemHandle != NULL)
    {
      (void)osSemaphoreRelease(LcdDmaSemHandle);
    }
  }
}

/**
 * @brief  绘制一张RGB565图片
 * @param  x 图片左上角X坐标
 * @param  y 图片左上角Y坐标
 * @param  width 图片宽度，单位为像素
 * @param  height 图片高度，单位为像素
 * @param  image RGB565大端字节流：每个像素依次为高字节、低字节
 * @retval None
 *
 * @note   图片是不可裁剪的连续二维数据。若简单缩小width，下一行起点
 *         仍按原图宽度排列，图像会错位。因此本函数选择“完整可见才绘制”，
 *         与LCD_FillRectangle可以安全裁剪纯色区域的策略不同。
 */
void LCD_DrawImage(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *image)
{
  uint32_t total_pixels;  /* 图片像素总数 */
  uint32_t total_bytes;   /* RGB565字节总数，等于像素数×2 */

  if(image == NULL)
  {
    return;
  }

  if(x >= LCD_WIDTH || y >= LCD_HEIGHT || width == 0U || height == 0U)
  {
    return;
  }

  if(width > LCD_WIDTH - x || height > LCD_HEIGHT - y)
  {
    return;
  }

  total_pixels = (uint32_t)width * (uint32_t)height;
  total_bytes = total_pixels * 2U;

  LCD_SetWindow(x, y, x + width - 1U, y + height - 1U);
  LCD_Select();
  LCD_WriteData(image, total_bytes);
  LCD_Unselect();
}
