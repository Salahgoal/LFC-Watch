#ifndef OV_LCD_H
#define OV_LCD_H

/*
 * ST7789 LCD驱动对外接口。
 *
 * 坐标约定：
 * - 原点(0, 0)位于当前旋转方向下的左上角。
 * - X向右增大，Y向下增大。
 * - 颜色统一使用RGB565，每个像素占2字节。
 *
 * FreeRTOS并发约定：
 * - 驱动内部会等待SPI DMA完成，但不会自动申请LcdMutex。
 * - 多任务同时访问LCD时，调用者必须在一组完整绘图操作外层加互斥锁。
 */

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* 显示方向：0、1为竖屏，2、3为横屏 */
#define LCD_ROTATION 0U

#if LCD_ROTATION == 0

#define LCD_WIDTH       240U
#define LCD_HEIGHT      280U
#define LCD_X_OFFSET    0U
#define LCD_Y_OFFSET    20U
#define LCD_MADCTL      0x00U

#elif LCD_ROTATION == 1

#define LCD_WIDTH       240U
#define LCD_HEIGHT      280U
#define LCD_X_OFFSET    0U
#define LCD_Y_OFFSET    20U
#define LCD_MADCTL      0xC0U

#elif LCD_ROTATION == 2

#define LCD_WIDTH       280U
#define LCD_HEIGHT      240U
#define LCD_X_OFFSET    20U
#define LCD_Y_OFFSET    0U
#define LCD_MADCTL      0x70U

#elif LCD_ROTATION == 3

#define LCD_WIDTH       280U
#define LCD_HEIGHT      240U
#define LCD_X_OFFSET    20U
#define LCD_Y_OFFSET    0U
#define LCD_MADCTL      0xA0U

#else

#error "LCD_ROTATION must be 0, 1, 2 or 3"

#endif

/* RGB565颜色 */
#define LCD_COLOR_BLACK    0x0000U
#define LCD_COLOR_WHITE    0xFFFFU
#define LCD_COLOR_RED      0xF800U
#define LCD_COLOR_GREEN    0x07E0U
#define LCD_COLOR_BLUE     0x001FU
#define LCD_COLOR_YELLOW   0xFFE0U
#define LCD_COLOR_CYAN     0x07FFU
#define LCD_COLOR_MAGENTA  0xF81FU

/* 复位并初始化ST7789控制器。 */
void LCD_Init(void);

/* 让ST7789进入低功耗睡眠模式。 */
void LCD_EnterSleep(void);

/* 让ST7789退出睡眠并恢复显示扫描。 */
void LCD_Wake(void);

/* 使用一种RGB565颜色填满整个逻辑屏幕。 */
void LCD_FillColor(uint16_t color);

/* 在指定坐标绘制一个像素，越界时不执行。 */
void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color);

/* 填充矩形，超出右边或下边时自动裁剪。 */
void LCD_FillRectangle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);

/* 绘制水平线，超出屏幕部分由矩形填充函数裁剪。 */
void LCD_DrawHorizontalLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color);

/* 绘制垂直线，超出屏幕部分由矩形填充函数裁剪。 */
void LCD_DrawVerticalLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color);

/* 绘制空心矩形边框。 */
void LCD_DrawRectangle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);

/* 使用Bresenham算法绘制任意方向直线。 */
void LCD_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);

/* 使用中点圆算法绘制空心圆。 */
void LCD_DrawCircle(int16_t center_x, int16_t center_y, int16_t radius, uint16_t color);

/* 绘制一个5x7点阵字符，可按整数倍缩放。 */
void LCD_DrawChar5x7(uint16_t x, uint16_t y, char character, uint16_t foreground, uint16_t background, uint8_t scale);

/* 绘制以'\0'结尾的5x7字符串，支持换行和自动折行。 */
void LCD_DrawString5x7(uint16_t x, uint16_t y, const char *text, uint16_t foreground, uint16_t background, uint8_t scale);

/* 处理SPI发送完成回调并释放LCD DMA信号量。 */
void LCD_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);

/* 绘制完整位于屏幕内的RGB565大端字节序图片。 */
void LCD_DrawImage(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *image);

#endif
