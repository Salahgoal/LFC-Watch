#ifndef OV_TOUCH_H
#define OV_TOUCH_H

/*
 * 文件作用：
 * - 声明 CST816 触摸驱动对外提供的接口。
 * - FreeRTOS TouchTask和touch_event.c只通过这些函数读取触摸状态，
 *   不直接访问软件I2C时序和驱动内部变量。
 */

#include <stdint.h>

/*
 * CST816触摸模块状态。
 * 把同一次触摸采样的所有字段收在一个结构体中，避免在应用层散落
 * 多个可被任意修改的全局变量。结构体实体只存在于touch.c内部。
 */
typedef struct
{
  uint8_t pressed;       /* 1表示本轮有有效触摸，0表示松手或读取失败 */
  uint8_t finger_num;    /* CST816返回的触点数量，本项目只处理单指 */
  uint16_t x;            /* CST816寄存器给出的原始X坐标 */
  uint16_t y;            /* CST816寄存器给出的原始Y坐标 */
  uint16_t draw_x;       /* 按当前屏幕方向映射后的LCD X坐标 */
  uint16_t draw_y;       /* 按当前屏幕方向映射后的LCD Y坐标 */
  uint8_t ack_error;     /* 软件I2C应答错误：0正常，1通信失败 */
  uint8_t chip_id;       /* 初始化时读取的芯片ID，当前硬件实测为0xB5 */
} Touch_Info;

/* 初始化软件I2C和CST816，并读取芯片ID。 */
void Touch_Init(void);

/* 让CST816进入深度睡眠，返回1U表示命令写入成功。 */
uint8_t Touch_EnterSleep(void);

/* 通过RST硬件复位唤醒CST816并重新读取芯片ID。 */
void Touch_Wake(void);

/* 读取一次触摸寄存器并刷新驱动内部状态。 */
void Touch_Update(void);

/* 返回1U表示当前有有效触摸，0U表示没有。 */
uint8_t Touch_IsPressed(void);

/* 获取映射到LCD坐标系后的X坐标。 */
uint16_t Touch_GetX(void);

/* 获取映射到LCD坐标系后的Y坐标。 */
uint16_t Touch_GetY(void);

/* 获取CST816寄存器中的原始X坐标。 */
uint16_t Touch_GetRawX(void);

/* 获取CST816寄存器中的原始Y坐标。 */
uint16_t Touch_GetRawY(void);

/* 获取CST816报告的触点数量。 */
uint8_t Touch_GetFingerNum(void);

/* 获取初始化时读取到的CST816芯片ID。 */
uint8_t Touch_GetChipID(void);

/* 获取最近一次软件I2C通信的ACK错误状态。 */
uint8_t Touch_GetAckError(void);

/* 获取驱动内部完整触摸状态的只读指针。 */
const Touch_Info *Touch_GetInfo(void);

#endif
