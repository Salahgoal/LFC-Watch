#ifndef LV_PORT_INDEV_H
#define LV_PORT_INDEV_H

#include "lvgl.h"
#include <stdint.h>

/*创建并注册LVGL触摸输入设备*/
void LVGL_PortInput_Init(void);

/* 设置输入层是否处于“触摸只唤醒”模式 */
void LVGL_PortInput_SetWakeOnly(uint8_t enable);

/* 读取并清除一次唤醒请求 */
uint8_t LVGL_PortInput_TakeWakeRequest(void);

#endif
