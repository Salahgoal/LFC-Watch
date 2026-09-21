#ifndef __APP_UI_BRIDGE_H
#define __APP_UI_BRIDGE_H

#include <stdint.h>

/*
 * UI能够观察到的蓝牙状态。
 *
 * OFF表示模块未供电；
 * READY表示模块已开启，可以等待手机通信；
 * ACTIVE表示本次开机后已经收到过合法LFC协议帧。
 *
 * 当前背板没有将KT6368A连接状态脚引到STM32，
 * 因此ACTIVE只表示应用通信已验证，不等于实时物理连接状态。
 */
typedef enum
{
  APP_BLE_STATUS_OFF = 0U, // BLE模块关闭
  APP_BLE_STATUS_READY = 1U, // BLE模块已开启
  APP_BLE_STATUS_ACTIVE = 2U, // 已收到过合法LFC协议帧
} APP_BLEStatus_t;

/* 查询当前模块与LFC应用会话组合得到的UI状态。 */
APP_BLEStatus_t APP_BLEGetStatus(void);

/*
 * SquareLine界面层调用的应用接口。
 * UI只提交新的亮度百分比，不直接操作TIM3。
 */
void APP_BrightnessChanged(uint8_t percent);
void APP_BrightnessSaveRequested(uint8_t percent);

/* SquareLine蓝牙开关只提交请求，BLETask负责实际的上下电与DMA启停。 */
void APP_BLEPowerChanged(uint8_t enabled);

/* 查询蓝牙模块当前的软件目标电源状态。 */
uint8_t APP_BLEPowerIsEnabled(void);

#endif /* __APP_UI_BRIDGE_H */
