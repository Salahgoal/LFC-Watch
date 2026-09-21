#ifndef BLE_COMMAND_H
#define BLE_COMMAND_H

#include <stdint.h>

#include "rtc_service.h"

/*
 * BLE应用层支持的命令类型。
 *
 * 协议模块只负责取出payload，命令模块再根据payload内容
 * 判断手机要求手表执行什么业务。
 */
typedef enum
{
  BLE_COMMAND_UNKNOWN = 0U, // 未知命令
  BLE_COMMAND_PING = 1U, // 手机PING手表，手表回应PONG
  BLE_COMMAND_SET_TIME = 2U, // 手机设置手表时间
  BLE_COMMAND_GET_VERSION = 3U, // 手机查询手表软件版本
  BLE_COMMAND_SEND_DATA = 4U, // 手机请求一次手表数据快照
  BLE_COMMAND_ENV_STREAM_ON = 5U, // 开启AHT21蓝牙连续推送
  BLE_COMMAND_ENV_STREAM_OFF = 6U, // 关闭AHT21蓝牙连续推送
  BLE_COMMAND_ENTER_OTA = 7U, // LFC+OTA：由BLETask应答并请求ControlTask交接
} BLE_CommandType_t;

/* 先识别负载对应的业务命令，TIME命令再单独解析日期时间字段。 */
BLE_CommandType_t BLE_Command_GetType(const uint8_t *payload, uint16_t payload_length);

/* 将TIME负载转换为日期时间字段，日期合法性由RTC服务继续检查。 */
uint8_t BLE_Command_ParseTime(const uint8_t *payload, uint16_t payload_length, RTC_DateTime_t *date_time);

#endif /* BLE_COMMAND_H */
