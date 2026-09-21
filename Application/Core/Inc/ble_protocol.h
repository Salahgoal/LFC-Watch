#ifndef BLE_PROTOCOL_H
#define BLE_PROTOCOL_H

#include <stdint.h>

/* LFC协议层只处理字节数组，不访问UART、FreeRTOS或业务模块。 */
#define BLE_MAX_PAYLOAD_LENGTH 48U // 单帧允许的最大负载字节数
#define BLE_MAX_ENCODED_LENGTH (BLE_MAX_PAYLOAD_LENGTH + 9U) // 负载加固定帧字段和CRLF

/*
 * 蓝牙应用协议帧
 *
 * payload_length表示payload中实际有效的字节数。
 * payload额外保留一个字节，用来保存C字符串结束符'\0'。
 */
typedef struct
{
  uint16_t payload_length; // payload中的实际有效字节数
  uint8_t payload[BLE_MAX_PAYLOAD_LENGTH + 1U]; // 负载内容及末尾'\0'
} BLE_Frame_t;

/* 解析不含CRLF的LFC帧，成功返回1，失败返回0。 */
uint8_t BLE_Protocol_Parse(const uint8_t *data, uint16_t length, BLE_Frame_t *frame);

/* 构建含CRLF的发送帧，返回完整字节数；失败返回0。 */
uint16_t BLE_Protocol_Build(const uint8_t *payload, uint16_t payload_length, uint8_t *output, uint16_t output_capacity);

#endif /* BLE_PROTOCOL_H */
