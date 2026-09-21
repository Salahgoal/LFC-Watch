#include "ble_protocol.h"

#include <string.h>

/*
 * LFC帧格式：@LL:PAYLOAD*CC\r\n
 * LL是两位十六进制负载长度，CC是负载所有字节按位异或的校验和。
 * 本文件只在内存中解析和构建帧，不关心数据来自UART还是其他传输方式。
 * APP中的LFC+OTA也使用此格式；进入Bootloader后的YMODEM不经过本模块。
 */

/**
 * @brief 将ASCII十六进制字符转换成数值
 *
 * @param character 待转换字符，支持0~9、A~F和a~f
 * @param value 用于接收转换后的0~15
 * @retval 1U=转换成功，0U=字符非法
 */
static uint8_t BLE_Protocol_HexToNibble(uint8_t character, uint8_t *value)
{
  /* 输出指针无效时无法写回转换结果，直接返回失败。 */
  if(value == NULL) return 0U;

  if(character >= '0' && character <= '9')
  {
    *value = character - '0';
    return 1U;
  }

  if(character >= 'A' && character <= 'F')
  {
    *value = character - 'A' + 10U;
    return 1U;
  }

  if(character >= 'a' && character <= 'f')
  {
    *value = character - 'a' + 10U;
    return 1U;
  }

  return 0U;
}

/**
 * @brief 将数值的低4位转换为大写ASCII十六进制字符
 * @param value 待转换数值，函数内部只保留低4位
 * @retval '0'~'9'或'A'~'F'
 */
static uint8_t BLE_Protocol_NibbleToHex(uint8_t value)
{
  value &= 0x0FU; // 只保留低4位，将输入限制到0~15

  return value < 10U ? value + '0' : value - 10U + 'A';
}

/**
 * @brief 解析一条不包含CRLF的蓝牙协议帧
 *
 * 帧格式为“@LL:PAYLOAD*CC”。LL是两个十六进制长度字符，
 * CC是负载所有字节按位异或得到的校验和。
 *
 * @param data 待解析的数据
 * @param length 数据长度，不包括CRLF
 * @param frame 用于接收解析结果
 * @retval 1U=帧合法，0U=格式、长度或校验错误
 */
uint8_t BLE_Protocol_Parse(const uint8_t *data, uint16_t length, BLE_Frame_t *frame)
{
    /* 帧头长度字段与校验字段的十六进制半字节。 */
    uint8_t length_high, length_low;
    uint8_t checksum_high, checksum_low;

    /* 解析后的负载长度、校验位置与校验值。 */
    uint8_t calculated_checksum = 0U; // 本地对payload计算的异或校验和
    uint8_t received_checksum; // 从帧尾CC字段解析得到的校验和
    uint16_t payload_length; // LL字段声明的负载长度
    uint16_t checksum_position; // '*'在当前帧中的下标
    uint16_t index; // payload遍历下标

    /* 基本参数、帧头或长度字段无效时立即拒绝，不写入frame。 */
    if(data == NULL || frame == NULL || length < 7U) return 0U;
    if(data[0] != '@' || data[3] != ':') return 0U;

    if(BLE_Protocol_HexToNibble(data[1], &length_high) == 0U) return 0U;
    if(BLE_Protocol_HexToNibble(data[2], &length_low) == 0U) return 0U;

    payload_length = ((uint16_t)length_high << 4U) | length_low;
    if(payload_length > BLE_MAX_PAYLOAD_LENGTH) return 0U;

    /*
     * 不含CRLF时固定开销为7字节：@、两位长度、冒号、星号和两位校验。
     * 实际长度不符时立即拒绝，避免按错误的LL继续定位校验字段。
     */
    if(length != payload_length + 7U) return 0U;

    checksum_position = 4U + payload_length;
    if(data[checksum_position] != '*') return 0U;

    if(BLE_Protocol_HexToNibble(data[checksum_position + 1U], &checksum_high) == 0U) return 0U;
    if(BLE_Protocol_HexToNibble(data[checksum_position + 2U], &checksum_low) == 0U) return 0U;

    received_checksum = (checksum_high << 4U) | checksum_low;

    for(index = 0U; index < payload_length; index++ )
    {
        calculated_checksum ^= data[4U + index];
    }

    /* 本地计算值与帧尾CC不一致时拒绝帧；一致时才复制payload到输出结构。 */
    if(calculated_checksum != received_checksum) return 0U;

    frame->payload_length = payload_length;
    if(payload_length > 0U) memcpy(frame->payload, &data[4U], payload_length);
    frame->payload[payload_length] = '\0'; // 确保C字符串结束符

    return 1U;

}

/**
 * @brief 将负载封装成完整蓝牙协议帧
 *
 * 输出内容包含帧头、长度、负载、校验和以及CRLF，
 * 返回值可以直接作为HAL_UART_Transmit()的发送长度。
 *
 * @retval 完整帧长度，返回0U表示参数或缓冲区错误
 */
uint16_t BLE_Protocol_Build(const uint8_t *payload, uint16_t payload_length, uint8_t *output, uint16_t output_capacity)
{
  uint8_t checksum = 0U;
  uint16_t frame_length;
  uint16_t index;

  if(output == NULL) return 0U;
  if(payload_length > 0U && payload == NULL) return 0U;
  if(payload_length > BLE_MAX_PAYLOAD_LENGTH) return 0U;

  frame_length = payload_length + 9U; // 帧头、长度、冒号、负载、星号、校验和、CRLF
  if(output_capacity < frame_length) return 0U;

  output[0] = '@';
  output[1] = BLE_Protocol_NibbleToHex((uint8_t)(payload_length >> 4U));
  output[2] = BLE_Protocol_NibbleToHex((uint8_t)payload_length);
  output[3] = ':';

  for(index = 0U; index < payload_length; index++)
  {
    output[4U + index] = payload[index];
    checksum ^= payload[index];
  }

  output[4U + payload_length] = '*';
  output[5U + payload_length] = BLE_Protocol_NibbleToHex(checksum >> 4U);
  output[6U + payload_length] = BLE_Protocol_NibbleToHex(checksum);
  output[7U + payload_length] = '\r';
  output[8U + payload_length] = '\n';

  return frame_length;
}
