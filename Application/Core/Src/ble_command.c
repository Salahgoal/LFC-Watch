#include "ble_command.h"

#include <string.h>

/*
 * 命令层接收BLE_Protocol_Parse()已经验证合法的payload，识别手机请求的业务类型。
 * 本文件只识别命令和解析TIME文本，不操作RTC、UART、传感器或FreeRTOS对象。
 * LFC+OTA识别为升级请求后，交回BLETask应答，再由ControlTask完成软件交接。
 * LFC+OTA只返回命令编号；应答、控制消息和Bootloader交接在freertos.c中完成。
 */

/**
 * @brief 将固定数量的ASCII十进制字符转换为无符号整数
 *
 * 例如输入"2026"和digit_count=4，输出数值2026。
 * 本函数是TIME命令解析的内部工具，不向其他模块公开。
 *
 * @param data 待转换字符的起始地址
 * @param digit_count 需要转换的字符数量
 * @param value 用于接收转换结果
 * @retval 1U=转换成功，0U=参数错误或存在非数字字符
 */
static uint8_t BLE_Command_ParseDecimal(const uint8_t *data, uint8_t digit_count, uint16_t *value)
{
    uint16_t result = 0U; // 按从左到右的顺序累积十进制结果
    uint8_t index; // 当前检查的ASCII数字下标

    /* 输入、输出指针或数字个数无效时，不修改value指向的结果。 */
    if(data == NULL || value == NULL || digit_count == 0U) return 0U;

    for(index = 0U; index < digit_count; index++)
    {
        /* 任意一个字符不是'0'~'9'时立即失败，整段数字不会写回。 */
        if(data[index] < '0' || data[index] > '9') return 0U;

        result *= 10U;
        result += (uint16_t)(data[index] - '0');
    }

    *value = result;

    return 1U;
}

/**
 * @brief 根据负载内容识别BLE业务命令
 *
 * PING必须正好为4个字节。
 * TIME命令只要以"TIME="开头，就交给时间解析函数继续检查，
 * 这样格式错误的TIME命令能够返回ERR:TIME而不是ERR:UNKNOWN。
 *
 * @param payload 已通过协议层验证的负载
 * @param payload_length 负载字节数
 * @retval 对应的BLE_CommandType_t
 */
BLE_CommandType_t BLE_Command_GetType(const uint8_t *payload, uint16_t payload_length)
{
    /* payload无效时无法进行字节比较，直接归类为未知命令。 */
    if(payload == NULL) return BLE_COMMAND_UNKNOWN;

    /* 固定命令必须同时满足精确长度和完整内容匹配，多字或少字都不会误识别。 */
    if(payload_length == 4U && memcmp(payload, "PING" , 4U) == 0)
    {
        return BLE_COMMAND_PING;
    }

    /*
     * 这里只比较协议层已校验的7字节负载，不包含@、长度、校验和或CRLF。
     * 匹配后返回命令类型；不匹配则继续识别其他命令，不触发升级动作。
     */
    if(payload_length == 7U && memcmp(payload, "LFC+OTA", 7U) == 0)
    {
        return BLE_COMMAND_ENTER_OTA;
    }

    if(payload_length == 11U && memcmp(payload, "LFC+VERSION", 11U) == 0)
    {
        return BLE_COMMAND_GET_VERSION;
    }

    if(payload_length == 8U && memcmp(payload, "LFC+SEND", 8U) == 0)
    {
        return BLE_COMMAND_SEND_DATA;
    }

    /* TIME=前缀匹配后先归类为校时命令，完整长度和各字段格式由ParseTime()继续检查。 */
    if(payload_length >= 5U && memcmp(payload, "TIME=", 5U) == 0)
    {
        return BLE_COMMAND_SET_TIME;
    }

    if(payload_length == 10U && memcmp(payload, "LFC+ENV=ON", 10U) == 0)
    {
        return BLE_COMMAND_ENV_STREAM_ON;
    }

    if(payload_length == 11U && memcmp(payload, "LFC+ENV=OFF", 11U) == 0)
    {
        return BLE_COMMAND_ENV_STREAM_OFF;
    }

    return BLE_COMMAND_UNKNOWN;
}

/**
 * @brief 将TIME命令负载解析为RTC日期时间结构
 *
 * 固定格式为TIME=YYYY-MM-DD,HH:MM:SS，共24个字节。
 * 本函数只检查文本格式，日期是否合法由RTC_Service_SetDateTime()
 * 继续判断。
 *
 * @param payload TIME命令负载
 * @param payload_length 负载字节数
 * @param date_time 用于接收解析后的日期和时间
 * @retval 1U=格式解析成功，0U=命令格式错误
 */
uint8_t BLE_Command_ParseTime(const uint8_t *payload, uint16_t payload_length, RTC_DateTime_t *date_time)
{
    RTC_DateTime_t parsed_date_time = {0}; // 所有字段都解析成功后再整体写回
    uint16_t value; // 暂存当前十进制字段的转换结果

    /* 参数、固定24字节长度或TIME=前缀无效时，不修改date_time。 */
    if(payload == NULL || date_time == NULL) return 0U;
    if(payload_length != 24U) return 0U; // TIME命令必须正好24个字节
    if(memcmp(payload, "TIME=", 5U) != 0) return 0U; // 必须以"TIME="开头

    /*
    * TIME=2026-08-27,18:30:55
    *          ^  ^  ^  ^  ^
    * 下标：    9 12 15 18 21
    */
    /* 任意一个分隔符位置不正确时立即拒绝，避免按错误下标解释后续数字。 */
    if(payload[9] != '-') return 0U;
    if(payload[12] != '-') return 0U;
    if(payload[15] != ',') return 0U;
    if(payload[18] != ':') return 0U;
    if(payload[21] != ':') return 0U;

    if(BLE_Command_ParseDecimal(&payload[5], 4U, &value) == 0U) return 0U;
    parsed_date_time.year = value;

    if(BLE_Command_ParseDecimal(&payload[10], 2U, &value) == 0U) return 0U;
    parsed_date_time.month = (uint8_t)value;

    if(BLE_Command_ParseDecimal(&payload[13], 2U, &value) == 0U) return 0U;
    parsed_date_time.date = (uint8_t)value;

    if(BLE_Command_ParseDecimal(&payload[16], 2U, &value) == 0U) return 0U;
    parsed_date_time.hour = (uint8_t)value;

    if(BLE_Command_ParseDecimal(&payload[19], 2U, &value) == 0U) return 0U;
    parsed_date_time.minute = (uint8_t)value;

    if(BLE_Command_ParseDecimal(&payload[22], 2U, &value) == 0U) return 0U;
    parsed_date_time.second = (uint8_t)value;

    /* 所有字段都转换成功后再整体写回，不会留下只更新一部分的日期时间。 */
    *date_time = parsed_date_time;
    return 1U;
}
