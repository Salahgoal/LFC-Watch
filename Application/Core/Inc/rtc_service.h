#ifndef RTC_SERVICE_H
#define RTC_SERVICE_H

#include <stdint.h>

/*
 * 应用层日期时间结构。
 * STM32 HAL把时间和日期拆成两个结构体，这里将它们合并，
 * 方便FreeRTOS任务和后续UI、蓝牙同步功能统一使用。
*/

typedef struct
{
    uint16_t year;
    uint8_t month;
    uint8_t date;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} RTC_DateTime_t ;

/**
 * @brief 读取RTC当前日期和时间
 * @param date_time 用于接收读取结果的结构体地址
 * @retval 1=读取成功，0=参数错误或HAL读取失败
*/
uint8_t RTC_Service_Read(RTC_DateTime_t *date_time);

/**
 * @brief 向RTC写入新的日期和时间
 * @param date_time 待写入的完整日期时间
 * @retval 1=写入成功，0=参数非法或HAL写入失败
*/
uint8_t RTC_Service_SetDateTime(const RTC_DateTime_t *date_time);


#endif
