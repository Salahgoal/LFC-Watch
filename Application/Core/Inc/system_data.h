#ifndef __SYSTEM_DATA_H
#define __SYSTEM_DATA_H

#include <stdint.h>
#include "rtc_service.h" // RTC服务接口，读取当前日期和时间

typedef struct
{
    RTC_DateTime_t date_time; // 当前日期和时间  这里是结构体内嵌套了一个结构体
    uint16_t battery_voltage_mv; // 当前电池电压，单位mV
    uint8_t battery_percent; // 当前电池百分比，0~100
    uint8_t heart_rate_bpm; // 当前心率，单位BPM
} SystemData_t;

#endif /* __SYSTEM_DATA_H */
