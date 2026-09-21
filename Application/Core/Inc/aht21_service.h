#ifndef __AHT21_SERVICE_H
#define __AHT21_SERVICE_H

#include "aht21.h"
#include <stdint.h>

/*
 * 保存经过单位换算的AHT21温湿度数据。
 *
 * temperature_centi_c使用0.01℃；
 * humidity_centi_rh使用0.01%RH。
 *
 * 例如：
 * 2536表示25.36℃；
 * 6248表示62.48%RH。
 *
 * 使用带缩放比例的整数，可以避免SensorTask消息和UiTask显示过程
 * 依赖浮点运算，同时保留两位小数。
 */
typedef struct
{
    int32_t temperature_centi_c;    // 环境温度，单位0.01℃
    int32_t humidity_centi_rh;      // 相对湿度，单位0.01%RH
} AHT21_Data_t;

/*
 * 将AHT21的20位原始数据换算成0.01℃和0.01%RH。
 * 返回1表示换算成功，返回0表示传入了空指针。
 */
uint8_t AHT21_Service_ConvertRawData(const AHT21_RawData_t *raw_data, AHT21_Data_t *data);

#endif /* __AHT21_SERVICE_H */
