#include "aht21_service.h"
#include <stddef.h>

#define AHT21_RAW_FULL_SCALE 1048576UL // 2^20，AHT21的20位原始数据满量程

uint8_t AHT21_Service_ConvertRawData(const AHT21_RawData_t *raw_data, AHT21_Data_t *data)
{
    uint64_t converted_value;

    if(raw_data == NULL || data == NULL) return 0U;

    /*
     * 相对湿度换算公式：
     *
     * RH = humidity_raw / 2^20 × 100%
     *
     * 输出单位为0.01%RH，因此将公式中的100扩大为10000。
     * 例如计算结果为6248，就表示62.48%RH。
     *
     * 20位原始值最大可达到1048575，乘以10000后会超出32位有符号整数
     * 范围，因此中间计算使用uint64_t，避免乘法溢出。
     */
    converted_value = (uint64_t)raw_data->humidity_raw * 10000ULL;
    data->humidity_centi_rh = (uint32_t)(converted_value / AHT21_RAW_FULL_SCALE);

    /*
     * 理论换算结果不应超过100%RH。
     * 当通信噪声、舍入或异常原始数据使结果越界时，将其限制为100.00%RH，
     * 避免UI显示不合理的湿度值。
     */
    if(data->humidity_centi_rh > 10000U) data->humidity_centi_rh = 10000U;

    /*
     * 温度换算公式：
     *
     * T = temperature_raw / 2^20 × 200 - 50
     *
     * 输出单位为0.01℃，因此将200扩大为20000，将50℃写成5000。
     * 整个换算过程使用整数完成，最终结果仍可以在UI上显示两位小数。
     */
    converted_value = (uint64_t)raw_data->temperature_raw * 20000ULL;
    data->temperature_centi_c = (int32_t)(converted_value / AHT21_RAW_FULL_SCALE);
    data->temperature_centi_c -= 5000L;

    return 1U;
}
