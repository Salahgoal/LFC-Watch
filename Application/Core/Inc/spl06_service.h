#ifndef __SPL06_SERVICE_H
#define __SPL06_SERVICE_H

#include "spl06.h"
#include <stdint.h>

/*
 * 保存经过出厂系数补偿和单位换算的SPL06数据。
 *
 * temperature_centi_c使用0.01℃，例如3516表示35.16℃；
 * pressure_pa使用Pa，例如100017表示100017Pa，也就是1000.17hPa；
 * altitude_cm使用cm，例如11825表示118.25m。
 *
 * 最终结果使用缩放整数，方便通过UIMessage_t传递，并避免UiTask
 * 再进行浮点计算和传感器单位换算。
 */
typedef struct
{
    int32_t temperature_centi_c;    // 补偿温度，单位0.01℃
    int32_t pressure_pa;            // 补偿气压，单位Pa
    int32_t altitude_cm;            // 根据标准海平面气压估算的海拔，单位cm
} SPL06_Data_t;

/*
 * 使用24位原始数据和出厂校准系数计算温度、气压及估算海拔。
 * 返回1表示换算成功，返回0表示参数错误或补偿结果无效。
 */
uint8_t SPL06_Service_ConvertRawData(const SPL06_RawData_t *raw_data, const SPL06_Calibration_t *calibration, SPL06_Data_t *data);

#endif /* __SPL06_SERVICE_H */
