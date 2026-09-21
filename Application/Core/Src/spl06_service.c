#include "spl06_service.h"
#include <stddef.h>
#include <math.h>

#define SPL06_TEMPERATURE_SCALE_FACTOR   524288.0f  // 温度单次采样对应的Kt
#define SPL06_PRESSURE_SCALE_FACTOR      1572864.0f // 气压2倍过采样对应的Kp
#define SPL06_SEA_LEVEL_PRESSURE_PA      101325.0f  // 标准海平面气压，单位Pa

/*
 * 将SPL06原始数据换算成经过出厂系数补偿的温度、气压和估算海拔。
 *
 * SPL06输出的24位数据不能直接作为温度或气压使用。处理过程为：
 *
 * 原始数据
 * -> 按当前过采样率进行缩放
 * -> 代入芯片出厂校准系数
 * -> 得到实际温度和气压
 * -> 根据标准海平面气压估算海拔
 * -> 转换成适合消息队列传递的缩放整数
 *
 * 气压补偿公式同时使用气压和温度原始值，是因为MEMS压力元件会受到
 * 温度变化影响。忽略温度交叉补偿，会使最终气压产生明显偏差。
 */
uint8_t SPL06_Service_ConvertRawData(const SPL06_RawData_t *raw_data, const SPL06_Calibration_t *calibration, SPL06_Data_t *data)
{
    float temperature_raw_scaled;
    float pressure_raw_scaled;
    float temperature_c;
    float pressure_pa;
    float altitude_m;

    if(raw_data == NULL || calibration == NULL || data == NULL) return 0U;

    /*
     * 原始数据的缩放因子由测量过采样率决定。
     *
     * 当前spl06.c中配置：
     * 温度为单次采样，对应Kt = 524288；
     * 气压为2倍过采样，对应Kp = 1572864。
     *
     * 如果以后修改PRS_CFG或TMP_CFG中的过采样率，必须同步修改这里，
     * 否则补偿公式结构虽然正确，计算结果仍会出现较大偏差。
     */
    temperature_raw_scaled = (float)raw_data->temperature_raw / SPL06_TEMPERATURE_SCALE_FACTOR;
    pressure_raw_scaled = (float)raw_data->pressure_raw / SPL06_PRESSURE_SCALE_FACTOR;

    /*
     * 温度补偿公式：
     *
     * Tcomp = c0 × 0.5 + c1 × Traw_sc
     *
     * Traw_sc是经过Kt缩放后的温度原始值；
     * c0和c1是当前芯片内部保存的出厂温度补偿系数。
     */
    temperature_c = (float)calibration->c0 * 0.5f + (float)calibration->c1 * temperature_raw_scaled;

    /*
     * 气压补偿公式：
     *
     * Pcomp = c00
     *       + Praw_sc × [c10 + Praw_sc × (c20 + Praw_sc × c30)]
     *       + Traw_sc × c01
     *       + Traw_sc × Praw_sc × (c11 + Praw_sc × c21)
     *
     * Pcomp：补偿后的气压，单位Pa；
     * Praw_sc：经过Kp缩放的气压原始值；
     * Traw_sc：经过Kt缩放的温度原始值；
     * c00～c30：当前SPL06内部的出厂校准系数。
     *
     * 公式既包含气压自身的多阶修正，也包含温度对压力元件影响的
     * 交叉补偿。括号顺序必须保持一致，否则会改变多项式含义。
     */
    pressure_pa = (float)calibration->c00;
    pressure_pa += pressure_raw_scaled * ((float)calibration->c10 + pressure_raw_scaled * ((float)calibration->c20 + pressure_raw_scaled * (float)calibration->c30));
    pressure_pa += temperature_raw_scaled * (float)calibration->c01;
    pressure_pa += temperature_raw_scaled * pressure_raw_scaled * ((float)calibration->c11 + pressure_raw_scaled * (float)calibration->c21);

    if(pressure_pa <= 0.0f) return 0U; // 实际气压不可能为负数

    /*
     * 根据气压估算海拔：
     *
     * altitude = 44330 × [1 - (pressure / sea_level_pressure)^0.19029495]
     *
     * 当前使用固定的标准海平面气压101325Pa。现实中的海平面气压会随
     * 天气和地区变化，因此这里的海拔只能用于学习公式和观察相对变化，
     * 不能替代经过当天气象站基准气压校准的专业高度计。
     *
     * 这也解释了设备在广州低海拔区域仍可能显示一百多米的原因：
     * 误差主要来自固定海平面气压基准，而不一定是SPL06通信或补偿错误。
     */
    altitude_m = 44330.0f * (1.0f - powf(pressure_pa / SPL06_SEA_LEVEL_PRESSURE_PA, 0.19029495f));

    data->temperature_centi_c = (int32_t)(temperature_c * 100.0f); // ℃转换为0.01℃
    data->pressure_pa = (int32_t)(pressure_pa + 0.5f);             // 四舍五入到整数Pa
    data->altitude_cm = (int32_t)(altitude_m * 100.0f + 0.5f);    // m转换为cm

    return 1U;
}
