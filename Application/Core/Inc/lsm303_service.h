#ifndef __LSM303_SERVICE_H
#define __LSM303_SERVICE_H

#include "lsm303.h"
#include <stdint.h>

/*
 * 保存磁力计校准过程中采集到的范围和计算结果。
 *
 * min和max记录多方向旋转过程中各轴出现过的极值；
 * span表示该轴已经覆盖的磁场范围；
 * offset是最大值与最小值的中点，用于修正固定零偏；
 * range_ready表示三个轴都达到了基础覆盖要求。
 *
 * 该结构只实现当前项目使用的基础Min-Max校准，不等同于完整的
 * 磁场椭球拟合，因此可以改善结果，但无法保证专业指南针精度。
 */
typedef struct
{
    int16_t min_x;
    int16_t max_x;
    int16_t min_y;
    int16_t max_y;
    int16_t min_z;
    int16_t max_z;

    uint16_t span_x;
    uint16_t span_y;
    uint16_t span_z;

    int16_t offset_x;
    int16_t offset_y;
    int16_t offset_z;

    uint32_t sample_count;    // 已用于校准的样本数量
    uint8_t started;          // 1表示已经接收第一组样本
    uint8_t range_ready;      // 1表示三个轴的变化范围均达到基础要求
} LSM303_MagCalibration_t;

/*
 * 保存校正后的磁场数据、方向角和板子姿态。
 *
 * heading_degrees是不考虑板子倾斜的基础方向角；
 * tilt_heading_degrees是结合加速度计姿态进行倾斜补偿后的方向角；
 * pitch和roll用于表示当前板子的前后、左右倾斜程度。
 */
typedef struct
{
    int16_t corrected_mag_x;
    int16_t corrected_mag_y;
    int16_t corrected_mag_z;

    uint16_t heading_degrees;         // 水平方向角，范围0～359°
    uint16_t tilt_heading_degrees;    // 倾斜补偿方向角，范围0～359°

    int16_t pitch_degrees;            // 俯仰角，范围约为-90～+90°
    int16_t roll_degrees;             // 横滚角，范围约为-180～+180°
} LSM303_CompassData_t;

/* 清空旧校准数据，准备开始新的磁场范围采集。 */
void LSM303_Service_InitMagCalibration(LSM303_MagCalibration_t *calibration);

/*
 * 使用一组新磁力计数据更新各轴极值、范围和零偏。
 *
 * 注意参数顺序：先传calibration，再传raw_data。
 */
uint8_t LSM303_Service_UpdateMagCalibration(LSM303_MagCalibration_t *calibration, const LSM303_RawData_t *raw_data);

/* 使用基础校准结果计算未经过倾斜补偿的水平方向角。 */
uint8_t LSM303_Service_CalculateHorizontalHeading(const LSM303_RawData_t *raw_data, const LSM303_MagCalibration_t *calibration, LSM303_CompassData_t *compass_data);

/* 使用加速度计姿态对磁场方向进行倾斜补偿。 */
uint8_t LSM303_Service_CalculateTiltHeading(const LSM303_RawData_t *raw_data, const LSM303_MagCalibration_t *calibration, LSM303_CompassData_t *compass_data);

#endif /* __LSM303_SERVICE_H */
