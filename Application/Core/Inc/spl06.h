#ifndef __SPL06_H
#define __SPL06_H

#include <stdint.h>

#define SPL06_I2C_ADDRESS    0x76U // SPL06的7位I2C地址
#define SPL06_ID_REGISTER    0x0DU // 产品ID寄存器地址
#define SPL06_EXPECTED_ID    0x10U // 当前SPL06预期返回的产品ID

/*
 * 保存SPL06内部出厂写入的校准系数。
 *
 * 每颗SPL06在制造时都存在细微差异，因此不能只使用原始气压和温度数据。
 * 必须读取芯片内部独有的校准系数，并代入补偿公式，才能得到实际温度和气压。
 */
typedef struct
{
    int16_t c0;     // 温度补偿系数，12位有符号数
    int16_t c1;     // 温度补偿系数，12位有符号数
    int32_t c00;    // 气压补偿系数，20位有符号数
    int32_t c10;    // 气压补偿系数，20位有符号数
    int16_t c01;    // 气压温度交叉补偿系数
    int16_t c11;    // 气压温度交叉补偿系数
    int16_t c20;    // 二阶气压补偿系数
    int16_t c21;    // 高阶气压温度补偿系数
    int16_t c30;    // 三阶气压补偿系数
} SPL06_Calibration_t;

/* 保存SPL06返回的24位有符号气压和温度原始数据。 */
typedef struct
{
    int32_t pressure_raw;       // 原始气压数据，24位有符号数
    int32_t temperature_raw;    // 原始温度数据，24位有符号数
} SPL06_RawData_t;

/* 检查SPL06是否在0x76地址返回ACK。 */
uint8_t SPL06_IsReady(void);

/* 读取SPL06产品ID寄存器。 */
uint8_t SPL06_ReadDeviceID(uint8_t *dev_id);

/* 读取并解析SPL06内部保存的全部出厂校准系数。 */
uint8_t SPL06_ReadCalibration(SPL06_Calibration_t *calibration);

/* 配置SPL06并进入气压、温度连续测量模式。 */
uint8_t SPL06_StartContinuousMeasurement(void);

/* 停止连续测量，使SPL06进入空闲模式。 */
uint8_t SPL06_StopMeasurement(void);

/*
 * 读取最新一组气压和温度原始数据。
 * 返回1表示两种数据均已就绪并读取成功。
 */
uint8_t SPL06_ReadRawData(SPL06_RawData_t *raw_data);

#endif /* __SPL06_H */
