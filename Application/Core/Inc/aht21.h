#ifndef __AHT21_H
#define __AHT21_H

#include <stdint.h>

#define AHT21_I2C_ADDRESS              0x38U // AHT21的7位I2C地址
#define AHT21_STATUS_BUSY_MASK         0x80U // 状态位Bit7：1表示正在测量
#define AHT21_STATUS_CALIBRATED_MASK   0x08U // 状态位Bit3：1表示校准功能已经启用

/*
 * 保存AHT21返回的20位温湿度原始数据。
 *
 * 这里暂时不进行单位换算，只负责保存从6字节测量结果中
 * 组合出来的原始值。实际温湿度由AHT21 Service层统一计算。
 */
typedef struct
{
    uint32_t humidity_raw;       // 20位原始湿度数据
    uint32_t temperature_raw;    // 20位原始温度数据
} AHT21_RawData_t;

/* 检查AHT21是否在0x38地址返回ACK。 */
uint8_t AHT21_IsReady(void);

/* 读取AHT21当前状态字节。 */
uint8_t AHT21_ReadStatus(uint8_t *status);

/* 判断状态字节是否表示传感器正在测量。 */
uint8_t AHT21_IsBusy(uint8_t status);

/* 判断AHT21内部校准功能是否已经启用。 */
uint8_t AHT21_IsCalibrated(uint8_t status);

/* 发送内部初始化命令BE 08 00。 */
uint8_t AHT21_SendInitCommand(void);

/* 发送温湿度测量命令AC 33 00。 */
uint8_t AHT21_SendMeasureCommand(void);

/*
 * 读取AHT21返回的6字节测量结果，并组合出20位温湿度原始值。
 * 返回1表示读取成功，返回0表示通信失败或传感器仍处于忙状态。
 */
uint8_t AHT21_ReadMeasurement(AHT21_RawData_t *raw_data, uint8_t *status);

#endif /* __AHT21_H */
