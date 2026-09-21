#ifndef __EM7028_H
#define __EM7028_H

#include <stdint.h>

#define EM7028_I2C_ADDRESS    0x24U // EM7028的7位I2C地址
#define EM7028_ID_REGISTER    0x00U // 芯片身份寄存器
#define EM7028_EXPECTED_ID    0x36U // EM7028预期返回的设备ID

/* 检查EM7028是否在0x24地址返回ACK。 */
uint8_t EM7028_IsReady(void);

/* 读取设备ID，正常结果应为0x36。 */
uint8_t EM7028_ReadDeviceID(uint8_t *device_id);

/*
 * 验证设备身份并配置光电采样参数。
 * 初始化完成后仍保持停止状态，由SensorTask根据页面状态启动测量。
 */
uint8_t EM7028_Init(void);

/* 启动EM7028光电采样。 */
uint8_t EM7028_StartMeasurement(void);

/* 停止EM7028光电采样。 */
uint8_t EM7028_StopMeasurement(void);

/* 读取HRS1通道的16位原始PPG数据。 */
uint8_t EM7028_ReadRawData(uint16_t *raw_data);

#endif /* __EM7028_H */
