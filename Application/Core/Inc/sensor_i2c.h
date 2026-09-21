#ifndef __SENSOR_I2C_H
#define __SENSOR_I2C_H

#include <stdint.h>

/*
 * 初始化背板传感器共用的软件I2C总线。
 *
 * MPU6050、AHT21、SPL06、LSM303和EM7028共用PB13和PB14。
 * 本模块统一保存总线引脚，具体传感器驱动只需要提供7位设备地址，
 * 不需要重复关心GPIO端口和软件I2C底层时序。
 */
void SensorI2C_Init(void);

/* 检查指定7位I2C地址是否有设备返回ACK。 */
uint8_t SensorI2C_IsDeviceReady(uint8_t dev_addr);

/* 向指定设备的一个寄存器写入一个字节。 */
uint8_t SensorI2C_WriteRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

/* 从指定设备的一个寄存器读取一个字节。 */
uint8_t SensorI2C_ReadRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);

/* 从指定设备的连续寄存器中读取多个字节。 */
uint8_t SensorI2C_ReadRegisters(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t length);

/* 向指定设备直接发送多个数据字节，不附带寄存器地址。 */
uint8_t SensorI2C_WriteData(uint8_t dev_addr, const uint8_t *data, uint8_t length);

/* 从指定设备直接读取多个数据字节，不先发送寄存器地址。 */
uint8_t SensorI2C_ReadData(uint8_t dev_addr, uint8_t *data, uint8_t length);

#endif /* __SENSOR_I2C_H */
