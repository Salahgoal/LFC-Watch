#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>

#define MPU6050_DEVICE_ADDRESS 0x68U // MPU6050的7位I2C地址
#define MPU6050_WHO_AM_I_VALUE 0x68U // WHO_AM_I寄存器的正常返回值

/*
 * 保存MPU6050一次连续读取获得的原始数据。
 *
 * 每项数据都是传感器寄存器直接输出的16位有符号数，尚未换算成
 * mg、mdps或摄氏度。物理单位换算由mpu6050_service.c完成。
 *
 * 寄存器排列顺序：
 * 加速度XYZ -> 温度 -> 陀螺仪XYZ
 */
typedef struct
{
    int16_t accel_x; // X轴加速度原始值
    int16_t accel_y; // Y轴加速度原始值
    int16_t accel_z; // Z轴加速度原始值
    int16_t temperature; // 芯片内部温度原始值
    int16_t gyro_x; // X轴角速度原始值
    int16_t gyro_y; // Y轴角速度原始值
    int16_t gyro_z; // Z轴角速度原始值
} MPU6050_RawData_t;

/* 检查MPU6050是否在I2C总线上返回ACK。 */
uint8_t MPU6050_IsReady(void);

/* 读取WHO_AM_I身份寄存器。 */
uint8_t MPU6050_ReadDeviceID(uint8_t *dev_id);

/* 配置电源、滤波、采样率和测量量程。 */
uint8_t MPU6050_Init(void);

/* 进入仅加速度计工作的低功耗循环采样模式。 */
uint8_t MPU6050_EnterLowPowerMode(void);

/* 恢复加速度计、温度传感器和陀螺仪正常工作。 */
uint8_t MPU6050_EnterNormalMode(void);

/* 连续读取加速度、温度和陀螺仪原始数据。 */
uint8_t MPU6050_ReadRawData(MPU6050_RawData_t *raw_data);

#endif /* __MPU6050_H */
