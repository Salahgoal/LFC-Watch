#ifndef __LSM303_H
#define __LSM303_H

#include <stdint.h>

/*
 * LSM303DLHC内部包含两个相对独立的功能模块：
 *
 * 加速度计使用I2C地址0x19；
 * 磁力计使用I2C地址0x1E。
 *
 * 因此它虽然是同一颗芯片，但在I2C总线上表现为两个从机设备。
 */
#define LSM303_ACCEL_I2C_ADDRESS          0x19U
#define LSM303_MAG_I2C_ADDRESS            0x1EU

#define LSM303_ACCEL_CTRL_REG1            0x20U // 加速度计控制寄存器1
#define LSM303_ACCEL_CTRL_REG1_DEFAULT    0x07U // 三轴使能，但输出速率为0

#define LSM303_MAG_ID_START_REGISTER      0x0AU // 磁力计身份寄存器起始地址
#define LSM303_MAG_EXPECTED_ID_A          0x48U // 字符H
#define LSM303_MAG_EXPECTED_ID_B          0x34U // 字符4
#define LSM303_MAG_EXPECTED_ID_C          0x33U // 字符3

/* 保存磁力计三个连续身份寄存器返回的“H43”。 */
typedef struct
{
    uint8_t id_a;
    uint8_t id_b;
    uint8_t id_c;
} LSM303_MagID_t;

/*
 * 保存LSM303DLHC的两组三轴原始数据。
 *
 * accel_x、accel_y、accel_z是右对齐后的12位加速度数据。
 * 当前使用±2g高分辨率模式，每个最低有效位约对应1mg。
 *
 * mag_x、mag_y、mag_z是磁力计输出的有符号原始值。
 * 计算指南针方向前，还需要进行零偏校正、三轴范围修正和倾斜补偿。
 */
typedef struct
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
} LSM303_RawData_t;

/* 检查加速度计地址是否返回ACK。 */
uint8_t LSM303_AccelIsReady(void);

/* 检查磁力计地址是否返回ACK。 */
uint8_t LSM303_MagIsReady(void);

/* 读取加速度计CTRL_REG1_A，用于验证寄存器通信。 */
uint8_t LSM303_ReadAccelControl(uint8_t *register_value);

/* 读取磁力计三个身份寄存器，正常结果应为“H43”。 */
uint8_t LSM303_ReadMagDeviceID(LSM303_MagID_t *device_id);

/* 检查设备身份，并配置加速度计和磁力计进入连续测量状态。 */
uint8_t LSM303_Init(void);

/* 从空闲状态恢复加速度计和磁力计连续测量。 */
uint8_t LSM303_StartMeasurement(void);

/* 停止加速度计采样，并让磁力计进入休眠模式。 */
uint8_t LSM303_StopMeasurement(void);

/* 连续读取加速度和磁场两组三轴原始数据。 */
uint8_t LSM303_ReadRawData(LSM303_RawData_t *raw_data);

#endif /* __LSM303_H */
