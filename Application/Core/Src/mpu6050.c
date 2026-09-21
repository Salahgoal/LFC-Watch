#include "mpu6050.h"
#include "sensor_i2c.h"
#include <stddef.h>

#define MPU6050_SMPLRT_DIV_REG   0x19U // 采样率分频寄存器
#define MPU6050_CONFIG_REG       0x1AU // 数字低通滤波配置寄存器
#define MPU6050_GYRO_CONFIG_REG  0x1BU // 陀螺仪量程配置寄存器
#define MPU6050_ACCEL_CONFIG_REG 0x1CU // 加速度计量程配置寄存器
#define MPU6050_ACCEL_XOUT_H_REG 0x3BU // 连续传感器数据的起始寄存器
#define MPU6050_PWR_MGMT_1_REG   0x6BU // 电源状态和时钟源配置寄存器
#define MPU6050_PWR_MGMT_2_REG   0x6CU // 各测量轴待机控制寄存器
#define MPU6050_WHO_AM_I_REG     0x75U // 设备身份寄存器

#define MPU6050_PWR_MGMT_1_INTERNAL_CLOCK 0x00U // 退出循环模式，暂时使用内部时钟
#define MPU6050_PWR_MGMT_1_NORMAL         0x01U // 正常模式，使用X轴陀螺仪时钟
#define MPU6050_PWR_MGMT_1_LOW_POWER      0x28U // 低功耗循环模式，关闭温度传感器

#define MPU6050_PWR_MGMT_2_ALL_AXES       0x00U // 加速度计和陀螺仪全部开启
#define MPU6050_PWR_MGMT_2_ACCEL_20HZ     0x87U // 20Hz唤醒，仅保留加速度计

/*
 * 将高、低两个8位寄存器组合成16位有符号数。
 *
 * MPU6050使用高字节在前的存储顺序。先把高字节移动到bit15~bit8，
 * 再拼接低字节，最后转换成int16_t，使二进制补码形式的负数能够
 * 被C语言正确解释。
 */
static int16_t MPU6050_CombinSignedData(uint8_t h_byte, uint8_t l_byte)
{
    uint16_t value;

    value = (uint16_t)h_byte << 8U;
    value |= l_byte;

    return (int16_t)value;
}

uint8_t MPU6050_IsReady(void)
{
    return SensorI2C_IsDeviceReady(MPU6050_DEVICE_ADDRESS);
}

/*
 * 读取MPU6050的WHO_AM_I身份寄存器。
 *
 * 正常芯片应返回0x68。读取该寄存器可以同时验证：
 * 1. 设备能够在0x68地址返回ACK；
 * 2. 软件I2C写阶段能够发送寄存器地址；
 * 3. 重复起始信号能够切换到读取阶段；
 * 4. 读取数据与ACK/NACK时序工作正常。
 *
 * 因此WHO_AM_I比单独检查ACK更适合作为驱动首次通信测试。
 *
 * @param dev_id 用于保存身份寄存器结果
 * @retval 1U=读取成功，0U=参数错误或通信失败
 */
uint8_t MPU6050_ReadDeviceID(uint8_t *dev_id)
{
    if(dev_id == NULL) return 0U;

    return SensorI2C_ReadRegister(MPU6050_DEVICE_ADDRESS, MPU6050_WHO_AM_I_REG, dev_id);
}

/*
 * 切换到仅加速度计工作的低功耗循环采样模式。
 *
 * 先切换到内部时钟，再关闭陀螺仪，避免关闭陀螺仪的同时
 * 仍选择X轴陀螺仪作为时钟源。最后才开启CYCLE循环采样。
 */
uint8_t MPU6050_EnterLowPowerMode(void)
{
    if(SensorI2C_WriteRegister(MPU6050_DEVICE_ADDRESS, MPU6050_PWR_MGMT_1_REG, MPU6050_PWR_MGMT_1_INTERNAL_CLOCK) == 0U) return 0U;
    if(SensorI2C_WriteRegister(MPU6050_DEVICE_ADDRESS, MPU6050_PWR_MGMT_2_REG, MPU6050_PWR_MGMT_2_ACCEL_20HZ) == 0U) return 0U;
    if(SensorI2C_WriteRegister(MPU6050_DEVICE_ADDRESS, MPU6050_PWR_MGMT_1_REG, MPU6050_PWR_MGMT_1_LOW_POWER) == 0U) return 0U;

    return 1U;
}

/*
 * 恢复六轴正常测量。
 *
 * 先退出CYCLE并使用内部时钟，再开启全部测量轴。
 * X轴陀螺仪启动后，最后将它选为稳定时钟源。
 */
uint8_t MPU6050_EnterNormalMode(void)
{
    if(SensorI2C_WriteRegister(MPU6050_DEVICE_ADDRESS, MPU6050_PWR_MGMT_1_REG, MPU6050_PWR_MGMT_1_INTERNAL_CLOCK) == 0U) return 0U;
    if(SensorI2C_WriteRegister(MPU6050_DEVICE_ADDRESS, MPU6050_PWR_MGMT_2_REG, MPU6050_PWR_MGMT_2_ALL_AXES) == 0U) return 0U;
    if(SensorI2C_WriteRegister(MPU6050_DEVICE_ADDRESS, MPU6050_PWR_MGMT_1_REG, MPU6050_PWR_MGMT_1_NORMAL) == 0U) return 0U;

    return 1U;
}

/*
 * 初始化MPU6050的测量参数。
 *
 * 配置流程：
 * 1. 退出睡眠并选择稳定的时钟源；
 * 2. 开启全部加速度计和陀螺仪测量轴；
 * 3. 配置数字低通滤波；
 * 4. 配置内部输出采样率；
 * 5. 配置陀螺仪和加速度计量程。
 *
 * 驱动把MPU6050内部输出速率配置为100Hz，而SensorTask目前约每20ms
 * 读取一次，即应用层实际使用约50Hz数据。传感器内部可以产生更高频
 * 的新结果，SensorTask每次读取当前最新的一组数据。
 *
 * @retval 1U=全部寄存器配置成功，0U=任意寄存器写入失败
 */
uint8_t MPU6050_Init(void)
{
    /* 初始化时先进入六轴正常模式，再配置滤波、采样率和量程。 */
    if(MPU6050_EnterNormalMode() == 0U) return 0U;

    /*
     * CONFIG写入0x03，将DLPF_CFG配置为3。
     * 数字低通滤波可以抑制手表振动和电路噪声，使姿态、计步和抬腕
     * 使用的加速度与角速度数据更加稳定。
     */
    if(SensorI2C_WriteRegister(MPU6050_DEVICE_ADDRESS, MPU6050_CONFIG_REG, 0x03U) == 0U) return 0U;

    /*
     * DLPF开启后内部采样基准为1kHz。
     * SMPLRT_DIV=9时，输出采样率为1000 / (1 + 9) = 100Hz。
     */
    if(SensorI2C_WriteRegister(MPU6050_DEVICE_ADDRESS, MPU6050_SMPLRT_DIV_REG, 0x09U) == 0U) return 0U;

    /*
     * GYRO_CONFIG写入0x00，将陀螺仪配置为±250°/s。
     * 这是灵敏度最高的量程，适合观察手表的日常转动和抬腕动作。
     */
    if(SensorI2C_WriteRegister(MPU6050_DEVICE_ADDRESS, MPU6050_GYRO_CONFIG_REG, 0x00U) == 0U) return 0U;

    /*
     * ACCEL_CONFIG写入0x00，将加速度计配置为±2g。
     * 日常静止、行走和抬腕通常不会超出该范围，同时可以获得最高分辨率。
     */
    if(SensorI2C_WriteRegister(MPU6050_DEVICE_ADDRESS, MPU6050_ACCEL_CONFIG_REG, 0x00U) == 0U) return 0U;

    return 1U;
}

/*
 * 连续读取MPU6050的一组完整原始数据。
 *
 * 从ACCEL_XOUT_H开始连续读取14个字节，可以在同一次I2C事务中取得：
 *
 * buffer[0] ~ buffer[5]：加速度计XYZ
 * buffer[6] ~ buffer[7]：内部温度
 * buffer[8] ~ buffer[13]：陀螺仪XYZ
 *
 * 使用一次连续读取可以保证七项数据来自相近的采样时刻，也比逐个
 * 寄存器读取减少了START、设备地址和寄存器地址的重复传输开销。
 *
 * 本函数只负责组合原始寄存器数据，不进行物理单位换算。
 *
 * @param raw_data 用于保存七项原始数据
 * @retval 1U=读取成功，0U=参数错误或通信失败
 */
uint8_t MPU6050_ReadRawData(MPU6050_RawData_t *raw_data)
{
    uint8_t buffer[14];

    if(raw_data == NULL) return 0U;
    if(SensorI2C_ReadRegisters(MPU6050_DEVICE_ADDRESS, MPU6050_ACCEL_XOUT_H_REG, buffer, 14U) == 0U) return 0U;

    raw_data->accel_x = MPU6050_CombinSignedData(buffer[0], buffer[1]);
    raw_data->accel_y = MPU6050_CombinSignedData(buffer[2], buffer[3]);
    raw_data->accel_z = MPU6050_CombinSignedData(buffer[4], buffer[5]);
    raw_data->temperature = MPU6050_CombinSignedData(buffer[6], buffer[7]);
    raw_data->gyro_x = MPU6050_CombinSignedData(buffer[8], buffer[9]);
    raw_data->gyro_y = MPU6050_CombinSignedData(buffer[10], buffer[11]);
    raw_data->gyro_z = MPU6050_CombinSignedData(buffer[12], buffer[13]);

    return 1U;
}
