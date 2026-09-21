#include "lsm303.h"
#include "sensor_i2c.h"
#include <stddef.h>

#define LSM303_ACCEL_CTRL_REG4            0x23U // 数据格式、量程和高分辨率配置
#define LSM303_ACCEL_OUT_X_L              0x28U // 加速度计X轴低字节起始地址
#define LSM303_ACCEL_AUTO_INCREMENT       0x80U // 加速度计连续读取地址自增位

#define LSM303_MAG_CRA_REG                0x00U // 磁力计输出速率配置
#define LSM303_MAG_CRB_REG                0x01U // 磁力计量程配置
#define LSM303_MAG_MODE_REG               0x02U // 磁力计工作模式配置
#define LSM303_MAG_OUT_X_H                0x03U // 磁力计X轴高字节起始地址

#define LSM303_ACCEL_CTRL_REG1_VALUE      0x27U // 10Hz正常模式，开启XYZ三轴
#define LSM303_ACCEL_CTRL_REG4_VALUE      0x88U // BDU锁定、±2g、高分辨率
#define LSM303_MAG_CRA_VALUE              0x10U // 磁力计15Hz输出速率
#define LSM303_MAG_CRB_VALUE              0x80U // 磁力计±4Gauss量程
#define LSM303_MAG_MODE_VALUE             0x00U // 磁力计连续转换模式
#define LSM303_MAG_SLEEP_VALUE            0x03U // 磁力计休眠模式

/*
 * 解析一个加速度轴的原始数据。
 *
 * 加速度计每个轴使用两个寄存器，低字节在前、高字节在后。
 * 高分辨率模式下，有效数据位于16位结果的高12位，因此组合后除以16，
 * 将数据右对齐为便于观察和处理的有符号整数。
 */
static int16_t LSM303_DecodeAcceleration(uint8_t low_byte, uint8_t high_byte)
{
    int16_t value = (int16_t)(((uint16_t)high_byte << 8U) | low_byte);

    return (int16_t)(value / 16);
}

/*
 * 解析一个磁力轴的原始数据。
 *
 * 磁力计采用高字节在前、低字节在后的排列方式，
 * 与加速度计的字节顺序相反。
 */
static int16_t LSM303_DecodeMagnetic(uint8_t high_byte, uint8_t low_byte)
{
    return (int16_t)(((uint16_t)high_byte << 8U) | low_byte);
}

uint8_t LSM303_AccelIsReady(void)
{
    return SensorI2C_IsDeviceReady(LSM303_ACCEL_I2C_ADDRESS);
}

uint8_t LSM303_MagIsReady(void)
{
    return SensorI2C_IsDeviceReady(LSM303_MAG_I2C_ADDRESS);
}

uint8_t LSM303_ReadAccelControl(uint8_t *register_value)
{
    if(register_value == NULL) return 0U;

    return SensorI2C_ReadRegister(LSM303_ACCEL_I2C_ADDRESS, LSM303_ACCEL_CTRL_REG1, register_value);
}

uint8_t LSM303_ReadMagDeviceID(LSM303_MagID_t *device_id)
{
    uint8_t buffer[3] = {0};

    if(device_id == NULL) return 0U;

    /*
     * 磁力计内部0x0A、0x0B和0x0C是连续的身份寄存器。
     * 从0x0A开始连续读取三个字节，可以一次取得“H43”身份信息。
     */
    if(SensorI2C_ReadRegisters(LSM303_MAG_I2C_ADDRESS, LSM303_MAG_ID_START_REGISTER, buffer, sizeof(buffer)) == 0U) return 0U;

    device_id->id_a = buffer[0];
    device_id->id_b = buffer[1];
    device_id->id_c = buffer[2];

    return 1U;
}

/*
 * 初始化LSM303DLHC的加速度计和磁力计。
 *
 * 初始化流程：
 *
 * 1. 分别检查0x19和0x1E是否返回ACK；
 * 2. 读取磁力计身份寄存器，确认结果为“H43”；
 * 3. 配置加速度计量程、分辨率和输出速率；
 * 4. 配置磁力计量程、输出速率和连续转换模式；
 * 5. 回读关键寄存器，确认配置真正写入芯片。
 *
 * 回读非常重要。I2C写函数返回成功只能说明总线上收到了ACK，
 * 回读正确才能进一步确认目标寄存器中的内容符合程序预期。
 */
uint8_t LSM303_Init(void)
{
    LSM303_MagID_t device_id = {0};
    uint8_t register_value = 0U;

    /*
     * 加速度计和磁力计可以独立工作，因此必须检查两个地址。
     * 只检查其中一个地址，不能证明整颗LSM303的两个模块都正常。
     */
    if(LSM303_AccelIsReady() == 0U || LSM303_MagIsReady() == 0U) return 0U;
    if(LSM303_ReadMagDeviceID(&device_id) == 0U) return 0U;

    if(device_id.id_a != LSM303_MAG_EXPECTED_ID_A || device_id.id_b != LSM303_MAG_EXPECTED_ID_B || device_id.id_c != LSM303_MAG_EXPECTED_ID_C) return 0U;

    /*
     * 加速度计先配置量程、分辨率和BDU，再启动采样。
     *
     * BDU开启后，一组高低字节在全部读出前不会被新数据覆盖，
     * 可以降低高字节和低字节来自不同采样时刻的风险。
     */
    if(SensorI2C_WriteRegister(LSM303_ACCEL_I2C_ADDRESS, LSM303_ACCEL_CTRL_REG4, LSM303_ACCEL_CTRL_REG4_VALUE) == 0U) return 0U;
    if(SensorI2C_WriteRegister(LSM303_ACCEL_I2C_ADDRESS, LSM303_ACCEL_CTRL_REG1, LSM303_ACCEL_CTRL_REG1_VALUE) == 0U) return 0U;

    /*
     * 磁力计先配置输出速率和量程，最后写MODE寄存器，
     * 使其进入连续转换模式。
     */
    if(SensorI2C_WriteRegister(LSM303_MAG_I2C_ADDRESS, LSM303_MAG_CRA_REG, LSM303_MAG_CRA_VALUE) == 0U) return 0U;
    if(SensorI2C_WriteRegister(LSM303_MAG_I2C_ADDRESS, LSM303_MAG_CRB_REG, LSM303_MAG_CRB_VALUE) == 0U) return 0U;
    if(SensorI2C_WriteRegister(LSM303_MAG_I2C_ADDRESS, LSM303_MAG_MODE_REG, LSM303_MAG_MODE_VALUE) == 0U) return 0U;

    /*
     * 回读三个关键寄存器：
     *
     * CTRL_REG1_A：确认10Hz采样和XYZ三轴使能；
     * CTRL_REG4_A：确认±2g、高分辨率和BDU生效；
     * MODE_REG_M：确认磁力计进入连续转换模式。
     */
    if(SensorI2C_ReadRegister(LSM303_ACCEL_I2C_ADDRESS, LSM303_ACCEL_CTRL_REG1, &register_value) == 0U) return 0U;
    if(register_value != LSM303_ACCEL_CTRL_REG1_VALUE) return 0U;

    if(SensorI2C_ReadRegister(LSM303_ACCEL_I2C_ADDRESS, LSM303_ACCEL_CTRL_REG4, &register_value) == 0U) return 0U;
    if(register_value != LSM303_ACCEL_CTRL_REG4_VALUE) return 0U;

    if(SensorI2C_ReadRegister(LSM303_MAG_I2C_ADDRESS, LSM303_MAG_MODE_REG, &register_value) == 0U) return 0U;
    if(register_value != LSM303_MAG_MODE_VALUE) return 0U;

    return 1U;
}

/*
 * 从低功耗状态恢复LSM303测量。
 *
 * 初始化时已经完成量程、分辨率和输出速率等配置，因此从休眠恢复时
 * 只需要重新启动加速度计采样，并让磁力计回到连续转换模式。
 */
uint8_t LSM303_StartMeasurement(void)
{
    if(SensorI2C_WriteRegister(LSM303_ACCEL_I2C_ADDRESS, LSM303_ACCEL_CTRL_REG1, LSM303_ACCEL_CTRL_REG1_VALUE) == 0U) return 0U;
    if(SensorI2C_WriteRegister(LSM303_MAG_I2C_ADDRESS, LSM303_MAG_MODE_REG, LSM303_MAG_MODE_VALUE) == 0U) return 0U;

    return 1U;
}

/*
 * 停止LSM303持续测量。
 *
 * CTRL_REG1_A写入0x07后，XYZ轴使能位仍然保留，但输出速率为0，
 * 加速度计采样部分进入掉电模式。
 *
 * MODE_REG_M写入0x03后，磁力计退出连续转换并进入休眠模式。
 * 离开CompassScreen时停止测量，可以减少指南针未使用期间的功耗。
 */
uint8_t LSM303_StopMeasurement(void)
{
    if(SensorI2C_WriteRegister(LSM303_MAG_I2C_ADDRESS, LSM303_MAG_MODE_REG, LSM303_MAG_SLEEP_VALUE) == 0U) return 0U;
    if(SensorI2C_WriteRegister(LSM303_ACCEL_I2C_ADDRESS, LSM303_ACCEL_CTRL_REG1, LSM303_ACCEL_CTRL_REG1_DEFAULT) == 0U) return 0U;

    return 1U;
}

/*
 * 连续读取加速度计和磁力计两组三轴数据。
 *
 * 加速度计和磁力计的数据寄存器布局不同：
 *
 * 加速度计：低字节在前，寄存器顺序为X、Y、Z；
 * 磁力计：高字节在前，寄存器顺序为X、Z、Y。
 *
 * 解析时必须按照各自实际顺序保存，否则会出现轴交换、方向反转或
 * 指南针旋转规律异常。
 */
uint8_t LSM303_ReadRawData(LSM303_RawData_t *raw_data)
{
    uint8_t accel_buffer[6] = {0};
    uint8_t mag_buffer[6] = {0};

    if(raw_data == NULL) return 0U;

    /*
     * 加速度计连续读取时，必须把起始寄存器Bit7置1以开启地址自动递增。
     * 实际发送的起始地址为0x28 | 0x80，也就是0xA8。
     */
    if(SensorI2C_ReadRegisters(LSM303_ACCEL_I2C_ADDRESS, LSM303_ACCEL_OUT_X_L | LSM303_ACCEL_AUTO_INCREMENT, accel_buffer, 6U) == 0U) return 0U;

    /*
     * 磁力计地址指针会自动递增，不需要额外设置Bit7。
     * 注意寄存器顺序为X、Z、Y，而不是通常理解的X、Y、Z。
     */
    if(SensorI2C_ReadRegisters(LSM303_MAG_I2C_ADDRESS, LSM303_MAG_OUT_X_H, mag_buffer, 6U) == 0U) return 0U;

    raw_data->accel_x = LSM303_DecodeAcceleration(accel_buffer[0], accel_buffer[1]);
    raw_data->accel_y = LSM303_DecodeAcceleration(accel_buffer[2], accel_buffer[3]);
    raw_data->accel_z = LSM303_DecodeAcceleration(accel_buffer[4], accel_buffer[5]);

    raw_data->mag_x = LSM303_DecodeMagnetic(mag_buffer[0], mag_buffer[1]);
    raw_data->mag_z = LSM303_DecodeMagnetic(mag_buffer[2], mag_buffer[3]);
    raw_data->mag_y = LSM303_DecodeMagnetic(mag_buffer[4], mag_buffer[5]);

    return 1U;
}
