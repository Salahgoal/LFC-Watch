#include "spl06.h"
#include "sensor_i2c.h"
#include <stddef.h>

#define SPL06_CALIBRATION_START_REGISTER     0x10U // 校准系数起始寄存器
#define SPL06_CALIBRATION_LENGTH             18U   // 全部校准数据长度，单位字节

#define SPL06_PRESSURE_DATA_START_REGISTER   0x00U // 气压原始数据起始寄存器
#define SPL06_PRESSURE_CONFIG_REGISTER       0x06U // 气压测量配置寄存器
#define SPL06_TEMPERATURE_CONFIG_REGISTER    0x07U // 温度测量配置寄存器
#define SPL06_MEASUREMENT_CONFIG_REGISTER    0x08U // 测量模式和数据状态寄存器
#define SPL06_GENERAL_CONFIG_REGISTER        0x09U // 数据移位和中断配置寄存器
#define SPL06_DATA_READY_MASK                0x30U // Bit5温度就绪，Bit4气压就绪
#define SPL06_RAW_DATA_LENGTH                6U    // 气压3字节加温度3字节

#define SPL06_MEASUREMENT_IDLE_VALUE         0x00U // 空闲模式
#define SPL06_MEASUREMENT_CONTINUOUS_VALUE   0x07U // 气压和温度连续测量模式

uint8_t SPL06_IsReady(void)
{
    return SensorI2C_IsDeviceReady(SPL06_I2C_ADDRESS);
}

uint8_t SPL06_ReadDeviceID(uint8_t *dev_id)
{
    if(dev_id == NULL) return 0U;

    return SensorI2C_ReadRegister(SPL06_I2C_ADDRESS, SPL06_ID_REGISTER, dev_id);
}

/*
 * 将指定位宽的二进制补码转换成int32_t。
 *
 * SPL06内部并不是所有校准系数都按照完整的16位或32位保存：
 * c0、c1为12位，c00、c10为20位，原始测量数据为24位。
 *
 * 例如12位数据0xFFF实际代表-1。如果直接转换成普通整数，
 * 它会被解释为4095，因此必须根据原始数据的位宽进行符号扩展。
 */
static int32_t SPL06_SignExtend(uint32_t value, uint8_t bit_width)
{
    uint32_t value_range = 1UL << bit_width;
    uint32_t sign_bit = 1UL << (bit_width - 1U);

    value &= value_range - 1UL; // 只保留指定位宽内的有效数据

    if((value & sign_bit) != 0U) return -(int32_t)(value_range - value);

    return (int32_t)value;
}

/*
 * 读取并解析SPL06的18字节出厂校准数据。
 *
 * 校准寄存器中的数据排列：
 *
 * buffer[0]：c0[11:4]
 * buffer[1]：高4位为c0[3:0]，低4位为c1[11:8]
 * buffer[2]：c1[7:0]
 *
 * buffer[3]：c00[19:12]
 * buffer[4]：c00[11:4]
 * buffer[5]：高4位为c00[3:0]，低4位为c10[19:16]
 * buffer[6]：c10[15:8]
 * buffer[7]：c10[7:0]
 *
 * buffer[8]～buffer[17]依次保存c01、c11、c20、c21和c30，
 * 每个系数占两个字节。
 *
 * 解析流程：
 * 读取连续寄存器 -> 移位组合分散的数据位 -> 按实际位宽符号扩展
 * -> 保存到SPL06_Calibration_t。
 */
uint8_t SPL06_ReadCalibration(SPL06_Calibration_t *calibration)
{
    uint8_t buffer[SPL06_CALIBRATION_LENGTH] = {0};
    uint32_t packed_value = 0U;

    if(calibration == NULL) return 0U;
    if(SensorI2C_ReadRegisters(SPL06_I2C_ADDRESS, SPL06_CALIBRATION_START_REGISTER, buffer, SPL06_CALIBRATION_LENGTH) == 0U) return 0U;

    /*
     * c0和c1各占12位，并且共用buffer[1]：
     * buffer[1]高4位属于c0，低4位属于c1。
     */
    packed_value = ((uint32_t)buffer[0U] << 4U) | ((uint32_t)buffer[1U] >> 4U);
    calibration->c0 = (int16_t)SPL06_SignExtend(packed_value, 12U);

    packed_value = ((uint32_t)(buffer[1U] & 0x0FU) << 8U) | (uint32_t)buffer[2U];
    calibration->c1 = (int16_t)SPL06_SignExtend(packed_value, 12U);

    /*
     * c00和c10各占20位，并且共用buffer[5]：
     * buffer[5]高4位属于c00，低4位属于c10。
     */
    packed_value = ((uint32_t)buffer[3U] << 12U) | ((uint32_t)buffer[4U] << 4U) | ((uint32_t)buffer[5U] >> 4U);
    calibration->c00 = SPL06_SignExtend(packed_value, 20U);

    packed_value = ((uint32_t)(buffer[5U] & 0x0FU) << 16U) | ((uint32_t)buffer[6U] << 8U) | (uint32_t)buffer[7U];
    calibration->c10 = SPL06_SignExtend(packed_value, 20U);

    /* 后面的五个系数均为连续存放的16位有符号数。 */
    calibration->c01 = (int16_t)SPL06_SignExtend(((uint32_t)buffer[8U] << 8U) | buffer[9U], 16U);
    calibration->c11 = (int16_t)SPL06_SignExtend(((uint32_t)buffer[10U] << 8U) | buffer[11U], 16U);
    calibration->c20 = (int16_t)SPL06_SignExtend(((uint32_t)buffer[12U] << 8U) | buffer[13U], 16U);
    calibration->c21 = (int16_t)SPL06_SignExtend(((uint32_t)buffer[14U] << 8U) | buffer[15U], 16U);
    calibration->c30 = (int16_t)SPL06_SignExtend(((uint32_t)buffer[16U] << 8U) | buffer[17U], 16U);

    return 1U;
}

/*
 * 配置SPL06并启动气压、温度连续测量。
 *
 * PRS_CFG = 0x01：
 * 气压测量频率为1Hz，每个测量结果使用2倍过采样。
 *
 * TMP_CFG = 0x80：
 * 使用当前配置对应的温度数据源，温度测量频率为1Hz，
 * 每个测量结果使用单次采样。
 *
 * CFG_REG = 0x00：
 * 当前不使用中断和FIFO。气压、温度过采样均未超过8倍，
 * 因此不需要开启结果移位。
 *
 * MEAS_CFG = 0x07：
 * MEAS_CTRL[2:0]写入111，启动气压和温度连续测量。
 *
 * Service层使用的原始数据缩放因子必须与这里配置的过采样率一致。
 * 如果以后修改过采样率，也要同步修改spl06_service.c中的缩放因子。
 */
uint8_t SPL06_StartContinuousMeasurement(void)
{
    if(SensorI2C_WriteRegister(SPL06_I2C_ADDRESS, SPL06_PRESSURE_CONFIG_REGISTER, 0x01U) == 0U) return 0U;
    if(SensorI2C_WriteRegister(SPL06_I2C_ADDRESS, SPL06_TEMPERATURE_CONFIG_REGISTER, 0x80U) == 0U) return 0U;
    if(SensorI2C_WriteRegister(SPL06_I2C_ADDRESS, SPL06_GENERAL_CONFIG_REGISTER, 0x00U) == 0U) return 0U;
    if(SensorI2C_WriteRegister(SPL06_I2C_ADDRESS, SPL06_MEASUREMENT_CONFIG_REGISTER, SPL06_MEASUREMENT_CONTINUOUS_VALUE) == 0U) return 0U;

    return 1U;
}

/*
 * 停止SPL06温度和气压连续转换。
 *
 * MEAS_CFG寄存器低三位决定测量模式，写入000后芯片进入空闲状态。
 * 已经读取到MCU内存中的出厂校准系数，以及芯片中的气压和温度配置，
 * 不会因为停止测量而丢失。
 *
 * 离开EnvScreen时停止测量，可以减少不需要环境数据时的功耗。
 * 下次进入页面重新调用SPL06_StartContinuousMeasurement()即可恢复测量。
 */
uint8_t SPL06_StopMeasurement(void)
{
    return SensorI2C_WriteRegister(SPL06_I2C_ADDRESS, SPL06_MEASUREMENT_CONFIG_REGISTER, SPL06_MEASUREMENT_IDLE_VALUE);
}

/*
 * 读取一组完整的气压和温度原始数据。
 *
 * MEAS_CFG的Bit5和Bit4分别表示温度与气压数据是否就绪。只有两种数据
 * 全部准备好后才连续读取6字节，确保本次气压和温度属于同一测量阶段，
 * 后续可以进行完整的温度补偿和气压补偿。
 *
 * 返回数据排列：
 *
 * buffer[0]：气压[23:16]
 * buffer[1]：气压[15:8]
 * buffer[2]：气压[7:0]
 * buffer[3]：温度[23:16]
 * buffer[4]：温度[15:8]
 * buffer[5]：温度[7:0]
 *
 * 两个原始量都是24位二进制补码。组合三个字节后，需要进行24位
 * 符号扩展，否则负数会被错误解释为很大的正数。
 */
uint8_t SPL06_ReadRawData(SPL06_RawData_t *raw_data)
{
    uint8_t measurement_status = 0U;
    uint8_t buffer[SPL06_RAW_DATA_LENGTH] = {0};
    uint32_t packed_value = 0U;

    if(raw_data == NULL) return 0U;
    if(SensorI2C_ReadRegister(SPL06_I2C_ADDRESS, SPL06_MEASUREMENT_CONFIG_REGISTER, &measurement_status) == 0U) return 0U;
    if((measurement_status & SPL06_DATA_READY_MASK) != SPL06_DATA_READY_MASK) return 0U;

    if(SensorI2C_ReadRegisters(SPL06_I2C_ADDRESS, SPL06_PRESSURE_DATA_START_REGISTER, buffer, SPL06_RAW_DATA_LENGTH) == 0U) return 0U;

    packed_value = ((uint32_t)buffer[0U] << 16U) | ((uint32_t)buffer[1U] << 8U) | buffer[2U];
    raw_data->pressure_raw = SPL06_SignExtend(packed_value, 24U);

    packed_value = ((uint32_t)buffer[3U] << 16U) | ((uint32_t)buffer[4U] << 8U) | buffer[5U];
    raw_data->temperature_raw = SPL06_SignExtend(packed_value, 24U);

    return 1U;
}
