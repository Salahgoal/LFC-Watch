#include "em7028.h"
#include "sensor_i2c.h"
#include <stddef.h>

#define EM7028_HRS_CONFIG_REGISTER       0x01U // 心率测量启动、停止和模式配置
#define EM7028_HRS2_OFFSET_REGISTER      0x08U // HRS2通道数据偏移配置
#define EM7028_HRS2_GAIN_REGISTER        0x0AU // HRS2通道增益配置
#define EM7028_HRS1_CONTROL_REGISTER     0x0DU // HRS1增益、量程和采样参数配置
#define EM7028_LED_CONTROL_REGISTER      0x0EU // LED驱动参数配置

#define EM7028_HRS1_DATA_LOW_REGISTER    0x28U // HRS1原始数据低字节
#define EM7028_HRS1_DATA_HIGH_REGISTER   0x29U // HRS1原始数据高字节

#define EM7028_MEASUREMENT_DISABLED      0x00U // 停止心率测量
#define EM7028_MEASUREMENT_ENABLED       0x08U // 启动心率测量
#define EM7028_HRS1_CONTROL_VALUE        0x47U // 当前使用的HRS1采样配置

uint8_t EM7028_IsReady(void)
{
    return SensorI2C_IsDeviceReady(EM7028_I2C_ADDRESS);
}

uint8_t EM7028_ReadDeviceID(uint8_t *device_id)
{
    if(device_id == NULL) return 0U;

    return SensorI2C_ReadRegister(EM7028_I2C_ADDRESS, EM7028_ID_REGISTER, device_id);
}

/*
 * 初始化EM7028光电采样模块。
 *
 * 处理流程：
 *
 * 1. 检查0x24地址是否返回ACK；
 * 2. 读取身份寄存器，确认设备ID为0x36；
 * 3. 保持测量关闭并配置光电模拟前端；
 * 4. 回读HRS1关键控制寄存器，确认配置生效。
 *
 * 初始化完成后不立即启动采样。SensorTask根据HomeScreen是否需要心率
 * 数据，调用StartMeasurement()或StopMeasurement()控制传感器功耗。
 */
uint8_t EM7028_Init(void)
{
    uint8_t device_id = 0U;
    uint8_t register_value = 0U;

    /*
     * I2C地址应答只能证明0x24上存在从机。
     * 只有身份寄存器返回0x36，才能进一步确认该设备是EM7028。
     */
    if(EM7028_IsReady() == 0U) return 0U;
    if(EM7028_ReadDeviceID(&device_id) == 0U) return 0U;
    if(device_id != EM7028_EXPECTED_ID) return 0U;

    /*
     * 配置过程中先关闭测量，避免模拟前端和LED参数尚未配置完整时
     * 产生无效采样。
     */
    if(SensorI2C_WriteRegister(EM7028_I2C_ADDRESS, EM7028_HRS_CONFIG_REGISTER, EM7028_MEASUREMENT_DISABLED) == 0U) return 0U;
    if(SensorI2C_WriteRegister(EM7028_I2C_ADDRESS, EM7028_HRS2_OFFSET_REGISTER, 0x00U) == 0U) return 0U;
    if(SensorI2C_WriteRegister(EM7028_I2C_ADDRESS, EM7028_HRS2_GAIN_REGISTER, 0x7FU) == 0U) return 0U;
    if(SensorI2C_WriteRegister(EM7028_I2C_ADDRESS, EM7028_HRS1_CONTROL_REGISTER, EM7028_HRS1_CONTROL_VALUE) == 0U) return 0U;
    if(SensorI2C_WriteRegister(EM7028_I2C_ADDRESS, EM7028_LED_CONTROL_REGISTER, 0x00U) == 0U) return 0U;

    /* 回读HRS1控制寄存器，确认关键采样配置已经真正写入芯片。 */
    if(SensorI2C_ReadRegister(EM7028_I2C_ADDRESS, EM7028_HRS1_CONTROL_REGISTER, &register_value) == 0U) return 0U;
    if(register_value != EM7028_HRS1_CONTROL_VALUE) return 0U;

    return 1U;
}

uint8_t EM7028_StartMeasurement(void)
{
    return SensorI2C_WriteRegister(EM7028_I2C_ADDRESS, EM7028_HRS_CONFIG_REGISTER, EM7028_MEASUREMENT_ENABLED);
}

uint8_t EM7028_StopMeasurement(void)
{
    return SensorI2C_WriteRegister(EM7028_I2C_ADDRESS, EM7028_HRS_CONFIG_REGISTER, EM7028_MEASUREMENT_DISABLED);
}

/*
 * 读取HRS1通道的一次16位PPG原始采样。
 *
 * PPG原始值表示光电接收通道测得的信号强度，它本身不是BPM。
 * 需要连续周期采样并识别脉搏波峰，才能根据相邻波峰的时间间隔
 * 计算心率。
 */
uint8_t EM7028_ReadRawData(uint16_t *raw_data)
{
    uint8_t high_byte = 0U;
    uint8_t low_byte = 0U;

    if(raw_data == NULL) return 0U;

    /*
     * HRS1结果由高、低两个寄存器组成。
     * 高字节左移8位后与低字节组合，得到完整的16位PPG采样值。
     */
    if(SensorI2C_ReadRegister(EM7028_I2C_ADDRESS, EM7028_HRS1_DATA_HIGH_REGISTER, &high_byte) == 0U) return 0U;
    if(SensorI2C_ReadRegister(EM7028_I2C_ADDRESS, EM7028_HRS1_DATA_LOW_REGISTER, &low_byte) == 0U) return 0U;

    *raw_data = ((uint16_t)high_byte << 8U) | low_byte;

    return 1U;
}
