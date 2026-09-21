#include "aht21.h"
#include "sensor_i2c.h"
#include <stddef.h>

uint8_t AHT21_IsReady(void)
{
    return SensorI2C_IsDeviceReady(AHT21_I2C_ADDRESS);
}

uint8_t AHT21_ReadStatus(uint8_t *status)
{
    if(status == NULL) return 0U;

    /*
     * AHT21的状态读取不使用寄存器地址。
     *
     * 主机直接发送设备读地址，然后读取一个状态字节。
     * SensorI2C_ReadData()接收的是7位地址0x38，并在底层自动添加读写位，
     * 因此这里不能手动把地址左移，也不能直接传入0x71。
     */
    return SensorI2C_ReadData(AHT21_I2C_ADDRESS, status, 1U);
}

uint8_t AHT21_IsBusy(uint8_t status)
{
    return ((status & AHT21_STATUS_BUSY_MASK) != 0U) ? 1U : 0U;
}

uint8_t AHT21_IsCalibrated(uint8_t status)
{
    return ((status & AHT21_STATUS_CALIBRATED_MASK) != 0U) ? 1U : 0U;
}

uint8_t AHT21_SendInitCommand(void)
{
    static const uint8_t command[3] = {0xBEU, 0x08U, 0x00U};

    /*
     * 状态字节中的校准使能位未置位时，需要发送初始化命令。
     *
     * AHT21接收命令后需要一定时间完成内部初始化。因此本函数只发送命令，
     * 调用任务不能立即读取状态，应等待至少10ms后再检查校准使能位。
     *
     * 等待过程应由SensorTask的执行流程控制，驱动层不负责操作FreeRTOS延时。
     */
    return SensorI2C_WriteData(AHT21_I2C_ADDRESS, command, (uint8_t)sizeof(command));
}

uint8_t AHT21_SendMeasureCommand(void)
{
    static const uint8_t command[3] = {0xACU, 0x33U, 0x00U};

    /*
     * 该命令用于触发一次新的温湿度测量。
     *
     * 传感器收到命令后，需要依次进行采集和内部计算。在此期间状态字节
     * 的Busy位会保持为1，因此不能发送命令后立即读取测量数据。
     */
    return SensorI2C_WriteData(AHT21_I2C_ADDRESS, command, (uint8_t)sizeof(command));
}

uint8_t AHT21_ReadMeasurement(AHT21_RawData_t *raw_data, uint8_t *status)
{
    uint8_t buffer[6];

    if(raw_data == NULL || status == NULL) return 0U;
    if(SensorI2C_ReadData(AHT21_I2C_ADDRESS, buffer, (uint8_t)sizeof(buffer)) == 0U) return 0U;

    *status = buffer[0];

    /*
     * 状态字节Bit7为1表示传感器仍在测量。
     *
     * 此时后面的数据还没有准备完成，不能将其当作有效结果。
     * 状态字节仍然通过status返回，方便调用者判断本次失败原因。
     */
    if(AHT21_IsBusy(buffer[0]) != 0U) return 0U;

    /*
     * AHT21返回的6字节数据排列：
     *
     * buffer[0]：状态字节
     * buffer[1]：湿度[19:12]
     * buffer[2]：湿度[11:4]
     * buffer[3]：高4位为湿度[3:0]，低4位为温度[19:16]
     * buffer[4]：温度[15:8]
     * buffer[5]：温度[7:0]
     *
     * 湿度和温度各占20位，并在buffer[3]中共用一个字节，因此需要通过
     * 移位和按位或操作，分别重新组合成两个完整的原始值。
     */
    raw_data->humidity_raw = (uint32_t)buffer[1] << 12U;
    raw_data->humidity_raw |= (uint32_t)buffer[2] << 4U;
    raw_data->humidity_raw |= (uint32_t)buffer[3] >> 4U;

    raw_data->temperature_raw = (uint32_t)(buffer[3] & 0x0FU) << 16U;
    raw_data->temperature_raw |= (uint32_t)buffer[4] << 8U;
    raw_data->temperature_raw |= (uint32_t)buffer[5];

    return 1U;
}

