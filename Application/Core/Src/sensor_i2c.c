#include "sensor_i2c.h"
#include "soft_i2c.h"

/*
 * 背板上的五个传感器共用同一组SDA和SCL。
 *
 * 总线对象集中定义在本文件中，避免每个传感器驱动重复保存PB13和PB14。
 * 以后即使硬件引脚发生改变，也只需要修改CubeMX中的引脚标签，
 * 各个传感器驱动不需要跟着修改。
 */
static const SoftI2C_Bus_t sensor_i2c_bus =
{
    SENSOR_SDA_GPIO_Port,
    SENSOR_SDA_Pin,
    SENSOR_SCL_GPIO_Port,
    SENSOR_SCL_Pin
};

/*
 * 将7位设备地址转换成I2C总线上实际发送的地址字节。
 *
 * 数据手册通常使用0x68、0x38这类7位地址，而I2C传输时需要将地址
 * 左移一位，再使用最低位表示读写方向：
 *
 * 写地址 = 7位地址 << 1
 * 读地址 = (7位地址 << 1) | 1
 *
 * 对外接口始终接收7位地址，可以避免不同驱动混用7位地址和8位地址。
 */
static uint8_t SensorI2C_GetWriteAddress(uint8_t dev_addr)
{
    return (uint8_t)(dev_addr << 1U);
}

static uint8_t SensorI2C_GetReadAddress(uint8_t dev_addr)
{
    return (uint8_t)((dev_addr << 1U) | 0x01U);
}

void SensorI2C_Init(void)
{
    SoftI2C_Init(&sensor_i2c_bus);
}

/*
 * 检查指定地址上的设备是否能够正常应答。
 *
 * 主机只发送设备写地址，不继续发送寄存器地址或数据。
 * 从机返回ACK表示该地址上存在能够正常通信的设备。
 *
 * @param dev_addr 传感器的7位I2C地址
 * @retval 1U=设备返回ACK，0U=设备未返回ACK
 */
uint8_t SensorI2C_IsDeviceReady(uint8_t dev_addr)
{
    uint8_t result;

    SoftI2C_Start(&sensor_i2c_bus);
    SoftI2C_WriteByte(&sensor_i2c_bus, SensorI2C_GetWriteAddress(dev_addr));
    result = SoftI2C_WaitAck(&sensor_i2c_bus);
    SoftI2C_Stop(&sensor_i2c_bus);

    if(result != 0U) return 0U;

    return 1U;
}

/*
 * 向传感器的指定寄存器写入一个字节。
 *
 * 通信顺序：
 * START -> 设备写地址 -> 寄存器地址 -> 数据 -> STOP
 *
 * 每发送一个字节都必须等待从机ACK。任意阶段没有收到ACK时，
 * 立即发送STOP释放总线，并向调用者返回失败。
 *
 * @param dev_addr 传感器的7位I2C地址
 * @param reg_addr 目标寄存器地址
 * @param data     要写入的一个字节
 * @retval 1U=写入成功，0U=通信失败
 */
uint8_t SensorI2C_WriteRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    SoftI2C_Start(&sensor_i2c_bus);

    SoftI2C_WriteByte(&sensor_i2c_bus, SensorI2C_GetWriteAddress(dev_addr));
    if(SoftI2C_WaitAck(&sensor_i2c_bus) != 0U)
    {
        SoftI2C_Stop(&sensor_i2c_bus);
        return 0U;
    }

    SoftI2C_WriteByte(&sensor_i2c_bus, reg_addr);
    if(SoftI2C_WaitAck(&sensor_i2c_bus) != 0U)
    {
        SoftI2C_Stop(&sensor_i2c_bus);
        return 0U;
    }

    SoftI2C_WriteByte(&sensor_i2c_bus, data);
    if(SoftI2C_WaitAck(&sensor_i2c_bus) != 0U)
    {
        SoftI2C_Stop(&sensor_i2c_bus);
        return 0U;
    }

    SoftI2C_Stop(&sensor_i2c_bus);
    return 1U;
}

uint8_t SensorI2C_ReadRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data)
{
    return SensorI2C_ReadRegisters(dev_addr, reg_addr, data, 1U);
}

/*
 * 从传感器的连续寄存器中读取一个或多个字节。
 *
 * 寄存器读取分为两个阶段：
 *
 * 1. 使用写模式发送寄存器地址，设置从机内部地址指针；
 * 2. 发送重复起始信号，切换到读模式并读取数据。
 *
 * 读取前面的数据字节后发送ACK，告诉从机继续发送；读取最后一个
 * 字节后发送NACK，再产生STOP，表示本次连续读取结束。
 *
 * 通信顺序：
 * START -> 设备写地址 -> 寄存器地址
 * RESTART -> 设备读地址 -> 数据 -> NACK -> STOP
 *
 * @param dev_addr 传感器的7位I2C地址
 * @param reg_addr 连续读取的起始寄存器地址
 * @param data     用于保存读取结果的缓冲区
 * @param length   需要读取的字节数
 * @retval 1U=读取成功，0U=参数错误或通信失败
 */
uint8_t SensorI2C_ReadRegisters(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t length)
{
    uint8_t i;

    if(data == NULL || length == 0U) return 0U;

    SoftI2C_Start(&sensor_i2c_bus);

    SoftI2C_WriteByte(&sensor_i2c_bus, SensorI2C_GetWriteAddress(dev_addr));
    if(SoftI2C_WaitAck(&sensor_i2c_bus) != 0U)
    {
        SoftI2C_Stop(&sensor_i2c_bus);
        return 0U;
    }

    SoftI2C_WriteByte(&sensor_i2c_bus, reg_addr);
    if(SoftI2C_WaitAck(&sensor_i2c_bus) != 0U)
    {
        SoftI2C_Stop(&sensor_i2c_bus);
        return 0U;
    }

    SoftI2C_Start(&sensor_i2c_bus);

    SoftI2C_WriteByte(&sensor_i2c_bus, SensorI2C_GetReadAddress(dev_addr));
    if(SoftI2C_WaitAck(&sensor_i2c_bus) != 0U)
    {
        SoftI2C_Stop(&sensor_i2c_bus);
        return 0U;
    }

    for(i = 0U; i < length; i++)
    {
        data[i] = SoftI2C_ReadByte(&sensor_i2c_bus);

        if(i < (uint8_t)(length - 1U))
        {
            SoftI2C_SendAck(&sensor_i2c_bus); // 还有数据需要读取
        }
        else
        {
            SoftI2C_SendNack(&sensor_i2c_bus); // 当前字节是最后一个数据
        }
    }

    SoftI2C_Stop(&sensor_i2c_bus);
    return 1U;
}

/*
 * 向传感器直接发送一个或多个命令字节。
 *
 * 某些传感器采用命令式通信，不存在独立的寄存器地址阶段。
 * 主机发送设备写地址后，直接依次发送调用者提供的数据。
 *
 * AHT21的初始化命令和测量命令都使用这种通信方式。
 *
 * 通信顺序：
 * START -> 设备写地址 -> 命令数据 -> STOP
 *
 * @param dev_addr 传感器的7位I2C地址
 * @param data     需要发送的数据缓冲区
 * @param length   需要发送的字节数
 * @retval 1U=发送成功，0U=参数错误或通信失败
 */
uint8_t SensorI2C_WriteData(uint8_t dev_addr, const uint8_t *data, uint8_t length)
{
    uint8_t i;

    if(data == NULL || length == 0U) return 0U;

    SoftI2C_Start(&sensor_i2c_bus);

    SoftI2C_WriteByte(&sensor_i2c_bus, SensorI2C_GetWriteAddress(dev_addr));
    if(SoftI2C_WaitAck(&sensor_i2c_bus) != 0U)
    {
        SoftI2C_Stop(&sensor_i2c_bus);
        return 0U;
    }

    for(i = 0U; i < length; i++)
    {
        SoftI2C_WriteByte(&sensor_i2c_bus, data[i]);

        if(SoftI2C_WaitAck(&sensor_i2c_bus) != 0U)
        {
            SoftI2C_Stop(&sensor_i2c_bus);
            return 0U;
        }
    }

    SoftI2C_Stop(&sensor_i2c_bus);
    return 1U;
}

/*
 * 从传感器直接读取一个或多个数据字节。
 *
 * 该接口不提前发送寄存器地址，适用于直接返回状态或测量结果的设备。
 * 除最后一个字节外，每读取一个字节都发送ACK；最后一个字节发送
 * NACK，再产生STOP结束通信。
 *
 * AHT21的状态读取和测量结果读取都使用这种通信方式。
 *
 * 通信顺序：
 * START -> 设备读地址 -> 数据 -> NACK -> STOP
 *
 * @param dev_addr 传感器的7位I2C地址
 * @param data     用于保存读取结果的缓冲区
 * @param length   需要读取的字节数
 * @retval 1U=读取成功，0U=参数错误或通信失败
 */
uint8_t SensorI2C_ReadData(uint8_t dev_addr, uint8_t *data, uint8_t length)
{
    uint8_t i;

    if(data == NULL || length == 0U) return 0U;

    SoftI2C_Start(&sensor_i2c_bus);

    SoftI2C_WriteByte(&sensor_i2c_bus, SensorI2C_GetReadAddress(dev_addr));
    if(SoftI2C_WaitAck(&sensor_i2c_bus) != 0U)
    {
        SoftI2C_Stop(&sensor_i2c_bus);
        return 0U;
    }

    for(i = 0U; i < length; i++)
    {
        data[i] = SoftI2C_ReadByte(&sensor_i2c_bus);

        if(i < (uint8_t)(length - 1U))
        {
            SoftI2C_SendAck(&sensor_i2c_bus); // 还有数据需要读取
        }
        else
        {
            SoftI2C_SendNack(&sensor_i2c_bus); // 当前字节是最后一个数据
        }
    }

    SoftI2C_Stop(&sensor_i2c_bus);
    return 1U;
}
