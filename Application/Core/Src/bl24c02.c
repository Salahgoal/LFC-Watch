#include "bl24c02.h"
#include "soft_i2c.h"

/*
 * BL24C02使用7位I2C设备地址0x50。
 * 实际发送到总线上的地址字节需要将7位地址左移一位，再用最低位表示传输方向：
 * - 最低位为0：写操作，地址字节为0xA0；
 * - 最低位为1：读操作，地址字节为0xA1。
 */
#define BL24C02_ADDRESS_7BIT  0x50U
#define BL24C02_WRITE_ADDRESS (BL24C02_ADDRESS_7BIT << 1U)
#define BL24C02_READ_ADDRESS  ((BL24C02_ADDRESS_7BIT << 1U) | 0x01U)

/*
 * BL24C02使用独立的软件I2C总线配置。
 * 将GPIO端口和引脚封装成SoftI2C_Bus_t后，底层软件I2C代码可以复用于
 * EEPROM以及其他使用不同GPIO的软件I2C设备。
 */
static const SoftI2C_Bus_t bl24c02_bus =
{
    EEPROM_SDA_GPIO_Port, EEPROM_SDA_Pin, // EEPROM数据线
    EEPROM_SCL_GPIO_Port, EEPROM_SCL_Pin  // EEPROM时钟线
};

/**
 * @brief 初始化BL24C02使用的软件I2C总线
 *
 * 初始化过程会释放SDA和SCL，使两条线路都由上拉电阻保持为高电平，
 * 让总线进入空闲状态。EEPROM本身不需要额外的寄存器初始化。
 *
 * @retval None
 */
void BL24C02_Init(void)
{
    SoftI2C_Init(&bl24c02_bus);
}

/**
 * @brief 向BL24C02指定地址写入一个字节
 *
 * BL24C02单字节写入流程：
 * 1. 发送START信号；
 * 2. 发送设备写地址0xA0，选择BL24C02并进入写模式；
 * 3. 发送EEPROM内部存储地址；
 * 4. 发送需要保存的数据；
 * 5. 发送STOP信号，启动EEPROM内部写周期。
 *
 * 设备地址、存储地址和数据发送完成后都必须检查ACK。任意阶段没有收到
 * ACK，都说明本次通信失败，需要先发送STOP释放总线，再返回失败。
 *
 * STOP信号后，EEPROM还需要把接收到的数据写入内部非易失存储单元。
 * 在此期间芯片处于忙状态，因此等待10ms后再进行下一次访问。
 *
 * @param memory_address EEPROM内部字节地址，范围为0x00~0xFF
 * @param data 要写入的数据
 * @retval 1U=写入流程成功，0U=某一阶段未收到ACK
 */
uint8_t BL24C02_WriteByte(uint8_t memory_address, uint8_t data)
{
    SoftI2C_Start(&bl24c02_bus);

    /* 发送设备写地址，选择BL24C02并进入写模式。 */
    SoftI2C_WriteByte(&bl24c02_bus, BL24C02_WRITE_ADDRESS);
    if(SoftI2C_WaitAck(&bl24c02_bus) != 0U)
    {
        SoftI2C_Stop(&bl24c02_bus);
        return 0U; // EEPROM没有响应设备地址
    }

    /* 设置EEPROM内部地址指针，指定数据的保存位置。 */
    SoftI2C_WriteByte(&bl24c02_bus, memory_address);
    if(SoftI2C_WaitAck(&bl24c02_bus) != 0U)
    {
        SoftI2C_Stop(&bl24c02_bus);
        return 0U; // EEPROM没有接收存储地址
    }

    /* 发送需要写入指定存储地址的数据。 */
    SoftI2C_WriteByte(&bl24c02_bus, data);
    if(SoftI2C_WaitAck(&bl24c02_bus) != 0U)
    {
        SoftI2C_Stop(&bl24c02_bus);
        return 0U; // EEPROM没有接收待写数据
    }

    SoftI2C_Stop(&bl24c02_bus);

    /*
     * STOP信号会启动EEPROM内部非易失写周期。
     * 等待写周期完成，避免下一次访问发生在芯片忙状态期间。
     */
    HAL_Delay(10U);

    return 1U;
}

/**
 * @brief 从BL24C02指定地址读取一个字节
 *
 * 本函数采用I2C随机读取流程：
 * 1. 发送START和设备写地址0xA0；
 * 2. 发送目标存储地址，设置EEPROM内部地址指针；
 * 3. 发送重复START，不释放当前总线；
 * 4. 发送设备读地址0xA1，将通信方向切换为读取；
 * 5. 读取一个字节；
 * 6. 主机发送NACK，表示不再读取后续数据；
 * 7. 发送STOP结束通信。
 *
 * 前两个阶段使用写模式不是向EEPROM写入数据，而是为了设置内部地址指针。
 * 重复START可以在不结束当前事务的情况下，将总线从写方向切换到读方向。
 *
 * @param memory_address EEPROM内部字节地址，范围为0x00~0xFF
 * @param data 用于接收读取结果的变量地址
 * @retval 1U=读取成功，0U=参数错误或某一阶段未收到ACK
 */
uint8_t BL24C02_ReadByte(uint8_t memory_address, uint8_t *data)
{
    if(data == NULL) return 0U;

    /*
     * 先使用写方向发送目标存储地址。
     * 该阶段只设置EEPROM内部地址指针，不会改写存储内容。
     */
    SoftI2C_Start(&bl24c02_bus);

    SoftI2C_WriteByte(&bl24c02_bus, BL24C02_WRITE_ADDRESS);
    if(SoftI2C_WaitAck(&bl24c02_bus) != 0U)
    {
        SoftI2C_Stop(&bl24c02_bus);
        return 0U; // EEPROM没有响应设备写地址
    }

    SoftI2C_WriteByte(&bl24c02_bus, memory_address);
    if(SoftI2C_WaitAck(&bl24c02_bus) != 0U)
    {
        SoftI2C_Stop(&bl24c02_bus);
        return 0U; // EEPROM没有接收目标存储地址
    }

    /*
     * 发送重复START并切换到读方向。
     * 此时EEPROM会从刚才设置的内部地址开始输出数据。
     */
    SoftI2C_Start(&bl24c02_bus);

    SoftI2C_WriteByte(&bl24c02_bus, BL24C02_READ_ADDRESS);
    if(SoftI2C_WaitAck(&bl24c02_bus) != 0U)
    {
        SoftI2C_Stop(&bl24c02_bus);
        return 0U; // EEPROM没有响应设备读地址
    }

    *data = SoftI2C_ReadByte(&bl24c02_bus);

    /*
     * 本次只读取一个字节，因此主机发送NACK表示读取结束，
     * 随后发送STOP释放总线。
     */
    SoftI2C_SendNack(&bl24c02_bus);
    SoftI2C_Stop(&bl24c02_bus);

    return 1U;
}
