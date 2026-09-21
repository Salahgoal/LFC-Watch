#include "soft_i2c.h"

/**
 * @brief 提供软件I2C所需的短延时
 *
 * 软件I2C使用GPIO手动产生SDA和SCL波形。每次改变引脚电平后都需要
 * 等待一小段时间，使信号稳定并满足从机对建立时间和保持时间的要求。
 *
 * 循环变量使用volatile修饰，防止编译器认为空循环没有作用而将其删除。
 * 该延时长度与系统主频和编译器优化等级有关，并不是精确的时间基准。
 *
 * @retval None
 */
static void SoftI2C_Delay(void)
{
    volatile uint8_t i; // 延时循环计数器

    for(i = 0U; i < 20U; i++)
    {
    }
}

/**
 * @brief 初始化一组软件I2C总线
 *
 * SDA和SCL配置为开漏输出。向引脚写入GPIO_PIN_SET并不是主动输出高电平，
 * 而是释放总线，由上拉电阻将线路拉高。初始化时释放两条线路，使总线
 * 回到I2C规定的空闲状态：SDA和SCL均为高电平。
 *
 * @param bus 软件I2C总线配置，包含SDA和SCL的GPIO端口及引脚
 * @retval None
 */
void SoftI2C_Init(const SoftI2C_Bus_t *bus)
{
    HAL_GPIO_WritePin(bus->sda_port, bus->sda_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, GPIO_PIN_SET);
    SoftI2C_Delay();
}

/**
 * @brief 产生I2C起始信号
 *
 * I2C总线空闲时，SDA和SCL均为高电平。主机在SCL保持高电平期间，
 * 将SDA从高电平拉到低电平，便形成START条件。随后拉低SCL，
 * 为发送设备地址或数据做好准备。
 *
 * 本函数也可以在一次通信尚未结束时调用，此时产生的是重复起始信号，
 * 常用于EEPROM随机读取过程中由写地址阶段切换到读数据阶段。
 *
 * @param bus 使用的软件I2C总线
 * @retval None
 */
void SoftI2C_Start(const SoftI2C_Bus_t *bus)
{
    HAL_GPIO_WritePin(bus->sda_port, bus->sda_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, GPIO_PIN_SET);
    SoftI2C_Delay();

    HAL_GPIO_WritePin(bus->sda_port, bus->sda_pin, GPIO_PIN_RESET);
    SoftI2C_Delay();

    HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, GPIO_PIN_RESET);
    SoftI2C_Delay();
}

/**
 * @brief 产生I2C停止信号
 *
 * 主机先在SCL为低电平时拉低SDA，然后将SCL释放为高电平；
 * 在SCL保持高电平期间，再将SDA从低电平释放为高电平，
 * 便形成STOP条件并结束本次I2C通信。
 *
 * @param bus 使用的软件I2C总线
 * @retval None
 */
void SoftI2C_Stop(const SoftI2C_Bus_t *bus)
{
    HAL_GPIO_WritePin(bus->sda_port, bus->sda_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, GPIO_PIN_SET);
    SoftI2C_Delay();

    HAL_GPIO_WritePin(bus->sda_port, bus->sda_pin, GPIO_PIN_SET);
    SoftI2C_Delay();
}

/**
 * @brief 通过软件I2C发送一个字节
 *
 * I2C按照最高位优先的顺序发送数据。每一位的传输过程如下：
 * 1. 拉低SCL，在时钟低电平期间设置SDA；
 * 2. 根据当前最高位决定释放SDA还是拉低SDA；
 * 3. 拉高SCL，让从机在时钟高电平期间采样SDA；
 * 4. 数据左移一位，继续发送下一位。
 *
 * 8位数据发送完成后将SCL保持为低电平。调用者随后应调用
 * SoftI2C_WaitAck()，产生第9个时钟并检查从机应答。
 *
 * @param bus 使用的软件I2C总线
 * @param data 待发送的8位数据
 * @retval None
 */
void SoftI2C_WriteByte(const SoftI2C_Bus_t *bus, uint8_t data)
{
    uint8_t i; // 已发送的数据位计数

    for(i = 0U; i < 8U; i++)
    {
        HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, GPIO_PIN_RESET);

        if((data & 0x80U) != 0U)
        {
            HAL_GPIO_WritePin(bus->sda_port, bus->sda_pin, GPIO_PIN_SET); // 释放SDA，发送逻辑1
        }
        else
        {
            HAL_GPIO_WritePin(bus->sda_port, bus->sda_pin, GPIO_PIN_RESET); // 拉低SDA，发送逻辑0
        }

        data <<= 1U; // 将下一位移动到最高位
        SoftI2C_Delay();

        HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, GPIO_PIN_SET); // 从机在SCL高电平期间采样SDA
        SoftI2C_Delay();
    }

    HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, GPIO_PIN_RESET);
}

/**
 * @brief 等待从机返回ACK应答
 *
 * 主机发送完8位数据后释放SDA，并拉高SCL产生第9个时钟。
 * 如果从机正确接收到设备地址或数据，会主动拉低SDA：
 *
 * - SDA为低电平：收到ACK，通信可以继续；
 * - SDA保持高电平：未收到ACK，可能是地址错误、从机未连接或从机正忙。
 *
 * timeout用于避免从机没有应答时程序永久停留在等待循环中。
 *
 * @param bus 使用的软件I2C总线
 * @retval 0U=收到ACK，1U=等待超时且未收到ACK
 */
uint8_t SoftI2C_WaitAck(const SoftI2C_Bus_t *bus)
{
    uint16_t timeout = 0U; // ACK等待循环计数器

    HAL_GPIO_WritePin(bus->sda_port, bus->sda_pin, GPIO_PIN_SET); // 释放SDA，交给从机控制
    SoftI2C_Delay();

    HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, GPIO_PIN_SET); // 产生第9个时钟
    SoftI2C_Delay();

    while(HAL_GPIO_ReadPin(bus->sda_port, bus->sda_pin) == GPIO_PIN_SET)
    {
        timeout++;

        if(timeout > 500U)
        {
            HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, GPIO_PIN_RESET);
            return 1U; // 超时后结束本次ACK等待
        }
    }

    HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, GPIO_PIN_RESET);
    return 0U;
}

/**
 * @brief 通过软件I2C读取一个字节
 *
 * 读取数据前，主机必须释放SDA，由从机控制数据线。每一位的读取过程如下：
 * 1. 拉低SCL，准备读取下一位；
 * 2. 拉高SCL，使从机输出的数据位稳定；
 * 3. 读取SDA电平，并把该位组合到data中。
 *
 * I2C按照最高位优先传输，因此每次读取前先将data左移一位，
 * 再根据SDA电平决定是否把最低位置1。
 *
 * 读取完8位后，本函数只返回数据，不自动发送应答。调用者需要根据
 * 是否继续读取，选择调用SoftI2C_SendAck()或SoftI2C_SendNack()。
 *
 * @param bus 使用的软件I2C总线
 * @retval 从机发送的8位数据
 */
uint8_t SoftI2C_ReadByte(const SoftI2C_Bus_t *bus)
{
    uint8_t i; // 已读取的数据位计数
    uint8_t data = 0U; // 组合后的8位数据

    HAL_GPIO_WritePin(bus->sda_port, bus->sda_pin, GPIO_PIN_SET); // 释放SDA，交给从机输出数据

    for(i = 0U; i < 8U; i++)
    {
        data <<= 1U; // 为即将读取的数据位腾出最低位

        HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, GPIO_PIN_RESET);
        SoftI2C_Delay();

        HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, GPIO_PIN_SET);
        SoftI2C_Delay();

        if(HAL_GPIO_ReadPin(bus->sda_port, bus->sda_pin) == GPIO_PIN_SET)
        {
            data |= 0x01U; // SDA为高电平，本次读取位为1
        }
    }

    HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, GPIO_PIN_RESET);
    return data;
}

/**
 * @brief 主机向从机发送ACK
 *
 * 主机读取一个字节后，在第9个时钟期间拉低SDA，表示当前字节已经
 * 正确接收，并且还希望从机继续发送下一个字节。
 *
 * ACK发送完成后重新释放SDA，避免影响下一字节的数据传输。
 *
 * @param bus 使用的软件I2C总线
 * @retval None
 */
void SoftI2C_SendAck(const SoftI2C_Bus_t *bus)
{
    HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(bus->sda_port, bus->sda_pin, GPIO_PIN_RESET); // 拉低SDA表示ACK
    SoftI2C_Delay();

    HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, GPIO_PIN_SET); // 产生应答时钟
    SoftI2C_Delay();

    HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(bus->sda_port, bus->sda_pin, GPIO_PIN_SET); // 应答结束后释放SDA
    SoftI2C_Delay();
}

/**
 * @brief 主机向从机发送NACK
 *
 * 主机读取最后一个字节后，在第9个时钟期间保持SDA为高电平，
 * 表示不再需要从机继续发送数据。发送NACK后，调用者通常还需要
 * 产生STOP信号，以完整结束本次读取操作。
 *
 * @param bus 使用的软件I2C总线
 * @retval None
 */
void SoftI2C_SendNack(const SoftI2C_Bus_t *bus)
{
    HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(bus->sda_port, bus->sda_pin, GPIO_PIN_SET); // 释放SDA表示NACK
    SoftI2C_Delay();

    HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, GPIO_PIN_SET); // 产生应答时钟
    SoftI2C_Delay();

    HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, GPIO_PIN_RESET);
    SoftI2C_Delay();
}
