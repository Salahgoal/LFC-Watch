#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H

#include "main.h"
#include <stdint.h>

/*
 * 描述一组软件I2C总线使用的GPIO。
 *
 * 软件I2C协议函数不需要知道这些引脚连接的是触摸芯片还是EEPROM，
 * 调用者只需要传入对应的总线对象即可。
*/
typedef struct
{
    GPIO_TypeDef *sda_port; // SDA引脚所在GPIO组
    uint16_t sda_pin; // SDA引脚编号
    GPIO_TypeDef *scl_port; // SCL引脚所在GPIO组
    uint16_t scl_pin; // SCL引脚编号
}SoftI2C_Bus_t;

/*释放SDA和SCL，使总线进入空闲状态*/
void SoftI2C_Init(const SoftI2C_Bus_t *bus);

/*产生I2C开始和停止信号*/
void SoftI2C_Start(const SoftI2C_Bus_t *bus);
void SoftI2C_Stop(const SoftI2C_Bus_t *bus);

/*发送一个字节，并检查从机是否应答*/
void SoftI2C_WriteByte(const SoftI2C_Bus_t *bus, uint8_t data);
uint8_t SoftI2C_WaitAck(const SoftI2C_Bus_t *bus);

/*读取一个字节，并向从机发送ACK/NACK*/
uint8_t SoftI2C_ReadByte(const SoftI2C_Bus_t *bus);
void SoftI2C_SendAck(const SoftI2C_Bus_t *bus);
void SoftI2C_SendNack(const SoftI2C_Bus_t *bus);

#endif /* __SOFT_I2C_H */
