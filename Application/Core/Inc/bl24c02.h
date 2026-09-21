#ifndef __BL24C02_H
#define __BL24C02_H

#include "main.h"

void BL24C02_Init(void);
uint8_t BL24C02_WriteByte(uint8_t memory_address, uint8_t data);
uint8_t BL24C02_ReadByte(uint8_t memory_address, uint8_t *data);

#endif /* __BL24C02_H */
