#ifndef __SETTINGS_SERVICE_H
#define __SETTINGS_SERVICE_H

#include <stdint.h>

#define SETTINGS_DEFAULT_BRIGHTNESS 66U //EEPROM无有效数据时默认使用66%亮度

void SettingsService_Init(void);
uint8_t SettingsService_LoadBrightness(uint8_t *percent);
uint8_t SettingsService_SaveBrightness(uint8_t percent);

#endif /* __SETTINGS_SERVICE_H */
