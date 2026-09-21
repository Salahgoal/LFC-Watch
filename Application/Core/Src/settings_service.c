#include "settings_service.h"
#include "bl24c02.h" // BL24C02 EEPROM驱动，提供读写单字节数据的接口


/*
 * EEPROM地址规划：
 * 0x20保存有效标记，0x21保存亮度百分比。
 *
 * 有效标记用于判断EEPROM中的数据是否真正由程序写入。
 * 因为新EEPROM中的内容可能是0xFF，不能直接当作亮度使用。
 */
#define SETTINGS_MAGIC_ADDRESS      0X20U
#define SETTINGS_BRIGHTNESS_ADDRESS 0X21U
#define SETTINGS_MAGIC_VALUE        0X5AU
#define SETTINGS_BRIGHTNESS_MAX     100U


/**
 * @brief 初始化设置存储模块
 *
 * 当前设置数据保存在BL24C02 EEPROM中，因此这里只需要初始化EEPROM驱动。
 */
void SettingsService_Init(void)
{
    BL24C02_Init();
}

/**
 * @brief 从EEPROM读取保存的亮度百分比
 *
 * 读取流程：
 * 1. 检查输出指针；
 * 2. 读取有效标记；
 * 3. 判断EEPROM是否保存过设置；
 * 4. 读取亮度；
 * 5. 检查亮度是否处于0～100范围。
 *
 * @param percent 用于接收亮度百分比
 * @return 1表示读取成功，0表示没有有效设置或读取失败
*/
uint8_t SettingsService_LoadBrightness(uint8_t *percent)
{
    uint8_t magic;
    uint8_t save_percent;

    if(percent == NULL) return 0U;

    if(BL24C02_ReadByte(SETTINGS_MAGIC_ADDRESS, &magic) == 0U) return 0U;

    if(magic != SETTINGS_MAGIC_VALUE) return 0U; // EEPROM中没有有效数据

    if(BL24C02_ReadByte(SETTINGS_BRIGHTNESS_ADDRESS, &save_percent) == 0U) return 0U;

    if(save_percent > SETTINGS_BRIGHTNESS_MAX) return 0U; // EEPROM中数据异常

    *percent = save_percent;

    return 1U; // 成功读取亮度百分比
}

/**
 * @brief 将亮度百分比保存到EEPROM
 *
 * 先写亮度数据，再写有效标记。这样即使写入过程意外中断，
 * 也不会出现“标记有效，但亮度数据还没写完”的情况。
 *
 * @param percent 要保存的亮度百分比，范围为0～100
 * @return 1表示保存成功，0表示参数错误或写入失败
*/
uint8_t SettingsService_SaveBrightness(uint8_t percent)
{
    if(percent > SETTINGS_BRIGHTNESS_MAX ) return 0U; // 参数错误

    if(BL24C02_WriteByte(SETTINGS_BRIGHTNESS_ADDRESS, percent) == 0U) return 0U;

    if(BL24C02_WriteByte(SETTINGS_MAGIC_ADDRESS, SETTINGS_MAGIC_VALUE) == 0U) return 0U;

    return 1U; // 成功保存亮度百分比
}
