#include "rtc_service.h"
#include "rtc.h"

/**
 * @brief 判断指定年份是否为闰年
 *
 * 闰年规则：
 * 1. 能被400整除的年份是闰年；
 * 2. 能被4整除但不能被100整除的年份也是闰年；
 * 3. 其他年份均为平年。
 *
 * @param year 完整年份，例如2026
 * @retval 1U=闰年，0U=平年
 */
static uint8_t RTC_Service_IsLeapYear(uint16_t year)
{
    if((year % 400U) == 0U) return 1U;
    if(((year % 4U) == 0U) && ((year % 100U) != 0U)) return 1U;

    return 0U;
}

/**
 * @brief 获取指定月份的实际天数
 *
 * 首先从固定月份表中取得天数；如果当前月份是二月且指定年份为闰年，
 * 则将二月天数修正为29天。该结果用于检查日期是否合法，例如拒绝
 * “2月31日”或“4月31日”等无效日期。
 *
 * @param year 完整年份，例如2026
 * @param month 月份，范围为1~12
 * @retval 指定月份的天数；月份非法时返回0U
 */
static uint8_t RTC_service_GetMonthDays(uint16_t year, uint8_t month)
{
    static const uint8_t month_days[12] = {
        31U, 28U, 31U, 30U, 31U, 30U,
        31U, 31U, 30U, 31U, 30U, 31U
    };

    uint8_t days; // 指定月份的实际天数

    if((month < 1U) || (month > 12U)) return 0U;

    days = month_days[month - 1U];

    if((month == 2U) && (RTC_Service_IsLeapYear(year) != 0U))
    {
        days = 29U;
    }

    return days;
}

/**
 * @brief 根据年月日计算星期
 *
 * 算法计算结果使用0~6表示星期日到星期六，其中0表示星期日。
 * STM32 HAL使用RTC_WEEKDAY_MONDAY到RTC_WEEKDAY_SUNDAY表示星期，
 * 因此需要将算法得到的星期日单独转换为RTC_WEEKDAY_SUNDAY。
 *
 * 一月和二月在公式中视为上一年的第十三月和第十四月，所以当月份
 * 小于3时，需要先将参与计算的年份减1。
 *
 * @param year 完整年份，例如2026
 * @param month 月份，范围为1~12
 * @param date 日期
 * @retval STM32 HAL定义的星期值
 */
static uint8_t RTC_Service_CalculateWeekday(uint16_t year, uint8_t month, uint8_t date)
{
    static const uint8_t month_offset[12] = {
        0U, 3U, 2U, 5U, 0U, 3U,
        5U, 1U, 4U, 6U, 2U, 4U
    };

    uint16_t adjusted_year = year; // 参与星期公式计算的年份
    uint8_t week_day; // 算法得到的星期值，范围为0~6

    if(month < 3U)
    {
        adjusted_year--;
    }

    week_day = (uint8_t)((adjusted_year
                        + adjusted_year / 4U
                        - adjusted_year / 100U
                        + adjusted_year / 400U
                        + month_offset[month - 1U]
                        + date) % 7U);

    if(week_day == 0U) return RTC_WEEKDAY_SUNDAY;

    return week_day;
}

/**
 * @brief 读取RTC当前日期和时间
 *
 * STM32 RTC通过影子寄存器保证读取到的时间和日期来自同一个采样时刻。
 * 使用HAL库时必须先调用HAL_RTC_GetTime()读取时间，再调用
 * HAL_RTC_GetDate()读取日期并解锁影子寄存器。
 *
 * 如果读取时间后没有继续读取日期，影子寄存器可能无法正常解锁，
 * 后续读取时便可能出现时间不再更新的问题。
 *
 * 本服务使用RTC_FORMAT_BIN读取二进制格式数据，并将HAL中以0~99表示的
 * 年份转换为应用层使用的完整年份，例如26转换为2026。
 *
 * @param date_time 用于接收完整日期和时间的结构体
 * @retval 1U=读取成功，0U=参数错误或HAL读取失败
 */
uint8_t RTC_Service_Read(RTC_DateTime_t *date_time)
{
    RTC_TimeTypeDef rtc_time = {0}; // HAL时间结构
    RTC_DateTypeDef rtc_date = {0}; // HAL日期结构

    if(date_time == NULL) return 0U;

    /* 必须先读取时间，再读取日期。 */
    if(HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN) != HAL_OK) return 0U;

    /*
     * 读取日期不仅用于获得年月日，还会解锁RTC影子寄存器，
     * 使下一次调用能够读取到更新后的时间数据。
     */
    if(HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN) != HAL_OK) return 0U;

    date_time->year = rtc_date.Year + 2000U;
    date_time->month = rtc_date.Month;
    date_time->date = rtc_date.Date;
    date_time->hour = rtc_time.Hours;
    date_time->minute = rtc_time.Minutes;
    date_time->second = rtc_time.Seconds;

    return 1U;
}

/**
 * @brief 设置RTC日期和时间
 *
 * 写入前会依次检查年份、月份、日期、小时、分钟和秒是否合法。
 * 日期上限由年份和月份共同决定，因此能够正确处理大小月和闰年二月。
 *
 * 应用层使用2000~2099的完整年份，而STM32 RTC的Year字段只保存
 * 0~99，因此写入前需要减去2000。星期不要求调用者提供，而是根据
 * 年月日自动计算，避免日期与星期不一致。
 *
 * @param date_time 待写入RTC的完整日期和时间
 * @retval 1U=设置成功，0U=参数非法或HAL写入失败
 */
uint8_t RTC_Service_SetDateTime(const RTC_DateTime_t *date_time)
{
    RTC_TimeTypeDef rtc_time = {0}; // 准备写入HAL的时间数据
    RTC_DateTypeDef rtc_date = {0}; // 准备写入HAL的日期数据
    uint8_t month_days; // 指定年月对应的最大日期

    if(date_time == NULL) return 0U;

    /* STM32 RTC的年份字段只支持表示2000~2099年。 */
    if((date_time->year < 2000U) || (date_time->year > 2099U)) return 0U;

    month_days = RTC_service_GetMonthDays(date_time->year, date_time->month);
    if(month_days == 0U) return 0U; // 月份非法

    if((date_time->date < 1U) || (date_time->date > month_days)) return 0U;
    if(date_time->hour > 23U) return 0U;
    if(date_time->minute > 59U) return 0U;
    if(date_time->second > 59U) return 0U;

    rtc_time.Hours = date_time->hour;
    rtc_time.Minutes = date_time->minute;
    rtc_time.Seconds = date_time->second;
    rtc_time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    rtc_time.StoreOperation = RTC_STOREOPERATION_RESET;

    /*
     * HAL的Year字段使用0~99表示2000~2099年。
     * 星期根据年月日自动计算，不要求应用层单独维护。
     */
    rtc_date.Year = (uint8_t)(date_time->year - 2000U);
    rtc_date.Month = date_time->month;
    rtc_date.Date = date_time->date;
    rtc_date.WeekDay = RTC_Service_CalculateWeekday(date_time->year, date_time->month, date_time->date);

    if(HAL_RTC_SetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN) != HAL_OK) return 0U;
    if(HAL_RTC_SetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN) != HAL_OK) return 0U;

    return 1U;
}
