#include "em7028_service.h"
#include <stddef.h>
#include <string.h>

#define EM7028_CONTACT_RAW_THRESHOLD    15000U // 手指接触判断阈值
#define EM7028_MIN_PEAK_PROMINENCE      60U    // 波峰相对窗口最小值的最小突出量
#define EM7028_MIN_BEAT_INTERVAL_MS     550U   // 两个有效波峰的最短间隔
#define EM7028_MAX_BEAT_INTERVAL_MS     1500U  // 两个连续波峰的最长有效间隔

/*
 * 判断5点窗口的中间样本是否为局部波峰。
 *
 * 当前窗口排列：
 *
 * samples[0] samples[1] samples[2] samples[3] samples[4]
 *                           ↑
 *                     被判断的中间点
 *
 * 使用中间点而不是最新样本，可以同时观察波峰左侧和右侧的变化。
 * 这会让波峰确认比真实出现时间晚两个采样点，但能够判断信号是否
 * 已经从上升转为下降。
 *
 * 有效波峰必须满足：
 *
 * 1. 中间点不小于窗口内其余四个点；
 * 2. 中间点不能同时与左右相邻点相等，避免识别平顶波形；
 * 3. 中间点与窗口最小值至少相差60，过滤微小噪声抖动。
 */
static uint8_t EM7028_Service_IsPeak(const uint16_t *samples)
{
    uint8_t i;
    uint16_t window_min = samples[0];
    uint16_t center = samples[2];

    for(i = 1U; i < EM7028_SERVICE_SAMPLE_WINDOW_SIZE; i++)
    {
        if(samples[i] < window_min) window_min = samples[i];
    }

    if(center < samples[0] || center < samples[1] || center < samples[3] || center < samples[4]) return 0U;
    if(center == samples[1] && center == samples[3]) return 0U;
    if((center - window_min) < EM7028_MIN_PEAK_PROMINENCE) return 0U;

    return 1U;
}

/*
 * 计算BPM历史数组中现有数据的算术平均值。
 *
 * 这里最多平均最近4次瞬时BPM，可以降低单个波峰时间误差导致的
 * 界面跳动，同时不会让显示结果过度滞后。
 */
static uint16_t EM7028_Service_CalculateAvgBPM(const EM7028_HeartRateState_t *state)
{
    uint8_t i;
    uint32_t sum = 0U;

    if(state->bpm_history_count == 0U) return 0U;

    for(i = 0U; i < state->bpm_history_count; i++)
    {
        sum += state->bpm_history[i];
    }

    return (uint16_t)(sum / state->bpm_history_count);
}

void EM7028_Service_Init(EM7028_HeartRateState_t *state)
{
    if(state == NULL) return;

    memset(state, 0, sizeof(EM7028_HeartRateState_t));
}

/*
 * 持续处理EM7028原始PPG样本，并在检测到稳定心率后输出BPM。
 *
 * 本函数由SensorTask周期调用，每次只输入一个新样本。state用于在多次
 * 调用之间保存滑动窗口、上一个波峰时间和BPM历史，因此只应在开始
 * 新测量或确认手指移开时清零。
 *
 * 处理流程：
 *
 * 原始PPG数据
 * -> 判断手指是否接触
 * -> 放入5点滑动窗口
 * -> 判断窗口中点是否为局部波峰
 * -> 检查两个波峰之间的时间间隔
 * -> 计算瞬时BPM
 * -> 保存最近4次BPM并求平均
 * -> 积累至少3次有效结果后输出
 *
 * 返回0并不一定表示错误，也可能是：
 *
 * 手指未接触、窗口尚未填满、当前不是波峰、正在等待第二个波峰、
 * 波峰间隔过短、测量连续性中断，或者有效BPM数量还不足。
 */
uint8_t EM7028_Service_ProcessSample(EM7028_HeartRateState_t *state, uint16_t raw_data, uint32_t current_tick, uint32_t tick_fre, uint16_t *bpm)
{
    uint8_t i;
    uint32_t interval_ticks;
    uint32_t min_interval_ticks;
    uint32_t max_interval_ticks;
    uint16_t instant_bpm;

    if(state == NULL || bpm == NULL || tick_fre == 0U) return 0U;

    *bpm = state->bpm;

    /*
     * 当前使用固定原始值阈值判断手指是否接触。
     *
     * 原始值低于15000时认为手指已经移开，此时必须清空波峰时间、
     * 滑动窗口和BPM历史。否则下一次放入手指时，可能会使用上一次
     * 测量留下的波峰时间，计算出错误的BPM。
     *
     * 该阈值来自当前实物测试，不是适用于所有佩戴松紧和环境光条件
     * 的通用值。
     */
    if(raw_data < EM7028_CONTACT_RAW_THRESHOLD)
    {
        EM7028_Service_Init(state);
        *bpm = 0U;
        return 0U;
    }

    state->finger_present = 1U;

    /*
     * 前5个样本用于填满窗口。窗口未填满时无法同时观察一个采样点
     * 左右两侧的波形，因此暂时不执行波峰判断。
     */
    if(state->sample_count < EM7028_SERVICE_SAMPLE_WINDOW_SIZE)
    {
        state->sample_window[state->sample_count] = raw_data;
        state->sample_count++;

        if(state->sample_count < EM7028_SERVICE_SAMPLE_WINDOW_SIZE) return 0U;
    }
    else
    {
        /*
         * 窗口填满后，每次丢弃最旧样本，并把其余四个样本向前移动，
         * 最后将本次新数据放到窗口末尾。
         */
        for(i = 0U; i < EM7028_SERVICE_SAMPLE_WINDOW_SIZE - 1U; i++)
        {
            state->sample_window[i] = state->sample_window[i + 1U];
        }

        state->sample_window[EM7028_SERVICE_SAMPLE_WINDOW_SIZE - 1U] = raw_data;
    }

    if(EM7028_Service_IsPeak(state->sample_window) == 0U) return 0U;

    /*
     * 波峰间隔阈值以毫秒保存，而current_tick使用FreeRTOS系统节拍，
     * 因此必须根据tick_fre换算成对应的Tick数量。
     *
     * 550ms对应的最高可接受心率约为109BPM；
     * 1500ms对应的最低连续心率约为40BPM。
     *
     * 当前550ms阈值是根据实测重复计峰现象调整后的结果。
     */
    min_interval_ticks = tick_fre * EM7028_MIN_BEAT_INTERVAL_MS / 1000U;
    max_interval_ticks = tick_fre * EM7028_MAX_BEAT_INTERVAL_MS / 1000U;

    /*
     * 第一个有效波峰只能作为计时起点。
     * 没有前一个波峰，就无法计算两个波峰之间的时间间隔。
     */
    if(state->has_last_peak == 0U)
    {
        state->has_last_peak = 1U;
        state->last_peak_tick = current_tick;
        state->peak_count++;
        return 0U;
    }

    /*
     * uint32_t无符号减法可以在正常Tick回绕情况下得到正确时间差，
     * 前提是两次波峰间隔远小于uint32_t的完整计数周期。
     */
    interval_ticks = current_tick - state->last_peak_tick;

    /*
     * 间隔小于550ms时，通常认为是噪声或同一次脉搏被重复识别。
     *
     * 这里不更新last_peak_tick，使后续真实波峰仍然相对于上一个
     * 已接受波峰计算，而不是相对于本次被拒绝的噪声计算。
     */
    if(interval_ticks < min_interval_ticks) return 0U;

    state->last_peak_tick = current_tick;
    state->peak_count++;

    /*
     * 间隔超过1500ms说明连续测量已经中断。
     *
     * 当前波峰被保留为下一轮测量的起点，但旧BPM历史必须清空，
     * 防止中断前后的两段数据被混合平均。
     */
    if(interval_ticks > max_interval_ticks)
    {
        state->bpm_history_count = 0U;
        state->bpm_history_index = 0U;
        state->bpm_valid = 0U;
        state->bpm = 0U;
        *bpm = 0U;
        return 0U;
    }

    /*
     * 心率计算公式：
     *
     * BPM = 60秒 × 每秒Tick数 ÷ 两个波峰之间的Tick数量
     */
    instant_bpm = (uint16_t)((60UL * tick_fre) / interval_ticks);

    /*
     * bpm_history是长度为4的循环数组。
     *
     * 写到末尾后从位置0重新开始，始终保留最近4次瞬时BPM，
     * 不需要每次移动整个数组。
     */
    state->bpm_history[state->bpm_history_index] = instant_bpm;
    state->bpm_history_index++;

    if(state->bpm_history_index >= EM7028_SERVICE_BPM_HISTORY_SIZE) state->bpm_history_index = 0U;
    if(state->bpm_history_count < EM7028_SERVICE_BPM_HISTORY_SIZE) state->bpm_history_count++;

    state->bpm = EM7028_Service_CalculateAvgBPM(state);

    /*
     * 至少积累3次有效瞬时BPM后才将结果标记为有效。
     *
     * 这会让刚放上手指时需要等待几个脉搏才能显示稳定结果，
     * 但可以减少只根据一两个偶然波峰就输出错误心率的情况。
     */
    state->bpm_valid = (state->bpm_history_count >= 3U) ? 1U : 0U;
    *bpm = state->bpm;

    return state->bpm_valid;
}
