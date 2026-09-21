#ifndef __EM7028_SERVICE_H
#define __EM7028_SERVICE_H

#include <stdint.h>

#define EM7028_SERVICE_SAMPLE_WINDOW_SIZE   5U // 局部波峰检测窗口长度
#define EM7028_SERVICE_BPM_HISTORY_SIZE     4U // 瞬时BPM平均历史长度

/*
 * 保存EM7028心率计算过程中的跨采样状态。
 *
 * sample_window保存最近5个PPG样本，用于判断局部波峰；
 * bpm_history保存最近4次瞬时心率，用于降低界面数值跳动；
 * last_peak_tick记录上一次有效波峰的系统时间；
 * peak_count记录已经接受的有效波峰数量；
 * bpm_valid表示当前平均BPM是否已经积累足够样本。
 *
 * 该结构必须在连续采样期间长期保留，不能在每次处理新样本前
 * 重新创建或初始化，否则滑动窗口和波峰时间都会丢失。
 */
typedef struct
{
    uint16_t sample_window[EM7028_SERVICE_SAMPLE_WINDOW_SIZE];
    uint16_t bpm_history[EM7028_SERVICE_BPM_HISTORY_SIZE];

    uint8_t sample_count;          // 当前窗口内已有的有效样本数
    uint8_t bpm_history_count;     // 当前已经保存的瞬时BPM数量
    uint8_t bpm_history_index;     // 下一次写入BPM历史的位置
    uint8_t finger_present;        // 1表示当前原始值超过手指接触阈值
    uint8_t has_last_peak;         // 1表示已经记录过前一个有效波峰
    uint8_t bpm_valid;             // 1表示当前平均BPM已经具备基础有效性

    uint32_t last_peak_tick;       // 上一次有效波峰的FreeRTOS Tick
    uint32_t peak_count;           // 当前测量过程累计接受的波峰数量
    uint16_t bpm;                  // 最近计算得到的平均心率
} EM7028_HeartRateState_t;

/* 清空心率计算状态，开始一轮新的测量。 */
void EM7028_Service_Init(EM7028_HeartRateState_t *state);

/*
 * 处理一个新的PPG原始样本。
 *
 * 返回1表示本次检测到有效波峰，并且已经积累出可用的平均BPM；
 * 返回0表示本次没有新的可显示结果，不代表I2C通信失败。
 *
 * tick_fre必须传入FreeRTOS每秒的系统节拍数量。
 */
uint8_t EM7028_Service_ProcessSample(EM7028_HeartRateState_t *state, uint16_t raw_data, uint32_t current_tick, uint32_t tick_fre, uint16_t *bpm);

#endif /* __EM7028_SERVICE_H */
