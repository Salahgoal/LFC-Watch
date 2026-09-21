#ifndef __MPU6050_SERVICE_H
#define __MPU6050_SERVICE_H

#include "mpu6050.h"
#include <stdint.h>

/*
 * 保存经过业务层换算后的MPU6050数据。
 *
 * 驱动层读取到的是传感器寄存器中的原始整数，Service层负责将其换算成
 * 更容易理解和传递的工程单位：
 *
 * 加速度：mg，1000mg等于1g；
 * 温度：0.01℃，例如3495表示34.95℃；
 * 角速度：mdps，1000mdps等于1°/s。
 *
 * 使用带缩放比例的整数，可以避免任务消息和UI更新依赖浮点数，
 * 同时保留界面显示小数所需的精度。
 */
typedef struct
{
    int32_t accel_x_mg;             // X轴加速度，单位mg
    int32_t accel_y_mg;             // Y轴加速度，单位mg
    int32_t accel_z_mg;             // Z轴加速度，单位mg
    int32_t temperature_centi_c;    // 芯片温度，单位0.01℃
    int32_t gyro_x_mdps;            // X轴角速度，单位mdps
    int32_t gyro_y_mdps;            // Y轴角速度，单位mdps
    int32_t gyro_z_mdps;            // Z轴角速度，单位mdps
    uint32_t step_count;            // 当前累计步数
    uint32_t wrist_raise_count;     // 当前累计抬腕次数
    uint8_t wrist_is_up;            // 1表示当前满足看表姿态，0表示不满足
} MPU6050_Data_t;

/*
 * 保存基础计步算法的运行状态。
 *
 * filtered_magnitude_mg保存经过低通滤波的三轴加速度模长；
 * peak_detected记录是否已经检测到超过高阈值的运动峰值；
 * cooldown_samples限制两次计步之间的最短间隔，避免同一次振动被重复计数。
 *
 * 这些变量必须在连续采样之间保留，因此统一放入状态结构体中，
 * 由SensorTask长期维护，而不是作为函数内部的临时变量。
 */
typedef struct
{
    uint32_t step_count;                // 当前累计步数
    int32_t filtered_magnitude_mg;      // 滤波后的加速度模长，单位mg
    uint16_t cooldown_samples;          // 冷却阶段剩余的采样次数
    uint8_t initialized;                // 1表示滤波器已经取得初始值
    uint8_t peak_detected;              // 1表示已经检测到超过高阈值的峰值
} MPU6050_StepCounterState_t;

/*
 * 抬腕识别不是只判断当前姿态，而是识别“垂手到抬手”的完整动作过程。
 *
 * WAIT_DOWN：等待并确认手臂已经自然下垂；
 * WAIT_UP：已经确认垂手，等待抬手看表；
 * COOLDOWN：已经触发一次抬腕，短时间内不再重复触发。
 */
typedef enum
{
    MPU6050_WRIST_WAIT_DOWN = 0U,
    MPU6050_WRIST_WAIT_UP,
    MPU6050_WRIST_COOLDOWN
} MPU6050_WristRaiseStateCode_t;

/*
 * 保存基础抬腕识别算法的运行状态。
 *
 * stable_samples要求目标姿态连续保持一段时间，过滤瞬间晃动；
 * cooldown_samples限制连续两次抬腕之间的最短时间；
 * wrist_is_up只表示当前采样是否满足看表姿态，不代表本次一定产生了抬腕事件。
 */
typedef struct
{
    uint32_t raise_count;                    // 当前累计抬腕次数
    uint16_t stable_samples;                 // 当前姿态连续满足条件的采样次数
    uint16_t cooldown_samples;               // 冷却阶段剩余的采样次数
    MPU6050_WristRaiseStateCode_t state;     // 当前抬腕识别阶段
    uint8_t wrist_is_up;                     // 1表示当前满足看表姿态
} MPU6050_WristRaiseState_t;

/*
 * 将MPU6050寄存器原始值转换为mg、0.01℃和mdps。
 * 这里只换算传感器物理数据，不修改步数和抬腕等业务状态。
 */
uint8_t MPU6050_Service_ConvertRawData(const MPU6050_RawData_t *raw_data, MPU6050_Data_t *data);

/* 初始化基础计步算法的全部状态。 */
void MPU6050_Service_InitStepCounter(MPU6050_StepCounterState_t *state);

/*
 * 输入一次新的加速度数据并执行计步检测。
 * 返回1表示本次采样确认产生了新的一步，返回0表示没有产生新步数。
 */
uint8_t MPU6050_Service_ProcessStep(const MPU6050_Data_t *data, MPU6050_StepCounterState_t *state);

/* 初始化基础抬腕识别状态机。 */
void MPU6050_Service_InitWristRaise(MPU6050_WristRaiseState_t *state);

/*
 * 输入一次新的加速度数据并执行抬腕识别。
 * 返回1表示本次采样刚刚确认了一次新抬腕，返回0表示没有新事件。
 */
uint8_t MPU6050_Service_ProcessWristRaise(const MPU6050_Data_t *data, MPU6050_WristRaiseState_t *state);

#endif /* __MPU6050_SERVICE_H */
