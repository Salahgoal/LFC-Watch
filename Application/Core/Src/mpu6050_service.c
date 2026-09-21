#include "mpu6050_service.h"
#include <stddef.h>
#include <math.h>

#define MPU6050_ACCEL_SCALE_LSB_PER_G       16384L // ±2g量程下，1g对应16384LSB
#define MPU6050_GYRO_SCALE_LSB_PER_DPS      131L   // ±250°/s量程下，1°/s约对应131LSB
#define MPU6050_TEMPERATURE_SCALE           340L   // 温度换算公式中的比例
#define MPU6050_TEMPERATURE_OFFSET_CENTI_C  3653L  // 温度换算偏移量，单位0.01℃

/*
 * 当前计步算法按照SensorTask约50Hz的采样频率设计。
 *
 * 高阈值用于确认运动峰值，低阈值用于确认峰值已经回落。
 * 使用两个不同阈值形成迟滞区间，可以避免数据在单一阈值附近波动时
 * 被重复计步。
 */
#define MPU6050_STEP_HIGH_THRESHOLD_MG  1150L // 超过1150mg认为出现运动峰值
#define MPU6050_STEP_LOW_THRESHOLD_MG   1050L // 回落到1050mg以下确认完成一步
#define MPU6050_STEP_COOLDOWN_SAMPLES   15U   // 50Hz采样下约为300ms

/*
 * 这些抬腕阈值来自当前手表背板实际安装方向和多组姿态测试。
 *
 * 垂手时重力主要落在负Y轴；
 * 抬手看表时重力主要转移到负Z轴。
 *
 * 稳定采样次数用于过滤瞬间晃动，冷却时间用于防止保持看表姿态时
 * 连续触发多次抬腕事件。这里同样以SensorTask约50Hz采样为前提。
 */
#define MPU6050_WRIST_DOWN_Y_THRESHOLD_MG  (-800L) // 垂手时Y轴接近-1g
#define MPU6050_WRIST_DOWN_Z_THRESHOLD_MG  (-650L) // 垂手时Z轴不能接近-1g
#define MPU6050_WRIST_UP_Z_THRESHOLD_MG    (-850L) // 看表时Z轴接近-1g
#define MPU6050_WRIST_UP_Y_LIMIT_MG         300L   // 看表时Y轴应低于该限制
#define MPU6050_WRIST_DOWN_STABLE_SAMPLES   10U    // 50Hz下连续约200ms确认垂手
#define MPU6050_WRIST_UP_STABLE_SAMPLES     5U     // 50Hz下连续约100ms确认抬手
#define MPU6050_WRIST_COOLDOWN_SAMPLES      100U   // 50Hz下冷却约2秒

uint8_t MPU6050_Service_ConvertRawData(const MPU6050_RawData_t *raw_data, MPU6050_Data_t *data)
{
    if(raw_data == NULL || data == NULL) return 0U;

    /*
     * 当前加速度计使用±2g量程，灵敏度为16384LSB/g。
     *
     * 输出单位选择mg，因此先乘1000再除以16384。
     * 运算前转换为int32_t，避免16位原始数据在乘法时溢出。
     */
    data->accel_x_mg = ((int32_t)raw_data->accel_x * 1000L) / MPU6050_ACCEL_SCALE_LSB_PER_G;
    data->accel_y_mg = ((int32_t)raw_data->accel_y * 1000L) / MPU6050_ACCEL_SCALE_LSB_PER_G;
    data->accel_z_mg = ((int32_t)raw_data->accel_z * 1000L) / MPU6050_ACCEL_SCALE_LSB_PER_G;

    /*
     * MPU6050芯片温度换算公式：
     *
     * temperature = raw / 340 + 36.53
     *
     * 为了保留两位小数，结果使用0.01℃表示。因此原始值先乘100，
     * 36.53℃也相应写成3653。
     *
     * 这里得到的是MPU6050芯片内部温度，主要用于温漂补偿和状态观察，
     * 不应当将它当作AHT21测得的环境温度。
     */
    data->temperature_centi_c = ((int32_t)raw_data->temperature * 100L) / MPU6050_TEMPERATURE_SCALE;
    data->temperature_centi_c += MPU6050_TEMPERATURE_OFFSET_CENTI_C;

    /*
     * 当前陀螺仪使用±250°/s量程，灵敏度约为131LSB/(°/s)。
     * 输出单位选择mdps，因此先乘1000再除以131。
     */
    data->gyro_x_mdps = ((int32_t)raw_data->gyro_x * 1000L) / MPU6050_GYRO_SCALE_LSB_PER_DPS;
    data->gyro_y_mdps = ((int32_t)raw_data->gyro_y * 1000L) / MPU6050_GYRO_SCALE_LSB_PER_DPS;
    data->gyro_z_mdps = ((int32_t)raw_data->gyro_z * 1000L) / MPU6050_GYRO_SCALE_LSB_PER_DPS;

    return 1U;
}

void MPU6050_Service_InitStepCounter(MPU6050_StepCounterState_t *state)
{
    if(state == NULL) return;

    state->step_count = 0U;
    state->filtered_magnitude_mg = 0;
    state->cooldown_samples = 0U;
    state->initialized = 0U;
    state->peak_detected = 0U;
}

/*
 * 基础计步处理流程：
 *
 * 三轴加速度 -> 计算矢量模长 -> 低通滤波 -> 超过高阈值记录峰值
 * -> 回落到低阈值确认一步 -> 进入短暂冷却阶段
 *
 * 手表静止时，加速度模长约为1000mg。使用三轴矢量模长，可以降低
 * 手表佩戴方向变化对计步结果的影响，因为无论重力主要落在哪一根轴上，
 * 三轴合成后的重力模长仍然接近1g。
 *
 * 返回值只表示“本次采样是否新增了一步”，累计结果保存在state->step_count。
 */
uint8_t MPU6050_Service_ProcessStep(const MPU6050_Data_t *data, MPU6050_StepCounterState_t *state)
{
    float accel_x, accel_y, accel_z;
    float magnitude;
    int32_t magnitude_mg;

    if(data == NULL || state == NULL) return 0U;

    accel_x = (float)data->accel_x_mg;
    accel_y = (float)data->accel_y_mg;
    accel_z = (float)data->accel_z_mg;

    /* 根据sqrt(x²+y²+z²)计算与当前安装方向无关的加速度模长。 */
    magnitude = sqrtf(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
    magnitude_mg = (int32_t)(magnitude + 0.5f);

    /*
     * 第一次采样还没有历史滤波值，因此直接使用当前模长作为初值。
     * 如果从0开始滤波，会让启动阶段出现一段没有实际意义的缓慢爬升。
     */
    if(state->initialized == 0U)
    {
        state->filtered_magnitude_mg = magnitude_mg;
        state->initialized = 1U;
        return 0U;
    }

    /*
     * 一阶低通滤波：
     *
     * filtered = old × 3/4 + new × 1/4
     *
     * 历史数据权重较大，可以削弱手部细小抖动和传感器噪声；
     * 新数据仍保留1/4权重，使正常走路产生的峰值能够及时反映出来。
     */
    state->filtered_magnitude_mg = (state->filtered_magnitude_mg * 3L + magnitude_mg) / 4L;

    if(state->cooldown_samples > 0U) state->cooldown_samples--;

    /*
     * 尚未检测到峰值时，只等待滤波结果超过高阈值。
     *
     * 这里检测到高峰后不能立即计步，因为一次普通晃动也可能短暂超过阈值。
     * 只有后续数据重新回落到低阈值，才认为完成了一个完整的步态波形。
     */
    if(state->peak_detected == 0U)
    {
        if(state->cooldown_samples == 0U && state->filtered_magnitude_mg >= MPU6050_STEP_HIGH_THRESHOLD_MG) state->peak_detected = 1U;
        return 0U;
    }

    /*
     * 已经检测到高峰，并且加速度模长回落到低阈值以下，
     * 说明一个“上升—峰值—回落”的完整过程已经结束，可以计为一步。
     */
    if(state->filtered_magnitude_mg <= MPU6050_STEP_LOW_THRESHOLD_MG)
    {
        state->peak_detected = 0U;
        state->cooldown_samples = MPU6050_STEP_COOLDOWN_SAMPLES;
        state->step_count++;
        return 1U;
    }

    return 0U;
}

void MPU6050_Service_InitWristRaise(MPU6050_WristRaiseState_t *state)
{
    if(state == NULL) return;

    state->raise_count = 0U;
    state->stable_samples = 0U;
    state->cooldown_samples = 0U;
    state->state = MPU6050_WRIST_WAIT_DOWN;
    state->wrist_is_up = 0U;
}

/*
 * 基础抬腕识别状态机：
 *
 * 1. WAIT_DOWN阶段连续检测垂手姿态；
 * 2. 垂手稳定后进入WAIT_UP，等待抬手看表姿态；
 * 3. 看表姿态稳定后确认一次抬腕，并进入COOLDOWN；
 * 4. 冷却结束后回到WAIT_DOWN，必须重新经历垂手才能再次触发。
 *
 * 不能只使用一个“当前是否抬手”的条件直接累计次数。否则用户保持看表
 * 姿态时，每次采样都会重复计数。状态机要求动作必须按照
 * “垂手 -> 抬手 -> 冷却 -> 再次垂手”的顺序发生。
 *
 * state->wrist_is_up表示当前姿态是否满足看表条件；
 * 函数返回1则表示本次调用刚刚确认了一次新的抬腕事件。
 */
uint8_t MPU6050_Service_ProcessWristRaise(const MPU6050_Data_t *data, MPU6050_WristRaiseState_t *state)
{
    uint8_t is_down;
    uint8_t is_up;

    if(data == NULL || state == NULL) return 0U;

    /*
     * 根据当前背板的实际安装方向和三组实测数据：
     *
     * 垂手时，重力主要落在负Y轴，Z轴不会接近-1g；
     * 抬手看表时，重力主要转移到负Z轴，同时Y轴应低于设定限制。
     *
     * X轴会随着手表松紧、佩戴位置和手腕转动产生较大变化，
     * 因此当前基础算法不使用X轴判断姿态。
     */
    is_down = (data->accel_y_mg <= MPU6050_WRIST_DOWN_Y_THRESHOLD_MG) && (data->accel_z_mg >= MPU6050_WRIST_DOWN_Z_THRESHOLD_MG);
    is_up = (data->accel_z_mg <= MPU6050_WRIST_UP_Z_THRESHOLD_MG) && (data->accel_y_mg <= MPU6050_WRIST_UP_Y_LIMIT_MG);

    state->wrist_is_up = is_up;

    /*
     * WAIT_DOWN：要求垂手条件连续满足约200ms。
     * 中途任何一次不满足都会清零计数，避免短暂晃动被当成有效垂手。
     */
    if(state->state == MPU6050_WRIST_WAIT_DOWN)
    {
        if(is_down != 0U)
        {
            if(state->stable_samples < MPU6050_WRIST_DOWN_STABLE_SAMPLES) state->stable_samples++;

            if(state->stable_samples >= MPU6050_WRIST_DOWN_STABLE_SAMPLES)
            {
                state->stable_samples = 0U;
                state->state = MPU6050_WRIST_WAIT_UP;
            }
        }
        else
        {
            state->stable_samples = 0U;
        }

        return 0U;
    }

    /*
     * WAIT_UP：已经确认手臂处于垂手状态，现在等待稳定的看表姿态。
     * 看表条件连续满足约100ms后，才确认完成一次抬腕动作。
     */
    if(state->state == MPU6050_WRIST_WAIT_UP)
    {
        if(is_up != 0U)
        {
            if(state->stable_samples < MPU6050_WRIST_UP_STABLE_SAMPLES) state->stable_samples++;

            if(state->stable_samples >= MPU6050_WRIST_UP_STABLE_SAMPLES)
            {
                state->raise_count++;
                state->stable_samples = 0U;
                state->cooldown_samples = MPU6050_WRIST_COOLDOWN_SAMPLES;
                state->state = MPU6050_WRIST_COOLDOWN;
                return 1U;
            }
        }
        else
        {
            state->stable_samples = 0U;
        }

        return 0U;
    }

    /*
     * COOLDOWN：一次抬腕触发后等待约2秒。
     * 即使用户继续保持看表姿态，这段时间也不会重复产生抬腕事件。
     */
    if(state->cooldown_samples > 0U)
    {
        state->cooldown_samples--;
        return 0U;
    }

    state->stable_samples = 0U;
    state->state = MPU6050_WRIST_WAIT_DOWN;

    return 0U;
}
