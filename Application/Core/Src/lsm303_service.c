#include "lsm303_service.h"
#include <stddef.h>
#include <math.h>

#define LSM303_PI                       3.1415926535f
#define LSM303_RAD_TO_DEGREE            (180.0f / LSM303_PI)
#define LSM303_CALIBRATION_MIN_SPAN      100U // 判断轴向覆盖是否足够的基础阈值

/*
 * 对磁力计数据进行基础硬铁偏移和三轴范围修正。
 *
 * 多方向旋转磁力计时，理想的三轴数据应该形成一个以零点为中心的球体。
 * 固定磁场干扰会使球体中心偏移，不同轴灵敏度和周围材料还可能使球体
 * 在三个方向上的直径不一致。
 *
 * 当前算法分两步处理：
 *
 * 1. corrected = raw - offset
 *    将各轴范围的中点移动回零点，修正基础硬铁零偏；
 *
 * 2. corrected = corrected × average_span / axis_span
 *    使用三轴平均范围作为目标，对三个轴进行简单比例修正。
 *
 * 这属于适合当前学习项目的基础Min-Max校准，不是完整的椭球拟合。
 */
static uint8_t LSM303_Service_CorrectMagnetic(const LSM303_RawData_t *raw_data, const LSM303_MagCalibration_t *calibration, float *mag_x, float *mag_y, float *mag_z)
{
    float average_span;

    if(raw_data == NULL || calibration == NULL || mag_x == NULL || mag_y == NULL || mag_z == NULL) return 0U;
    if(calibration->range_ready == 0U) return 0U;
    if(calibration->span_x == 0U || calibration->span_y == 0U || calibration->span_z == 0U) return 0U;

    average_span = ((float)calibration->span_x + calibration->span_y + calibration->span_z) / 3.0f;

    *mag_x = ((float)raw_data->mag_x - calibration->offset_x) * average_span / calibration->span_x;
    *mag_y = ((float)raw_data->mag_y - calibration->offset_y) * average_span / calibration->span_y;
    *mag_z = ((float)raw_data->mag_z - calibration->offset_z) * average_span / calibration->span_z;

    return 1U;
}

void LSM303_Service_InitMagCalibration(LSM303_MagCalibration_t *calibration)
{
    if(calibration == NULL) return;

    calibration->min_x = 0;
    calibration->max_x = 0;
    calibration->min_y = 0;
    calibration->max_y = 0;
    calibration->min_z = 0;
    calibration->max_z = 0;

    calibration->span_x = 0U;
    calibration->span_y = 0U;
    calibration->span_z = 0U;

    calibration->offset_x = 0;
    calibration->offset_y = 0;
    calibration->offset_z = 0;

    calibration->sample_count = 0U;
    calibration->started = 0U;
    calibration->range_ready = 0U;
}

/*
 * 使用实时磁场数据更新基础校准范围。
 *
 * 处理流程：
 *
 * 第一组数据作为三个轴各自的初始最大值和最小值；
 * 后续数据持续更新min和max；
 * span = max - min；
 * offset = (max + min) / 2；
 * 三个轴的span都达到阈值后，将range_ready置1。
 *
 * range_ready只说明板子在三个轴上发生过较明显的旋转，
 * 并不表示方向精度已经达到专业指南针水平。
 */
uint8_t LSM303_Service_UpdateMagCalibration(LSM303_MagCalibration_t *calibration, const LSM303_RawData_t *raw_data)
{
    if(calibration == NULL || raw_data == NULL) return 0U;

    /*
     * 第一组样本必须同时作为初始最大值和最小值。
     *
     * 不能简单把所有最小值初始化为0，因为某个轴可能始终处于正数范围，
     * 这样会把一个从未测量到的0错误地当成该轴最小值。
     */
    if(calibration->started == 0U)
    {
        calibration->min_x = raw_data->mag_x;
        calibration->max_x = raw_data->mag_x;
        calibration->min_y = raw_data->mag_y;
        calibration->max_y = raw_data->mag_y;
        calibration->min_z = raw_data->mag_z;
        calibration->max_z = raw_data->mag_z;
        calibration->started = 1U;
    }
    else
    {
        if(raw_data->mag_x < calibration->min_x) calibration->min_x = raw_data->mag_x;
        if(raw_data->mag_x > calibration->max_x) calibration->max_x = raw_data->mag_x;

        if(raw_data->mag_y < calibration->min_y) calibration->min_y = raw_data->mag_y;
        if(raw_data->mag_y > calibration->max_y) calibration->max_y = raw_data->mag_y;

        if(raw_data->mag_z < calibration->min_z) calibration->min_z = raw_data->mag_z;
        if(raw_data->mag_z > calibration->max_z) calibration->max_z = raw_data->mag_z;
    }

    calibration->span_x = (uint16_t)((int32_t)calibration->max_x - calibration->min_x);
    calibration->span_y = (uint16_t)((int32_t)calibration->max_y - calibration->min_y);
    calibration->span_z = (uint16_t)((int32_t)calibration->max_z - calibration->min_z);

    /*
     * 最大值和最小值的中点就是当前估算的硬铁零偏。
     * 先转换为int32_t再相加，避免两个int16_t直接相加发生溢出。
     */
    calibration->offset_x = (int16_t)(((int32_t)calibration->max_x + calibration->min_x) / 2);
    calibration->offset_y = (int16_t)(((int32_t)calibration->max_y + calibration->min_y) / 2);
    calibration->offset_z = (int16_t)(((int32_t)calibration->max_z + calibration->min_z) / 2);

    calibration->sample_count++;

    /*
     * 三个轴的变化范围都超过基础阈值后，允许使用校正和方向角算法。
     * 当前条件只检查覆盖范围，不检查样本分布是否均匀。
     */
    if(calibration->span_x >= LSM303_CALIBRATION_MIN_SPAN &&
       calibration->span_y >= LSM303_CALIBRATION_MIN_SPAN &&
       calibration->span_z >= LSM303_CALIBRATION_MIN_SPAN)
    {
        calibration->range_ready = 1U;
    }

    return 1U;
}

/*
 * 计算未经过倾斜补偿的基础指南针方向角。
 *
 * 处理流程：
 *
 * 磁场原始数据
 * -> 减去各轴固定零偏并修正范围
 * -> 使用atan2(Y, X)计算磁场矢量相对X轴的夹角
 * -> 弧度转换为角度
 * -> 将结果整理到0～359°
 *
 * 该算法只使用磁场X、Y分量，要求板子基本水平。板子明显倾斜时，
 * 应使用LSM303_Service_CalculateTiltHeading()的倾斜补偿结果。
 */
uint8_t LSM303_Service_CalculateHorizontalHeading(const LSM303_RawData_t *raw_data, const LSM303_MagCalibration_t *calibration, LSM303_CompassData_t *compass_data)
{
    float corrected_x, corrected_y, corrected_z;
    float heading;
    uint16_t rounded_heading;

    if(raw_data == NULL || calibration == NULL || compass_data == NULL) return 0U;
    if(calibration->range_ready == 0U) return 0U;

    if(LSM303_Service_CorrectMagnetic(raw_data, calibration, &corrected_x, &corrected_y, &corrected_z) == 0U) return 0U;
    if(corrected_x == 0.0f && corrected_y == 0.0f) return 0U;

    /*
     * atan2f(Y, X)能够保留磁场矢量所在象限。
     * 返回角度范围约为-180～+180°，负数加360°后整理为0～359°。
     */
    heading = atan2f(corrected_y, corrected_x) * LSM303_RAD_TO_DEGREE;

    if(heading < 0.0f) heading += 360.0f;
    if(heading >= 360.0f) heading -= 360.0f;

    rounded_heading = (uint16_t)(heading + 0.5f);
    if(rounded_heading >= 360U) rounded_heading = 0U;

    compass_data->corrected_mag_x = (int16_t)corrected_x;
    compass_data->corrected_mag_y = (int16_t)corrected_y;
    compass_data->corrected_mag_z = (int16_t)corrected_z;
    compass_data->heading_degrees = rounded_heading;

    return 1U;
}

/*
 * 使用加速度计姿态计算经过倾斜补偿的指南针方向角。
 *
 * 处理流程：
 *
 * 1. 使用三轴加速度计算roll和pitch；
 * 2. 对磁场数据执行零偏和三轴范围修正；
 * 3. 根据roll和pitch把磁场矢量旋转回水平坐标系；
 * 4. 使用水平磁场X、Y分量计算atan2方向角；
 * 5. 把弧度转换成0～359°的整数角度。
 *
 * 加速度计在静止时主要测量重力，因此可以用于估算板子姿态。
 * 快速移动时，加速度中还会包含人体运动产生的分量，此时计算得到的
 * roll、pitch和倾斜补偿方向角可能短暂波动。
 *
 * 当前算法没有完成高精度轴向映射、当地磁偏角修正和完整椭球校准，
 * 因此用于学习倾斜补偿流程和基础方向显示，不作为专业导航结果。
 */
uint8_t LSM303_Service_CalculateTiltHeading(const LSM303_RawData_t *raw_data, const LSM303_MagCalibration_t *calibration, LSM303_CompassData_t *compass_data)
{
    float accel_x, accel_y, accel_z;
    float mag_x, mag_y, mag_z;
    float accel_length_squared;
    float roll, pitch;
    float sin_roll, cos_roll, sin_pitch, cos_pitch;
    float horizontal_mag_x, horizontal_mag_y;
    float heading;
    uint16_t rounded_heading;

    if(raw_data == NULL || calibration == NULL || compass_data == NULL) return 0U;
    if(calibration->range_ready == 0U) return 0U;

    /*
     * 实物测试中，Core板和屏幕朝上时加速度Z轴约为-1000mg。
     *
     * 后面的姿态公式以水平放置时Z轴为正作为计算约定，因此这里将三个
     * 加速度轴统一取反，使屏幕朝上时Z轴约为+1000mg。
     */
    accel_x = -(float)raw_data->accel_x;
    accel_y = -(float)raw_data->accel_y;
    accel_z = -(float)raw_data->accel_z;

    accel_length_squared = accel_x * accel_x + accel_y * accel_y + accel_z * accel_z;
    if(accel_length_squared < 1.0f) return 0U;

    /*
     * 当前函数执行的是硬铁零偏和三轴范围比例修正。
     * 修正结果仍是用于方向计算的相对磁场数值，没有转换成Gauss。
     */
    if(LSM303_Service_CorrectMagnetic(raw_data, calibration, &mag_x, &mag_y, &mag_z) == 0U) return 0U;

    /*
     * roll表示左右倾斜角，pitch表示前后倾斜角。
     *
     * atan2f可以保留象限信息；pitch分母使用Y、Z合成值，
     * 减少板子接近特殊角度时简单除法产生除零的风险。
     */
    roll = atan2f(accel_y, accel_z);
    pitch = atan2f(-accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z));

    sin_roll = sinf(roll);
    cos_roll = cosf(roll);
    sin_pitch = sinf(pitch);
    cos_pitch = cosf(pitch);

    /*
     * 使用当前roll和pitch，将三轴磁场旋转回水平坐标系。
     *
     * horizontal_mag_x和horizontal_mag_y可以理解为：
     * 如果此刻把倾斜的板子恢复水平，应该得到的磁场X、Y分量。
     */
    horizontal_mag_x = mag_x * cos_pitch + mag_z * sin_pitch;
    horizontal_mag_y = mag_x * sin_roll * sin_pitch + mag_y * cos_roll - mag_z * sin_roll * cos_pitch;

    if(horizontal_mag_x == 0.0f && horizontal_mag_y == 0.0f) return 0U;

    heading = atan2f(horizontal_mag_y, horizontal_mag_x) * LSM303_RAD_TO_DEGREE;

    if(heading < 0.0f) heading += 360.0f;
    if(heading >= 360.0f) heading -= 360.0f;

    rounded_heading = (uint16_t)(heading + 0.5f);
    if(rounded_heading >= 360U) rounded_heading = 0U;

    compass_data->tilt_heading_degrees = rounded_heading;
    compass_data->pitch_degrees = (int16_t)(pitch * LSM303_RAD_TO_DEGREE);
    compass_data->roll_degrees = (int16_t)(roll * LSM303_RAD_TO_DEGREE);

    return 1U;
}
