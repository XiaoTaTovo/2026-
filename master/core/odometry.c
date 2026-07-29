#include "core/odometry.h"

#define CAR_PI_F (3.14159265358979323846f)

void CarOdometry_Init(CarOdometry *odometry)
{
    if (odometry == 0) {
        return;
    }
//指针为空返回
    *odometry = (CarOdometry){0};
}

CarStatus CarOdometry_Update(CarOdometry *odometry,
                             const CarConfig *config,
                             const CarEncoderSample *sample)
//sample是读取的原始数据，包括  左轮计数、右轮计数、时间戳、有效否
//odometry是里程计数据，包括  上一次左轮计数、上一次右轮计数、左轮走过的距离、右轮走过的距离、中心走过的距离、编码器角度、上一次更新时间戳、是否初始化
{
    float mm_per_count;
    int16_t left_delta;
    int16_t right_delta;
    float left_mm;
    float right_mm;

    if ((odometry == 0) || (config == 0) || (sample == 0) ||
        !sample->valid || (config->wheel_diameter_mm <= 0.0f) ||
        (config->track_width_mm <= 0.0f) ||
        (config->encoder_counts_per_wheel_rev <= 0.0f)) {
        return CAR_ERROR_ARG;
    }

    if (!odometry->initialized) {
        odometry->previous_left_count = sample->left_count;
        odometry->previous_right_count = sample->right_count;
        odometry->last_update_ms = sample->timestamp_ms;
        odometry->initialized = true;
        return CAR_OK;
    }//因为需要上一次的状态，而第一次的上一次却没有，所以需要特殊处理
    //注意到初始化的时候初始化居然标的0，就是在这里先指定好第一次的状态

    left_delta = (int16_t)((uint16_t)sample->left_count -
                           (uint16_t)odometry->previous_left_count);
    right_delta = (int16_t)((uint16_t)sample->right_count -
                            (uint16_t)odometry->previous_right_count);
    //编码器计数是16位，是-32768到32767，有符号数从32767到-32768，会以为-65535是瞬间倒退了这么多
    //实际上转换成无符号数字，就是32768-32767，合理
    odometry->previous_left_count = sample->left_count;
    odometry->previous_right_count = sample->right_count;

    mm_per_count = (CAR_PI_F * config->wheel_diameter_mm) /
                   config->encoder_counts_per_wheel_rev;//一圈pi d长度，对应每一圈编码器的计数
                   //相除得到每一计数对应的毫米数
    left_mm = (float)left_delta * mm_per_count;
    right_mm = (float)right_delta * mm_per_count;

    odometry->left_distance_mm += left_mm;
    odometry->right_distance_mm += right_mm;
    odometry->center_distance_mm += (left_mm + right_mm) * 0.5f;//中心走过的距离是两边的平均值
    odometry->encoder_heading_deg +=
        ((right_mm - left_mm) / config->track_width_mm) *
        (180.0f / CAR_PI_F);//角度等于弧长除以半径，算出来是弧度，乘以180/pi转换成角度
    odometry->last_update_ms = sample->timestamp_ms;

    return CAR_OK;
}
//注意轮距指的是两轮中心点之间的距离而不是内侧的距离，到时候要根据实际情况替换
