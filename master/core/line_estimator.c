#include "core/line_estimator.h"

static int16_t CarLine_ClampPosition(int32_t position)
{
    if (position > INT16_MAX) {
        return INT16_MAX;
    }
    if (position < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)position;
}

CarStatus CarLineEstimator_Update(const CarConfig *config,
                                  const CarGraySample *sample,
                                  CarLineEstimate *estimate)
//sample是读取的原始数据，包括  8个灰度传感器的归一化值、时间戳、有效否
//estimate是线的估计数据，包括  线的位置、置信度、活跃计数、时间戳、有效否
{
    static const int16_t weights[8] = {
        -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
    };
    uint32_t sum = 0U;
    uint32_t left_sum = 0U;
    uint32_t right_sum = 0U;
    int32_t weighted_sum = 0;
    uint8_t active_count = 0U;
    uint8_t first_active = 8U;
    uint8_t last_active = 0U;
    uint16_t minimum_1 = UINT16_MAX;
    uint16_t minimum_2 = UINT16_MAX;
    uint16_t maximum = 0U;
    uint32_t track_sum = 0U;
    int32_t track_weighted_sum = 0;
    uint8_t track_first_active = 8U;
    uint8_t track_last_active = 0U;
    bool previous_track_active = false;

    if ((config == 0) || (sample == 0) || (estimate == 0)) {
        return CAR_ERROR_ARG;
    }

    *estimate = (CarLineEstimate){0};
    estimate->timestamp_ms = sample->timestamp_ms;

    if (!sample->valid) {
        return CAR_OK;
    }

    for (uint8_t i = 0U; i < 8U; i++) {
        uint16_t value = sample->normalized[i];
        if (value < minimum_1) {
            minimum_2 = minimum_1;
            minimum_1 = value;
        } else if (value < minimum_2) {
            minimum_2 = value;
        }
        if (value > maximum) {
            maximum = value;
        }
        if (value >= config->gray_min_signal) {
            sum += value;
            weighted_sum += (int32_t)value * weights[i];
            active_count++;
            estimate->active_mask |= (uint8_t)(1U << i);
            if (i < 4U) {
                left_sum += value;
            } else {
                right_sum += value;
            }
            if (first_active == 8U) {
                first_active = i;
            }
            last_active = i;
        }
    }

    /* The two darkest channels estimate the local floor. Subtracting this
     * floor makes a thin coherent peak survive illumination drift without
     * changing the legacy absolute-threshold result above. */
    estimate->adaptive_background = (uint16_t)(
        ((uint32_t)minimum_1 + (uint32_t)minimum_2) / 2U);
    estimate->adaptive_contrast = (maximum > estimate->adaptive_background) ?
        (uint16_t)(maximum - estimate->adaptive_background) : 0U;
    for (uint8_t i = 0U; i < 8U; i++) {
        uint16_t value = sample->normalized[i];
        uint16_t relative = (value > estimate->adaptive_background) ?
            (uint16_t)(value - estimate->adaptive_background) : 0U;
        bool active = (relative > 0U) &&
                      (relative >= config->gray_relative_delta);

        if (active) {
            track_sum += relative;
            track_weighted_sum += (int32_t)relative * weights[i];
            estimate->track_active_count++;
            estimate->track_active_mask |= (uint8_t)(1U << i);
            if (track_first_active == 8U) {
                track_first_active = i;
            }
            track_last_active = i;
            if (!previous_track_active) {
                estimate->track_cluster_count++;
            }
        }
        previous_track_active = active;
    }
    estimate->track_confidence = (track_sum > UINT16_MAX) ?
        UINT16_MAX : (uint16_t)track_sum;
    estimate->track_active_span = (estimate->track_active_count == 0U) ? 0U :
        (uint8_t)(track_last_active - track_first_active + 1U);
    if (track_sum > 0U) {
        int32_t track_position = track_weighted_sum / (int32_t)track_sum;
        track_position -= (int32_t)config->gray_center_offset;
        estimate->track_position = CarLine_ClampPosition(track_position);
    }

    /* A broad footprint is WIDE only when even its two darkest channels are
     * strongly black. This rejects a true all-black area with local variation,
     * while preserving a narrow peak on a raised but still white background
     * where the legacy low threshold can also produce mask 0xFF. */
    if ((config->gray_wide_min_active > 0U) &&
        (active_count >= config->gray_wide_min_active) &&
        (estimate->adaptive_background >=
         config->gray_wide_min_background)) {
        estimate->pattern = CAR_LINE_PATTERN_WIDE_AREA;
    } else if ((track_sum >= config->gray_track_min_confidence) &&
        (estimate->track_active_count > 0U) &&
        (estimate->track_active_count <= config->gray_track_max_active) &&
        (estimate->track_active_span <= config->gray_track_max_span) &&
        (estimate->track_cluster_count == 1U)) {
        estimate->pattern = CAR_LINE_PATTERN_NARROW_TRACK;
    } else if (estimate->track_active_count > 0U) {
        estimate->pattern = CAR_LINE_PATTERN_SPLIT_NOISE;
    } else {
        estimate->pattern = CAR_LINE_PATTERN_NONE;
    }

    estimate->confidence = (sum > UINT16_MAX) ? UINT16_MAX : (uint16_t)sum;
    estimate->active_count = active_count;
    estimate->left_confidence = (left_sum > UINT16_MAX) ?
        UINT16_MAX : (uint16_t)left_sum;
    estimate->right_confidence = (right_sum > UINT16_MAX) ?
        UINT16_MAX : (uint16_t)right_sum;
    estimate->right_ratio_permille = (sum == 0U) ? 0U :
        (uint16_t)((right_sum * 1000U) / sum);
    estimate->active_span = (active_count == 0U) ? 0U :
        (uint8_t)(last_active - first_active + 1U);
    if ((sum >= config->gray_min_confidence) && (active_count > 0U)) {
        int32_t position = weighted_sum / (int32_t)sum;
        /* 改造3：减去标定的中心偏移，把传感器零点漂移补偿掉，
         * 使 position==0 真正对应车体中线。offset 默认 0 时行为与旧版一致。
         * 减完后夹到 int16_t 范围，避免极端标定值造成溢出。 */
        position -= (int32_t)config->gray_center_offset;
        estimate->position = CarLine_ClampPosition(position);
        estimate->valid = true;
    }

    return CAR_OK;
}
