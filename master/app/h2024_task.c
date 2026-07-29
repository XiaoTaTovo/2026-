#include "app/h2024_task.h"

#include "app/h2026_task.h"

#define H2024_AB_MM (1000.0f)
#define H2024_BC_MM (800.0f)
#define H2024_ARC_RADIUS_MM (400.0f)
#define H2024_DIAGONAL_MM (1280.6248f)
#define H2024_DIAGONAL_TURN_DEG (38.6598f)
#define H2024_LOOP_TURN_DEG (141.3402f)
#define H2024_DEBUG_TURN_DEG (-30.0f)//2024转向环调试参数，目前是向左为正，向右为负

static CarStatus H2024_Add(CarRoute *route,
                           CarSegmentType type,
                           float value,
                           float speed,
                           uint32_t timeout_ms,
                           bool use_line,
                           CarCue cue)
{
    CarRouteSegment *segment;

    if ((route == 0) || (route->count >= CAR_ROUTE_MAX_SEGMENTS)) {
        return CAR_ERROR_CAPACITY;
    }
    segment = &route->segments[route->count++];
    /* 2024 弧段继续由执行器的 0->180deg 兼容兜底处理。 */
    *segment = (CarRouteSegment){type, value, speed, timeout_ms,
                                 use_line, cue, 0.0f,
                                 CAR_LINE_FOLLOW_EXIT_DISTANCE, 0.0f, false};
    return CAR_OK;
}

static CarStatus H2024_AddCue(CarRoute *route, CarCue cue)
{
    return H2024_Add(route, CAR_SEGMENT_CUE, 0.0f, 0.0f, 0U, false, cue);
}

/* ============================================================
 * 改造4：2026 积分赛地图（新增，独立于 2024 对照路线）
 * 场地：220cm × 120cm，黑色半圆弧 + 矩形轨迹，四端点 A/B/C/D。
 *   A→B 顶边直线 = 100cm；B→C 是半径 40cm 的右半圆；
 *   C→D 底边直线 = 100cm。D 左侧还有一个 40cm 延长段，
 *   所以矩形回程必须走 40cm、右转、80cm、右转、40cm 才回到 A。
 * 这里的段全部新写，不复用 2024 的宏，方便单独调参与对照。
 * ============================================================ */
#define H2026_LONG_MM       (1000.0f) /* A→B / C→D 长边 100cm */
#define H2026_SHORT_MM      (400.0f)  /* D 到左角、左上角到 A 均为 40cm */
#define H2026_SIDE_MM       (800.0f)  /* 矩形左侧竖边 80cm */
#define H2026_ARC_RADIUS_MM (400.0f)  /* 半圆弧半径 40cm，带符号表方向 */
#define H2026_ARC_ANGLE_DEG (180.0f)  /* 改造1：半圆 = 180°，写进段字段 */
#define H2026_CORNER_DEG    (-90.0f)  /* 实测约定：左正右负，矩形两角均右转 */
#define H2026_DIAGONAL_MM   (1280.6248f) /* sqrt(1000^2 + 800^2) */
#define H2026_DIAGONAL_TURN_DEG (38.6598f) /* atan(800 / 1000) */
#define H2026_LINE_EVENT_ARM_RATIO (0.75f) /* 黑线边后25%才检测拐角/线尾 */
#define H2026_ITEM_4_LAPS   (3U)

/* 改造4 专用添加函数：比 H2024_Add 多一个 arc_angle_deg 入参，
 * 让每个弧段能独立指定退出角度（配合改造1）。2024 的 H2024_Add 不动。 */
static CarStatus H2026_Add(CarRoute *route,
                           CarSegmentType type,
                           float value,
                           float speed,
                           uint32_t timeout_ms,
                           bool use_line,
                           CarCue cue,
                           float arc_angle_deg)
{
    CarRouteSegment *segment;

    if ((route == 0) || (route->count >= CAR_ROUTE_MAX_SEGMENTS)) {
        return CAR_ERROR_CAPACITY;
    }
    segment = &route->segments[route->count++];
    *segment = (CarRouteSegment){type, value, speed, timeout_ms,
                                 use_line, cue, arc_angle_deg,
                                 CAR_LINE_FOLLOW_EXIT_DISTANCE, 0.0f,
                                 type == CAR_SEGMENT_ARC};
    return CAR_OK;
}

static CarStatus H2026_AddCue(CarRoute *route, CarCue cue)
{
    return H2026_Add(route, CAR_SEGMENT_CUE, 0.0f, 0.0f, 0U,
                     false, cue, 0.0f);
}

static CarStatus H2026_AddLineFollow(CarRoute *route,
                                     float distance_mm,
                                     float speed_mm_s,
                                     uint32_t timeout_ms,
                                     CarLineFollowExit exit_mode)
{
    CarStatus status = H2026_Add(route, CAR_SEGMENT_LINE_FOLLOW,
                                 distance_mm, speed_mm_s, timeout_ms,
                                 true, CAR_CUE_NONE, 0.0f);
    if (status == CAR_OK) {
        CarRouteSegment *segment = &route->segments[route->count - 1U];
        segment->line_follow_exit = exit_mode;
        segment->line_event_arm_ratio = H2026_LINE_EVENT_ARM_RATIO;
    }
    return status;
}

static CarStatus H2026_AddStraightHandoff(CarRoute *route,
                                          float distance_mm,
                                          const CarConfig *config,
                                          CarLineFollowExit handoff_mode)
{
    CarStatus status = H2026_Add(route, CAR_SEGMENT_STRAIGHT,
                                 distance_mm,
                                 config->straight_speed_mm_s,
                                 config->straight_timeout_ms,
                                 true, CAR_CUE_NONE, 0.0f);
    if (status == CAR_OK) {
        route->segments[route->count - 1U].line_follow_exit = handoff_mode;
    }
    return status;
}

static CarStatus H2026_AddArc(CarRoute *route,
                              float signed_radius_mm,
                              const CarConfig *config,
                              bool handoff_to_straight)
{
    CarStatus status = H2026_Add(route, CAR_SEGMENT_ARC,
                                 signed_radius_mm,
                                 config->arc_speed_mm_s,
                                 config->arc_timeout_ms,
                                 true, CAR_CUE_NONE,
                                 H2026_ARC_ANGLE_DEG);
    if (status == CAR_OK) {
        route->segments[route->count - 1U].handoff_arc_heading =
            handoff_to_straight;
    }
    return status;
}

/* 2026 任务1：从 A 直行到 B。后半程看到 B 点黑线可提前结束，
 * 里程 1000mm 仍是漏检时的停车兜底。终点 STOP 由公共逻辑追加。 */
static CarStatus H2026_BuildItem1Route(CarRoute *route,
                                       const CarConfig *config)
{
    if ((route == 0) || (config == 0)) {
        return CAR_ERROR_ARG;
    }

    if ((H2026_Add(route, CAR_SEGMENT_STRAIGHT, H2026_LONG_MM,
                   config->straight_speed_mm_s, config->straight_timeout_ms,
                   true, CAR_CUE_NONE, 0.0f) != CAR_OK) ||
        (H2026_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK)) {
        return CAR_ERROR_CAPACITY;
    }
    return CAR_OK;
}

/* 2026 任务2真实单圈：A→B→C→D→左下角→左上角→A。
 * A-B/C-D在白地直行并以黑线端点切段，B-C沿半圆巡线；D点之后立即
 * 切换为矩形黑线巡线，拐角/线尾事件受里程窗口约束并保留距离兜底。 */
static CarStatus H2026_BuildItem2Route(CarRoute *route,
                                       const CarConfig *config)
{
    if ((route == 0) || (config == 0)) {
        return CAR_ERROR_ARG;
    }

    /* 段1：A→B，100cm 直线；检测 B 点黑线，距离是兜底。 */
    if ((H2026_AddStraightHandoff(
             route, H2026_LONG_MM, config,
             CAR_LINE_FOLLOW_EXIT_SENSOR_AXLE) != CAR_OK) ||
        (H2026_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK) ||
        /* 段2：B→C，向右半圆 180°，半径 40cm，灰度贴弧。 */
        (H2026_Add(route, CAR_SEGMENT_ARC, -H2026_ARC_RADIUS_MM,
                   config->arc_speed_mm_s, config->arc_timeout_ms,
                   true, CAR_CUE_NONE, H2026_ARC_ANGLE_DEG) != CAR_OK) ||
        (H2026_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK) ||
        /* 段3：C→D，100cm 直线；检测 D 点黑线，距离是兜底。 */
        (H2026_AddStraightHandoff(
             route, H2026_LONG_MM, config,
             CAR_LINE_FOLLOW_EXIT_SENSOR_AXLE) != CAR_OK) ||
        (H2026_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK) ||
        /* 段4：D→左下角40cm；巡线，末段检测右拐角，距离兜底。 */
        (H2026_AddLineFollow(route, H2026_SHORT_MM,
                             config->arc_speed_mm_s,
                             config->straight_timeout_ms,
                             CAR_LINE_FOLLOW_EXIT_RIGHT_CORNER) != CAR_OK) ||
        /* 段5：矩形左下角右转 90°。 */
        (H2026_Add(route, CAR_SEGMENT_TURN, H2026_CORNER_DEG,
                   config->turn_wheel_speed_mm_s, config->turn_timeout_ms,
                   true, CAR_CUE_NONE, 0.0f) != CAR_OK) ||
        /* 段6：左侧竖边向上80cm；巡线，末段检测右拐角。 */
        (H2026_AddLineFollow(route, H2026_SIDE_MM,
                             config->arc_speed_mm_s,
                             config->straight_timeout_ms,
                             CAR_LINE_FOLLOW_EXIT_RIGHT_CORNER) != CAR_OK) ||
        /* 段7：矩形左上角右转 90°。 */
        (H2026_Add(route, CAR_SEGMENT_TURN, H2026_CORNER_DEG,
                   config->turn_wheel_speed_mm_s, config->turn_timeout_ms,
                   true, CAR_CUE_NONE, 0.0f) != CAR_OK) ||
        /* 段8：左上角→A 40cm；巡线，末段连续丢线判定A点。 */
        (H2026_AddLineFollow(route, H2026_SHORT_MM,
                             config->arc_speed_mm_s,
                             config->straight_timeout_ms,
                             CAR_LINE_FOLLOW_EXIT_LOST) != CAR_OK)) {
        return CAR_ERROR_CAPACITY;
    }
    return CAR_OK;
}

/* 2026 任务3变序单圈：车头在 A 点朝正东方向，程序先右转对准 C，
 * 再走 A->C 对角线、C->B 左半圆、B->D 对角线和矩形黑线回到 A。
 * 固定正向起步是为了消除人工斜着对准 C 带来的重复性误差。 */
static CarStatus H2026_BuildItem3Route(CarRoute *route,
                                       const CarConfig *config)
{
    if ((route == 0) || (config == 0)) {
        return CAR_ERROR_ARG;
    }

    /* A 点朝正东，先右转 atan(800/1000) 后才正对 C。 */
    if ((H2026_Add(route, CAR_SEGMENT_TURN, -H2026_DIAGONAL_TURN_DEG,
                   config->turn_wheel_speed_mm_s, config->turn_timeout_ms,
                   false, CAR_CUE_NONE, 0.0f) != CAR_OK) ||
        /* A->C：100cm x 80cm 矩形对角线，C 点黑线作为提前退出证据。 */
        (H2026_AddStraightHandoff(
             route, H2026_DIAGONAL_MM, config,
             CAR_LINE_FOLLOW_EXIT_DIAGONAL_AXLE) != CAR_OK) ||
        (H2026_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK) ||
        /* A->C 航向到 C 点圆弧东向切线：左转 38.66deg。 */
        (H2026_Add(route, CAR_SEGMENT_TURN, H2026_DIAGONAL_TURN_DEG,
                   config->turn_wheel_speed_mm_s, config->turn_timeout_ms,
                   false, CAR_CUE_NONE, 0.0f) != CAR_OK) ||
        /* C->B：从半圆下端沿右半圆左转 180deg 到上端。 */
        (H2026_AddArc(route, H2026_ARC_RADIUS_MM, config, false) != CAR_OK) ||
        (H2026_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK) ||
        /* B 点西向切线到 B->D 对角线：继续左转 38.66deg。 */
        (H2026_Add(route, CAR_SEGMENT_TURN, H2026_DIAGONAL_TURN_DEG,
                   config->turn_wheel_speed_mm_s, config->turn_timeout_ms,
                   false, CAR_CUE_NONE, 0.0f) != CAR_OK) ||
        /* B->D：第二条 128.06cm 对角线，D 点黑线作为退出证据。 */
        (H2026_AddStraightHandoff(
             route, H2026_DIAGONAL_MM, config,
             CAR_LINE_FOLLOW_EXIT_DIAGONAL_AXLE) != CAR_OK) ||
        (H2026_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK) ||
        /* B->D 航向转到 D 点向左的黑线：右转 38.66deg。 */
        (H2026_Add(route, CAR_SEGMENT_TURN, -H2026_DIAGONAL_TURN_DEG,
                   config->turn_wheel_speed_mm_s, config->turn_timeout_ms,
                   false, CAR_CUE_NONE, 0.0f) != CAR_OK) ||
        /* D->左下角 40cm，检测第一个右直角。 */
        (H2026_AddLineFollow(route, H2026_SHORT_MM,
                             config->arc_speed_mm_s,
                             config->straight_timeout_ms,
                             CAR_LINE_FOLLOW_EXIT_RIGHT_CORNER) != CAR_OK) ||
        (H2026_Add(route, CAR_SEGMENT_TURN, H2026_CORNER_DEG,
                   config->turn_wheel_speed_mm_s, config->turn_timeout_ms,
                   true, CAR_CUE_NONE, 0.0f) != CAR_OK) ||
        /* 左侧竖边 80cm，检测第二个右直角。 */
        (H2026_AddLineFollow(route, H2026_SIDE_MM,
                             config->arc_speed_mm_s,
                             config->straight_timeout_ms,
                             CAR_LINE_FOLLOW_EXIT_RIGHT_CORNER) != CAR_OK) ||
        (H2026_Add(route, CAR_SEGMENT_TURN, H2026_CORNER_DEG,
                   config->turn_wheel_speed_mm_s, config->turn_timeout_ms,
                   true, CAR_CUE_NONE, 0.0f) != CAR_OK) ||
        /* 左上角->A 40cm，连续丢线判定 A 点并停车。 */
        (H2026_AddLineFollow(route, H2026_SHORT_MM,
                             config->arc_speed_mm_s,
                             config->straight_timeout_ms,
                             CAR_LINE_FOLLOW_EXIT_LOST) != CAR_OK)) {
        return CAR_ERROR_CAPACITY;
    }
    return CAR_OK;
}

/* 2026 任务4只验证最终路线：连续复用任务3单圈三次，不接二维码。
 * 每圈任务3都会从 A 点正东方向执行同一个右转入口，因此三圈起步
 * 条件一致；圈间只插入到达 A 的提示，唯一 STOP 由公共逻辑放在末尾。 */
static CarStatus H2026_BuildItem4Route(CarRoute *route,
                                       const CarConfig *config)
{
    if ((route == 0) || (config == 0)) {
        return CAR_ERROR_ARG;
    }

    for (uint8_t lap = 0U; lap < H2026_ITEM_4_LAPS; lap++) {
        if ((H2026_BuildItem3Route(route, config) != CAR_OK) ||
            (H2026_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK)) {
            return CAR_ERROR_CAPACITY;
        }
    }
    return CAR_OK;
}

static CarStatus H2024_AddItem3Loop(CarRoute *route,
                                    const CarConfig *config,
                                    bool add_entry_turn)
{
    if (add_entry_turn &&
        (H2024_Add(route, CAR_SEGMENT_TURN, H2024_LOOP_TURN_DEG,
                   config->turn_wheel_speed_mm_s, config->turn_timeout_ms,
                   false, CAR_CUE_NONE) != CAR_OK)) {
        return CAR_ERROR_CAPACITY;
    }
    if ((H2024_Add(route, CAR_SEGMENT_STRAIGHT, H2024_DIAGONAL_MM,
                   config->straight_speed_mm_s, config->straight_timeout_ms,
                   false, CAR_CUE_NONE) != CAR_OK) ||
        (H2024_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK) ||
        (H2024_Add(route, CAR_SEGMENT_TURN, H2024_DIAGONAL_TURN_DEG,
                   config->turn_wheel_speed_mm_s, config->turn_timeout_ms,
                   false, CAR_CUE_NONE) != CAR_OK) ||
        (H2024_Add(route, CAR_SEGMENT_ARC, H2024_ARC_RADIUS_MM,
                   config->arc_speed_mm_s, config->arc_timeout_ms,
                   true, CAR_CUE_NONE) != CAR_OK) ||
        (H2024_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK) ||
        (H2024_Add(route, CAR_SEGMENT_TURN, H2024_DIAGONAL_TURN_DEG,
                   config->turn_wheel_speed_mm_s, config->turn_timeout_ms,
                   false, CAR_CUE_NONE) != CAR_OK) ||
        (H2024_Add(route, CAR_SEGMENT_STRAIGHT, H2024_DIAGONAL_MM,
                   config->straight_speed_mm_s, config->straight_timeout_ms,
                   false, CAR_CUE_NONE) != CAR_OK) ||
        (H2024_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK) ||
        (H2024_Add(route, CAR_SEGMENT_TURN, H2024_LOOP_TURN_DEG,
                   config->turn_wheel_speed_mm_s, config->turn_timeout_ms,
                   false, CAR_CUE_NONE) != CAR_OK) ||
        (H2024_Add(route, CAR_SEGMENT_ARC, H2024_ARC_RADIUS_MM,
                   config->arc_speed_mm_s, config->arc_timeout_ms,
                   true, CAR_CUE_NONE) != CAR_OK) ||
        (H2024_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK)) {
        return CAR_ERROR_CAPACITY;
    }
    return CAR_OK;
}

CarStatus H2024_BuildRoute(H2024Mode mode,
                           const CarConfig *config,
                           CarRoute *route)
{
    if ((config == 0) || (route == 0)) {
        return CAR_ERROR_ARG;
    }
    if (H2026_ModeIsOfficial(mode)) {
        return H2026_BuildRoute(mode, config, route);
    }
    *route = (CarRoute){0};

    if (H2024_AddCue(route, CAR_CUE_START) != CAR_OK) {
        return CAR_ERROR_CAPACITY;
    }

    switch (mode) {
        case H2024_MODE_ITEM_1:
            if ((H2024_Add(route, CAR_SEGMENT_STRAIGHT, H2024_AB_MM,
                           config->straight_speed_mm_s,
                           config->straight_timeout_ms, false,
                           CAR_CUE_NONE) != CAR_OK) ||
                (H2024_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK)) {
                return CAR_ERROR_CAPACITY;
            }
            break;

        case H2024_MODE_ITEM_2:
            if ((H2024_Add(route, CAR_SEGMENT_STRAIGHT, H2024_AB_MM,
                           config->straight_speed_mm_s,
                           config->straight_timeout_ms, false,
                           CAR_CUE_NONE) != CAR_OK) ||
                (H2024_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK) ||
                (H2024_Add(route, CAR_SEGMENT_ARC, -H2024_ARC_RADIUS_MM,
                           config->arc_speed_mm_s, config->arc_timeout_ms,
                           true, CAR_CUE_NONE) != CAR_OK) ||
                (H2024_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK) ||
                (H2024_Add(route, CAR_SEGMENT_STRAIGHT, H2024_AB_MM,
                           config->straight_speed_mm_s,
                           config->straight_timeout_ms, false,
                           CAR_CUE_NONE) != CAR_OK) ||
                (H2024_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK) ||
                (H2024_Add(route, CAR_SEGMENT_ARC, -H2024_ARC_RADIUS_MM,
                           config->arc_speed_mm_s, config->arc_timeout_ms,
                           true, CAR_CUE_NONE) != CAR_OK) ||
                (H2024_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK)) {
                return CAR_ERROR_CAPACITY;
            }
            break;

        case H2024_MODE_ITEM_3:
            if (H2024_AddItem3Loop(route, config, false) != CAR_OK) {
                return CAR_ERROR_CAPACITY;
            }
            break;

        case H2024_MODE_ITEM_4:
            for (uint8_t loop = 0U; loop < 4U; loop++) {
                if (H2024_AddItem3Loop(route, config, loop > 0U) != CAR_OK) {
                    return CAR_ERROR_CAPACITY;
                }
            }
            break;

        case H2024_MODE_TURN_DEBUG:
            if (H2024_Add(route, CAR_SEGMENT_TURN, H2024_DEBUG_TURN_DEG,
                          config->turn_wheel_speed_mm_s,
                          config->turn_timeout_ms, false,
                          CAR_CUE_NONE) != CAR_OK) {
                return CAR_ERROR_CAPACITY;
            }
            break;

        /* 2026 各项任务独立构建，不影响 2024 对照路线。 */
        case H2026_MODE_ITEM_1:
            if (H2026_BuildItem1Route(route, config) != CAR_OK) {
                return CAR_ERROR_CAPACITY;
            }
            break;

        case H2026_MODE_ITEM_2:
            if (H2026_BuildItem2Route(route, config) != CAR_OK) {
                return CAR_ERROR_CAPACITY;
            }
            break;

        case H2026_MODE_ITEM_3:
            if (H2026_BuildItem3Route(route, config) != CAR_OK) {
                return CAR_ERROR_CAPACITY;
            }
            break;

        case H2026_MODE_ITEM_4:
            if (H2026_BuildItem4Route(route, config) != CAR_OK) {
                return CAR_ERROR_CAPACITY;
            }
            break;

        default:
            return CAR_ERROR_ARG;
    }

    if (H2024_Add(route, CAR_SEGMENT_STOP, 0.0f, 0.0f, 0U,
                  false, CAR_CUE_FINISH) != CAR_OK) {
        return CAR_ERROR_CAPACITY;
    }
    return CAR_OK;
}
