#ifndef PROJECT_MODE_H
#define PROJECT_MODE_H

#define PROJECT_MODE_BLUETOOTH_TUNING (0)
#define PROJECT_MODE_H2024_ITEM_1      (1)
#define PROJECT_MODE_H2024_ITEM_2      (2)
#define PROJECT_MODE_H2024_ITEM_3      (3)
#define PROJECT_MODE_H2024_ITEM_4      (4)
#define PROJECT_MODE_GRAY_ADC_DEBUG    (5)
#define PROJECT_MODE_TURN_DEBUG        (6)
/* 2026 积分赛任务独立选择：ITEM_1=A->B，ITEM_2=A->B->C->D->A，
 * ITEM_3=固定正向起步的变序单圈，ITEM_4=同路线连续三圈。
 * MAIN 指向当前最终三圈路线。 */
#define PROJECT_MODE_H2026_ITEM_1      (7)
#define PROJECT_MODE_H2026_ITEM_2      (8)
#define PROJECT_MODE_H2026_ITEM_3      (9)
#define PROJECT_MODE_H2026_ITEM_4      (10)
#define PROJECT_MODE_H2026_MAIN        PROJECT_MODE_H2026_ITEM_4

/* Official 2026 H problem. B1 video is an independent subsystem. */
#define PROJECT_MODE_H2026_B2          (20)
#define PROJECT_MODE_H2026_B3          (21)
#define PROJECT_MODE_H2026_B4          (22)
#define PROJECT_MODE_H2026_B5          (23)
#define PROJECT_MODE_H2026_B6          (24)

/* Unlock each later item only after that exact end-to-end path is complete. */
#define H2026_B3_CONTROL_AVAILABLE     (0)
#define H2026_B4_CONTROL_AVAILABLE     (0)
#define H2026_B5_CONTROL_AVAILABLE     (0)
#define H2026_B6_CONTROL_AVAILABLE     (0)

/* The burnable baseline is official item B2. Select another mode explicitly. */
#ifndef PROJECT_MODE
#define PROJECT_MODE PROJECT_MODE_H2026_B2

#endif

/* Validate both the historical range and the official problem range. */
#if ((PROJECT_MODE < PROJECT_MODE_BLUETOOTH_TUNING) || \
     ((PROJECT_MODE > PROJECT_MODE_H2026_ITEM_4) && \
      (PROJECT_MODE < PROJECT_MODE_H2026_B2)) || \
     (PROJECT_MODE > PROJECT_MODE_H2026_B6))
#error "PROJECT_MODE is invalid"
#endif

#if (PROJECT_MODE == PROJECT_MODE_H2026_B3) && !H2026_B3_CONTROL_AVAILABLE
#error "B3 is unavailable until its ball-control path is implemented"
#elif (PROJECT_MODE == PROJECT_MODE_H2026_B4) && !H2026_B4_CONTROL_AVAILABLE
#error "B4 is unavailable until its combined control path is implemented"
#elif (PROJECT_MODE == PROJECT_MODE_H2026_B5) && !H2026_B5_CONTROL_AVAILABLE
#error "B5 is unavailable until its combined control path is implemented"
#elif (PROJECT_MODE == PROJECT_MODE_H2026_B6) && !H2026_B6_CONTROL_AVAILABLE
#error "B6 is unavailable until its arbitrary-position path is implemented"
#endif

#endif
