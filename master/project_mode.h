#ifndef PROJECT_MODE_H
#define PROJECT_MODE_H

/* Official 2026 H problem items. B1 video transmission is independent. */
#define PROJECT_MODE_H2026_B2 (2)
#define PROJECT_MODE_H2026_B3 (3)
#define PROJECT_MODE_H2026_B4 (4)
#define PROJECT_MODE_H2026_B5 (5)
#define PROJECT_MODE_H2026_B6 (6)

/* Unlock only after the corresponding control path passes its own tests. */
#define H2026_B3_CONTROL_AVAILABLE (0)
#define H2026_B4_CONTROL_AVAILABLE (0)
#define H2026_B5_CONTROL_AVAILABLE (0)
#define H2026_B6_CONTROL_AVAILABLE (0)

#ifndef PROJECT_MODE
#define PROJECT_MODE PROJECT_MODE_H2026_B2
#endif

#if (PROJECT_MODE < PROJECT_MODE_H2026_B2) || \
    (PROJECT_MODE > PROJECT_MODE_H2026_B6)
#error "PROJECT_MODE must select an official 2026 H item"
#endif

#if (PROJECT_MODE == PROJECT_MODE_H2026_B3) && !H2026_B3_CONTROL_AVAILABLE
#error "B3 is unavailable until the ball-control path is implemented"
#elif (PROJECT_MODE == PROJECT_MODE_H2026_B4) && !H2026_B4_CONTROL_AVAILABLE
#error "B4 is unavailable until the combined control path is implemented"
#elif (PROJECT_MODE == PROJECT_MODE_H2026_B5) && !H2026_B5_CONTROL_AVAILABLE
#error "B5 is unavailable until the combined control path is implemented"
#elif (PROJECT_MODE == PROJECT_MODE_H2026_B6) && !H2026_B6_CONTROL_AVAILABLE
#error "B6 is unavailable until arbitrary-position control is implemented"
#endif

#endif
