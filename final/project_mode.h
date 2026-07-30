#ifndef PROJECT_MODE_H
#define PROJECT_MODE_H

/* KEY3 selects all six official items at runtime. */
#define H2026_INITIAL_TASK_NUMBER (1)

#if (H2026_INITIAL_TASK_NUMBER < 1) || (H2026_INITIAL_TASK_NUMBER > 6)
#error "H2026_INITIAL_TASK_NUMBER must be in [1, 6]"
#endif

#endif
