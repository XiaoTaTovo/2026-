#ifndef LINE8_GPIO_DIAGNOSTIC_H
#define LINE8_GPIO_DIAGNOSTIC_H

/*
 * Set to 1 only after SysConfig has generated GPIO_LINE8_X1..X8. This build
 * reads every sensor channel, prints the raw digital state, and never enables
 * the motors.
 */
#ifndef H2026_LINE8_GPIO_DIAGNOSTIC_BUILD
#define H2026_LINE8_GPIO_DIAGNOSTIC_BUILD (1)
#endif

#endif
