#ifndef SAFETY_SUPERVISOR_H
#define SAFETY_SUPERVISOR_H

#include "car_config.h"
#include "car_types.h"

uint32_t CarSafety_Evaluate(const CarConfig *config,
                            uint32_t now_ms,
                            const CarInputSnapshot *input,
                            bool motion_requested,
                            bool gray_required);

#endif
