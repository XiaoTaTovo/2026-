#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* Set a sign to -1 if forward wheel motion produces negative counts. */
#ifndef ENC_LEFT_SIGN
#define ENC_LEFT_SIGN (-1)
#endif

#ifndef ENC_RIGHT_SIGN
#define ENC_RIGHT_SIGN (+1)
#endif

/*
 * Leave these at zero until one output-wheel revolution has been measured.
 * The active H2026 build instead uses the measured values in platform config.
 */
#ifndef ENC_LEFT_COUNTS_PER_REV
#define ENC_LEFT_COUNTS_PER_REV (0U)
#endif

#ifndef ENC_RIGHT_COUNTS_PER_REV
#define ENC_RIGHT_COUNTS_PER_REV (0U)
#endif

#if ((ENC_LEFT_SIGN != 1) && (ENC_LEFT_SIGN != -1))
#error "ENC_LEFT_SIGN must be +1 or -1"
#endif

#if ((ENC_RIGHT_SIGN != 1) && (ENC_RIGHT_SIGN != -1))
#error "ENC_RIGHT_SIGN must be +1 or -1"
#endif

extern volatile int32_t gEncoderLeftCount;
extern volatile int32_t gEncoderRightCount;

void Encoder_Init(void);
void Encoder_ResetCounts(void);

int32_t Encoder_GetLeftCount(void);
int32_t Encoder_GetRightCount(void);

/* Return cumulative_count - previous_sample without clearing total counts. */
int32_t Encoder_GetLeftDelta(void);
int32_t Encoder_GetRightDelta(void);

#endif
