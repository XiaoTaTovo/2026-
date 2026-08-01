#ifndef VOFA_TELEMETRY_H
#define VOFA_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "firmware.h"

void VofaTelemetry_Init(void);
void VofaTelemetry_SendBanner(void);
void VofaTelemetry_SendSpeedLoopBanner(void);
void VofaTelemetry_SendFrame(const CarFirmware *firmware,
                             uint32_t uptime_ms);
void VofaTelemetry_SendSpeedLoopFrame(const CarFirmware *firmware,
                                      uint32_t uptime_ms);
void VofaTelemetry_SendSpeedLoopArm(uint32_t trial_id,
                                    int16_t target_mm_s,
                                    uint32_t uptime_ms);
void VofaTelemetry_SendSpeedLoopDone(uint32_t trial_id,
                                     const char *reason,
                                     uint32_t duration_ms,
                                     uint32_t faults);
void VofaTelemetry_SendLineTrialArm(uint32_t trial_id,
                                    uint32_t uptime_ms);
void VofaTelemetry_SendLineTrialDone(uint32_t trial_id,
                                     uint32_t duration_ms,
                                     uint32_t faults,
                                     uint32_t exit_reason);
void VofaTelemetry_SendLineTrialReject(uint32_t action,
                                       int32_t status,
                                       uint32_t uptime_ms);
void VofaTelemetry_TxIrqHandler(void);
void VofaTelemetry_PushRxFromIsr(uint8_t byte);
bool VofaTelemetry_ProcessCommands(CarFirmware *firmware,
                                   uint32_t uptime_ms);

#endif
