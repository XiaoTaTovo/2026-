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
void VofaTelemetry_SendLineTuningBanner(void);
void VofaTelemetry_SendLineTuningBoot(const char *stage);
void VofaTelemetry_SendLineTuningKeyLevels(bool key1_level,
                                           bool key2_level,
                                           bool key3_level);
void VofaTelemetry_SendLineTuningCalibrationState(uint32_t state);
void VofaTelemetry_SendLineTuningOledStatus(uint32_t status,
                                             uint32_t page);
void VofaTelemetry_SendLineTuningArm(uint32_t trial_id,
                                     uint32_t mode,
                                     uint32_t uptime_ms);
void VofaTelemetry_SendLineTuningDone(uint32_t trial_id,
                                      const char *reason,
                                      uint32_t duration_ms,
                                      uint32_t faults);
void VofaTelemetry_SendLineTuningFrame(const CarFirmware *firmware,
                                       uint32_t uptime_ms);
void VofaTelemetry_ServiceTx(void);
void VofaTelemetry_TxIrqHandler(void);
void VofaTelemetry_PushRxFromIsr(uint8_t byte);
bool VofaTelemetry_ProcessCommands(CarFirmware *firmware,
                                   uint32_t uptime_ms);
bool VofaTelemetry_TakeParameterUpdateOk(void);

#endif
