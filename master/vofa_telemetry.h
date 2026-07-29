#ifndef VOFA_TELEMETRY_H
#define VOFA_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "firmware.h"

void VofaTelemetry_Init(void);
void VofaTelemetry_SendBanner(void);
void VofaTelemetry_SendFrame(const CarFirmware *firmware,
                             uint32_t uptime_ms);
void VofaTelemetry_TxIrqHandler(void);
void VofaTelemetry_PushRxFromIsr(uint8_t byte);
bool VofaTelemetry_ProcessCommands(CarFirmware *firmware,
                                   uint32_t uptime_ms);

#endif
