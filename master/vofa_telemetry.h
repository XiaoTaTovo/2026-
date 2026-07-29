#ifndef VOFA_TELEMETRY_H
#define VOFA_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "bluetooth_control.h"
#include "firmware.h"

void VofaTelemetry_SendBanner(void);
void VofaTelemetry_Send(
    const BluetoothControlStatus *status, uint32_t uptimeMs);
void VofaTelemetry_SendRouteBanner(void);
void VofaTelemetry_SendRoute(const CarFirmware *firmware, uint32_t uptimeMs);
void VofaTelemetry_TxInit(void);
void VofaTelemetry_TxIrqHandler(void);
void VofaTelemetry_RouteCommandInit(void);
void VofaTelemetry_RouteCommandPushRxFromIsr(uint8_t byte);
bool VofaTelemetry_ProcessRouteCommands(CarFirmware *firmware,
                                        uint32_t uptimeMs);

#endif
