#ifndef CHASSIS_FEEDFORWARD_LINK_H
#define CHASSIS_FEEDFORWARD_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "core/chassis_feedforward.h"

#define CHASSIS_FEEDFORWARD_LINK_TX_RING_SIZE (128U)
#define CHASSIS_FEEDFORWARD_LINK_RX_RING_SIZE (128U)

typedef struct {
    uint8_t tx_ring[CHASSIS_FEEDFORWARD_LINK_TX_RING_SIZE];
    uint8_t rx_ring[CHASSIS_FEEDFORWARD_LINK_RX_RING_SIZE];
    uint16_t tx_head;
    uint16_t tx_tail;
    uint16_t rx_head;
    uint16_t rx_tail;
    uint16_t next_sequence;
    uint32_t tx_frame_count;
    uint32_t tx_drop_count;
    uint32_t tx_byte_count;
    uint32_t rx_byte_count;
    uint32_t rx_drop_count;
    bool initialized;
} ChassisFeedforwardLink;

void ChassisFeedforwardLink_Init(ChassisFeedforwardLink *link);
bool ChassisFeedforwardLink_QueueState(ChassisFeedforwardLink *link,
                                       const ChassisFeedforwardSample *sample,
                                       uint16_t flags);
void ChassisFeedforwardLink_Service(ChassisFeedforwardLink *link);

#endif
