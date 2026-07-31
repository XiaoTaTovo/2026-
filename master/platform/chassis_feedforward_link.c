#include "platform/chassis_feedforward_link.h"

#include "ti_msp_dl_config.h"

/* Source: master/Debug/syscfg/ti_msp_dl_config.h generated from
 * empty_mspm0g3507.syscfg: UART_DEBUG is UART0, PA10/PA11, 115200 8N1. */

static uint16_t ChassisFeedforwardLink_Next(uint16_t index, uint16_t size)
{
    index++;
    return (index == size) ? 0U : index;
}

static uint16_t ChassisFeedforwardLink_Free(uint16_t head,
                                            uint16_t tail,
                                            uint16_t size)
{
    if (head >= tail) {
        return (uint16_t)(size - (head - tail) - 1U);
    }
    return (uint16_t)(tail - head - 1U);
}

void ChassisFeedforwardLink_Init(ChassisFeedforwardLink *link)
{
    if (link == 0) {
        return;
    }
    *link = (ChassisFeedforwardLink){0};
    link->initialized = true;
}

bool ChassisFeedforwardLink_QueueState(ChassisFeedforwardLink *link,
                                       const ChassisFeedforwardSample *sample,
                                       uint16_t flags)
{
    uint8_t frame[CHASSIS_FEEDFORWARD_FRAME_SIZE];

    if ((link == 0) || (sample == 0) || !link->initialized) {
        return false;
    }
    if (ChassisFeedforwardLink_Free(link->tx_head, link->tx_tail,
                                    CHASSIS_FEEDFORWARD_LINK_TX_RING_SIZE) <
        CHASSIS_FEEDFORWARD_FRAME_SIZE) {
        link->tx_drop_count++;
        return false;
    }
    if (!ChassisFeedforward_EncodeState(sample, flags, link->next_sequence,
                                        frame)) {
        return false;
    }
    link->next_sequence++;
    for (uint16_t index = 0U; index < CHASSIS_FEEDFORWARD_FRAME_SIZE;
         ++index) {
        link->tx_ring[link->tx_head] = frame[index];
        link->tx_head = ChassisFeedforwardLink_Next(
            link->tx_head, CHASSIS_FEEDFORWARD_LINK_TX_RING_SIZE);
    }
    link->tx_frame_count++;
    return true;
}

void ChassisFeedforwardLink_Service(ChassisFeedforwardLink *link)
{
    if ((link == 0) || !link->initialized) {
        return;
    }
    while (!DL_UART_isRXFIFOEmpty(UART_DEBUG_INST)) {
        uint16_t next = ChassisFeedforwardLink_Next(
            link->rx_head, CHASSIS_FEEDFORWARD_LINK_RX_RING_SIZE);

        if (next == link->rx_tail) {
            (void)DL_UART_receiveData(UART_DEBUG_INST);
            link->rx_drop_count++;
        } else {
            link->rx_ring[link->rx_head] =
                (uint8_t)DL_UART_receiveData(UART_DEBUG_INST);
            link->rx_head = next;
            link->rx_byte_count++;
        }
    }
    while ((link->tx_tail != link->tx_head) &&
           !DL_UART_isTXFIFOFull(UART_DEBUG_INST)) {
        DL_UART_transmitData(UART_DEBUG_INST, link->tx_ring[link->tx_tail]);
        link->tx_tail = ChassisFeedforwardLink_Next(
            link->tx_tail, CHASSIS_FEEDFORWARD_LINK_TX_RING_SIZE);
        link->tx_byte_count++;
    }
}
