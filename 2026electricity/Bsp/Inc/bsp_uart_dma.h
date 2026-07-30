#ifndef BSP_UART_DMA_H
#define BSP_UART_DMA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#define BSP_UART_DMA_RX_DMA_SIZE 128U
#define BSP_UART_DMA_RX_RING_SIZE 512U
#define BSP_UART_DMA_TX_RING_SIZE 512U
#define BSP_UART_DMA_MAX_PORTS 4U

typedef enum
{
    BSP_UART_DMA_OK = 0,
    BSP_UART_DMA_EMPTY,
    BSP_UART_DMA_FULL,
    BSP_UART_DMA_INVALID_ARGUMENT,
    BSP_UART_DMA_INVALID_CONFIG,
    BSP_UART_DMA_HAL_ERROR
} BspUartDmaResult;

typedef struct
{
    UART_HandleTypeDef *uart;

    uint8_t rx_dma_buffer[BSP_UART_DMA_RX_DMA_SIZE];
    uint8_t rx_ring[BSP_UART_DMA_RX_RING_SIZE];
    uint8_t tx_ring[BSP_UART_DMA_TX_RING_SIZE];

    volatile uint16_t rx_dma_last_pos;
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    volatile uint16_t tx_head;
    volatile uint16_t tx_tail;
    volatile uint16_t tx_dma_length;

    volatile bool started;
    volatile bool tx_dma_active;
    volatile bool rx_restart_pending;

    volatile uint32_t rx_bytes;
    volatile uint32_t tx_bytes;
    volatile uint32_t rx_overflow_count;
    volatile uint32_t tx_full_count;
    volatile uint32_t rx_idle_event_count;
    volatile uint32_t rx_half_event_count;
    volatile uint32_t rx_full_event_count;
    volatile uint32_t rx_duplicate_event_count;
    volatile uint32_t uart_error_count;
    volatile uint32_t dma_start_error_count;
    volatile uint32_t last_hal_error;
} BspUartDmaPort;

BspUartDmaResult BspUartDma_Init(
    BspUartDmaPort *port,
    UART_HandleTypeDef *uart);

BspUartDmaResult BspUartDma_Start(BspUartDmaPort *port);
void BspUartDma_Service(BspUartDmaPort *port);

size_t BspUartDma_RxAvailable(const BspUartDmaPort *port);
size_t BspUartDma_TxFree(const BspUartDmaPort *port);

size_t BspUartDma_Read(
    BspUartDmaPort *port,
    uint8_t *data,
    size_t capacity);

BspUartDmaResult BspUartDma_Write(
    BspUartDmaPort *port,
    const uint8_t *data,
    size_t length);

#endif
