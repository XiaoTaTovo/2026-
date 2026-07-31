#include "bsp_uart_dma.h"

#include <string.h>

/*
 * Sources:
 * - STM32CubeF4 V1.28.3 HAL UART enhanced DMA reception API:
 *   Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
 * - Vendor STM32F407 X42S example DMA ownership:
 *   USART1_RX = DMA2 Stream2 Channel4, circular
 *   USART1_TX = DMA2 Stream7 Channel4, normal
 *
 * The HAL RxEvent Size argument is treated as the DMA write position, not as
 * an event byte count. This makes HT/TC/IDLE callbacks safe even when two
 * events report the same position.
 */

#if (BSP_UART_DMA_RX_RING_SIZE == 0U) || \
    ((BSP_UART_DMA_RX_RING_SIZE & (BSP_UART_DMA_RX_RING_SIZE - 1U)) != 0U)
#error BSP_UART_DMA_RX_RING_SIZE_must_be_a_power_of_two
#endif

#if (BSP_UART_DMA_TX_RING_SIZE == 0U) || \
    ((BSP_UART_DMA_TX_RING_SIZE & (BSP_UART_DMA_TX_RING_SIZE - 1U)) != 0U)
#error BSP_UART_DMA_TX_RING_SIZE_must_be_a_power_of_two
#endif

static BspUartDmaPort *registered_ports[BSP_UART_DMA_MAX_PORTS];

static uint16_t ring_next(uint16_t index, uint16_t size)
{
    return (uint16_t)((index + 1U) & (uint16_t)(size - 1U));
}

static size_t ring_used(uint16_t head, uint16_t tail, uint16_t size)
{
    return (size_t)((head - tail) & (uint16_t)(size - 1U));
}

static size_t ring_free(uint16_t head, uint16_t tail, uint16_t size)
{
    return (size_t)(size - 1U) - ring_used(head, tail, size);
}

static uint32_t critical_enter(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return primask;
}

static void critical_exit(uint32_t primask)
{
    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static BspUartDmaPort *find_port(UART_HandleTypeDef *uart)
{
    size_t index;

    for (index = 0U; index < BSP_UART_DMA_MAX_PORTS; ++index)
    {
        if ((registered_ports[index] != NULL) &&
            (registered_ports[index]->uart == uart))
        {
            return registered_ports[index];
        }
    }

    return NULL;
}

static BspUartDmaResult register_port(BspUartDmaPort *port)
{
    size_t index;

    for (index = 0U; index < BSP_UART_DMA_MAX_PORTS; ++index)
    {
        if (registered_ports[index] == port)
        {
            return BSP_UART_DMA_OK;
        }
        if (registered_ports[index] == NULL)
        {
            registered_ports[index] = port;
            return BSP_UART_DMA_OK;
        }
    }

    return BSP_UART_DMA_FULL;
}

static void push_rx_byte(BspUartDmaPort *port, uint8_t byte)
{
    uint16_t next = ring_next(port->rx_head, BSP_UART_DMA_RX_RING_SIZE);

    if (next == port->rx_tail)
    {
        port->rx_overflow_count++;
        return;
    }

    port->rx_ring[port->rx_head] = byte;
    __DMB();
    port->rx_head = next;
    port->rx_bytes++;
}

static void copy_rx_dma_delta(BspUartDmaPort *port, uint16_t new_pos)
{
    uint16_t last;

    if (new_pos > BSP_UART_DMA_RX_DMA_SIZE)
    {
        port->uart_error_count++;
        port->last_hal_error = 0xFFFFFFFFU;
        return;
    }

    last = port->rx_dma_last_pos;

    if (new_pos == last)
    {
        port->rx_duplicate_event_count++;
        return;
    }

    if (new_pos > last)
    {
        uint16_t index;
        for (index = last; index < new_pos; ++index)
        {
            push_rx_byte(port, port->rx_dma_buffer[index]);
        }
    }
    else
    {
        uint16_t index;
        for (index = last; index < BSP_UART_DMA_RX_DMA_SIZE; ++index)
        {
            push_rx_byte(port, port->rx_dma_buffer[index]);
        }
        for (index = 0U; index < new_pos; ++index)
        {
            push_rx_byte(port, port->rx_dma_buffer[index]);
        }
    }

    /* Keep BSP_UART_DMA_RX_DMA_SIZE as a valid position. If TC and IDLE
     * report the same full-buffer position, the second event is a duplicate.
     * The next wrapped position (for example 10) then copies bytes [0, 10). */
    port->rx_dma_last_pos = new_pos;
}

static void start_tx_locked(BspUartDmaPort *port)
{
    uint16_t length;
    HAL_StatusTypeDef status;

    if (port->tx_dma_active || (port->tx_head == port->tx_tail))
    {
        return;
    }

    if (port->tx_head > port->tx_tail)
    {
        length = (uint16_t)(port->tx_head - port->tx_tail);
    }
    else
    {
        length = (uint16_t)(BSP_UART_DMA_TX_RING_SIZE - port->tx_tail);
    }

    status = HAL_UART_Transmit_DMA(
        port->uart,
        &port->tx_ring[port->tx_tail],
        length);

    if (status == HAL_OK)
    {
        port->tx_dma_length = length;
        port->tx_dma_active = true;
    }
    else
    {
        port->dma_start_error_count++;
        port->last_hal_error = port->uart->ErrorCode;
    }
}

static BspUartDmaResult start_rx(BspUartDmaPort *port)
{
    HAL_StatusTypeDef status;

    port->rx_dma_last_pos = 0U;
    status = HAL_UARTEx_ReceiveToIdle_DMA(
        port->uart,
        port->rx_dma_buffer,
        BSP_UART_DMA_RX_DMA_SIZE);

    if (status != HAL_OK)
    {
        port->dma_start_error_count++;
        port->last_hal_error = port->uart->ErrorCode;
        return BSP_UART_DMA_HAL_ERROR;
    }

    port->rx_restart_pending = false;
    return BSP_UART_DMA_OK;
}

BspUartDmaResult BspUartDma_Init(
    BspUartDmaPort *port,
    UART_HandleTypeDef *uart)
{
    BspUartDmaResult result;

    if ((port == NULL) || (uart == NULL))
    {
        return BSP_UART_DMA_INVALID_ARGUMENT;
    }

    memset(port, 0, sizeof(*port));
    port->uart = uart;

    result = register_port(port);
    if (result != BSP_UART_DMA_OK)
    {
        port->uart = NULL;
    }

    return result;
}

BspUartDmaResult BspUartDma_Start(BspUartDmaPort *port)
{
    BspUartDmaResult result;

    if ((port == NULL) || (port->uart == NULL))
    {
        return BSP_UART_DMA_INVALID_ARGUMENT;
    }

    if ((port->uart->hdmarx == NULL) || (port->uart->hdmatx == NULL) ||
        (port->uart->hdmarx->Init.Mode != DMA_CIRCULAR) ||
        (port->uart->hdmatx->Init.Mode != DMA_NORMAL))
    {
        return BSP_UART_DMA_INVALID_CONFIG;
    }

    result = start_rx(port);
    if (result == BSP_UART_DMA_OK)
    {
        port->started = true;
    }

    return result;
}

void BspUartDma_Service(BspUartDmaPort *port)
{
    uint32_t primask;

    if ((port == NULL) || !port->started)
    {
        return;
    }

    if (port->rx_restart_pending &&
        (port->uart->RxState == HAL_UART_STATE_READY))
    {
        (void)start_rx(port);
    }

    primask = critical_enter();
    start_tx_locked(port);
    critical_exit(primask);
}

size_t BspUartDma_RxAvailable(const BspUartDmaPort *port)
{
    if (port == NULL)
    {
        return 0U;
    }
    return ring_used(port->rx_head, port->rx_tail, BSP_UART_DMA_RX_RING_SIZE);
}

size_t BspUartDma_TxFree(const BspUartDmaPort *port)
{
    if (port == NULL)
    {
        return 0U;
    }
    return ring_free(port->tx_head, port->tx_tail, BSP_UART_DMA_TX_RING_SIZE);
}

size_t BspUartDma_Read(
    BspUartDmaPort *port,
    uint8_t *data,
    size_t capacity)
{
    size_t count = 0U;

    if ((port == NULL) || ((data == NULL) && (capacity != 0U)))
    {
        return 0U;
    }

    while ((count < capacity) && (port->rx_tail != port->rx_head))
    {
        data[count++] = port->rx_ring[port->rx_tail];
        __DMB();
        port->rx_tail = ring_next(port->rx_tail, BSP_UART_DMA_RX_RING_SIZE);
    }

    return count;
}

BspUartDmaResult BspUartDma_Write(
    BspUartDmaPort *port,
    const uint8_t *data,
    size_t length)
{
    size_t index;
    uint32_t primask;

    if ((port == NULL) || (port->uart == NULL) ||
        ((data == NULL) && (length != 0U)))
    {
        return BSP_UART_DMA_INVALID_ARGUMENT;
    }

    if (length == 0U)
    {
        return BSP_UART_DMA_OK;
    }

    primask = critical_enter();

    if (length > ring_free(port->tx_head, port->tx_tail, BSP_UART_DMA_TX_RING_SIZE))
    {
        port->tx_full_count++;
        critical_exit(primask);
        return BSP_UART_DMA_FULL;
    }

    for (index = 0U; index < length; ++index)
    {
        port->tx_ring[port->tx_head] = data[index];
        port->tx_head = ring_next(port->tx_head, BSP_UART_DMA_TX_RING_SIZE);
    }

    __DMB();
    start_tx_locked(port);
    critical_exit(primask);
    return BSP_UART_DMA_OK;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    BspUartDmaPort *port = find_port(uart);
    HAL_UART_RxEventTypeTypeDef event_type;

    if (port == NULL)
    {
        return;
    }

    event_type = HAL_UARTEx_GetRxEventType(uart);
    if (event_type == HAL_UART_RXEVENT_IDLE)
    {
        port->rx_idle_event_count++;
    }
    else if (event_type == HAL_UART_RXEVENT_HT)
    {
        port->rx_half_event_count++;
    }
    else if (event_type == HAL_UART_RXEVENT_TC)
    {
        port->rx_full_event_count++;
    }

    copy_rx_dma_delta(port, size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    BspUartDmaPort *port = find_port(uart);
    uint32_t primask;

    if ((port == NULL) || !port->tx_dma_active)
    {
        return;
    }

    primask = critical_enter();
    port->tx_tail = (uint16_t)((port->tx_tail + port->tx_dma_length) &
                               (BSP_UART_DMA_TX_RING_SIZE - 1U));
    port->tx_bytes += port->tx_dma_length;
    port->tx_dma_length = 0U;
    port->tx_dma_active = false;
    start_tx_locked(port);
    critical_exit(primask);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    BspUartDmaPort *port = find_port(uart);

    if (port == NULL)
    {
        return;
    }

    port->uart_error_count++;
    port->last_hal_error = uart->ErrorCode;
    port->rx_restart_pending = true;
}
