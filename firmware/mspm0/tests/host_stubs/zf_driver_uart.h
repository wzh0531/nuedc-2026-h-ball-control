#ifndef _HOST_STUB_ZF_DRIVER_UART_H_
#define _HOST_STUB_ZF_DRIVER_UART_H_

#include "zf_common_typedef.h"

typedef enum { UART_1 = 1, UART_2 = 2 }uart_index_enum;
typedef enum { UART1_TX_A8 = 0, UART2_TX_B15 = 1 }uart_tx_pin_enum;
typedef enum { UART1_RX_A9 = 0, UART2_RX_B16 = 1 }uart_rx_pin_enum;
typedef enum
{
    UART_INTERRUPT_CONFIG_RX_ENABLE = 1,
}uart_interrupt_config_enum;
typedef enum
{
    UART_INTERRUPT_STATE_NONE = 0,
    UART_INTERRUPT_STATE_RX = 1,
}uart_interrupt_state_enum;
typedef void (*host_uart_callback)(uint32 event, void *ptr);

void uart_write_buffer(uart_index_enum uart_index, const uint8 *buff,
    uint32 len);
uint8 uart_query_byte(uart_index_enum uart_index, uint8 *data);
void uart_set_callback(uart_index_enum uart_index,
    host_uart_callback callback, void *ptr);
void uart_set_interrupt_config(uart_index_enum uart_index,
    uart_interrupt_config_enum config);
void uart_init(uart_index_enum uart_index, uint32 baud,
    uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin);

#endif
