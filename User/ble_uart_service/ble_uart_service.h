#ifndef _BLE_UART_SERVICE_H
#define _BLE_UART_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define UART_SERVICE_UUID 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
#define UART_RXCHAR_UUID 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E
#define UART_TXCHAR_UUID 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E

#define BLE_UART_RX_BUFF_SIZE    1

typedef enum
{
    BLE_UART_EVT_TX_NOTI_DISABLED = 1,
    BLE_UART_EVT_TX_NOTI_ENABLED,
    BLE_UART_EVT_BLE_DATA_RECIEVED,
} ble_uart_evt_type_t;

typedef struct
{
    uint8_t const *p_data; /**< A pointer to the buffer with received data. */
    uint16_t       length; /**< Length of received data. */
} ble_uart_evt_rx_data_t;

typedef struct
{
    ble_uart_evt_type_t    type;
    ble_uart_evt_rx_data_t data;
} ble_uart_evt_t;

typedef void (*ble_uart_ProfileChangeCB_t)(uint16_t connection_handle, ble_uart_evt_t *p_evt);

bStatus_t ble_uart_add_service(ble_uart_ProfileChangeCB_t cb);
bStatus_t ble_uart_notify(uint16_t connHandle, attHandleValueNoti_t *pNoti, uint8_t taskId);
uint8_t ble_uart_notify_is_ready(uint16_t connHandle);


#ifdef __cplusplus
}
#endif

#endif /* _BLE_UART_SERVICE_H */
