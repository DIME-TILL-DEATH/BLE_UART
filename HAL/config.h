#ifndef __CONFIG_H
#define __CONFIG_H

#define	ID_CH32V208							0x0208

#define CHIP_ID								ID_CH32V208

#ifdef WCHBLE_ROM
#include "WCHBLE_ROM.H"
#else
#include "wchble.H"
#endif

#include "ch32v20x.h"


#ifndef BLE_MAC
#define BLE_MAC                             FALSE
#endif
#ifndef HAL_SLEEP
#define HAL_SLEEP                           FALSE
#endif
#ifndef WAKE_UP_MAX_TIME_US
#define WAKE_UP_MAX_TIME_US                 2400
#endif
#ifndef HAL_KEY
#define HAL_KEY                             FALSE
#endif
#ifndef HAL_LED
#define HAL_LED                             FALSE
#endif
#ifndef TEM_SAMPLE
#define TEM_SAMPLE                          TRUE
#endif
#ifndef BLE_CALIBRATION_ENABLE
#define BLE_CALIBRATION_ENABLE              TRUE
#endif
#ifndef BLE_CALIBRATION_PERIOD
#define BLE_CALIBRATION_PERIOD              120000
#endif
#ifndef BLE_SNV
#define BLE_SNV                             TRUE
#endif
#ifndef BLE_SNV_ADDR
#define BLE_SNV_ADDR                        0x08077C00
#endif
#ifndef BLE_SNV_NUM
#define BLE_SNV_NUM                         3
#endif
#ifndef CLK_OSC32K
#define CLK_OSC32K                          1
#endif
#ifndef BLE_MEMHEAP_SIZE
#define BLE_MEMHEAP_SIZE                    (1024*7)
#endif
#ifndef BLE_BUFF_MAX_LEN
#define BLE_BUFF_MAX_LEN                    512 //27  // Max packet length 27-516
#endif
#ifndef BLE_BUFF_NUM
#define BLE_BUFF_NUM                        16 //5
#endif
#ifndef BLE_TX_NUM_EVENT
#define BLE_TX_NUM_EVENT                    1
#endif
#ifndef BLE_TX_POWER
#define BLE_TX_POWER                        LL_TX_POWEER_7_DBM //LL_TX_POWEER_0_DBM
#endif
#ifndef PERIPHERAL_MAX_CONNECTION
#define PERIPHERAL_MAX_CONNECTION           1
#endif
#ifndef CENTRAL_MAX_CONNECTION
#define CENTRAL_MAX_CONNECTION              3
#endif

extern uint32_t MEM_BUF[BLE_MEMHEAP_SIZE / 4];
extern const uint8_t MacAddr[6];

#endif

