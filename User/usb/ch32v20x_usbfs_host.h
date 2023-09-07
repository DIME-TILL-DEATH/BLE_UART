/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v20x_usbfs_host.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : USBOTG full speed host header file
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/


#ifndef __MY_CH32V20x_USBFS_HOST_H__
#define __MY_CH32V20x_USBFS_HOST_H__

#ifdef __cplusplus
 extern "C" {
#endif

/*******************************************************************************/
/* Header File */
#include "ch32v20x.h"
#include "ch32v20x_usb.h"

#include "usb_defines.h"

 //endpoint
#define DEF_ENDP_0     0
#define DEF_ENDP_1     1
#define DEF_ENDP_2     2
#define DEF_ENDP_3     3
#define DEF_ENDP_4     4
#define DEF_ENDP_5     5
#define DEF_ENDP_6     6
#define DEF_ENDP_7     7
#define DEF_ENDP_8     8
#define DEF_ENDP_9     9
#define DEF_ENDP_10    10
#define DEF_ENDP_11    11
#define DEF_ENDP_12    12
#define DEF_ENDP_13    13
#define DEF_ENDP_14    14
#define DEF_ENDP_15    15

/******************************************************************************/
/* USB Host Defines */

#define DEF_TOTAL_ROOT_HUB          1
#define DEF_USBFS_PORT_EN           1
#define DEF_USBFS_PORT_INDEX        0x00
#define DEF_ONE_USB_SUP_DEV_TOTAL   5
#define DEF_NEXT_HUB_PORT_NUM_MAX   4
#define DEF_INTERFACE_NUM_MAX       4

/* USB Root Device Status */
#define ROOT_DEV_DISCONNECT         0
#define ROOT_DEV_CONNECTED          1
#define ROOT_DEV_FAILED             2
#define ROOT_DEV_SUCCESS            3

/* USB Device Address */
#define USB_DEVICE_ADDR             0x02

/* USB Speed */
#define USB_LOW_SPEED               0x00
#define USB_FULL_SPEED              0x01
#define USB_HIGH_SPEED              0x02
#define USB_SPEED_CHECK_ERR         0xFF

/* Configuration Descriptor Type */
#define DEF_DECR_CONFIG             0x02
#define DEF_DECR_INTERFACE          0x04
#define DEF_DECR_ENDPOINT           0x05
#define DEF_DECR_HID                0x21

/* USB Communication Status Code */
#define ERR_SUCCESS                 0x00
#define ERR_USB_CONNECT             0x15
#define ERR_USB_DISCON              0x16
#define ERR_USB_BUF_OVER            0x17
#define ERR_USB_DISK_ERR            0x1F
#define ERR_USB_TRANSFER            0x20
#define ERR_USB_UNSUPPORT           0xFB
#define ERR_USB_UNAVAILABLE         0xFC
#define ERR_USB_UNKNOWN             0xFE

/* USB Device Enumeration Status Code */
#define DEF_DEV_DESCR_GETFAIL       0x45
#define DEF_DEV_ADDR_SETFAIL        0x46
#define DEF_CFG_DESCR_GETFAIL       0x47
#define DEF_REP_DESCR_GETFAIL       0x48
#define DEF_CFG_VALUE_SETFAIL       0x49
#define DEF_DEV_TYPE_UNKNOWN        0xFF

/* USB Communication Time */
#define DEF_BUS_RESET_TIME          11          // USB bus reset time
#define DEF_RE_ATTACH_TIMEOUT       100         // Wait for the USB device to reconnect after reset, 100mS timeout
#define DEF_WAIT_USB_TRANSFER_CNT   1000        // Wait for the USB transfer to complete
#define DEF_CTRL_TRANS_TIMEOVER_CNT 200000/20   // Control transmission delay timing

/* USB Buffer Size */
#ifndef USBFS_MAX_PACKET_SIZE
#define USBFS_MAX_PACKET_SIZE 64
#endif

 typedef struct __attribute__((packed)) _ENDP_INFO
 {
     uint16_t  endpMaxSize;
     uint8_t   endpAddress;
     uint8_t   direction;
     uint8_t   type;
     uint8_t   toggle;
 }USBENDP_INFO;

 typedef struct __attribute__((packed)) _ITF_INFO
 {
     USBENDP_INFO*   endpInfo;
     uint8_t         endpCount;

     uint8_t     itfNumber;
     uint8_t     itfClass;
 }USBITF_INFO;

 typedef struct  __attribute__((packed)) _DEV_INFO
 {
     uint8_t   devClass;
     uint8_t   devSubClass;
     uint16_t  VID;
     uint16_t  PID;

     char* manufacturerString;
     uint8_t manufacturerStringLen;
     char* productString;
     uint8_t productStringLen;

     USBITF_INFO* itfInfo;
     uint8_t     itfNum;

     uint8_t   endp0Size;
     uint8_t   devCfgValue;

     uint8_t   devStatus;
     uint8_t   devAddress;
 }USBDEV_INFO;

 /*******************************************************************************/
 /* Variable Declaration */
 extern __attribute__((aligned(4))) uint8_t  endpRXbuf[ ];
 extern __attribute__((aligned(4))) uint8_t  endpTXbuf[ ];


void USB_RCCInit();
void USB_HostInit(FunctionalState state);
void USB_SetBusReset();
void USB_ResetRootHubPort();
void USB_SetSelfAddr(uint8_t address);
void USB_SetSelfSpeed(uint8_t speed);

uint8_t USB_RawTransaction(uint8_t endpPid, uint8_t endpToggle, uint32_t timeout);
uint8_t USB_HostCtrlTransfer(USBDEV_INFO* usbDevice_ptr, USB_SETUP_REQ* request_ptr, uint8_t** replyBuf_ptr);

uint8_t USB_GetEndpData(USBENDP_INFO* endpInfo_ptr, uint8_t *buf_ptr, uint16_t *len_ptr);
uint8_t USB_SendEndpData(USBENDP_INFO* endpInfo_ptr, uint8_t *buf_ptr, int16_t len);

#ifdef __cplusplus
}
#endif

#endif
