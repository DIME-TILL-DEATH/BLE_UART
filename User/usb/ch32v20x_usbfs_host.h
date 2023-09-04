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


#ifndef __CH32V20x_USBFS_HOST_H__
#define __CH32V20x_USBFS_HOST_H__

#ifdef __cplusplus
 extern "C" {
#endif

/*******************************************************************************/
/* Header File */
#include "ch32v20x.h"
#include "ch32v20x_usb.h"

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

/*******************************************************************************/
/* Macro Definition */

/* USB Setup Request */
#define pUSBFS_SetupRequest        ((PUSB_SETUP_REQ)endpTXbuf)

/* USB Buffer Size */
#ifndef USBFS_MAX_PACKET_SIZE
#define USBFS_MAX_PACKET_SIZE      64
#endif

/*******************************************************************************/
/* Constant Definition */
#ifndef DEF_USB_GEN_ENUM_CMD
#define DEF_USB_GEN_ENUM_CMD
/* Get Device Descriptor Command Packet */
__attribute__((aligned(4))) static const uint8_t  SetupGetDevDesc[ ] =
{
    USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_DEVICE, 0x00, 0x00, sizeof( USB_DEV_DESCR ), 0x00
};

/* Get Configuration Descriptor Command Packet */
__attribute__((aligned(4))) static const uint8_t SetupGetCfgDesc[ ] =
{
    USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_CONFIG, 0x00, 0x00, 0x04, 0x00
};

/* Get String Descriptor Command Packet */
__attribute__((aligned(4))) static const uint8_t SetupGetStrDesc[ ] =
{
    USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_STRING, 0x09, 0x04, 0x04, 0x00
};

/* Set USB Address Command Packet */
__attribute__((aligned(4))) static const uint8_t SetupSetAddr[ ] =
{
    USB_REQ_TYP_OUT, USB_SET_ADDRESS, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* Set USB Configuration Command Packet */
__attribute__((aligned(4))) static const uint8_t SetupSetConfig[ ] =
{
    USB_REQ_TYP_OUT, USB_SET_CONFIGURATION, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* Clear Endpoint STALL Command Packet */
__attribute__((aligned(4))) static const uint8_t SetupClearEndpStall[ ] =
{
    USB_REQ_TYP_OUT | USB_REQ_RECIP_ENDP, USB_CLEAR_FEATURE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* Set Device Interface Command Packet */
__attribute__((aligned(4))) static const uint8_t SetupSetInterface[ ] =
{
    USB_REQ_RECIP_INTERF, USB_SET_INTERFACE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
#endif

/*******************************************************************************/
/* Variable Declaration */
extern __attribute__((aligned(4))) uint8_t  endpTXbuf[ ];
extern __attribute__((aligned(4))) uint8_t  endpRXbuf[ ];

/*******************************************************************************/

void USBFS_RCC_Init();
extern void USBFS_Host_Init( FunctionalState sta );
extern uint8_t USBFSH_CheckRootHubPortStatus( uint8_t dev_sta );
extern uint8_t USBFSH_CheckRootHubPortEnable( void );
extern uint8_t USBFSH_CheckRootHubPortSpeed( void );
extern void USBFSH_SetSelfAddr( uint8_t addr );
extern void USBFSH_SetSelfSpeed( uint8_t speed );
extern void USBFSH_ResetRootHubPort( uint8_t mode );
extern uint8_t USBFSH_EnableRootHubPort( uint8_t *pspeed );
extern uint8_t USBFSH_Transact( uint8_t endp_pid, uint8_t endp_tog, uint16_t timeout );
extern uint8_t USBFSH_CtrlTransfer( uint8_t ep0_size, uint8_t *pbuf, uint16_t *plen );
extern uint8_t USBFSH_GetDeviceDescr( uint8_t *pep0_size, uint8_t *pbuf );
extern uint8_t USBFSH_GetConfigDescr( uint8_t ep0_size, uint8_t *pbuf, uint16_t buf_len, uint16_t *pcfg_len );
extern uint8_t USBFSH_GetStrDescr( uint8_t ep0_size, uint8_t str_num, uint8_t *pbuf );
extern uint8_t USBFSH_SetUsbAddress( uint8_t ep0_size, uint8_t addr );
extern uint8_t USBFSH_SetUsbConfig( uint8_t ep0_size, uint8_t cfg_val );
extern uint8_t USBFSH_ClearEndpStall( uint8_t ep0_size, uint8_t endp_num );
extern uint8_t USBFSH_GetEndpData( uint8_t endp_num, uint8_t *pendp_tog, uint8_t *pbuf, uint16_t *plen );
extern uint8_t USBFSH_SendEndpData( uint8_t endp_num, uint8_t *pendp_tog, uint8_t *pbuf, uint16_t len );


#ifdef __cplusplus
}
#endif

#endif
