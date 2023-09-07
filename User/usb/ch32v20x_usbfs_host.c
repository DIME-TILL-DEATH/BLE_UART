/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32f20x_usbfs_host.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2022/09/01
* Description        : USB full-speed port host operation functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/


/*******************************************************************************/
/* Header File */
//#include "usb_host_config.h"

#include "string.h"
#include "debug.h"

#include "ch32v20x_usbfs_host.h"

/*******************************************************************************/
/* Variable Definition */
__attribute__((aligned(4))) uint8_t  endpTXbuf[USBFS_MAX_PACKET_SIZE];
__attribute__((aligned(4))) uint8_t  endpRXbuf[USBFS_MAX_PACKET_SIZE];

/*********************************************************************
 * @fn      USB_RCC_Init
 *
 * @brief   Set USB port clock.
 *          Note: If the SystemCoreClock is selected as the USB clock source,
 *          only the frequency specified below can be used.
 *
 * @return  none
 */
void USB_RCCInit()
{
    if( SystemCoreClock == 144000000 )
    {
        RCC_USBCLKConfig( RCC_USBCLKSource_PLLCLK_Div3 );
    }
    else if( SystemCoreClock == 96000000 )
    {
        RCC_USBCLKConfig( RCC_USBCLKSource_PLLCLK_Div2 );
    }
    else if( SystemCoreClock == 48000000 )
    {
        RCC_USBCLKConfig( RCC_USBCLKSource_PLLCLK_Div1 );
    }
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_OTG_FS, ENABLE );
}
/*********************************************************************
 * @fn      USB_Host_Init
 *
 * @brief   Initialize USB port host configuration.
 *
 * @param   state - ENABLE or DISABLE
 *
 * @return  none
 */
void USB_HostInit(FunctionalState sta)
{
    if( sta == ENABLE )
    {
        /* Reset USB module */
        USBOTG_H_FS->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
        Delay_Us( 10 );
        USBOTG_H_FS->BASE_CTRL = 0;

        /* Initialize USB host configuration */
        USBOTG_H_FS->BASE_CTRL = USBFS_UC_HOST_MODE | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
        USBOTG_H_FS->HOST_EP_MOD = USBFS_UH_EP_TX_EN | USBFS_UH_EP_RX_EN;
        USBOTG_H_FS->HOST_RX_DMA = (uint32_t)endpRXbuf;
        USBOTG_H_FS->HOST_TX_DMA = (uint32_t)endpTXbuf;
    }
    else
    {
        USBOTG_H_FS->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
        Delay_Us( 10 );
        USBOTG_H_FS->BASE_CTRL = 0;
    }

    // trig pin
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*********************************************************************
 * @fn      SetBusReset
 *
 * @brief   Reset USB bus
 *
 * @return  none
 */
void USB_SetBusReset()
{
//    USBOTG_H_FS->HOST_CTRL |= USBFS_UH_BUS_RESET;                              //bus reset
//    Delay_Ms(15);
//    USBOTG_H_FS->HOST_CTRL &= ~USBFS_UH_BUS_RESET;
//
////    while((USBOTG_H_FS->MIS_ST & USBFS_UMS_BUS_RESET));                     //wait bus idle;
//
//    USBOTG_H_FS->HOST_SETUP |= USBFS_UH_SOF_EN;                                 //sof enable
    USBOTG_H_FS->HOST_CTRL = (USBOTG_H_FS->HOST_CTRL & ~USBFS_UH_LOW_SPEED) | USBFS_UH_BUS_RESET;
    USBOTG_H_FS->HOST_CTRL = USBOTG_H_FS->HOST_CTRL & ~USBFS_UH_BUS_RESET;
}

/*********************************************************************
 * @fn      USBFSH_SetSelfAddr
 *
 * @brief   Set the USB device address.
 *
 * @para    addr: USB device address.
 *
 * @return  none
 */
void USB_SetSelfAddr(uint8_t address)
{
    USBOTG_H_FS->DEV_ADDR = (USBOTG_H_FS->DEV_ADDR & USBFS_UDA_GP_BIT) | (address & USBFS_USB_ADDR_MASK);
}

/*********************************************************************
 * @fn      USBFSH_SetSelfSpeed
 *
 * @brief   Set USB speed.
 *
 * @para    speed: USB speed.
 *
 * @return  none
 */
void USBFSH_SetSelfSpeed( uint8_t speed )
{
    if( speed == USB_FULL_SPEED )
    {
        USBOTG_H_FS->BASE_CTRL &= ~USBFS_UC_LOW_SPEED;
        USBOTG_H_FS->HOST_CTRL &= ~USBFS_UH_LOW_SPEED;
        USBOTG_H_FS->HOST_SETUP &= ~USBFS_UH_PRE_PID_EN;
    }
    else
    {
        USBOTG_H_FS->BASE_CTRL |= USBFS_UC_LOW_SPEED;
        USBOTG_H_FS->HOST_CTRL |= USBFS_UH_LOW_SPEED;
        USBOTG_H_FS->HOST_SETUP |= USBFS_UH_PRE_PID_EN;
    }
}

/*********************************************************************
 * @fn      USBFSH_ResetRootHubPort
 *
 * @brief   Reset USB port.
 *
 * @para    mod: Reset host port operating mode.
 *               0 -> reset and wait end
 *               1 -> begin reset
 *               2 -> end reset
 *
 * @return  none
 */
void USB_ResetRootHubPort()
{
    USB_SetSelfAddr(0x00);
    USBFSH_SetSelfSpeed(USB_FULL_SPEED);

    USBOTG_H_FS->HOST_CTRL |= USBFS_UH_BUS_RESET; // Start reset
    Delay_Ms(DEF_BUS_RESET_TIME); // Reset time from 10mS to 20mS
    USBOTG_H_FS->HOST_CTRL &= ~USBFS_UH_BUS_RESET; // End reset
    Delay_Ms(2);

//    if( USBOTG_H_FS->INT_FG & USBFS_UIF_DETECT )
//    {
//        if( USBOTG_H_FS->MIS_ST & USBFS_UMS_DEV_ATTACH )
//        {
//            USBOTG_H_FS->INT_FG = USBFS_UIF_DETECT;
//        }
//    }
}

/*********************************************************************
 * @fn      USB_RawTransaction
 *
 * @brief   Perform USB transaction.
 *
 * @para    endpPid: Token PID.
 *          endpToggle: Toggle
 *          timeout: Timeout time.
 *
 * @return  USB transfer result.
 */
uint8_t USB_RawTransaction(uint8_t endpPid, uint8_t endpToggle, uint32_t timeout)
{
    uint8_t trans_retry = 0;

    USBOTG_H_FS->HOST_TX_CTRL = USBOTG_H_FS->HOST_RX_CTRL = endpToggle;

    do
    {
        USBOTG_H_FS->HOST_EP_PID = endpPid;       // Specify token PID and endpoint number
        USBOTG_H_FS->INT_FG |= USBFS_UIF_TRANSFER;  // Allow transfer
        GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_RESET);

        for(uint32_t i = DEF_WAIT_USB_TRANSFER_CNT; (i != 0) && ((USBOTG_H_FS->INT_FG & USBFS_UIF_TRANSFER) == 0); i--)
        {
            Delay_Us(1); // Delay for USB transfer
        }

        USBOTG_H_FS->HOST_EP_PID = 0x00;  // Stop USB transfer

        if((USBOTG_H_FS->INT_FG & USBFS_UIF_TRANSFER) == 0) return ERR_USB_UNKNOWN;

//        if(USBOTG_H_FS->INT_ST & USBFS_UIS_TOG_OK) return ERR_SUCCESS;

        if(USBOTG_H_FS->INT_FG & USBFS_UIF_DETECT)
        {
            USBOTG_H_FS->INT_FG = USBFS_UIF_DETECT;
            Delay_Us(200);

            if(USBOTG_H_FS->MIS_ST & USBFS_UMS_DEV_ATTACH)
            {
//                if(USBOTG_H_FS->HOST_CTRL & SEND_SOF_EN)
//                {
                    return ERR_USB_CONNECT;
//                }
            }
            else return ERR_USB_DISCON;
        }
        else if(USBOTG_H_FS->INT_FG & USBFS_UIF_TRANSFER)
        {
            uint8_t responsePID = USBOTG_H_FS->INT_ST & USBFS_UIS_H_RES_MASK; // Response status of current USB transaction

            if(responsePID == USB_PID_STALL) return responsePID|ERR_USB_TRANSFER;

            if((endpPid >> 4) == USB_PID_IN)
            {
                if (USBOTG_H_FS->INT_ST & USBFS_UIS_TOG_OK) return ERR_SUCCESS;
            }
            else
            {
                if ((responsePID == USB_PID_ACK) || (responsePID == USB_PID_NYET)) return ERR_SUCCESS;
            }

            if(responsePID == USB_PID_NAK)
            {
                if(timeout == 0) return responsePID|ERR_USB_TRANSFER;
                if(timeout < 0xFFFF) timeout--;
                --trans_retry;
            }
            else switch (endpPid >> 4)
            {
                case USB_PID_SETUP:
                    break; //????? no break in original!
                case USB_PID_OUT:
                    if(responsePID) return responsePID|ERR_USB_TRANSFER;
                    break;
                case USB_PID_IN:
                    if((responsePID == USB_PID_DATA0) || (responsePID == USB_PID_DATA1))
                    {
                    }
                    else if(responsePID)
                    {
                        return responsePID|ERR_USB_TRANSFER;
                    }
                    break;
                default:
                    return ERR_USB_UNKNOWN;
            }
        }
//        else
//        {
//            USBOTG_H_FS->INT_FG = 0x3F;
//        }

        Delay_Us(20);

    }while(++trans_retry < 5);

    return ERR_USB_TRANSFER; // Reply timeout
}

/*********************************************************************
 * @fn      USB_HostCtrlTransfer
 *
 * @brief   Host control transfer.
 *
 * @param   usbDevice_ptr - target USB device pointer
 *          request_ptr - USB setup request pointer
 *          replyBuf_ptr - pointer to answer
 *
 * @return  Error state
 */
uint8_t USB_HostCtrlTransfer(USBDEV_INFO* usbDevice_ptr, USB_SETUP_REQ* request_ptr, uint8_t** replyBuf_ptr)
{
    uint8_t   retVal;

    uint32_t len_ptr = 0;
    if(replyBuf_ptr) *replyBuf_ptr = 0;

    memcpy(endpTXbuf, request_ptr, sizeof(USB_SETUP_REQ));
    Delay_Us(100);

    USB_SetSelfAddr(0x00);

    USBOTG_H_FS->HOST_TX_LEN = sizeof(USB_SETUP_REQ);

    GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_SET);

    retVal = USB_RawTransaction((USB_PID_SETUP << 4) | DEF_ENDP_0, 0x00, DEF_CTRL_TRANS_TIMEOVER_CNT);
    if(retVal != ERR_SUCCESS) return retVal;

    // ***DATA stage**********************
    uint16_t requestLen = request_ptr->wLength;
    USBOTG_H_FS->HOST_TX_CTRL = USBOTG_H_FS->HOST_RX_CTRL = USBFS_UH_T_TOG | USBFS_UH_R_TOG; // Default DATA1
    if(requestLen && endpRXbuf)
    {
        if((request_ptr->bRequestType) & USB_REQ_TYP_IN)
        {
            /* Receive data */
            while(requestLen)
            {
//                Delay_Us(100);

                USBOTG_H_FS->HOST_RX_DMA = (uint32_t)endpRXbuf + len_ptr;
                retVal = USB_RawTransaction((USB_PID_IN << 4)|DEF_ENDP_0, USBOTG_H_FS->HOST_RX_CTRL, DEF_CTRL_TRANS_TIMEOVER_CNT);  // IN
                if(retVal != ERR_SUCCESS) return retVal;

                USBOTG_H_FS->HOST_RX_CTRL ^= USBFS_UH_R_TOG;

                uint32_t rxLen = (USBOTG_H_FS->RX_LEN < requestLen) ? USBOTG_H_FS->RX_LEN : requestLen;
                requestLen -= rxLen;
                len_ptr += rxLen;

                if((USBOTG_H_FS->RX_LEN == 0) || (USBOTG_H_FS->RX_LEN & (usbDevice_ptr->endp0Size - 1)))  break; // Short package
            }
            USBOTG_H_FS->HOST_TX_LEN = 0; // Status stage is OUT
        }
        else
        {
            /* Send data */
            while(requestLen)
            {
//                Delay_Us(100);

                USBOTG_H_FS->HOST_TX_DMA = (uint32_t)endpTXbuf + len_ptr;
                USBOTG_H_FS->HOST_TX_LEN = (requestLen >= usbDevice_ptr->endp0Size) ? usbDevice_ptr->endp0Size : requestLen;

                retVal = USB_RawTransaction(USB_PID_OUT << 4|DEF_ENDP_0, USBOTG_H_FS->HOST_TX_CTRL, DEF_CTRL_TRANS_TIMEOVER_CNT); // OUT
                if(retVal != ERR_SUCCESS) return retVal;

                USBOTG_H_FS->HOST_TX_CTRL ^= USBFS_UH_T_TOG;

                requestLen -= USBOTG_H_FS->HOST_TX_LEN;
                len_ptr += USBOTG_H_FS->HOST_TX_LEN;
            }
        }
    }

//    Delay_Us(100);

    // ***STATUS stage**********************
    retVal = USB_RawTransaction(USBOTG_H_FS->HOST_TX_LEN ? (USB_PID_IN << 4|DEF_ENDP_0) : (USB_PID_OUT << 4|DEF_ENDP_0),
            USBFS_UH_R_TOG|USBFS_UH_T_TOG, DEF_CTRL_TRANS_TIMEOVER_CNT );

    if(retVal != ERR_SUCCESS) return retVal;

    if(replyBuf_ptr) *replyBuf_ptr = endpRXbuf;

    if(USBOTG_H_FS->HOST_TX_LEN == 0) return ERR_SUCCESS;
    if(USBOTG_H_FS->RX_LEN == 0) return ERR_SUCCESS;

    return ERR_USB_BUF_OVER;
}


/*********************************************************************
 * @fn      USB_GetEndpData
 *
 * @brief   Get data from USB device input endpoint.
 *
 * @param    endpInfo_ptr: Endpoint pointer
 *          buf_ptr: Data Buffer
 *          len_ptr: Data length
 *
 * @return  The result of getting data.
 */
uint8_t USB_GetEndpData(USBENDP_INFO* endpInfo_ptr, uint8_t *buf_ptr, uint16_t *len_ptr)
{
    if(!endpInfo_ptr || !buf_ptr) return 0xFF;

    if(len_ptr) *len_ptr = 0;

    memset(endpRXbuf, '\0', USBFS_MAX_PACKET_SIZE);

    USBOTG_H_FS->HOST_RX_DMA = (uint32_t)endpRXbuf;
    uint8_t retVal = USB_RawTransaction((USB_PID_IN << 4) | endpInfo_ptr->endpAddress, endpInfo_ptr->toggle,  2000);
    if(retVal == ERR_SUCCESS)
    {
        endpInfo_ptr->toggle ^= USBFS_UH_R_TOG;
        *len_ptr = USBOTG_H_FS->RX_LEN;
        memcpy(buf_ptr, endpRXbuf, *len_ptr);
    }
    
    return retVal;
}
/*********************************************************************
 * @fn      USBHSH_SendEndpData
 *
 * @brief   Send data to the USB device output endpoint.
 *
 * @para    endpInfo_ptr: Endpoint pointer
 *          pbuf: Data Buffer
 *          plen: Data length
 *
 * @return  The result of sending data.
 */
uint8_t USB_SendEndpData(USBENDP_INFO* endpInfo_ptr, uint8_t *buf_ptr, int16_t len)
{
    if(!endpInfo_ptr || !buf_ptr) return 0xFF;

    uint16_t packetSize = (endpInfo_ptr->endpMaxSize <= USBFS_MAX_PACKET_SIZE) ? endpInfo_ptr->endpMaxSize : USBFS_MAX_PACKET_SIZE;
    uint32_t len_ptr = 0;
    uint8_t retVal = 0xFF;

    memcpy(endpTXbuf, buf_ptr, len); // no more than MAX_PACKET_SIZE!!!!!!!

    while(len>0)
    {
         USBOTG_H_FS->HOST_TX_DMA = (uint32_t)endpTXbuf + len_ptr;
         USBOTG_H_FS->HOST_TX_LEN = (len >= endpInfo_ptr->endpMaxSize) ? packetSize : len;

         retVal = USB_RawTransaction(( USB_PID_OUT << 4 ) | endpInfo_ptr->endpAddress, endpInfo_ptr->toggle,  2000);
         if(retVal != ERR_SUCCESS)
         {
             return retVal;
         }

         endpInfo_ptr->toggle ^= USBFS_UH_T_TOG;
         len -= USBOTG_H_FS->HOST_TX_LEN;
         len_ptr += USBOTG_H_FS->HOST_TX_LEN;
    }

    return retVal;
}
