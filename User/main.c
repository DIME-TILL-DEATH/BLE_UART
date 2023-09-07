#include <malloc.h>

#include "config.h"
#include "HAL.h"

#include "ble.h"

//#include "exmpl_ch32v20x_usbfs_host.h"
//#include "app_km.h"

#include "ch32v20x_usbfs_host.h"

#include "usb_driver.h"
#include "usb_device_classes.h"
//
////void USBHD_IRQHandler()  __attribute__((interrupt("WCH-Interrupt-fast")));
//
////*********************************************************************************************

#define PANGAEA_CDC_INTERFACE_NUM 1

typedef enum
{
    DISCONNECTED,
    VCOM_PORT,
    PACNGAEA_CP16,
    FLASH_DRIVE,
    OTHER
}CONNECTED_TYPE;
CONNECTED_TYPE connectedType = DISCONNECTED;

USBDEV_INFO* lastConnectedDevice_ptr;

void checkConnectedDevice()
{
    if(lastConnectedDevice_ptr->devClass == USB_CLASS_MSD) connectedType = FLASH_DRIVE;

    if(lastConnectedDevice_ptr->devClass == USB_CLASS_CDC)
    {
        connectedType = VCOM_PORT;
        if(lastConnectedDevice_ptr->VID == 0x483 && lastConnectedDevice_ptr->PID == 0x5740)
        {
            connectedType = PACNGAEA_CP16;
            printf("Pangaea device found!\r\n");
        }
    }
}
//*******************************************************************************************

__attribute__((aligned(4))) u32 MEM_BUF[BLE_MEMHEAP_SIZE / 4];

__attribute__((section(".highcode")))
__attribute__((noinline))
void Main_Circulation(void)
{
    while(1)
    {
//        TMOS_SystemProcess();
//        app_uart_process();
    }
}

//=============================================
int main(void)
{
    Delay_Init();
#ifdef DEBUG
    USART_Printf_Init(115200);
#endif
    printf("BLE-USB Host converter\r\n");
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf("%s\r\n", VER_LIB);

//    WCHBLE_Init();
//    HAL_Init();
//    GAPRole_PeripheralInit();
//    Ble_Init();

//    app_uart_init();
    USB_RCCInit();
    USB_HostInit(ENABLE);

//    USBFS_RCC_Init( );
//    USBFS_Host_Init( ENABLE );

//    USBOTG_H_FS->INT_EN |= USBFS_UIE_DETECT;
//    NVIC_EnableIRQ(USBHD_IRQn);

//    Main_Circulation();

    while(1)
    {

        if(USBOTG_H_FS->INT_FG & USBFS_UIF_DETECT)// Check that there is a device connection or disconnection event on the port
        {
            USBOTG_H_FS->INT_FG = USBFS_UIF_DETECT;
            if(USBOTG_H_FS->MIS_ST & USBFS_UMS_DEV_ATTACH)
            {
                printf("\r\nNew device connected.\n");

                USB_FreeDevStruct(lastConnectedDevice_ptr); // clear old ptr
                lastConnectedDevice_ptr = (USBDEV_INFO*)malloc(sizeof(USBDEV_INFO));

                uint8_t retVal = USB_HostEnum(lastConnectedDevice_ptr);

                if(retVal == ERR_SUCCESS)
                {
                    printf("Enum success\n");

                    USB_PrintDevInfo(lastConnectedDevice_ptr);
                    checkConnectedDevice();
                }
                else
                {
                    printf("Enum error\n");
                    connectedType = DISCONNECTED;
                    USB_FreeDevStruct(lastConnectedDevice_ptr);
                }
            }
            else
            {
                USB_HostInit(DISABLE);
                USB_HostInit(ENABLE);

                connectedType = DISCONNECTED;
                USB_FreeDevStruct(lastConnectedDevice_ptr);

                printf("Disconnect\n");
            }
        }
    }
}
////===============================================
//
//void USBHD_IRQHandler()
//{
//    USBOTG_H_FS->INT_FG = USBFS_UIF_DETECT;
//    if(USBOTG_H_FS->MIS_ST & USBFS_UMS_DEV_ATTACH)
//    {
//        printf("\r\nNew device connected.\n");
//
//        USB_FreeDevStruct(lastConnectedDevice_ptr); // clear old ptr
//        lastConnectedDevice_ptr = (USBDEV_INFO*)malloc(sizeof(USBDEV_INFO));
//        uint8_t retVal = USB_HostEnum(lastConnectedDevice_ptr);
//
//        if(retVal == ERR_SUCCESS)
//        {
//            printf("Enum success\n");
//
//            USB_PrintDevInfo(lastConnectedDevice_ptr);
//            checkConnectedDevice();
//        }
//        else
//        {
//            printf("Enum error\n");
//            connectedType = DISCONNECTED;
//            USB_FreeDevStruct(lastConnectedDevice_ptr);
//        }
//    }
//    else
//    {
//        USB_HostInit(DISABLE);
//        USB_HostInit(ENABLE);
//
//        connectedType = DISCONNECTED;
//        USB_FreeDevStruct(lastConnectedDevice_ptr);
//
//        printf("Disconnect\n");
//    }
//}
