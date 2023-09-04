#include "config.h"
#include "HAL.h"

#include "ble.h"

__attribute__((aligned(4))) u32 MEM_BUF[BLE_MEMHEAP_SIZE / 4];

__attribute__((section(".highcode")))
__attribute__((noinline))
void Main_Circulation(void)
{
    while(1)
    {
        TMOS_SystemProcess();
        app_uart_process();
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

    WCHBLE_Init();
    HAL_Init();
    GAPRole_PeripheralInit();
    Ble_Init();

    app_uart_init();
    Main_Circulation();
}
