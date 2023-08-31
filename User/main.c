#include "config.h"
#include "HAL.h"

#include "peripheral.h"


__attribute__((aligned(4))) u32 MEM_BUF[BLE_MEMHEAP_SIZE / 4];

#if(defined(BLE_MAC)) && (BLE_MAC == TRUE)
uint8_t const MacAddr[6] = {0x84, 0xC2, 0xE4, 0x03, 0x02, 0x02};
#endif

extern void app_uart_process(void);
extern void app_uart_init(void);

/*********************************************************************
 * @fn      Main_Circulation
 *
 * @brief   Main loop
 *
 * @return  none
 */
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


int main(void)
{
    Delay_Init();
#ifdef DEBUG
    USART_Printf_Init(115200);
#endif
    printf("%s\r\n", VER_LIB);

    WCHBLE_Init();
    HAL_Init();
    GAPRole_PeripheralInit();
    Peripheral_Init();

    app_uart_init();
    Main_Circulation();
}