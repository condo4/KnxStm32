#include "KnxDevice.h"
#include "KnxTPUART.h"
#include "debug.h"
#include "gpio.h"
#include "string.h"
#include "stdlib.h"
#include "FreeRTOS.h"
#include <task.h>
#include <queue.h>
#include <semphr.h>


#ifdef DEBUG_KNXDEVICE
#define debug_printf(...) do { printf(__VA_ARGS__); } while (0)
#else
#define debug_printf(...) do { if(0) printf(__VA_ARGS__); } while (0)
#endif


void fatal_error(int error)
{
    if(error < FATAL_ERROR_TPUART_APPS) return;
    for(;;)
    {
        for(int i = 0; i < error; i++)
        {
            HAL_Delay(300);
            HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
            HAL_Delay(50);
            HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        }
        HAL_Delay(2000);
    }
}


/*
 * Telegram:
 * => Fields details :
 *      -Control Field : "FFR1 PP00" format with
 *         FF = Frame Format (10 = Std Length L_DATA service, 00 = extended L_DATA service, 11 = L_POLLDATA service)
 *          R = Repeatflag (1 = not repeated, 0 = repeated)
 *         PP = Priority (00 = system, 10 = alarm, 01 = high, 11 = normal)
 *      -Routing Field  : "TCCC LLLL" format with
 *          T = Target Addr type (1 = group address/muticast, 0 = individual address/unicast)
 *        CCC = Counter
 *       LLLL = Payload Length (1-15)
 *      -Command Field : "00XX XXCC CCDD DDDD" format with
 *         XX = Not used
 *         CC = command (0000 = Value Read, 0001 = Value Response, 0010 = Value Write, 1010 = Memory Write)
 *         DD = Payload Data (1st payload byte)
 */

void telegram_recived_callback(uint8_t *telegram, uint8_t size)
{
#ifdef DEBUG_KNXDEVICE_TELEGRAM
    char *sep;
    uint8_t command;

    printf("KnxDevice: Rx: ");
    for(uint8_t *ptr = telegram; ptr < (telegram + size); ptr++)
    {
        printf("%02x ", *ptr);
    }
    for(int i = 0; i < 16 - size; i++) printf("   ");

    /* Decode command */
    command = (((*(telegram + TELEGRAM_COMMAND_H) ) & 0x3) << 2 ) | (((*(telegram + TELEGRAM_COMMAND_L) ) & 0xC0) >> 6);
    switch(command)
    {
        case 0b0000:
            printf("> READ: from ");
            break;
        case 0b0001:
            printf("> RESPONSE: from ");
            break;
        case 0b0010:
            printf("> WRITE: from ");
            break;
        case 0b1010:
            printf("> MEMORY WRITE: from ");
            break;
        default:
            printf("> ERROR CMD: from ");
    }

    if((*(telegram + TELEGRAM_ROUTING) ) & 0x80)
    {
        /* Group Address Destination */
        sep = "/";
    }
    else
    {
        /* Individual Address Destination */
        sep = ".";
    }

    printf("%i.%i.%i to %i%s%i%s%i\r\n",
           ((*(telegram + TELEGRAM_SRC_ADDR_H) >> 4) & 0xF),
           ((*(telegram + TELEGRAM_SRC_ADDR_H) ) & 0xF),
           ((*(telegram + TELEGRAM_SRC_ADDR_L) ) & 0xFF),
           ((*(telegram + TELEGRAM_DST_ADDR_H) >> 3) & 0x1F),
           sep,
           ((*(telegram + TELEGRAM_DST_ADDR_H) ) & 0x7),
           sep,
           ((*(telegram + TELEGRAM_DST_ADDR_L) ) & 0xFF)
          );

#endif
}


void knx_send_telegram(uint8_t telegram[], uint8_t len)
{
    static struct telegram_to_send msg;
    msg.telegram = telegram;
    msg.len = len;
    tpuart_send_telegram(msg);
}



static phyaddress_t device_address = 0;

phyaddress_t get_device_address(void)
{
    return device_address;
}

void knx_start_device(void)
{
    tpuart_wait_started();
    
    debug_printf("TPUART started, now start KNX Device\r\n");
    
    tpuart_U_Reset_request();

    while((tpuart_get_state() & 0xFF) != TPUART_RESET_OK)
    {
        vTaskDelay(10);
    }

    debug_printf("TPUART reseted\r\n");

    tpuart_U_SetAddress(device_address);

    debug_printf("KnxDevice started\r\n");
    
    tpuart_U_State_request();
    
    vTaskDelay(10);
}

int knx_create_device(UART_HandleTypeDef *uart, phyaddress_t phyaddress)
{
    printf("Create new KNX Device on address %i.%i.%i\r\n", 
           ((phyaddress >> 12) & 0xF),
           ((phyaddress >> 8) & 0xF),
           ((phyaddress) & 0xFF)
    );
    device_address = phyaddress;

    if(tpuart_create_tasks(uart) != 0)
    {
        printf("!!! KnxDevice: Error when initialize TPUART\r\n");
    }
    tpuart_set_telegram_recived(&telegram_recived_callback);
    
    return 0;
}
