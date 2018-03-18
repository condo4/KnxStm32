/*
 * Low level access to TP-UART2 component
 * The TPUART contains the physical coupling to KNX Bus.
 * You can find TP-UART2 in a Siemens 5WG1117-2AB12
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * AUTHOR : Fabien Proriol       START DATE :  2017/11/04
 *
 */

#ifndef __KNXDEVICE__
#define __KNXDEVICE__

#include <stdint.h>
#include "usart.h"

#if !defined(GA_LEVEL)
#define GA_LEVEL 3
#endif

/* Header */
#define TELEGRAM_CTRL           0
#define TELEGRAM_SRC_ADDR_H     1
#define TELEGRAM_SRC_ADDR_L     2
#define TELEGRAM_DST_ADDR_H     3
#define TELEGRAM_DST_ADDR_L     4
#define TELEGRAM_ROUTING        5
#define TELEGRAM_COMMAND_H      6
#define TELEGRAM_COMMAND_L      7 /* + 6 bits payload */
#define TELEGRAM_PAYLOAD        8 /* Optional */

#define TELEGRAM_DPT_9_CRC      10
#define TELEGRAM_DPT_9_LEN      11

/*
 * GA is a 16 bit value
 * - 3-level: H/M/L: HHHHHMMM LLLLLLLL
 * - 2-level: H/L:   HHHHHLLL LLLLLLLL
 * - 1-level: L:     LLLLLLLL LLLLLLLL
 */
#if GA_LEVEL == 3
#define GA(h,m,l) ((h << 11) | (m << 8) | l)
#elif GA_LEVEL == 2
#define GA(h,l) ((h << 11) | l)
#else
#define GA(l) (l)
#endif

typedef uint16_t groupaddress_t;
typedef uint16_t phyaddress_t;


#define FATAL_ERROR_TPUART_RESET (1)
#define FATAL_ERROR_TPUART_PRDID (2)
#define FATAL_ERROR_TPUART_PADDR (3)
#define FATAL_ERROR_TPUART_APPS  (4)


void fatal_error(int error);
void knx_start_device(void);
void knx_start_service(UART_HandleTypeDef *uart);
void knx_send_telegram(uint8_t telegram[], uint8_t len);

phyaddress_t get_device_address(void);

int knx_create_device(UART_HandleTypeDef *uart, phyaddress_t phyaddress);

#endif
