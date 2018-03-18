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

#ifndef __KNXTPUART__
#define __KNXTPUART__

#include <stdint.h>
#include "usart.h"

#define TPUART_MAX_KNX_TELEGRAM_SIZE    32 /* The last byte is the telegram len */

#define TPUART_SERVICE_RUNNING   (1)
#define TPUART_SERVICE_WAITSTOP  (2)
#define TPUART_SERVICE_STOP      (0)
#define TPUART_RESET_MASK        (0x3)
#define TPUART_RESET_OK          (0x3)
#define TPUART_REQUESTED_STATE   (0xFF)
#define TPUART_REQUESTED_STATE16 (0xFFFF)

typedef void (*telegram_recived)(uint8_t *, uint8_t);

struct telegram_to_send {
    uint8_t *telegram;
    uint8_t len;
};

void tpuart_send_telegram(struct telegram_to_send msg);

/*
 * Set callback call each time a complete telegram is recived
 * Must be define as:
 * void my_telegram_reciver(uint8_t *eib_telegram, uint8_t lenght);
 * tpuart_set_telegram_recived(&my_telegram_reciver);
 */
void tpuart_set_telegram_recived(telegram_recived callback);


/* 
 * tpuart_get_state:
 * Read TPUART State (32bit)
 * 0x00CCBBAA
 * Where:
 * AA: Thread RX state: 
 *     - TPUART_SERVICE_RUNNING
 *     - TPUART_SERVICE_WAITSTOP
 *     - TPUART_SERVICE_STOP
 * BB: Reset state return by TPUART
 *     - 0x0 Wait Reset state
 *     - 0x3 Reset State OK
 * CC: TP-UART State
 *     - [SC RE TE PE TW 1 1 1]
 *       SC: Slave Collision
 *       RE: Receive Error (checksum, parity or bit error)
 *       TE: Transmitter Error (send 0 receive 1)
 *       PE: Protocol Error (e.g. illegal control byte)
 *       TW: Temperature Warning
 */
uint32_t tpuart_get_state(void);


/*
 * Return tpuart ProductID
 * need to call tpuart_U_ProductID_request before
 * return 0 if never asked
 * return TPUART_REQUESTED_STATE16 while waiting response
 * return real value when recived
 */
uint16_t tpuart_get_product_id(void);

/*
 * Start RX task
 * This blocking function must be call in a seperate thread
 */
void tpuart_start_service(UART_HandleTypeDef *uart);

/*
 * Service to TP-UART methods (see TP-UART2 documentation)
 */
int tpuart_U_Reset_request(void);
int tpuart_U_State_request(void);
int tpuart_U_ActiveBusmon(void);
int tpuart_U_ProductID_request(void);
int tpuart_U_ActiveBusyMode(void);
int tpuart_U_ResetBusyMode(void);
int tpuart_U_SetAddress(uint16_t phyaddress);
int tpuart_U_AckInformation(uint8_t nack, uint8_t busy, uint8_t addresses);
int tpuart_U_MxRstCnt(uint8_t busycnt, uint8_t nack_cnt);
int tpuart_U_ActivateCRC(uint8_t busycnt, uint8_t nack_cnt);
int tpuart_U_PollingState(uint8_t slot, uint16_t polladdr, uint8_t state);

int tpuart_create_tasks(UART_HandleTypeDef *uart);
void tpuart_wait_started(void);

#endif
