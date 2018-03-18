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
#include "usart.h"
#include "debug.h"
#include "KnxTPUART.h"
#include "FreeRTOS.h"
#include <task.h>
#include <queue.h>
#include <semphr.h>

#ifdef DEBUG_TPUART
#define debug_printf(...) do { printf(__VA_ARGS__); } while (0)
#else
#define debug_printf(...) do { if(0) printf(__VA_ARGS__); } while (0)
#endif

/*
 * KnxTPUART
 * Manage TPUART2 device throw USART with Hal library
 * TPUART2 can be found in Siemens 5WG1117-2AB12 (25€)
 */
#define TPUART_SEND_PACKET_TIMEOUT      200
#define TPUART_RECIVED_PACKET_TIMEOUT      200


static uint8_t tpuart_reset_state = 0;
static uint8_t tpuart_state = 0;
static uint8_t tpuart_l_data_confirm = 0;


static uint16_t tpuart_productid = 0;

/* 8bit, we need 16 to code TPUART_REQUESTED_STATE */

static UART_HandleTypeDef *tpuart_uart = NULL;
static telegram_recived telegram_recived_callback = NULL;




uint32_t tpuart_get_state(void)
{
    return 
        (tpuart_reset_state) | 
        (tpuart_state << 8);
}

uint16_t tpuart_get_product_id(void)
{
    return tpuart_productid;
}

void tpuart_set_telegram_recived(telegram_recived callback)
{
    telegram_recived_callback = callback;
}

/* 
 * Thread KNX TP-UART RX
 * TPUART to Services (TPUART -> hostcontroller)
 */
#define TPUART_RECIVED_U_RESET_INDICATION               0x03
#define TPUART_RECIVED_STATE_INDICATION                 0x07


// Services to TPUART (hostcontroller -> TPUART)
#define TPUART_U_RESET_REQUEST          0x01
#define TPUART_U_STATE_REQUEST          0x02
#define TPUART_U_ACTIVATEBUSMON         0x05
#define TPUART_U_ACKINFORMATION         0x10 /* With nack as 0x04; busy as 0x02 and addressed as 0x01 */
#define TPUART_U_PRODUCTID_REQUEST      0x20
#define TPUART_U_ACTIVATEBUSYMODE       0x21
#define TPUART_U_RESETBUSYMODE          0x22
#define TPUART_U_MXRSTCNT               0x24
#define TPUART_U_ACTIVATECRC            0x25
#define TPUART_U_SETADDRESS             0x28
#define TPUART_U_L_DATASTART            0x80
#define TPUART_U_L_DATACONTINUE         0x80 /* With index as 0x3F in range [1..62] */
#define TPUART_U_L_DATAEND              0x40 /* With lenght as 0x3F in range [7..63] */
#define TPUART_U_POLLINGSTATE           0xE0 /* With slotnumber as 0x0F in range [0..14] */



static inline int tpuart_send_packet(uint8_t *buf, uint8_t len, const char *name)
{
#ifdef DEBUG_TPUART
    printf("\t%s TX:", name);
    for(int i=0; i < len; i++)
    {
        printf("%02x ", buf[i]);
    }
    printf("\r\n");
#endif
    
    if(HAL_UART_Transmit(tpuart_uart, buf, len, 100) != HAL_OK)
    {
        printf("!!! TPUART Error: %s failed\r\n", name);
        return -1;
    }
    return 0;
}


/*
 * Resets the TP-UART-IC to the initial state. At start-up the TP-UART-IC waits for a bus free
 * time-out before sending a U_Reset.indication-Service to the host controller. To be sure that
 * TP-UART-IC is in reset state the host controller has to wait for 50 ms and after that the
 * U_Reset.request-Service can be send. 
 */
int tpuart_U_Reset_request(void)
{
    uint8_t cmd = TPUART_U_RESET_REQUEST;
    tpuart_reset_state = TPUART_REQUESTED_STATE;
    return tpuart_send_packet(&cmd, 1, __FUNCTION__);
}


/*
 * Requests the internal communication state from the TP-UART-IC. The TP-UART-IC answers with the
 * Communication state.
 */
int tpuart_U_State_request(void)
{
    uint8_t cmd = TPUART_U_STATE_REQUEST;

    tpuart_state = TPUART_REQUESTED_STATE;
    return tpuart_send_packet(&cmd, 1, __FUNCTION__);
}


/*
 * Activates the busmonitor mode. That means each byte which is received on the EIB is sent
 * through the TP-UART-IC as well as illegal control bytes and not used immediate ACK. The
 * TP-UART-IC is absolute quiet (not sending) on the EIB. The busmonitor mode can only left
 * by using the U_Reset.request-Service.
 */
int tpuart_U_ActiveBusmon(void)
{
    uint8_t cmd = TPUART_U_ACTIVATEBUSMON;
    return tpuart_send_packet(&cmd, 1, __FUNCTION__);
}


/*
 * The ProductIdentifier.response service is sent if a U_ProductID.request service was received from
 * the host.
 */
int tpuart_U_ProductID_request(void)
{
    uint8_t cmd = TPUART_U_STATE_REQUEST;
    tpuart_productid = TPUART_REQUESTED_STATE16;
    return tpuart_send_packet(&cmd, 1, __FUNCTION__);
}

/*
 * If the host controller is temporarily not able to receive telegrams from the bus (e.g. due to no code
 * execution during flash erase), the TP-UART should reject frames from the bus with BUSY
 * acknowledges. The service activates the BUSY mode in the TP-UART for a fix period of 700 ms (+/-
 * 10ms). This means that all addressed telegrams (TP-UART has internally the "addressed" bit set) are
 * acknowledged with Busy.
 * Note: physical addressed frames which do not correspond with the TP-UART address will never be
 * acknowledged with BUSY!
 * All received telegrams are sent byte by byte from the TP-UART to the host as before. If the host
 * confirms a frame with the U_Ackinfo service (ACK, NACK or BUSY) the BusyMode will be
 * deactivated. After reset the BusyMode will be also deactivated.
 */
int tpuart_U_ActiveBusyMode(void)
{
    uint8_t cmd = TPUART_U_ACTIVATEBUSYMODE;
    return tpuart_send_packet(&cmd, 1, __FUNCTION__);
}

/*
 * The service U_ResetBusyMode deactivates immediately the BUSY mode in the TP-UART. All
 * addressed telegrams are answered according to the intern address flags. The host shall synchronize
 * its receiver before sending the U_ResetBusyMode.
 */
int tpuart_U_ResetBusyMode(void)
{
    uint8_t cmd = TPUART_U_RESETBUSYMODE;
    return tpuart_send_packet(&cmd, 1, __FUNCTION__);
}

/*
 * This service configures the physical address of the TP-UART. If this service is repeatedly sent (e.g.
 * the physical address changes), the new address will be active in the TP-UART after reception of the
 * complete U_SetAddress service and when no Layer 2 frame is currently received from the EIB. If the
 * address is set a complete address evaluation in the TP-UART is activated (group-addressed
 * telegrams are now generally confirmed).
 * Note: The TP-UART does not evaluate the received EIB telegrams which were sent itself.
 * After reset the address evaluation is deactivated again.
 * If the TP-UART is “addressed” and no error occurs then the TP-UART sends an IACK. If the TP-
 * UART is “addressed” and an error occurs (parity or checksum error) then it generates an INACK.
 * Note:
 * Also during activated address evaluation the host is able to manipulate the IACK generation of the
 * TP-UART by sending the U_AckInformation service.
 */
int tpuart_U_SetAddress(uint16_t phyaddress)
{
    uint8_t cmd[3] = {TPUART_U_SETADDRESS, (phyaddress >> 8) & 0xFF, phyaddress & 0xFF};
    return tpuart_send_packet(cmd, 3, __FUNCTION__);
}

/*
 * The U_AckInformation-Service is only sent to the TP-UART if a host controller wants to check the
 * destination address itself. This service is sent by the host after the address evaluation and must be
 * sent latest 1,7 ms after receiving the address type octet of an addressed frame. The Nack-/ Busy-/
 * Addr-Bits sets internal flags in the TP-UART. The NACK flag is set by the TP-UART itself if it is
 * detecting any frame error.
 * If the TP-UART receives this service and the addressed bit is set it will generate a ACK, NACK or
 * BUSY-frame on the EIB depending on the settings of the NACK/busy-flags.
 * UART-Controlfield
 */
int tpuart_U_AckInformation(uint8_t nack, uint8_t busy, uint8_t addresses)
{
    uint8_t cmd = TPUART_U_ACKINFORMATION | ((nack & 0x1) << 2) | ((busy & 0x1) << 1) | (addresses & 0x1);
    return tpuart_send_packet(&cmd, 1, __FUNCTION__);
}


/*
 * The U_L_Data-Services are used to transfer the complete EIB-Linklayer-Frame (L_Data.request and
 * L_PollData.request) to the TP-UART.
 * If the host sends a second frame (the first frame buffer is yet active) the TP-UART rejects this and
 * reports it with a Status.indication (PE bit set).
 *
 * The U_L_DataStart-Service initialize the TP-UART-IC to receive a complete EIB-Linklayer-Frame
 * from the host. As additional data the EIB-Control-byte is transmitted which is the control field of the
 * L_Data-frame or L_Polldata-frame. If the repetition flag in the control byte is just cleared the TP-
 * UART transmits the frame only once time with repetition flag set.
 */
static int tpuart_U_L_Data_Start(uint8_t ctrl)
{
    uint8_t cmd[2] = {TPUART_U_L_DATASTART, ctrl};
    tpuart_l_data_confirm = TPUART_REQUESTED_STATE;
    return tpuart_send_packet(cmd, 2, __FUNCTION__);
}

/*
 * The U_L_DataContinue-Service transmits one byte of the contents of an EIB-L_Data-Frame to the
 * TP-UART. The data-index starts with 1 and the maximum value is 62 depending on the length of the
 * frame.
 */
static int tpuart_U_L_Data_Continue(uint8_t index, uint8_t data)
{
    uint8_t cmd[2] = {TPUART_U_L_DATACONTINUE | (index & 0x3F), data};
    return tpuart_send_packet(cmd, 2, __FUNCTION__);
}

/*
 * The U_L_DataEnd-Service marks the end of the transmission of the EIB-Frame. After receiving this
 * service the TP-UART checks the checksum and if correct the transmission starts on the EIB else the
 * UART returns a state-indication with Receive-Errorflag is set.
 */
static int tpuart_U_L_Data_End(uint8_t length, uint8_t checksum)
{
    uint8_t cmd[2] = {TPUART_U_L_DATAEND | ((length - 1) & 0x3F), checksum};
    return tpuart_send_packet(cmd, 2, __FUNCTION__);
}

/*
 * This service adjusts the maximum number of repetitions on bus after a frame has not been
 * acknowledged with IACK. Values from 0 to 7 are separately adjustable for BUSY or INACK. No
 * acknowledge will be handled as INACK. If the host clears the repetition flag in the U_L_DataStart
 * service always no repetitions will be sent.
 * After Reset 3 repetitions are active.
 */
int tpuart_U_MxRstCnt(uint8_t busycnt, uint8_t nack_cnt)
{
    uint8_t cmd[2] = {TPUART_U_MXRSTCNT, ((busycnt & 0x7) << 5) | (nack_cnt & 0x7) };
    return tpuart_send_packet(cmd, 2, __FUNCTION__);
}


/*
 * This service activates a 16 bit CRC calculation for every received Layer 2 Service from the bus
 * (except when the TP-UART is Polling Slave or the busmonitormode is active). However the CRC
 * calculation only becomes active at the host speed of 19200 Baud.
 * The TP-UART calculates over the complete received telegram (including the Layer 2 checksum) a
 * CRC16-CCITT with the following parameters:
 * - Width= 16 bit
 * - Truncated polynomial = 1021hex
 * - init value = FFFFh
 * - I/O not reflected
 * - no xor on output CRC
 * - Test string „123456789“ is 0xE5CC
 * and adds it to the receiving frame (sending order is highbyte then lowbyte).
 * After Reset CRC calculation is disabled.
 */
int tpuart_U_ActivateCRC(uint8_t busycnt, uint8_t nack_cnt)
{
    uint8_t cmd = TPUART_U_ACTIVATECRC;
    return tpuart_send_packet(&cmd, 1, __FUNCTION__);
}

/*
 * This service must be send to the TP-UART if an Pollingframe-Ctrlbyte is received. If the TP-UART
 * detects a collision during sending the slave slot to the EIB the TP-UART generates a State indication
 * with the Slave Collisionflag set.
 */
int tpuart_U_PollingState(uint8_t slot, uint16_t polladdr, uint8_t state)
{
    uint8_t cmd[4] = {TPUART_U_POLLINGSTATE | (slot & 0xF), (polladdr & 0xFF00) >> 8, polladdr & 0xFF, state};
    return tpuart_send_packet(cmd, 4, __FUNCTION__);
}






#define TPUART_TELEGRAM_POOL_SIZE 8
#define STACK_SIZE_TelegramSplitter 128
#define STACK_SIZE_TelegramDispatcher 128

static TaskHandle_t xTaskTelegramSplitterHandle = NULL;
static TaskHandle_t xTaskTelegramDispatcherHandle = NULL;
static TaskHandle_t xTaskTelegramSenderHandle = NULL;
static QueueHandle_t xQueueControlerRx = NULL;
static QueueHandle_t xQueueTelegramEmpty = NULL;
static QueueHandle_t xQueueTelegramReady = NULL;
static QueueHandle_t xQueueTelegramToSendReady = NULL;

struct tpuart_rx_byte_t {
    uint8_t byte;
    unsigned int tick;
};

#if configSUPPORT_STATIC_ALLOCATION
StaticTask_t xTaskBufferTelegramSplitter;
StackType_t xStackTelegramSplitter[ STACK_SIZE_TelegramSplitter ];

StaticTask_t xTaskBufferTelegramDispatcher;
StackType_t xStackTelegramDispatcher[ STACK_SIZE_TelegramDispatcher ];

static StaticQueue_t xQueueStaticControlerRx;
uint8_t ucQueueControlerRxStorageArea[ 128 * sizeof( struct tpuart_rx_byte_t ) ];

static StaticQueue_t xQueueStaticTelegramEmpty;
uint8_t ucQueueTelegramEmptyStorageArea[ TPUART_TELEGRAM_POOL_SIZE * sizeof( void *) ];

static StaticQueue_t xQueueStaticTelegramReady;
uint8_t ucQueueTelegramReadyStorageArea[ TPUART_TELEGRAM_POOL_SIZE * sizeof( void *) ];

static StaticQueue_t xQueueStaticTelegramToSendReady;
uint8_t ucQueueTelegramToSendReadyStorageArea[  4 * sizeof( struct telegram_to_send ) ];
#endif



static struct tpuart_rx_byte_t tpuart_rx_byte = {0,0};

typedef uint8_t telegram_t[TPUART_MAX_KNX_TELEGRAM_SIZE];
static telegram_t telegrams_pool[TPUART_TELEGRAM_POOL_SIZE];
static SemaphoreHandle_t tpuart_spliter_started;
static SemaphoreHandle_t tpuart_started;
static uint8_t tpuart_started_bool = 0;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken;

    if(huart == tpuart_uart)
    {
        tpuart_rx_byte.tick = HAL_GetTick();
        if( xQueueSendFromISR( xQueueControlerRx, &tpuart_rx_byte, &xHigherPriorityTaskWoken ) != pdPASS )
        {
            printf("!!! TIMOUT to send byte in RX Queue\r\n");
        }

        if(HAL_UART_Receive_IT(tpuart_uart, &(tpuart_rx_byte.byte), 1) != HAL_OK)
        {
            printf("!!! Error on HAL_UART_Receive_IT\r\n");
        }
    }
}


void tpuart_complete_telegram(telegram_t **telegram, uint8_t **currentByte)
{
    void *ptr = *telegram;
    /* Save telegram length in last telegram byte */
    int len =  (*currentByte - &((*(*telegram))[0]));
    *currentByte = &((*(*telegram))[0]);
    (*currentByte) += TPUART_MAX_KNX_TELEGRAM_SIZE - 1;
    **currentByte = len;
    debug_printf("Telegram length (at %p) is %i\r\n", *currentByte, len);
    
    if(xQueueSend(xQueueTelegramReady, &ptr, TPUART_SEND_PACKET_TIMEOUT) != pdTRUE)
    {
        printf("!!! Fail to send ready telegram\r\n");
    }
    if(!xQueueReceive( xQueueTelegramEmpty, &ptr, 5000 ))
    {
        printf("!!! Can't get an empty telegram\r\n");
    }
    *telegram = ptr;
    *currentByte = &((*(*telegram))[0]);
    debug_printf("Use new telegram at 0x%p\r\n", *currentByte);
}

void vTaskTelegramSplitter( void * pvParameters )
{
    static telegram_t *telegram;
    static uint8_t *currentByte;
    static void *ptr;
    static unsigned int lastTick = 0;
    static struct tpuart_rx_byte_t rx_data;
    debug_printf("Start Telegram Splitter task\r\n");
    
    if(HAL_UART_Receive_IT(tpuart_uart, &tpuart_rx_byte.byte, 1) != HAL_OK)
    {
        printf("!!! Error to ARM RX IT\r\n");
        return;
    }

    if(!xQueueReceive( xQueueTelegramEmpty, &ptr, 5000 ))
    {
        printf("!!! Can't get an empty telegram\r\n");
    }
    telegram = ptr;
    currentByte = &((*telegram)[0]);
    debug_printf("Use new telegram at %p\r\n", currentByte);

    /* Ready to recive telegrams */
    xSemaphoreGive(tpuart_spliter_started);

    for( ;; )
    {
        /* Task code goes here. */
        if( xQueueReceive( xQueueControlerRx, &rx_data, 4 ) )
        {
            if(rx_data.tick - lastTick > 4)
            {
                if(lastTick != 0)
                {
                    debug_printf("End of telegram\r\n");
                    tpuart_complete_telegram(&telegram, &currentByte);
                }
            }
            debug_printf("Recive byte 0x%X after %i ticks\r\n", rx_data.byte, rx_data.tick - lastTick);
            lastTick = rx_data.tick;
            *currentByte++ = rx_data.byte;
        }
        else
        {
            if(lastTick != 0)
            {
                debug_printf("End of telegram\r\n");
                tpuart_complete_telegram(&telegram, &currentByte);
                lastTick = 0;
            }
        }
    }
}

void vTaskTelegramDispatcher( void * pvParameters )
{
    static telegram_t *telegram;
    static void *ptr;

    /* First wait reciver is ready */
    if( xSemaphoreTake(tpuart_spliter_started, 10000) != pdTRUE)
    {
        printf("!!! ERROR: Splitter never ready....\r\n");
    }

    /* TPUART Ready */
    xSemaphoreGive(tpuart_spliter_started);
    tpuart_started_bool = 1;

    for( ;; )
    {
        /* Task code goes here. */

        if(xQueueReceive( xQueueTelegramReady, &ptr, 10000 ))
        {
            telegram = ptr;

            if( ((*telegram)[0] & 0x3) == 0x3 ) /*  TP-UART-Control-Services */
            {
                /*
                * TP-UART-Control-Services
                * The TP-UART-Control-Services are services which exist only on this interface. They have to
                * reset the communication or to inform the host controller about the actual state.
                */
                if((*telegram)[0] == TPUART_RECIVED_U_RESET_INDICATION)
                {
                    /* 
                    * The Reset.indication-Service is sent after each reset (e.g. TP_UART_Reset.requ). 
                    */
                    tpuart_reset_state = TPUART_RECIVED_U_RESET_INDICATION;
#ifdef DEBUG_TPUART_HIGHTLEVEL
                    printf("TPUART: Enter in reset state\r\n");
#endif
                }
                else if(((*telegram)[0] & TPUART_RECIVED_STATE_INDICATION) == TPUART_RECIVED_STATE_INDICATION)
                {
                    /*
                    * The TP-UART-State.response-Service is sent if an U_State.request-Service was received
                    * from the host controller. In case of a slave collision, receive error, checksum error or
                    * protocol error the TP-UART-IC sends a State.indication-Service.
                    * [SC RE TE PE TW 1 1 1]
                    * SC: Slave Collision
                    * RE: Receive Error (checksum, parity or bit error)
                    * TE: Transmitter Error (send 0 receive 1)
                    * PE: Protocol Error (e.g. illegal control byte)
                    * TW: Temperature Warning
                    */
                    tpuart_state = (*telegram)[0];
                    printf("TPUART: TP UART state changed: 0x%X\r\n", tpuart_state);
#ifdef DEBUG_TPUART_HIGHTLEVEL
                    printf("TPUART: TP UART state changed: 0x%X\r\n", tpuart_state);
#endif
                }
                else {
                    /* 
                    * TP-UART-L_Data.confirm Service
                    * The L_DATA.confirm service is transmitted to the host controller if an acknowledge was
                    * received or the last repetition is transmitted and no acknowledge was received.
                    * TP-UART-Control Field
                    */
                    tpuart_l_data_confirm = (((*telegram)[0] & 0x8) != 0);
                }
            }
            else if( (*telegram)[0] == 0xCC || (*telegram)[0] == 0x0C || (*telegram)[0] == 0xC0 )
            {
                /*
                    * Acknowledge-Services (not managed yet)
                    * TODO: Manage it
                    * Acknowledge-Services are just transmitted to the host controller in busmonitor mode.
                    * The short acknowledgment frame format consists of 15 Tbit (1 Tbit = 1/9600 s) idle time
                    * followed by a single character which is used to acknowledge a L_Data.req frame. The
                    * following figure shows the corresponding codes of the short acknowledgment.
                    * 0xCC: ACK
                    * 0x0C: NACK
                    * 0xC0: BUSY
                    */
                printf("!!! TPUART: Acknowledge-Services telegram not managed (TODO)\r\n");
            }
            else
            {
                /*
                * Layer-2-Services
                * The Layer-2-Services include all standard EIB Link-Layer-Services. The control fields are
                * followed by the data of the EIB frame. All bytes received on the EIB are immediately sent to
                * the host controller. The host controller has to detect a end of packet time out by supervising
                * the EOP gap of 2 to 2.5 bittimes.
                */
                static uint8_t crc;
                static uint8_t valid;
                crc = (*telegram)[0];

                /* Check CRC */
                for(uint8_t *ptr = &((*telegram)[0]) + 1; ptr < (&((*telegram)[0]) + (*telegram)[TPUART_MAX_KNX_TELEGRAM_SIZE - 1] - 1); ptr++)
                {
                    crc ^= *ptr;
                }
                crc = (~crc) & 0xFF;
                valid = (crc == (*telegram)[(*telegram)[TPUART_MAX_KNX_TELEGRAM_SIZE - 1] - 1]);
                
#ifdef DEBUG_TPUART_HIGHTLEVEL
                printf("RX %02i bytes: ", (*telegram)[TPUART_MAX_KNX_TELEGRAM_SIZE - 1]);
                for(int i=0; i <  (*telegram)[TPUART_MAX_KNX_TELEGRAM_SIZE - 1]; i++)
                {
                    printf("%02X ", (*telegram)[i]);
                }
                printf(" %s\r\n", (valid)?("valid"):("not valid"));
#endif
                if(valid)
                {
                    if(telegram_recived_callback != NULL)
                    {
                        (*telegram_recived_callback)(*telegram, (*telegram)[TPUART_MAX_KNX_TELEGRAM_SIZE - 1]);
                    }
                }
            }

            if(xQueueSend(xQueueTelegramEmpty, &ptr, TPUART_SEND_PACKET_TIMEOUT) != pdTRUE)
            {
                printf("!!! Fail to send empty telegram\r\n");
            }
        }
    }
}

void vTaskTelegramSender( void * pvParameters)
{
    static struct telegram_to_send msg;
    printf("@@@@Start vTaskTelegramSender\r\n");

    for( ;; )
    {
        /* Task code goes here. */
        if(xQueueReceive( xQueueTelegramToSendReady, &msg, 10000 ))
        {
            static uint8_t crc;
            static uint8_t i = 0;

#ifdef DEBUG_TPUART
            printf("Task Sender: TX ");
            for(int i=0; i < msg.len; i++)
            {
                printf("%02x ", msg.telegram[i]);
            }
            printf("\r\n");
#endif
            crc = msg.telegram[0];
            debug_printf("Start to send telegram: \r\n");
            tpuart_U_L_Data_Start(msg.telegram[0]);
            for(i = 1; i < msg.len - 1; i++)
            {
                tpuart_U_L_Data_Continue(i, msg.telegram[i]);
                crc ^= msg.telegram[i];
            }
            tpuart_U_L_Data_End(msg.len, ((~crc) & 0xFF));
            if(msg.telegram[i] != ((~crc) & 0xFF))
            {
                debug_printf("ERROR: KnxDevice CRC Error %x %x\r\n", msg.telegram[i], ((~crc) & 0xFF));
            }
            debug_printf("Telegram sent\r\n");
            tpuart_U_State_request();
        }
    }
}

void tpuart_send_telegram(struct telegram_to_send msg)
{
    BaseType_t xHigherPriorityTaskWoken;
    if( xQueueSendFromISR( xQueueTelegramToSendReady, &msg, &xHigherPriorityTaskWoken ) != pdPASS )
    {
        printf("!!! Fail to register telegram into send queue\r\n");
    }
}

int tpuart_create_tasks(UART_HandleTypeDef *uart)
{
    debug_printf("TPUART Reset controler\r\n");
    tpuart_uart = uart;
    
    BaseType_t xReturned;
    
    /* Create the Receiver task */
#if configSUPPORT_STATIC_ALLOCATION
    xQueueControlerRx = xQueueCreateStatic( 128, sizeof( struct tpuart_rx_byte_t ) ,
                                 ucQueueControlerRxStorageArea, &xQueueStaticControlerRx );
#else
    xQueueControlerRx = xQueueCreate( 128, sizeof( struct tpuart_rx_byte_t ) );
#endif

    if(xQueueControlerRx == NULL)
    {
        printf("!!! Can't create Rx Queue\r\n");
        return -1;
    }
    
    /* Create the Telegram Sender Queue  */
#if configSUPPORT_STATIC_ALLOCATION
    xQueueTelegramToSendReady = xQueueCreateStatic( 128, sizeof( struct telegram_to_send ) ,
                                 ucQueueTelegramToSendReadyStorageArea, &xQueueStaticTelegramToSendReady );
#else
    xQueueTelegramToSendReady = xQueueCreate( 4, sizeof( struct telegram_to_send ) );
#endif
    if(xQueueTelegramToSendReady == NULL)
    {
        printf("!!! Can't create Rx Telegram Ready Queue\r\n");
        return -1;
    }

    /* Create the Telegram Pool  */
#if configSUPPORT_STATIC_ALLOCATION
    xQueueTelegramEmpty = xQueueCreateStatic( TPUART_TELEGRAM_POOL_SIZE, sizeof( void *) ,
                                 ucQueueTelegramEmptyStorageArea, &xQueueStaticTelegramEmpty );
#else
    xQueueTelegramEmpty = xQueueCreate( TPUART_TELEGRAM_POOL_SIZE, sizeof( void *) );
#endif
    if(xQueueTelegramEmpty == NULL)
    {
        printf("!!! Can't create Rx Telegram Empty Queue\r\n");
        return -1;
    }

#if configSUPPORT_STATIC_ALLOCATION
    xQueueTelegramReady = xQueueCreateStatic( TPUART_TELEGRAM_POOL_SIZE, sizeof( void * ) ,
                                 ucQueueTelegramReadyStorageArea, &xQueueStaticTelegramReady );
#else
    xQueueTelegramReady = xQueueCreate( TPUART_TELEGRAM_POOL_SIZE, sizeof( void * ) );
#endif
    
    if(xQueueTelegramReady == NULL)
    {
        printf("!!! Can't create Rx Telegram Ready Queue\r\n");
        return -1;
    }
    for(int i = 0; i < TPUART_TELEGRAM_POOL_SIZE; i++)
    {
        void *ptr = &(telegrams_pool[i]);
        debug_printf("New Telegram[%i] at 0x%p\r\n", i, ptr);
        if(xQueueSend(xQueueTelegramEmpty, &ptr, TPUART_SEND_PACKET_TIMEOUT) != pdTRUE)
        {
            printf("!!! Fail to register telegram %i into pool\r\n", i);
        }
    }




    /* Create the task, storing the handle. */
#if configSUPPORT_STATIC_ALLOCATION
    xTaskTelegramSplitterHandle = xTaskCreateStatic(
                    vTaskTelegramSplitter,       /* Function that implements the task. */
                    "TPU_SPL",          /* Text name for the task. */
                    STACK_SIZE_TelegramSplitter,      /* Number of indexes in the xStack array. */
                    ( void * ) uart,    /* Parameter passed into the task. */
                    3,/* Priority at which the task is created. */
                    xStackTelegramSplitter,          /* Array to use as the task's stack. */
                    &xTaskBufferTelegramSplitter );  /* Variable to hold the task's data structure. */
#else
    xReturned = xTaskCreate(
                    vTaskTelegramSplitter,       /* Function that implements the task. */
                    "TPU_SPL",          /* Text name for the task. */
                    STACK_SIZE_TelegramSplitter,      /* Stack size in words, not bytes. */
                    ( void * ) uart,    /* Parameter passed into the task. */
                    3,/* Priority at which the task is created. */
                    &xTaskTelegramSplitterHandle );      /* Used to pass out the created task's handle. */
    if(xReturned != pdPASS)
    {
        printf("!!! Can't create vTaskTelegramSplitter Task\r\n");
        return -1;
    }
#endif


    /* Create the task, storing the handle. */
#if configSUPPORT_STATIC_ALLOCATION
    xTaskTelegramDispatcherHandle = xTaskCreateStatic(
                    vTaskTelegramDispatcher,       /* Function that implements the task. */
                    "TPU_DIS",          /* Text name for the task. */
                    STACK_SIZE_TelegramSplitter,      /* Number of indexes in the xStack array. */
                    NULL,    /* Parameter passed into the task. */
                    2,/* Priority at which the task is created. */
                    xStackTelegramDispatcher,          /* Array to use as the task's stack. */
                    &xTaskBufferTelegramDispatcher );  /* Variable to hold the task's data structure. */
#else
    xReturned = xTaskCreate(
                    vTaskTelegramDispatcher,       /* Function that implements the task. */
                    "TPU_DIS",          /* Text name for the task. */
                    STACK_SIZE_TelegramDispatcher,      /* Stack size in words, not bytes. */
                    NULL,    /* Parameter passed into the task. */
                    2,/* Priority at which the task is created. */
                    &xTaskTelegramDispatcherHandle );      /* Used to pass out the created task's handle. */
    
    if(xReturned != pdPASS)
    {
        printf("!!! Can't create Rx Task\r\n");
        return -1;
    }
#endif
    /* Create the Sender task, storing the handle. */
    xReturned = xTaskCreate(
                    vTaskTelegramSender,       /* Function that implements the task. */
                    "TPU_TX",          /* Text name for the task. */
                    128,      /* Stack size in words, not bytes. */
                    NULL,    /* Parameter passed into the task. */
                    2,/* Priority at which the task is created. */
                    &xTaskTelegramSenderHandle );      /* Used to pass out the created task's handle. */
    
    if(xReturned != pdPASS)
    {
        printf("!!! Can't create Rx Task\r\n");
        return -1;
    }

    tpuart_spliter_started = xSemaphoreCreateBinary();
    if( tpuart_spliter_started == NULL )
    {
        printf("!!! ERROR: Can't create semaphore\r\n");
    }
    else
    {
        xSemaphoreGive(tpuart_spliter_started);
        xSemaphoreTake(tpuart_spliter_started, 1000);
    }

    tpuart_started = xSemaphoreCreateBinary();
    if( tpuart_started == NULL )
    {
        printf("!!! ERROR: Can't create semaphore\r\n");
    }
    else
    {
        xSemaphoreGive(tpuart_started);
        xSemaphoreTake(tpuart_started, 1000);
    }


    debug_printf("TPUART Task ready\r\n");
    return 0;
}

void tpuart_wait_started(void)
{
    if(tpuart_started_bool == 0)
    {
        printf("TPUART, wait to started\r\n");
        xSemaphoreTake(tpuart_started, 1000);
    }
}
