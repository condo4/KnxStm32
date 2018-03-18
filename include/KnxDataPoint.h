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

#ifndef __KNXDATAPOINT__
#define __KNXDATAPOINT__

#include <stdint.h>
#include "KnxConfig.h"
#include "KnxDevice.h"

/*
 * FLAG: xx  xx   C   R   W   T   U   I
 */
typedef uint8_t flag_t;
#define KNX_FLAG_U 0x02 /* Change internal value if Response Telegram is recive */
#define KNX_FLAG_T 0x04 /* Need to send a Write telegram if internal value change */
#define KNX_FLAG_W 0x08 /* Change internal value if Write Telegram is recived*/
#define KNX_FLAG_R 0x10 /* Need to respond to Read Request */
#define KNX_FLAG_I 0x01 /* Need tp send a Read Request on startup */
#define KNX_FLAG_C 0x20 /* Object Enable */

/* FLAG HELPERS */
#define KNX_SENSOR (KNX_FLAG_C | KNX_FLAG_R | KNX_FLAG_T) /* For all physical sensor (Temperature sensor...) */
#define KNX_IMAGE  (KNX_FLAG_C | KNX_FLAG_W | KNX_FLAG_U | KNX_FLAG_I) /* To have in device an updated image to another DPT value */

/* COMMAND */
#define KNX_COMMAND_WRITE    0x80
#define KNX_COMMAND_READ     0x00
#define KNX_COMMAND_RESPONSE 0x40


#if defined(KNX_ENABLE_DPT_9)
/*
 * DPT 9.xxx : 16bits Float
 * 2 Byte Float: SEEEEMMM MMMMMMMM
 *   - S: Sign [0, 1]
 *   - E: Exponent [0:15]
 *   - M: Significand (Mantissa) [-2048:2047]
 *
 * RAW to FLOAT:
 * sign = (data & 0x8000) >> 15
 * exp = (data & 0x7800) >> 11
 * mant = data & 0x07ff
 * if(sign != 0): mant = -(~(mant - 1) & 0x07ff)
 * value = (1 << exp) * 0.01 * mant
 *
 * FLOAT to RAW
 * sign = 0
 * exp = 0
 * if value < 0: sign = 1
 * mant = int(value * 100)
 * while not -2048 <= mant <= 2047:
 *     mant = mant >> 1
 *     exp += 1
 * data = (sign << 15) | (exp << 11) | (int(mant) & 0x07ff)
 *
 * For all Datapoint Types 9.xxx, the encoded value 7FFFh shall always be used to denote invalid data.
 *
 * DPT 9.001: Temperature in Celcius
 *
 */
void set_float(void *datapoint, void *value);


typedef unsigned char FRAME_DPT_9[TELEGRAM_DPT_9_CRC + 1];

/*
 * bc 00 65 11 01 2a e3 00 80 00
 * ctrl    = bc
 * src     = 00 65 vs 0x1165
 * dst     = 11 01 vs 0x2a01
 * routing = 2a
 * command = e3 00
 * data    = 00 80
 * crc     = 00
 */

typedef struct __KNX_DPT_9 {
    groupaddress_t address;
    flag_t flag;
    float data;
    uint8_t valid;
    void (*set)(void *datapoint, void *value);

    /* Send Telegram management */
    FRAME_DPT_9 telegram;
} KNX_DPT_9;

inline void init_KNX_DPT_9(KNX_DPT_9 *obj, groupaddress_t addr, flag_t flag)
{
    obj->address = addr;
    obj->flag = flag;
    obj->data = 0;
    obj->valid = 0;
    obj->set = &set_float;
    obj->telegram[TELEGRAM_CTRL] = 0xBC;
    obj->telegram[TELEGRAM_SRC_ADDR_H] = get_device_address() >> 8;
    obj->telegram[TELEGRAM_SRC_ADDR_L] = get_device_address() & 0xFF;
    obj->telegram[TELEGRAM_DST_ADDR_H] = addr >> 8;
    obj->telegram[TELEGRAM_DST_ADDR_L] = addr & 0xFF;
    obj->telegram[TELEGRAM_ROUTING] = 0xE3;
}


#endif




#endif
