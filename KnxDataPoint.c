#include "KnxDataPoint.h"
#include "debug.h"
#include "stdlib.h"

#ifdef DEBUG_KNXDATAPOINT
#define debug_printf(...) do { printf(__VA_ARGS__); } while (0)
#else
#define debug_printf(...) do { if(0) printf(__VA_ARGS__); } while (0)
#endif

const char DataPointBitsSize[] = {
    0,   /* ERROR CODE */
    1,   /* 1.yyy = boolean, like switching, move up/down, step */
    2,   /* 2.yyy = 2 x boolean, e.g. switching + priority control */
    4,   /* 3.yyy = boolean + 3-bit unsigned value, e.g. dimming up/down */
    8,   /* 4.yyy = character (8-bit) */
    8,   /* 5.yyy = 8-bit unsigned value, like dim value (0..100%), blinds position (0..100%) */
    8,   /* 6.yyy = 8-bit 2's complement, e.g. % */
    16,  /* 7.yyy = 2 x 8-bit unsigned value, i.e. pulse counter */
    16,  /* 8.yyy = 2 x 8-bit 2's complement, e.g. % */
    16,  /* 9.yyy = 16-bit float, e.g. temperature */
    24,  /* 10.yyy = time */
    24,  /* 11.yyy = date */
    32,  /* 12.yyy = 4 x 8-bit unsigned value, i.e. pulse counter */
    32,  /* 13.yyy = 4 x 8-bit 2's complement, i.e. pulse counter */
    32,  /* 14.yyy = 32-bit float, e.g. temperature */
    8,   /* 15.yyy = access control */
    112, /* 16.yyy = string -> 14 characters (14 x 8-bit) */
    8,   /* 17.yyy = scene number */
    8,   /* 18.yyy = scene control */
    64,  /* 19.yyy = time + data */
    8,   /* 20.yyy = 8-bit enumeration, e.g. HVAC mode ('auto', 'comfort', 'standby', 'economy', 'protection') */
    8,   /* 21.yyy = General Status */
    16,  /* 22.yyy = 16 Bit set */
    16   /* 23.yyy = Enum8 (8 * 2bit enum {0..3} */
};


#if defined(KNX_ENABLE_DPT_9)
void set_float(void *datapoint, void *value)
{
    KNX_DPT_9 *dpt = (KNX_DPT_9*)datapoint;
    debug_printf("(->DPT_9");
    float *val = (float*)value;

    if(dpt->data != *val)
    {
        dpt->data = *val;
        if(dpt->flag & (KNX_FLAG_T | KNX_FLAG_C))
        {
            /* Send Write telegram */
            uint8_t sign;
            uint8_t exp;
            uint16_t raw;
            int mant;
            sign = 0;
            exp = 0;
            if ((*val) < 0)
                sign = 1;
            mant = ((*val) * 100);
            while(mant > 2047 || mant < -2048)
            {
                mant = mant >> 1;
                ++exp;
            }
            raw = (sign << 15) | (exp << 11) | (mant & 0x07ff);
            dpt->telegram[TELEGRAM_PAYLOAD] = raw >> 8;
            dpt->telegram[TELEGRAM_PAYLOAD + 1] = raw & 0xFF;
            dpt->telegram[TELEGRAM_COMMAND_H] = 0;
            dpt->telegram[TELEGRAM_COMMAND_L] = KNX_COMMAND_WRITE;

            /* Calculate Checksum */
            dpt->telegram[TELEGRAM_DPT_9_CRC] = dpt->telegram[TELEGRAM_CTRL];
            for(int i = 1; i < 10; i++)
            {
                dpt->telegram[TELEGRAM_DPT_9_CRC] ^= dpt->telegram[i];
            }
            dpt->telegram[TELEGRAM_DPT_9_CRC] = ~dpt->telegram[TELEGRAM_DPT_9_CRC];
#ifdef DEBUG_KNXDATAPOINT
            printf(" >");
            for(int i = 0; i <= TELEGRAM_DPT_9_CRC; i++)
            {
                printf("%02x ", dpt->telegram[i]);
            }
            printf("->KNX\r\n");
#endif
            knx_send_telegram(dpt->telegram, TELEGRAM_DPT_9_LEN);
       }
    }
    debug_printf(")\r\n");
}
#endif
