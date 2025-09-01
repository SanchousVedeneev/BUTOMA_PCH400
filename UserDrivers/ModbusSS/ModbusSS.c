
#include "ModbusSS.h"

__STATIC_INLINE uint16_t _Crc16(uint8_t *pcBlock, uint16_t len);
__STATIC_INLINE ModbusSS_table_t *getTable(ModbusSS_t *core, uint8_t f, uint16_t r, uint16_t q);

#define GET_DATA(buf, idx) *(buf + idx)
#define WRITE_DATA(buf, idx, value) (*(buf + idx) = value)
#define GET_WORD(buf, idx) (((uint16_t) * (buf + idx + 1)) | (((uint16_t) * (buf + idx)) << 8))
#define WRITE_WORD(buf, idx, word) (*((uint16_t *)(buf) + idx) = (((word) >> 8) | (word) << 8))

#define MB_ECODE_IN_RANGE (2)
#define MODBUS_TCP_BUF_OFFSET (6)
#define MB_POS_SLAVE_ID (0)
#define MB_POS_FUNC (1)
#define MB_POS_REG (2)
#define MB_POS_ECODE (2)
#define MB_POS_BYTE_CNT (2)
#define MB_POS_DATA (3)
#define MB_POS_QUANTITY (4)

#define MB_ERR_FUNC  (0x01)
#define MB_ERR_RANGE (0x02)

int32_t ModbusSS_ParseRxData(ModbusSS_t *mbCore)
{
    int32_t retLenError = -1;
#ifdef MODBUS_SS_STATIC
    if (mbCore == NULL)
    {
        return retLenError;
    }
#else
    if ((mbCore == NULL) || mbCore->bufRxTx == NULL)
    {
        return retLenError;
    }
#endif
    uint8_t* buf = mbCore->bufRxTx;
    if (mbCore->rtuTcp == MODBUS_SS_TCP)
    {
        buf += MODBUS_TCP_BUF_OFFSET;
    }
    // Slave Id check
    if (mbCore->slaveId != GET_DATA(buf, MB_POS_SLAVE_ID))
    {
        return retLenError;
    }

    // get func
    uint8_t func = GET_DATA(buf, MB_POS_FUNC);

    // get req len
    uint16_t reqLen = 0;
    if (func >= 0x01 && func <= 0x06)
        reqLen = 6;
    else if (func == 0x0F || func == 0x10)
    {
        reqLen = 7 + *(buf + 6);
    }
    else
    {
        return retLenError;
    }

    // check Crc
    if (mbCore->rtuTcp == MODBUS_SS_RTU)
    {
        uint16_t readCrc = GET_DATA(buf, reqLen) | (GET_DATA(buf, reqLen+1)<<8);
        uint16_t getCrc = _Crc16(buf, reqLen);
        if (readCrc != getCrc)
        {
            return retLenError;
        }
    }

    // cb new req
    if (mbCore->cbNewRequest != NULL)
        mbCore->cbNewRequest();

    switch (func)
    {
    case 0x03:
    {
        uint16_t r = GET_WORD(buf, MB_POS_REG);
        uint16_t q = GET_WORD(buf, MB_POS_QUANTITY);
        ModbusSS_table_t *table = getTable(mbCore, func, r, q);
        if (table == NULL)
        {
            WRITE_WORD(buf + 1, 0, (func | 0x80)<<8 | MB_ERR_RANGE);
            retLenError = 3;
        }
        else
        {
            *(buf + MB_POS_BYTE_CNT) = (uint8_t)q * 2;
            if (mbCore->cbHoldingRequest != NULL)
                mbCore->cbHoldingRequest(table, r, q);
            for (int i = 0; i < q; i++)
            {
                WRITE_WORD(buf + 3, i, ModbusSS_GetWord(table, r + i));
            }
            retLenError = q * 2 + 3;
        }
    }
    break;
    case 0x06:
    {
        uint16_t r = GET_WORD(buf, MB_POS_REG);
        ModbusSS_table_t *table = getTable(mbCore, func, r, 1);
        if (table == NULL)
        {
            WRITE_WORD(buf, MB_POS_FUNC, func | 0x80 | MB_ERR_RANGE << 8);
            retLenError = 3;
        }
        else
        {
            asm("NOP");
            ModbusSS_SetWord(table,r,GET_WORD(buf,4));
            if (mbCore->cbHoldingUpdate != NULL)
                mbCore->cbHoldingUpdate(table, r, 1);
            retLenError = reqLen;
            asm("NOP");
            // if (mbCore->rtuTcp == MODBUS_SS_RTU){


            // }
        }
    }
    break;
    default:
        WRITE_WORD(buf, MB_POS_FUNC, func | 0x80 | MB_ERR_FUNC << 8);
        retLenError = 3;
        break;
    }

    if (mbCore->rtuTcp == MODBUS_SS_TCP)
    {
        WRITE_WORD(buf, -1, retLenError);
        retLenError += MODBUS_TCP_BUF_OFFSET;
    }
    else
    {
        uint16_t crc = _Crc16(buf, retLenError);
        WRITE_DATA(buf,retLenError,(uint8_t)crc);
        WRITE_DATA(buf,retLenError+1,(uint8_t)(crc>>8));
        retLenError += 2;
        asm("NOP");
        //mbCore->fpTxFunc(mbCore->bufRxTx,retLenError);
    }
    return retLenError;
}

__INLINE uint16_t ModbusSS_GetWord(ModbusSS_table_t *table, uint16_t regNo)
{
    uint16_t idx = regNo - table->regNo;
    return *((uint16_t*)table->buf + idx);
}

__INLINE void ModbusSS_SetWord(ModbusSS_table_t *table, uint16_t regNo, uint16_t value)
{
    // if(regNo < table->regNo || (regNo >= table->regNo + table->quantity)){
    //     return 0;
    // }
    *((uint16_t*)table->buf + regNo - table->regNo) = value;
}

__STATIC_INLINE ModbusSS_table_t *getTable(ModbusSS_t *core, uint8_t f, uint16_t r, uint16_t q)
{

    for (int i = 0; i < core->tablesCount; i++)
    {
        ModbusSS_table_t *table = core->tables[i];
        if (table == NULL)
            return NULL;
        if (r < table->regNo || r + q - 1 > table->regNo + table->quantity - 1)
        {
            continue;
        }
        switch (table->type)
        {
        case ModbusSS_Holding:
            if((f!=0x03)&&(f!=0x06)&&(f!=0x10)) continue;
            break;
        case ModbusSS_Input:
            if(f!=0x04) continue;
            break;
        case ModbusSS_Coils:
            if((f!=0x01)&&(f!=0x05)&&(f!=0x0F)) continue;
            break;
        case ModbusSS_DI:
            if(f!=0x02) continue;
            break;
        default:
            continue;
            break;
        }

        return table;
    }
    return NULL;
}

__STATIC_INLINE uint16_t _Crc16(uint8_t *pcBlock, uint16_t len)
{
    uint16_t w = 0;
    uint8_t shift_cnt;

    if (pcBlock)
    {
        w = 0xffffU;
        for (; len > 0; len--)
        {
            w = (uint16_t)((w / 256U) * 256U + ((w % 256U) ^ (*pcBlock++)));
            for (shift_cnt = 0; shift_cnt < 8; shift_cnt++)
            {
                if ((w & 0x1) == 1)
                    w = (uint16_t)((w >> 1) ^ 0xa001U);
                else
                    w >>= 1;
            }
        }
    }
    return w;
}