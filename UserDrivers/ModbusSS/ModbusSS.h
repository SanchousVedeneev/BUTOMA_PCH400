#ifndef __MODBUS_SS__H__
#define __MODBUS_SS__H__


#include "main.h"

#define MODBUS_SS_BUF_CNT (260)
//#define MODBUS_SS_STATIC

typedef enum{
    ModbusSS_Holding,
    ModbusSS_Input,
    ModbusSS_Coils,
    ModbusSS_DI
}ModbusSS_table_type_t;

typedef struct ModbusSS_table
{
    uint16_t regNo;
    uint16_t quantity;
    ModbusSS_table_type_t type;
    uint8_t* buf;
}ModbusSS_table_t;


#define MODBUS_SS_RTU (0)
#define MODBUS_SS_TCP (1)

typedef struct ModbusSS_struct
{
    //void (*fpTxFunc)(uint8_t *, uint8_t);
    void (*cbNewRequest)(void);
    void (*cbHoldingUpdate)(ModbusSS_table_t* table, uint16_t reg, uint16_t quantity);
    void (*cbHoldingRequest)(ModbusSS_table_t* table, uint16_t reg, uint16_t quantity);
    uint8_t rtuTcp;
#ifdef MODBUS_SS_STATIC
    uint8_t bufRxTx[MODBUS_SS_BUF_CNT];
#else
    uint8_t* bufRxTx;
#endif
    ModbusSS_table_t** tables;
    uint8_t tablesCount;
    uint8_t slaveId;
} ModbusSS_t;

int32_t ModbusSS_ParseRxData(ModbusSS_t *mbCore);

uint16_t ModbusSS_GetWord(ModbusSS_table_t* table, uint16_t regNo);
void ModbusSS_SetWord(ModbusSS_table_t* table, uint16_t regNo, uint16_t value);




#endif


