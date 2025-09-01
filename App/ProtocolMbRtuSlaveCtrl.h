#ifndef __ProtocolMbRtuSlaveCtrl__H__
#define __ProtocolMbRtuSlaveCtrl__H__

#include "BSP.h"
#include "ModbusSS.h"



typedef enum{
    protocol_cmd_debug = 1,
    protocol_cmd_save_param,
    protocol_cmd_reset,
    protocol_cmd_pwmOuts123,
    protocol_cmd_pwmOuts456,
}protocolMbRtuSlaveCtrl_control_typedef;

typedef struct
{
    protocolMbRtuSlaveCtrl_control_typedef cmd;
    uint16_t param;
} protocolMbRtuSlaveCtrl_typedef;



void protocolMbRtuSlaveCtrl_init(uint8_t portNo);


void protocolMbRtuSlaveCtrl_update_tables();


void protocolMbRtuSlaveCtrl_callback_H_READ(ModbusSS_table_t* table, uint16_t reg, uint16_t quantity);
void protocolMbRtuSlaveCtrl_callback_H_WRITE(ModbusSS_table_t* table, uint16_t reg, uint16_t quantity);







#endif