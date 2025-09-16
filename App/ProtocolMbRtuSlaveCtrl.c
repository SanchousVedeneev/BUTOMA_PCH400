
#include "ProtocolMbRtuSlaveCtrl.h"
#include "Program.h"

uint8_t modbusBufRxTxRtu485[MODBUS_SS_BUF_CNT];
//--------------------  PROTOCOL ---------------------//

//---1000
#define MDB_TABLE_BSP_REG_NO (1000)
enum mdb_table_bsp
{
  tab_bsp_din = MDB_TABLE_BSP_REG_NO,
  tab_bsp_dout_led_w1,
  tab_bsp_dout_led_w2,
  tab_bsp_dout,
  tab_bsp_analog = MDB_TABLE_BSP_REG_NO + 5, // ..1016
  tab_bsp_temp_1 = MDB_TABLE_BSP_REG_NO + 17,
  tab_bsp_temp_2
};
#define MDB_BSP_BUF_COUNT (tab_bsp_temp_2 - MDB_TABLE_BSP_REG_NO + 1)
uint16_t mdb_bsp_buf[MDB_BSP_BUF_COUNT];
ModbusSS_table_t mdb_table_bsp = 
{
    .buf = (uint8_t *)mdb_bsp_buf,
    .quantity = MDB_BSP_BUF_COUNT,
    .regNo = MDB_TABLE_BSP_REG_NO,
    .type = ModbusSS_Holding
};

//---1200
#define MDB_TABLE_PROGRAM_REG_NO (1200)
enum mdb_table_program
{
  tab_prg_cmd = MDB_TABLE_PROGRAM_REG_NO,
  tab_prg_param,
  tab_prg_step,
  tab_prg_target,
  tab_prg_ecode,
  tab_prg_flashCounter,
  tab_prg_prg_analog_AI0,
  tab_prg_prg_analog_AI1,
  tab_prg_prg_analog_AI2,
  tab_prg_prg_analog_AI3,
  tab_prg_prg_analog_AI4,
  tab_prg_prg_analog_AI5,
  tab_prg_prg_analog_AI6,
  tab_prg_prg_analog_AI7,
  tab_prg_prg_analog_AI8,
  tab_prg_prg_analog_AI9,
  tab_prg_prg_analog_AI10,
  tab_prg_k_modIn,
  tab_prg_сhoice_kPWM_or_kMod,
  tab_prg_kPWM_0,
  tab_prg_kPWM_1,
  tab_prg_kPWM_2,
  tab_prg_kPWM_3,
  tab_prg_kPWM_4,
  tab_prg_kPWM_5
};
#define MDB_PROGRAM_BUF_COUNT (tab_prg_kPWM_5 - MDB_TABLE_PROGRAM_REG_NO + 1)
uint16_t mdb_program_buf[MDB_PROGRAM_BUF_COUNT];
ModbusSS_table_t mdb_table_program = 
{
    .buf = (uint8_t *)mdb_program_buf,
    .quantity = MDB_PROGRAM_BUF_COUNT,
    .regNo = MDB_TABLE_PROGRAM_REG_NO,
    .type = ModbusSS_Holding
};

//---1300
#define MDB_TABLE_SETUP_REG_NO (1300)
enum mdb_table_setup
{
  tab_setup_analog_shift_0     = MDB_TABLE_SETUP_REG_NO,   // 1300
  tab_setup_analog_shift_10    = 1310,   // 1310
  tab_setup_analog_kMul_0      = 1311,   // 1311
  tab_setup_analog_kMul_10     = 1321,   // 1321
  tab_setup_analog_av_order_0  = 1322,   // 1322
  tab_setup_analog_av_order_10 = 1332,   // 1332
  tab_setup_analog_filter_N_0  = 1333,   // 1333
  tab_setup_analog_filter_N_10 = 1343,   // 1343
  tab_setup_protect,                     // 1344
  tab_setup_phase_count,                 // 1345
  tab_setup_f_out,                       // 1346
  tab_setup_U_out,                       // 1347
  tab_setup_PWM_freq,                    // 1348
  tab_setup_RegU_kp_phase_0    = 1349,   // 1349
  tab_setup_RegU_kp_phase_2    = 1351,   // 1351
  tab_setup_RegU_ki_phase_0    = 1352,   // 1352
  tab_setup_RegU_ki_phase_2    = 1354,   // 1354
  tab_setup_RegU_max_phase_0   = 1355,   // 1355
  tab_setup_RegU_max_phase_2   = 1357    // 1357
};
#define MDB_SETUP_BUF_COUNT (tab_setup_RegU_max_phase_2 - MDB_TABLE_SETUP_REG_NO + 1)
uint16_t mdb_setup_buf[MDB_SETUP_BUF_COUNT];
ModbusSS_table_t mdb_table_setup = 
{
  .buf = (uint8_t *)mdb_setup_buf,
  .quantity = MDB_SETUP_BUF_COUNT,
  .regNo = MDB_TABLE_SETUP_REG_NO,
  .type = ModbusSS_Holding
};

//---1400
#define MDB_TABLE_REGUL_REG_NO (1400)
enum mdb_table_regul
{
  tab_regul_u_in_phase_0 = MDB_TABLE_REGUL_REG_NO,
  tab_regul_u_in_phase_2      = 1402,    // 1402
  tab_regul_u_fb_phase_0      = 1403,    // 1403
  tab_regul_u_fb_phase_2      = 1405,    // 1405
  tab_regul_u_out_phase_0     = 1406,    // 1406  
  tab_regul_u_out_phase_2     = 1408,    // 1408
  tab_regul_u_kP_phase_0      = 1409,    // 1409
  tab_regul_u_kP_phase_2      = 1411,    // 1411
  tab_regul_u_kI_phase_0      = 1412,    // 1412
  tab_regul_u_kI_phase_2      = 1414,    // 1414
  tab_regul_u_outMax_phase_0  = 1415,    // 1415
  tab_regul_u_outMax_phase_2  = 1417,    // 1417
};
#define MDB_REGUL_BUF_COUNT (tab_regul_u_outMax_phase_2 - MDB_TABLE_REGUL_REG_NO + 1)
uint16_t mdb_regul_buf[MDB_REGUL_BUF_COUNT];
ModbusSS_table_t mdb_table_regul = 
{
  .buf = (uint8_t *)mdb_regul_buf,
  .quantity = MDB_REGUL_BUF_COUNT,
  .regNo = MDB_TABLE_REGUL_REG_NO,
  .type = ModbusSS_Holding
};

//--------------------  PROTOCOL END---------------------//

//--------------------  TABLES ARRAY ---------------------//
ModbusSS_table_t *modbusTables[] = 
{
  &mdb_table_bsp,
  &mdb_table_program,
  &mdb_table_setup,
  &mdb_table_regul
};
//--------------------  TABLES ARRAY END---------------------//

//--------------------  MODBUS STRUCT ---------------------//
ModbusSS_t modbusSS_rtu_rs485 = 
{
  .cbHoldingUpdate = protocolMbRtuSlaveCtrl_callback_H_WRITE,
  .cbHoldingRequest = NULL, // protocolMbRtuSlaveCtrl_callback_H_REQ, //modbusHoldingReq,
  .rtuTcp = MODBUS_SS_RTU,
  .bufRxTx = modbusBufRxTxRtu485,
  .slaveId = 1,
  .tables = modbusTables,
  .tablesCount = 4
};

protocolMbRtuSlaveCtrl_typedef modbusRtu_ctrlStruct; // protocol control struct
//--------------------  MODBUS STRUCT END---------------------//

//------------------------ EXTERN ------------------------
extern bsp_dInOut_typedef bsp_dInOut_struct;
extern bsp_analogIn_typedef bsp_analogIn_struct;
extern Program_typedef programStruct;
//---------------------- EXTERN END-----------------------

//------------------------ REGULAR FCN ------------------------
void protocolMbRtuSlaveCtrl_init(uint8_t portNo)
{
  HAL_Delay(10);
  bsp_rs485_setPortToModbusRtu(portNo, modbusBufRxTxRtu485, MODBUS_SS_BUF_CNT);
}

__INLINE void protocolMbRtuSlaveCtrl_update_tables()
{
  // BSP -----------------------------
  uint16_t regNo = MDB_TABLE_BSP_REG_NO;
  float kMul_1000 = 1000.0f;

  ModbusSS_SetWord(&mdb_table_bsp, regNo++, bsp_dInOut_struct.in.w16); // 1000
  for (uint8_t i = 0; i < 3; i++)
  {
    ModbusSS_SetWord(&mdb_table_bsp, regNo++, bsp_dInOut_struct.out.w16[i]); // 1001 - 1003
  }
  regNo = MDB_TABLE_BSP_REG_NO + 5; // 1005
  for (uint8_t i = 0; i < 12; i++)
  {
    ModbusSS_SetWord(&mdb_table_bsp, regNo++, bsp_analogIn_struct.rawDataUI[i]); // 1005 - 1016
  }
  ModbusSS_SetWord(&mdb_table_bsp, regNo++, bsp_analogIn_getTemp(1));   // 1017
  ModbusSS_SetWord(&mdb_table_bsp, regNo,   bsp_analogIn_getTemp(2));   // 1018

  // PROGRAM -----------------------------
  // ModbusSS_SetWord(&mdb_table_program, tab_prg_cmd,            modbusRtu_ctrlStruct.cmd);         // 1200
  // ModbusSS_SetWord(&mdb_table_program, tab_prg_param,          modbusRtu_ctrlStruct.param);       // 1201
  ModbusSS_SetWord(&mdb_table_program, tab_prg_target,         programStruct.control.target);     // 1202
  ModbusSS_SetWord(&mdb_table_program, tab_prg_step,           programStruct.control.step);       // 1203
  ModbusSS_SetWord(&mdb_table_program, tab_prg_ecode,          programStruct.control.errorCode);  // 1204
  ModbusSS_SetWord(&mdb_table_program, tab_prg_flashCounter,   programStruct.sys.flash_counter);  // 1205
  
  uint16_t StartIdx = tab_prg_prg_analog_AI0;
  uint16_t StopIdx = tab_prg_prg_analog_AI10;
  int16_t val = 0.0f;
  for (uint8_t i = 0; i < (StopIdx - StartIdx + 1); i++)
  {
      val = Program_analogGetByIdx(i)->value;
      ModbusSS_SetWord(&mdb_table_program, StartIdx + i, val); // 1206
  }
  
  ModbusSS_SetWord(&mdb_table_program, tab_prg_k_modIn,             programStruct.control.remote.k_modIn*kMul_1000);    //1217

  ModbusSS_SetWord(&mdb_table_program, tab_prg_сhoice_kPWM_or_kMod, programStruct.control.remote.choise_kPWM_or_kMod);  //1218

  StartIdx = tab_prg_kPWM_0;
  StopIdx  = tab_prg_kPWM_5;
  for (uint8_t i = 0; i < (StopIdx - StartIdx + 1); i++)
  {
    ModbusSS_SetWord(&mdb_table_program, StartIdx + i,   programStruct.control.remote.kPWM[i]);  //1219
  }

  // SETUP -----------------------------
  StartIdx = tab_setup_analog_shift_0;
  StopIdx  = tab_setup_analog_shift_10;
  for (uint8_t i = 0; i < (StopIdx - StartIdx + 1); i++)
  {
    ModbusSS_SetWord(&mdb_table_setup, StartIdx + i, programStruct.setup.analog_shift[i]);
  }

  StartIdx = tab_setup_analog_kMul_0;
  StopIdx  = tab_setup_analog_kMul_10;
  for (uint8_t i = 0; i < (StopIdx - StartIdx + 1); i++)
  {
    ModbusSS_SetWord(&mdb_table_setup, StartIdx + i, (int16_t)(programStruct.setup.analog_kMul[i]*kMul_1000));
  }

  StartIdx = tab_setup_analog_av_order_0;
  StopIdx  = tab_setup_analog_av_order_10;
  for (uint8_t i = 0; i < (StopIdx - StartIdx + 1); i++)
  {
    ModbusSS_SetWord(&mdb_table_setup, StartIdx + i, programStruct.setup.analog_av_order[i]);
  }

  StartIdx = tab_setup_analog_filter_N_0;
  StopIdx  = tab_setup_analog_filter_N_10;
  for (uint8_t i = 0; i < (StopIdx - StartIdx + 1); i++)
  {
    ModbusSS_SetWord(&mdb_table_setup, StartIdx + i, programStruct.setup.analog_filter_N[i]);
  }

  ModbusSS_SetWord(&mdb_table_setup, tab_setup_protect,       programStruct.setup.protect_control);
  ModbusSS_SetWord(&mdb_table_setup, tab_setup_phase_count,   programStruct.setup.phaseCount);
  ModbusSS_SetWord(&mdb_table_setup, tab_setup_f_out,         programStruct.setup.f_out);
  ModbusSS_SetWord(&mdb_table_setup, tab_setup_U_out,         programStruct.setup.U_out);
  ModbusSS_SetWord(&mdb_table_setup, tab_setup_PWM_freq,      programStruct.setup.PWM_freq);

  StartIdx = tab_setup_RegU_kp_phase_0;
  StopIdx  = tab_setup_RegU_kp_phase_2;
  for (uint8_t i = 0; i < (StopIdx - StartIdx + 1); i++)
  {
    ModbusSS_SetWord(&mdb_table_setup, StartIdx + i,          programStruct.setup.RegU_kp[i]*kMul_1000);
  }

  StartIdx = tab_setup_RegU_ki_phase_0;
  StopIdx  = tab_setup_RegU_ki_phase_2;
  for (uint8_t i = 0; i < (StopIdx - StartIdx + 1); i++)
  {
    ModbusSS_SetWord(&mdb_table_setup, StartIdx + i,          programStruct.setup.RegU_ki[i]*kMul_1000);
  }

  StartIdx = tab_setup_RegU_max_phase_0;
  StopIdx  = tab_setup_RegU_max_phase_2;
  for (uint8_t i = 0; i < (StopIdx - StartIdx + 1); i++)
  {
    ModbusSS_SetWord(&mdb_table_setup, StartIdx + i,          programStruct.setup.RegU_max[i]*kMul_1000);
  }

  // REGUL -----------------------------
  StartIdx = tab_regul_u_in_phase_0;
  StopIdx  = tab_regul_u_in_phase_2;
  for (uint8_t i = 0; i < (StopIdx - StartIdx + 1); i++)
  {
    ModbusSS_SetWord(&mdb_table_regul, StartIdx + i,     programStruct.control.sau.voltageRegulator[i].In);
  }

  StartIdx = tab_regul_u_fb_phase_0;
  StopIdx  = tab_regul_u_fb_phase_2;
  for (uint8_t i = 0; i < (StopIdx - StartIdx + 1); i++)
  {
    ModbusSS_SetWord(&mdb_table_regul, StartIdx + i,     programStruct.control.sau.voltageRegulator[i].Fb);
  }

  StartIdx = tab_regul_u_out_phase_0;
  StopIdx  = tab_regul_u_out_phase_2;
  for (uint8_t i = 0; i < (StopIdx - StartIdx + 1); i++)
  {
    ModbusSS_SetWord(&mdb_table_regul, StartIdx + i,     programStruct.control.sau.voltageRegulator[i].Out);
  }

  StartIdx = tab_regul_u_kP_phase_0;
  StopIdx  = tab_regul_u_kP_phase_2;
  for (uint8_t i = 0; i < (StopIdx - StartIdx + 1); i++)
  {
    ModbusSS_SetWord(&mdb_table_regul, StartIdx + i,     programStruct.control.sau.voltageRegulator[i].k_P*kMul_1000);
  }

  StartIdx = tab_regul_u_kI_phase_0;
  StopIdx  = tab_regul_u_kI_phase_2;
  for (uint8_t i = 0; i < (StopIdx - StartIdx + 1); i++)
  {
    ModbusSS_SetWord(&mdb_table_regul, StartIdx + i,     programStruct.control.sau.voltageRegulator[i].k_Int*kMul_1000);
  }

  StartIdx = tab_regul_u_outMax_phase_0;
  StopIdx  = tab_regul_u_outMax_phase_2;
  for (uint8_t i = 0; i < (StopIdx - StartIdx + 1); i++)
  {
    ModbusSS_SetWord(&mdb_table_regul, StartIdx + i,     programStruct.control.sau.voltageRegulator[i].OutMax*kMul_1000);
  }
}
//------------------------ REGULAR FCN END------------------------

//------------------------------- MODBUS CALLBACKS -------------------------------------------//
#define PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_OK (0xAAAA)
#define PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_FAIL (0xF00F)
__weak void protocolMbRtuSlaveCtrl_callback_H_WRITE(ModbusSS_table_t *table, uint16_t reg, uint16_t quantity)
{
  uint16_t response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_FAIL;
  uint16_t param = 0;
  uint16_t value = 0;
  int16_t sign_val = 0;
  uint8_t idx = 0;
  float kMul_0_001 = 0.001f;
  asm("NOP");

  if (table == &mdb_table_program) // Диапазон PROGRAM
  {
    switch (reg)
    {
    case tab_prg_cmd:
      modbusRtu_ctrlStruct.cmd = ModbusSS_GetWord(&mdb_table_program, reg);
      switch (modbusRtu_ctrlStruct.cmd)
      {
      case protocol_cmd_debug:
        /* code */
        if (Program_GoDebug())
        {
          response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_OK;
        }
        break;
      case protocol_cmd_save_param:
        if (Program_ParamSave())
        {
          response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_OK;
        }
        break;
      case protocol_cmd_reset:
        if (Program_GoReset())
        {
          response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_OK;
        }
        break;
      case protocol_cmd_pwmOuts123:
      case protocol_cmd_pwmOuts456:
        param = ModbusSS_GetWord(&mdb_table_program, tab_prg_param);
        if (Program_set_pwmOuts_debug(modbusRtu_ctrlStruct.cmd - protocol_cmd_pwmOuts123, param))
        {
          response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_OK;
        }
        break;
      case protocol_cmd_param_set_defolt:
        Program_ParamSetToDefault();
        break;
      default:
        response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_FAIL;
        break;
      }
      break;
    case tab_prg_prg_analog_AI0 ... tab_prg_prg_analog_AI10:
      idx = reg - tab_prg_prg_analog_AI0;
      float value = ModbusSS_GetWord(&mdb_table_program, reg);
      if (value == 0.0f)
      {
        if (Program_analogSetZero(idx))
        {
          response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_OK;
        }
      }
      else
      {
        if (Program_analogCalibKMul(idx, value))
        {
          response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_OK;
        }
      }
      break;
    case tab_prg_k_modIn:
      Program_set_k_mod_debug(ModbusSS_GetWord(&mdb_table_program, reg));
      break;
    case tab_prg_сhoice_kPWM_or_kMod:
      if (Program_сhoice_kPWM_or_kMod_debug(ModbusSS_GetWord(&mdb_table_program, reg)))
      {
        response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_OK;
      }
      break;
    case tab_prg_kPWM_0 ... tab_prg_kPWM_5:
      idx = reg - tab_prg_kPWM_0;
      if (Program_set_pwm_debug(idx, ModbusSS_GetWord(&mdb_table_program, reg)))
      {
        response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_OK;
      }
      break;
    default:
      response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_FAIL;
      break;
    }
  }
  else if (table == &mdb_table_bsp) // Диапазон BSP
  {
    switch (reg)
    {
    case tab_bsp_dout:
      if (Program_set_dout_debug(ModbusSS_GetWord(&mdb_table_bsp, reg)))
      {
        response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_OK;
      }
      break;
    default:
      response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_FAIL;
      break;
    }
  }
  else if (table == &mdb_table_setup)
  {
    value = ModbusSS_GetWord(&mdb_table_setup, reg);
        switch (reg)
    {
    case tab_setup_analog_shift_0 ... tab_setup_analog_shift_10:
      idx = reg - tab_setup_analog_shift_0;
      if  (Program_analogSetShift(idx, value))
      {
        response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_OK;
      }
      break;
    case tab_setup_analog_kMul_0 ... tab_setup_analog_kMul_10:
      idx = reg - tab_setup_analog_kMul_0;
      sign_val = value;
      if (Program_analogSetKMul(idx, sign_val * kMul_0_001))
      {
        response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_OK;
      }
      break;
    case tab_setup_analog_av_order_0 ... tab_setup_analog_av_order_10:
      idx = reg - tab_setup_analog_av_order_0;
      if (Program_analogSetAvOrder(idx, value))
      {
        response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_OK;
      }
      break;
    case tab_setup_analog_filter_N_0 ... tab_setup_analog_filter_N_10:
      idx = reg - tab_setup_analog_filter_N_0;
      if (Program_analogSetFilterN(idx, value))
      {
        response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_OK;
      }
      break;
    case tab_setup_protect:
      programStruct.setup.protect_control = value;
      response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_OK;
      break;
    case tab_setup_phase_count:
      Program_set_phaseCount_debug(value);
      response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_OK;
      break;
    case tab_setup_f_out:
      Program_set_fOut_debug(value);
      response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_OK;
      break;
    case tab_setup_U_out:
      Program_set_uOut_debug(value);
      response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_OK;
      break;
    case tab_setup_PWM_freq:
      Program_set_PWM_freq_debug(value);
      response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_OK;
      break;
    case tab_setup_RegU_kp_phase_0 ... tab_setup_RegU_kp_phase_2:
      idx = reg - tab_setup_RegU_kp_phase_0;
      Program_set_regul_kp(idx, value*kMul_0_001);
      response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_OK;
      break;
    case tab_setup_RegU_ki_phase_0 ... tab_setup_RegU_ki_phase_2:
      idx = reg - tab_setup_RegU_ki_phase_0;
      Program_set_regul_ki(idx, value*kMul_0_001);
      response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_OK;
      break;
    case tab_setup_RegU_max_phase_0 ... tab_setup_RegU_max_phase_2:
      idx = reg - tab_setup_RegU_max_phase_0;
      Program_set_regul_uOut_max(idx, value*kMul_0_001);
      response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_OK;
      break;
    default:
      response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_FAIL;
      break;
    }
  }
  else // Вне диапазона
  {
    response = PROTOCOL_MB_RTU_SLAVE_CTRL_CMD_FAIL;
  }

  ModbusSS_SetWord(&mdb_table_program, tab_prg_cmd, response);
}

__weak void protocolMbRtuSlaveCtrl_callback_H_READ(ModbusSS_table_t *table, uint16_t reg, uint16_t quantity)
{

  asm("NOP");
  //             if (table == &modbusTableHolding1)
  // {
  //   for (int r = reg; r < reg + quantity; r++)
  //   {
  //     asm("NOP");
  //     switch (r)
  //     {
  //     case MBP_AI1_X:
  //       ModbusSS_SetWord(table, r, bsp_ai_read_cache(BSP_AI1));
  //       asm("NOP");
  //       break;
  //     case MBP_AI2_Y:
  //       ModbusSS_SetWord(table, r, bsp_ai_read_cache(BSP_AI2));
  //       asm("NOP");
  //       break;
  //     case MBP_DI_STATE:
  //       ModbusSS_SetWord(table, r, bsp_di_get_cache_pack16());
  //       asm("NOP");
  //       break;
  //     default:
  //       break;
  //     }
  //   }
  // }
}
//------------------------------- MODBUS CALLBACKS END-------------------------------------------//

//------------------------------- HW CALLBACK -------------------------------------------//
void bsp_rs485_callback_rxBlockReady(uint8_t portNo)
{

  int32_t blockSizeByte = 0;
  if ((blockSizeByte = ModbusSS_ParseRxData(&modbusSS_rtu_rs485)) == 0)
  {
    // bug with reset modbus!!!
    asm("NOP");
  }
  else if (blockSizeByte != -1)
  {
    asm("NOP");
    bsp_rs485_sendBlock(portNo, modbusSS_rtu_rs485.bufRxTx, blockSizeByte);
    asm("NOP");
  }
}
//------------------------------- HW CALLBACK END-------------------------------------------//




