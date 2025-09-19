#include "Program.h"
#include "ProtocolMbRtuSlaveCtrl.h"
#include "FlashWorker.h"

Program_typedef program;

/*----------------------------- PRIVATE FCN MACRO DCL ---------------------------------*/
__STATIC_INLINE uint8_t Program_analogInit();
__STATIC_INLINE void Program_pwmInit();
__STATIC_INLINE void Program_pwmOutsControl(bsp_pwm_outs_group_typedef group, uint8_t enable);
__STATIC_INLINE void Program_regulatorInit();
__STATIC_INLINE void Program_setDout(Program_dout_typedef dout);
__STATIC_INLINE void Program_resetDout(Program_dout_typedef dout);
__STATIC_INLINE uint8_t Program_checkDin(Program_din_typedef din);
__STATIC_INLINE void Program_fastStop();
__STATIC_INLINE uint8_t Program_setError(Program_ERROR_typedef error);

__STATIC_INLINE void setPhasePWM(Program_PHASE_typedef *phase, float value);
__STATIC_INLINE float saturate(float in, float min, float max);

#define SET_PWM(IDX, VALUE_1000)  bsp_pwm_set_ccrPercentX10((IDX), (VALUE_1000));
/*----------------------------- PRIVATE FCN MACRO END ---------------------------------*/

// --------------------- EXTERN ---------------------//
extern bsp_analogIn_typedef bsp_analogIn_struct;
// --------------------- EXTERN END---------------------//

/*----------------------------- STEPS ---------------------------------*/
#define PROGRAM_PWM_ON  (1)
#define PROGRAM_PWM_OFF (0)

#define PROGRAM_SETUP_VOLTAGE_OK_ERR (1.0f)
#define PROGRAM_I_AC_K_OVERLOAD      (1.3f)
#define PROGRAM_TIMER_I_AC_OVERLOAD  (10)

#define PROGRAM_TIMER_CHECK_UDC (1000)
#define PROGRAM_TIMER_VOLTAGE_UP (1000)
#define PROGRAM_TIMER_WAIT_AC_OK (3000)
__STATIC_INLINE void __stepCheckUdc()
{
    static uint16_t timerCheckUdc = 0;


    // Button Stop
    if (Program_checkDin(prg_din2_Stop) == PRG_DIN_STOP_VAL)
    {
        Program_switchTarget(target_waitOp);
    }

    // Check error
    if (program.analog.aIn[prg_analog_Udc_invL].value < program.setup.Udc_low)
    {
        Program_setError(error_Udc_L_low);
    }
    if (program.analog.aIn[prg_analog_Udc_invL].value > program.setup.Udc_high)
    {
        Program_setError(error_Udc_L_high);
    }
    if (program.setup.phaseCount > 1)
    {
        if (program.analog.aIn[prg_analog_Udc_invR].value < program.setup.Udc_low)
        {
            Program_setError(error_Udc_R_low);  
        }
        else if (program.analog.aIn[prg_analog_Udc_invR].value > program.setup.Udc_high)
        {
            Program_setError(error_Udc_R_high); 
        }
    } 

    // Check target and program timer
    if (program.control.target == target_Work) 
    {
        if (timerCheckUdc++ < PROGRAM_TIMER_CHECK_UDC)
        {
            return;
        }
        else
        {
            // set zero pwm, regulators and intensSetter
            for (uint8_t phase = 0; phase < program.setup.phaseCount; phase++)
            {
                program.phase[phase].k_modOut = 0.0f;
                dsp_regulatorReset(&program.control.sau.voltageRegulator[phase]);
            }
            dsp_intensSetterReset(&program.control.sau.ZI);

            Program_setDout(prg_dout5_KM1);
            Program_pwmOutsControl(bsp_pwm_outs_group_123, PROGRAM_PWM_ON);
            Program_pwmOutsControl(bsp_pwm_outs_group_456, PROGRAM_PWM_ON);
        }
    }
    timerCheckUdc = 0;

    // Switch
    switch (program.control.target)
    {
    case target_Work:
        program.control.step = step_Voltage_Up; 
        break;    
    case target_waitOp:
        program.control.step = step_Stop; 
        break;
    case target_error:
        program.control.step = step_error;
        break;
    default:
        break;
    }
}
__STATIC_INLINE void __stepVoltageUp()
{
    static uint16_t timer_Iac_overload[PROGRAM_FHASE_COUNT] = {[0 ... 2] = 0};
    static uint16_t timerVoltageUp = 0;
    static uint16_t timerWaitAcOk[PROGRAM_FHASE_COUNT] = {[0 ... 2] = 0};

    float voltage_in = 0.0f;
    uint8_t uFlag[3] = {[0 ... 2] = RESET};

    // Button Stop
    if (Program_checkDin(prg_din2_Stop) == PRG_DIN_STOP_VAL)
    {
        Program_switchTarget(target_waitOp);
    }

    // Check error
    if (program.analog.aIn[prg_analog_Udc_invL].value < program.setup.Udc_low)
    {
        Program_setError(error_Udc_L_low);
    }
    if (program.analog.aIn[prg_analog_Udc_invL].value > program.setup.Udc_high)
    {
        Program_setError(error_Udc_L_high);
    }
    if (program.setup.phaseCount > 1)
    {
        if (program.analog.aIn[prg_analog_Udc_invR].value < program.setup.Udc_low)
        {
            Program_setError(error_Udc_R_low);
        }
        if (program.analog.aIn[prg_analog_Udc_invR].value > program.setup.Udc_high)
        {
            Program_setError(error_Udc_R_high);
        }
    }
    
    if ((program.analog.aIn[prg_analog_I_u].value >= program.setup.Iac_nominal) &&
        (program.analog.aIn[prg_analog_I_u].value <= program.setup.Iac_nominal*PROGRAM_I_AC_K_OVERLOAD))
    {
        if (timer_Iac_overload[PROGRAM_FHASE_U]++ > PROGRAM_TIMER_I_AC_OVERLOAD)
        {
            Program_setError(error_Iac_U_overload);
        } 
    }
    else
    {
        timer_Iac_overload[PROGRAM_FHASE_U] = 0;
    }

    if ((program.analog.aIn[prg_analog_I_v].value >= program.setup.Iac_nominal) &&
        (program.analog.aIn[prg_analog_I_v].value <= program.setup.Iac_nominal*PROGRAM_I_AC_K_OVERLOAD))
    {
        if (timer_Iac_overload[PROGRAM_FHASE_V]++ > PROGRAM_TIMER_I_AC_OVERLOAD)
        {
            Program_setError(error_Iac_V_overload);
        } 
    }
    else
    {
        timer_Iac_overload[PROGRAM_FHASE_V] = 0;
    }

    if ((program.analog.aIn[prg_analog_I_w].value >= program.setup.Iac_nominal) &&
        (program.analog.aIn[prg_analog_I_w].value <= program.setup.Iac_nominal*PROGRAM_I_AC_K_OVERLOAD))
    {
        if (timer_Iac_overload[PROGRAM_FHASE_W]++ > PROGRAM_TIMER_I_AC_OVERLOAD)
        {
            Program_setError(error_Iac_W_overload);
        } 
    }
    else
    {
        timer_Iac_overload[PROGRAM_FHASE_W] = 0;
    }

    if (program.analog.aIn[prg_analog_I_u].value > program.setup.Iac_nominal*PROGRAM_I_AC_K_OVERLOAD)
    {
        Program_setError(error_Iac_U_KZ);
    }
    if (program.analog.aIn[prg_analog_I_v].value > program.setup.Iac_nominal*PROGRAM_I_AC_K_OVERLOAD)
    {
        Program_setError(error_Iac_V_KZ);
    }
    if (program.analog.aIn[prg_analog_I_w].value > program.setup.Iac_nominal*PROGRAM_I_AC_K_OVERLOAD)
    {
        Program_setError(error_Iac_W_KZ);
    }

    if ((bsp_analogIn_struct.currentTemp[0] > program.setup.Tradiator_high) ||
        (bsp_analogIn_struct.currentTemp[1] > program.setup.Tradiator_high))
    {
        Program_setError(error_Tradiator_high);
    }
    
    // VoltageUp process
    if (program.control.target == target_Work)
    {
        program.control.sau.ZI.in = program.setup.U_out;
        dsp_intensSetterUpProcess(&program.control.sau.ZI);
        voltage_in = program.control.sau.ZI.out;

        for (uint8_t phase = 0; phase < program.setup.phaseCount; phase++)
        {
            program.control.sau.voltageRegulator[phase].In = voltage_in;
            program.control.sau.voltageRegulator[phase].Fb = *program.phase[phase].voltageFb;
            program.phase[phase].k_modOut = dsp_regulatorProcess(&program.control.sau.voltageRegulator[phase]);
            if (voltage_in == program.setup.U_out) // задатчик вышел
            {
                if (program.control.sau.voltageRegulator[phase].d < PROGRAM_SETUP_VOLTAGE_OK_ERR)
                {
                    uFlag[phase] = SET;
                    if ((uFlag[0] + uFlag[1] + uFlag[2]) == program.setup.phaseCount)
                    {
                        if (timerVoltageUp++ >= PROGRAM_TIMER_VOLTAGE_UP)
                        {
                            Program_setDout(prg_dout6_KM2);
                            Program_setDout(prg_dout2_LampWork);
                            Program_setDout(prg_dout7_FanPower);
                        }
                        else
                        {
                            return;
                        }
                    }
                    else
                    {
                        return;
                    }
                }
                else if (timerWaitAcOk[phase]++ >= PROGRAM_TIMER_WAIT_AC_OK)
                {
                    Program_setError(error_Uac_voltageUp);
                }
                else
                {
                    return;
                }
            }
            else
            {
                return;
            }
        }
    }
    timerWaitAcOk[PROGRAM_FHASE_U] = 0;
    timerWaitAcOk[PROGRAM_FHASE_V] = 0;
    timerWaitAcOk[PROGRAM_FHASE_W] = 0;
    timerVoltageUp = 0;

    // Switch
    switch (program.control.target)
    {
    case target_Work:
        program.control.step = step_Work;
        break;
    case target_waitOp:
        program.control.step = step_Stop;
        break;
    case target_error:
        program.control.step = step_error;
        break;
    default:
        break;
    }
}
__STATIC_INLINE void __stepWork()
{
    static uint16_t timer_Iac_overload[PROGRAM_FHASE_COUNT] = {[0 ... 2] = 0};
    float Utemp = 0;

    // Button Stop
    if (Program_checkDin(prg_din2_Stop) == PRG_DIN_STOP_VAL)
    {
        Program_switchTarget(target_waitOp);
    }

  // Check error
    if (program.analog.aIn[prg_analog_Udc_invL].value < program.setup.Udc_low)
    {
        Program_setError(error_Udc_L_low);
    }
    if (program.analog.aIn[prg_analog_Udc_invL].value > program.setup.Udc_high)
    {
        Program_setError(error_Udc_L_high);
    }
    if (program.setup.phaseCount > 1)
    {
        if (program.analog.aIn[prg_analog_Udc_invR].value < program.setup.Udc_low)
        {
            Program_setError(error_Udc_R_low);
        }
        else if (program.analog.aIn[prg_analog_Udc_invR].value > program.setup.Udc_high)
        {
            Program_setError(error_Udc_R_high);
        }
    }
    
    if ((program.analog.aIn[prg_analog_I_u].value >= program.setup.Iac_nominal) &&
        (program.analog.aIn[prg_analog_I_u].value <= program.setup.Iac_nominal*PROGRAM_I_AC_K_OVERLOAD))
    {
        if (timer_Iac_overload[PROGRAM_FHASE_U]++ > PROGRAM_TIMER_I_AC_OVERLOAD)
        {
            Program_setError(error_Iac_U_overload);
        } 
    }
    else
    {
        timer_Iac_overload[PROGRAM_FHASE_U] = 0;
    }

    if ((program.analog.aIn[prg_analog_I_v].value >= program.setup.Iac_nominal) &&
        (program.analog.aIn[prg_analog_I_v].value <= program.setup.Iac_nominal*PROGRAM_I_AC_K_OVERLOAD))
    {
        if (timer_Iac_overload[PROGRAM_FHASE_V]++ > PROGRAM_TIMER_I_AC_OVERLOAD)
        {
            Program_setError(error_Iac_V_overload);
        } 
    }
    else
    {
        timer_Iac_overload[PROGRAM_FHASE_V] = 0;
    }

    if ((program.analog.aIn[prg_analog_I_w].value >= program.setup.Iac_nominal) &&
        (program.analog.aIn[prg_analog_I_w].value <= program.setup.Iac_nominal*PROGRAM_I_AC_K_OVERLOAD))
    {
        if (timer_Iac_overload[PROGRAM_FHASE_W]++ > PROGRAM_TIMER_I_AC_OVERLOAD)
        {
            Program_setError(error_Iac_W_overload);
        } 
    }
    else
    {
        timer_Iac_overload[PROGRAM_FHASE_W] = 0;
    }

    if (program.analog.aIn[prg_analog_I_u].value > program.setup.Iac_nominal*PROGRAM_I_AC_K_OVERLOAD)
    {
        Program_setError(error_Iac_U_KZ);
    }
    if (program.analog.aIn[prg_analog_I_v].value > program.setup.Iac_nominal*PROGRAM_I_AC_K_OVERLOAD)
    {
        Program_setError(error_Iac_V_KZ);
    }
    if (program.analog.aIn[prg_analog_I_w].value > program.setup.Iac_nominal*PROGRAM_I_AC_K_OVERLOAD)
    {
        Program_setError(error_Iac_W_KZ);
    }

    Utemp = program.analog.aIn[prg_analog_U_u].value;
    if ((Utemp < Utemp * (1.0f - program.setup.Uac_no_ok_percent)) ||
        (Utemp > Utemp * (1.0f + program.setup.Uac_no_ok_percent)))
    {
        Program_setError(error_Uac_U_no_ok);
    }

    Utemp = program.analog.aIn[prg_analog_U_v].value;
    if ((Utemp < Utemp * (1.0f - program.setup.Uac_no_ok_percent)) ||
        (Utemp > Utemp * (1.0f + program.setup.Uac_no_ok_percent)))
    {
        Program_setError(error_Uac_V_no_ok);
    }

    Utemp = program.analog.aIn[prg_analog_U_w].value;
    if ((Utemp < Utemp * (1.0f - program.setup.Uac_no_ok_percent)) ||
        (Utemp > Utemp * (1.0f + program.setup.Uac_no_ok_percent)))
    {
        Program_setError(error_Uac_W_no_ok);
    }

    if ((bsp_analogIn_struct.currentTemp[0] > program.setup.Tradiator_high) ||
        (bsp_analogIn_struct.currentTemp[1] > program.setup.Tradiator_high))
    {
        Program_setError(error_Tradiator_high);
    }

    if (Program_checkDin(prg_din3_TV_KT1) == PRG_DIN_KT_ON)
    {
        Program_setError(error_Ttv_high);
    }
    if (Program_checkDin(prg_din4_TV_KT2) == PRG_DIN_KT_ON)
    {
        Program_setError(error_Ttv_high);
    }
    
    // Regulator
    for (uint8_t phase = 0; phase < program.setup.phaseCount; phase++)
    {
        program.control.sau.voltageRegulator[phase].In = program.setup.U_out;
        program.control.sau.voltageRegulator[phase].Fb = *program.phase[phase].voltageFb;
        program.phase[phase].k_modOut = dsp_regulatorProcess(&program.control.sau.voltageRegulator[phase]);
    }

    // Switch
    switch (program.control.target)
    { 
    case target_waitOp:
        program.control.step = step_Stop; 
        break;
    case target_error:
        program.control.step = step_error;
        break;
    default:
        break;
    }
}
__STATIC_INLINE void __stepStop()
{
    Program_pwmOutsControl(bsp_pwm_outs_group_123, PROGRAM_PWM_OFF);
    Program_pwmOutsControl(bsp_pwm_outs_group_456, PROGRAM_PWM_OFF);

    for (uint8_t phase = 0; phase < program.setup.phaseCount; phase++)
    {
        program.phase[phase].k_modOut = 0.0f;
        dsp_regulatorReset(&program.control.sau.voltageRegulator[phase]);
    }
    dsp_intensSetterReset(&program.control.sau.ZI);

    bsp_dInOut_setDouts1_10(0);

    // Switch
    switch (program.control.target)
    { 
    case target_waitOp:
        program.control.step = step_waitOp; 
        break;
    case target_error:
        program.control.step = step_error;
        break;
    default:
        break;
    }
}


#define PRG_LED_FAULT_BLINK_PERIOD (200)
__STATIC_INLINE void __stepWaitInit()
{
    switch (program.control.target)
    {
    case target_waitOp:
        program.control.step = step_init;
        break;
    default:
        break;
    }
}
__STATIC_INLINE void __stepInit()
{
    static uint8_t ledCounter = 1;
    static uint16_t timer = 0;

    if (((timer++) % 30) == 0)
    {
        if (ledCounter < 24)
        {
            bsp_dInOut_setDout(bsp_dInOut_led_a1_y + ledCounter);
            bsp_dInOut_resetDout(bsp_dInOut_led_a1_y + ledCounter - 1);
        }
        ledCounter++;
    }

    if (ledCounter < 25)
    {
        return;
    }

    for (uint8_t i = 0; i < 24; i++)
    {
        bsp_dInOut_resetDout(bsp_dInOut_led_a1_y + i);
    }

    switch (program.control.target)
    {
    case target_waitOp:
        program.control.step = step_waitOp;
        break;
    default:
        break;
    }
}
__STATIC_INLINE void __stepDebug()
{
    static uint16_t cnt_led = 0;

    if(((cnt_led++) % 100) == 0)
    {
        BSP_LED_TOGGLE(BSP_LED_RDY);
    }

    bsp_dInOut_setDouts1_10(program.control.remote.dout.w16);
    Program_pwmOutsControl(bsp_pwm_outs_group_123, program.control.remote.pwmEnable123);
    Program_pwmOutsControl(bsp_pwm_outs_group_456, program.control.remote.pwmEnable456);

    switch (program.control.target)
    {
    case target_reset:
        program.control.step = step_reset;
        break;
    default:
        break;
    }
}
__STATIC_INLINE void __stepReset()
{
    bsp_sys_reset();
}
__STATIC_INLINE void __stepError()
{
    static uint16_t timerFaultBlink = 0;

    Program_fastStop();

    if (((timerFaultBlink++) % PRG_LED_FAULT_BLINK_PERIOD) == 0)
    {
        BSP_LED_TOGGLE(BSP_LED_FAULT);
    }

    switch (program.control.target)
    {
    case target_debug:
        program.control.step = step_debug; 
        break;
    default:
        break;
    }
}
__STATIC_INLINE void __stepWaitOp()
{
    static uint16_t cnt = 0;

    if(((cnt++) % 1000) == 0)
    {
        BSP_LED_TOGGLE(BSP_LED_RDY);
    }

    switch (program.control.target)
    {
    case target_debug:
        program.control.step = step_debug;
        break;
    default:
        break;
    }
}
/*--------------------------- STEPS END -------------------------------*/

//------------  ФУНКЦИИ   ------------//
void Program_start()
{
    protocolMbRtuSlaveCtrl_init(1);
    protocolMbRtuSlaveCtrl_init(2);

    // Загрузка параметров
    Program_ParamSetToDefault();

    // @DEBUG
     if (Program_ParamLoad() == 0)
     {
        // Неудачная попытка
        asm("NOP");
     }

    Program_regulatorInit();
    Program_pwmInit();
    Program_analogInit();

    Program_sinBuf_init();
    Program_phase_init();

    bsp_sys_tick1k_start();
    
    asm("NOP");
    Program_switchTarget(target_waitOp);
}

extern float sinBuf[3][320];
void Program_sinBuf_init()
{
    program.sin.f_pwm  = program.setup.PWM_freq;
    program.sin.f_out = program.setup.f_out;
    program.sin.bufLen = bsp_writeSinBuf(program.sin.f_pwm, program.sin.f_out);
    program.sin.currentIdx = 0;
    for (uint8_t i = 0; i < 3; i++)
    {
        for (uint16_t j = 0; j < program.sin.bufLen; j++)
        {
            program.sin.sinBuf[i][j] = sinBuf[i][j];
        }
    }
    return;
}

void Program_phase_init()
{
    program.phase[PROGRAM_FHASE_U].invNo1 = PROGRAM_INV_L;
    program.phase[PROGRAM_FHASE_U].channelNo1 = PROGRAM_OUT_U_INV_L;
    program.phase[PROGRAM_FHASE_U].invNo2 = PROGRAM_INV_R;
    program.phase[PROGRAM_FHASE_U].channelNo2 = PROGRAM_OUT_U_INV_R;
    program.phase[PROGRAM_FHASE_U].k_modOut = 0.0f;
    program.phase[PROGRAM_FHASE_U].voltageFb = &program.analog.aIn[prg_analog_U_u].value;

    program.phase[PROGRAM_FHASE_V].invNo1 = PROGRAM_INV_L;
    program.phase[PROGRAM_FHASE_V].channelNo1 = PROGRAM_OUT_V_INV_L;
    program.phase[PROGRAM_FHASE_V].invNo2 = PROGRAM_INV_R;
    program.phase[PROGRAM_FHASE_V].channelNo2 = PROGRAM_OUT_V_INV_R;
    program.phase[PROGRAM_FHASE_V].k_modOut = 0.0f;
    program.phase[PROGRAM_FHASE_V].voltageFb = &program.analog.aIn[prg_analog_U_v].value;

    program.phase[PROGRAM_FHASE_W].invNo1 = PROGRAM_INV_L;
    program.phase[PROGRAM_FHASE_W].channelNo1 = PROGRAM_OUT_W_INV_L;
    program.phase[PROGRAM_FHASE_W].invNo2 = PROGRAM_INV_R;
    program.phase[PROGRAM_FHASE_W].channelNo2 = PROGRAM_OUT_W_INV_R;
    program.phase[PROGRAM_FHASE_W].k_modOut = 0.0f;
    program.phase[PROGRAM_FHASE_W].voltageFb = &program.analog.aIn[prg_analog_U_w].value;

    return;
}

__STATIC_INLINE void Program_setDout(Program_dout_typedef dout){
        bsp_dInOut_setDout(bsp_dInOut_out1 + dout);
}
__STATIC_INLINE void Program_resetDout(Program_dout_typedef dout){
        bsp_dInOut_resetDout(bsp_dInOut_out1 + dout);
}
__STATIC_INLINE uint8_t Program_checkDin(Program_din_typedef din){
    return bsp_dInOut_readDin(bsp_dInOut_in1 + din);
}
__STATIC_INLINE void Program_fastStop()
{
    /* снять все импульсы, выключить все контакторы*/
    Program_pwmOutsControl(bsp_pwm_outs_group_123, PROGRAM_PWM_OFF);
    Program_pwmOutsControl(bsp_pwm_outs_group_456, PROGRAM_PWM_OFF);

    for (uint8_t phase = 0; phase < program.setup.phaseCount; phase++)
    {
        program.phase[phase].k_modOut = 0.0f;
        dsp_regulatorReset(&program.control.sau.voltageRegulator[phase]);
    }
    dsp_intensSetterReset(&program.control.sau.ZI);

    bsp_dInOut_setDouts1_10(0);
}

/* ЗАДАТЬ НАСТРОЙКИ ПО УМОЛЧАНИЮ */
#define PROGRAM_FHASE_COUNT_1 (1)
#define PROGRAM_FHASE_COUNT_3 (3)

#define PROGRAM_F_OUT_50HZ    (50)
#define PROGRAM_F_OUT_400HZ   (400)

// PWM_LIST
#define PROGRAM_PWM_FREQ_4000HZ (4000)
#define PROGRAM_PWM_FREQ_4800HZ (4800)
#define PROGRAM_PWM_FREQ_5600HZ (5600)
#define PROGRAM_PWM_FREQ_6000HZ (6000)
#define PROGRAM_PWM_FREQ_6400HZ (6400)
#define PROGRAM_PWM_FREQ_8000HZ (8000)
void Program_ParamSetToDefault()
{
    for (uint8_t i = 0; i < PRG_ANALOG_COUNT; i++)
    {
        program.setup.analog_av_order[i] = 12;  // 1 - фильтр отключен
        program.setup.analog_filter_N[i] = 1;   // 1 - Фильтр отключен

        program.setup.analog_kMul[i] = 1.0f;
        program.setup.analog_shift[i] = 0;
    }

    program.setup.phaseCount = PROGRAM_FHASE_COUNT_3;
    program.setup.f_out = PROGRAM_F_OUT_400HZ;
    program.setup.U_out = 120;
    program.setup.PWM_freq = PROGRAM_PWM_FREQ_4000HZ; 

    // Regul -----------------------------------------------------------
    for (uint8_t phase = 0; phase < PROGRAM_FHASE_COUNT; phase++)
    {
        program.setup.RegU_kp[phase]  = 0.001f;
        program.setup.RegU_ki[phase]  = 0.001f;
        program.setup.RegU_max[phase] = 1.0f;
    }

    // уставки для формирования аварий
    program.setup.Udc_low           = 800;     // V
    program.setup.Udc_high          = 1030;    // V
    program.setup.Uac_no_ok_percent = 0.10f;   // %
    program.setup.Iac_nominal       = 40;      // A
    program.setup.Tradiator_high    = 85;      // C

    return;
}

#define PROGRAM_PARAM_SIZE_BYTE sizeof(Program_PARAM_typedef)
uint8_t Program_ParamLoad()
{
   program.sys.flash_counter = FlashWorker_getAvailableRecords(PROGRAM_PARAM_SIZE_BYTE);
   return FlashWorker_load((void*)(&program.setup),PROGRAM_PARAM_SIZE_BYTE);
}

#define SAVE_COUNT_MAX_PER_SESSION (10)
uint8_t Program_ParamSave()
{
    static uint8_t saveCounter = 0;

    if(++saveCounter >= SAVE_COUNT_MAX_PER_SESSION ){
        return 0;
    }

    if (program.control.step == step_debug)
    {
        FlashWorker_save((void *)(&program.setup), PROGRAM_PARAM_SIZE_BYTE);
        program.sys.flash_counter = FlashWorker_getAvailableRecords(PROGRAM_PARAM_SIZE_BYTE);
        return 1;
    }
    return 0;
}

__INLINE uint8_t Program_GoDebug()
{
    if ((program.control.step != step_waitOp) && (program.control.step != step_error))
    {
        return 0;
    }
    /*
        Перед отладкой обнулить REMOTE структуру!
    */
    program.control.remote.dout.w16 = 0;
    for (uint8_t i = 0; i < 6; i++)
    {
        //program.control.remote.pwmArray[i] = 0;
    }

    program.control.remote.pwmEnable123 = 0;
    program.control.remote.pwmEnable456 = 0;

   /*
        Перед отладкой обнулить REMOTE структуру!
    */
    Program_switchTarget(target_debug);
    return 1;
}

__INLINE uint8_t Program_set_dout_debug(uint16_t douts)
{
    if(program.control.step == step_debug){

        program.control.remote.dout.w16 = douts;
        return 1;
    }
    return 0;
}

#define PWM_REMOTE_MAX_PERCENT (1000)
#define PWM_REMOTE_MIN_PERCENT (0)
__INLINE uint8_t Program_set_pwm_debug(uint8_t channel_IDx, uint16_t pwm1000Perc)
{

    if (channel_IDx > 5)
    {
        return 0;
    }
    if (program.control.step != step_debug)
    {
        return 0; 
    }

    if (pwm1000Perc > PWM_REMOTE_MAX_PERCENT)
    {
        pwm1000Perc = PWM_REMOTE_MAX_PERCENT;
    }
    else if (pwm1000Perc < PWM_REMOTE_MIN_PERCENT)
    {
        pwm1000Perc = PWM_REMOTE_MIN_PERCENT;
    }

    for (uint8_t i = 0; i < COUNT_CHANNEL_PWM; i++)
    {
        program.control.remote.kPWM[channel_IDx] = pwm1000Perc;
    }
    return 1;
}

#define PROGRAM_KMOD_MDB_MAX (990)
#define PROGRAM_KMOD_MDB_MIN (0)
__INLINE uint8_t Program_set_k_mod_debug (uint16_t kMod_mdb)
{
    if (program.control.step != step_debug)
    {
        return 0;
    }

    if (kMod_mdb > PROGRAM_KMOD_MDB_MAX)
    {
        kMod_mdb = PROGRAM_KMOD_MDB_MAX;
    }
    else if (kMod_mdb < 0)
    {
        kMod_mdb = 0;
    }
    
    for (int phase = 0; phase < program.setup.phaseCount; phase++)
    {
        program.control.remote.k_modIn = ((float)kMod_mdb)/1000.0f;
    }
    return 1;
}

__INLINE uint8_t Program_сhoice_kPWM_or_kMod_debug(uint16_t choise)
{
    if (program.control.step != step_debug)
    {
        return 0;
    }

    if (choise == REMOTE_KMOD)
    {
        program.control.remote.choise_kPWM_or_kMod = REMOTE_KMOD;
    }
    else if (choise == REMOTE_KPWM)
    {
        program.control.remote.choise_kPWM_or_kMod = REMOTE_KPWM;
    }

    return 1;
}

__INLINE uint8_t Program_set_phaseCount_debug(uint16_t phaseCount)
{
    if (program.control.step != step_debug)
    {
        return 0;
    }

    if (phaseCount == PROGRAM_FHASE_COUNT_1)
    {
        program.setup.phaseCount = PROGRAM_FHASE_COUNT_1;
    }
    else if (phaseCount == PROGRAM_FHASE_COUNT_3)
    {
        program.setup.phaseCount = PROGRAM_FHASE_COUNT_3;
    }
    else 
    {
        program.setup.phaseCount = PROGRAM_FHASE_COUNT_1;
        return 0;
    }
    return 1;
}

__INLINE uint8_t Program_set_fOut_debug(uint16_t fOut)
{
    if (program.control.step != step_debug)
    {
        return 0;
    }

    if (fOut == PROGRAM_F_OUT_50HZ)
    {
        program.setup.f_out = PROGRAM_F_OUT_50HZ;
    }
    else if (fOut == PROGRAM_F_OUT_400HZ)
    {
        program.setup.f_out = PROGRAM_F_OUT_400HZ;
    }
    else 
    {
        program.setup.f_out = PROGRAM_F_OUT_50HZ;
        return 0;
    }
    return 1; 
}

#define PROGRAM_U_OUT_MIN (0)
#define PROGRAM_U_OUT_MAX (230)
__INLINE uint8_t Program_set_uOut_debug(uint16_t uOut)
{
    if (program.control.step != step_debug)
    {
        return 0;
    }

    if (uOut <= PROGRAM_U_OUT_MIN)
    {
        uOut = PROGRAM_U_OUT_MIN;
    }
    else if (uOut >= PROGRAM_U_OUT_MAX)
    {
        uOut = PROGRAM_U_OUT_MAX;
    }
    program.setup.U_out = uOut;
    return 1;
}

__INLINE uint8_t Program_set_PWM_freq_debug(uint16_t PWM_freq)
{
    if (program.control.step != step_debug)
    {
        return 0;
    }

    switch (PWM_freq)
    {
    case PROGRAM_PWM_FREQ_4000HZ:
        program.setup.PWM_freq = PROGRAM_PWM_FREQ_4000HZ;
        break;
    case PROGRAM_PWM_FREQ_4800HZ:
        program.setup.PWM_freq = PROGRAM_PWM_FREQ_4800HZ;
        break; 
    case PROGRAM_PWM_FREQ_5600HZ:
        program.setup.PWM_freq = PROGRAM_PWM_FREQ_5600HZ;
        break;
    case PROGRAM_PWM_FREQ_6000HZ:
        program.setup.PWM_freq = PROGRAM_PWM_FREQ_6000HZ;
        break;
    case PROGRAM_PWM_FREQ_6400HZ:
        program.setup.PWM_freq = PROGRAM_PWM_FREQ_6400HZ;
        break;
    case PROGRAM_PWM_FREQ_8000HZ:
        program.setup.PWM_freq = PROGRAM_PWM_FREQ_8000HZ;
        break; 
    default:
        program.setup.PWM_freq = PROGRAM_PWM_FREQ_4000HZ;
        return 0;
        break;
    }

    return 1;
}

#define PROGRAM_REGUL_KP_MIN (0.0f)
#define PROGRAM_REGUL_KP_MAX (0.05f)
__INLINE uint8_t Program_set_regul_kp (uint8_t phase_idx, float kp)
{
    if ((program.control.step != step_debug) || (phase_idx >= PROGRAM_FHASE_COUNT))
    {
        return 0;
    }
    
    if (kp < PROGRAM_REGUL_KP_MIN)
    {
        kp = PROGRAM_REGUL_KP_MIN;
    }
    else if (kp > PROGRAM_REGUL_KP_MAX)
    {
        kp = PROGRAM_REGUL_KP_MAX;
    }
    program.setup.RegU_kp[phase_idx] = kp;
    return 1;
}

#define PROGRAM_REGUL_KI_MIN (0.0f)
#define PROGRAM_REGUL_KI_MAX (0.05f)
__INLINE uint8_t Program_set_regul_ki (uint8_t phase_idx, float ki)
{
    if ((program.control.step != step_debug) || (phase_idx >= PROGRAM_FHASE_COUNT))
    {
        return 0;
    }
    
    if (ki < PROGRAM_REGUL_KI_MIN)
    {
        ki = PROGRAM_REGUL_KI_MIN;
    }
    else if (ki > PROGRAM_REGUL_KI_MAX)
    {
        ki = PROGRAM_REGUL_KI_MAX;
    }
    program.setup.RegU_ki[phase_idx] = ki;
    return 1;
}

#define PROGRAM_REGUL_U_OUT_MIN (0.0f)
#define PROGRAM_REGUL_U_OUT_MAX (1.0f)
uint8_t Program_set_regul_uOut_max (uint8_t phase_idx, float uOut_max)
{
    if ((program.control.step != step_debug) || (phase_idx >= PROGRAM_FHASE_COUNT))
    {
        return 0;
    }

    if (uOut_max < PROGRAM_REGUL_U_OUT_MIN)
    {
        uOut_max = PROGRAM_REGUL_U_OUT_MIN;
    }
    else if (uOut_max > PROGRAM_REGUL_U_OUT_MAX)
    {
        uOut_max = PROGRAM_REGUL_U_OUT_MAX;
    }
    program.setup.RegU_max[phase_idx] = uOut_max;
    return 1;

}

#define PROGRAM_UAC_PERCENT_NO_OK_MIN (0.05f)
#define PROGRAM_UAC_PERCENT_NO_OK_MAX (0.15f)
uint8_t Program_set_Uac_no_ok_percent (float Uac_no_ok_percent)
{
    if (program.control.step != step_debug)
    {
        return 0;
    }

    if (Uac_no_ok_percent <= PROGRAM_UAC_PERCENT_NO_OK_MIN)
    {
        Uac_no_ok_percent = PROGRAM_UAC_PERCENT_NO_OK_MIN;
    }
    else if (Uac_no_ok_percent >= PROGRAM_UAC_PERCENT_NO_OK_MAX)
    {
        Uac_no_ok_percent = PROGRAM_UAC_PERCENT_NO_OK_MAX;
    }
    program.setup.Uac_no_ok_percent = Uac_no_ok_percent;
    return 1;
}

#define PROGRAM_IAC_NOMINAL_MIN (20)
#define PROGRAM_IAC_NOMINAL_MAX (60)
uint8_t Program_set_Iac_nominal (uint16_t Iac_nominal)
{
    if (program.control.step != step_debug)
    {
        return 0;
    }

    if (Iac_nominal <= PROGRAM_IAC_NOMINAL_MIN)
    {
        Iac_nominal = PROGRAM_IAC_NOMINAL_MIN;
    }
    else if (Iac_nominal >= PROGRAM_IAC_NOMINAL_MAX)
    {
        Iac_nominal = PROGRAM_IAC_NOMINAL_MAX;
    }
    program.setup.Iac_nominal = Iac_nominal;
    return 1;
}

__INLINE uint8_t Program_GoReset()
{
    if(program.control.step == step_debug){

        Program_switchTarget(target_reset);
        return 1;
    }
    return 0;
}

__INLINE uint8_t Program_LoadDefaultParam_debug()
{
    if (program.control.step == step_debug)
    {
        Program_ParamSetToDefault();
        return 1;
    }
    return 0;
}

__STATIC_INLINE uint8_t Program_setError(Program_ERROR_typedef error)
{
    if (error == error_noError)
    {
        return 0;
    }
        
    if (program.setup.protect_control & (uint16_t)(1 << (error - 1)))
    {
        return 0;
    }
    program.control.errorCode = error;
    Program_switchTarget(target_error);
    return 1;
}

__INLINE void Program_switchTarget(Program_TARGET_typedef newTarget)
{
    program.control.target = newTarget;
}
//------------  ФУНКЦИИ КОНЕЦ ------------//

//------------   Задача 1 кГц   ------------//
#define PROGRAM_UPDATE_MDB (50)
#define PROGRAM_WAIT_START (1000)
void bsp_sys_tick_1k_callback()
{
    static uint8_t  count_mdb_update   = 0;
    static uint16_t count_button_start = 0;

    // Пуск ПЧ от кнопки Start, только из target_waitOp и step_wait_op 
    if ((Program_checkDin(prg_din1_Pusk) == PRG_DIN_PUSK_VAL) &&
        (program.control.target == target_waitOp) && 
        (program.control.step == step_waitOp))
    {
        if (count_button_start++ > PROGRAM_WAIT_START)
        {
            Program_switchTarget(target_Work);
        }
        else
        {
            asm("Nop");
        }
    }
    else
    {
        count_button_start = 0;
    }


    // обновить состояние буферов Modbus Slave
    if (count_mdb_update++ > PROGRAM_UPDATE_MDB)
    {
        protocolMbRtuSlaveCtrl_update_tables();
        count_mdb_update = 0;
    }

    asm("NOP");
    switch (program.control.step)
    {
    case step_waitInit:
        __stepWaitInit();
        break;
    case step_init:
        __stepInit();
        break;
    case step_waitOp:
        __stepWaitOp();
        break;
    case step_debug:
        __stepDebug();
        break;
    case step_reset:
        __stepReset();
        break;
    case step_error:
        __stepError();
        break;
    case step_Check_Udc:
        __stepCheckUdc();
        break;
    case step_Voltage_Up:
        __stepVoltageUp();
        break;
    case step_Work:
        __stepWork();
        break;
    case step_Stop:
        __stepStop();
        break;
    default:
        break;
    }
}
//------------   Задача 1 кГц КОНЕЦ  ------------//

//----------------------- PWM ----------------------
__STATIC_INLINE void Program_pwmOutsControl(bsp_pwm_outs_group_typedef group, uint8_t enable)
{

    if (enable == 0)
    {
        bsp_pwm_disable_outs_VT(group);
        return;
    }

    if (group == bsp_pwm_outs_group_123)
    {
        bsp_pwm_enable_out_VT_CONCEPT(0, bsp_pwm_outs_type_all);
        bsp_pwm_enable_out_VT_CONCEPT(1, bsp_pwm_outs_type_all);
        bsp_pwm_enable_out_VT_CONCEPT(2, bsp_pwm_outs_type_all);
    }
    else
    {
        bsp_pwm_enable_out_VT_CONCEPT(3, bsp_pwm_outs_type_all);
        bsp_pwm_enable_out_VT_CONCEPT(4, bsp_pwm_outs_type_all);
        bsp_pwm_enable_out_VT_CONCEPT(5, bsp_pwm_outs_type_all);
    }
}

__STATIC_INLINE void Program_pwmInit()
{
    bsp_pwm_set_tim(bsp_pwm_tim_mode_up_down, 300, BSP_PWM_POSITIVE_POLARITY);


    switch (program.setup.PWM_freq)
    {
    case PROGRAM_PWM_FREQ_4000HZ:
        bsp_pwm_set_freq(bsp_pwm_outs_group_123, bsp_pwm_freq_4000_hz, 1);
        bsp_pwm_set_freq(bsp_pwm_outs_group_456, bsp_pwm_freq_4000_hz, 1);
        break;
    case PROGRAM_PWM_FREQ_4800HZ:
        bsp_pwm_set_freq(bsp_pwm_outs_group_123, bsp_pwm_freq_4800_hz, 1);
        bsp_pwm_set_freq(bsp_pwm_outs_group_456, bsp_pwm_freq_4800_hz, 1);
        break; 
    case PROGRAM_PWM_FREQ_5600HZ:
        bsp_pwm_set_freq(bsp_pwm_outs_group_123, bsp_pwm_freq_5600_hz, 1);
        bsp_pwm_set_freq(bsp_pwm_outs_group_456, bsp_pwm_freq_5600_hz, 1);
        break;
    case PROGRAM_PWM_FREQ_6000HZ:
        bsp_pwm_set_freq(bsp_pwm_outs_group_123, bsp_pwm_freq_6000_hz, 1);
        bsp_pwm_set_freq(bsp_pwm_outs_group_456, bsp_pwm_freq_6000_hz, 1);
        break;
    case PROGRAM_PWM_FREQ_6400HZ:
        bsp_pwm_set_freq(bsp_pwm_outs_group_123, bsp_pwm_freq_6400_hz, 1);
        bsp_pwm_set_freq(bsp_pwm_outs_group_456, bsp_pwm_freq_6400_hz, 1);
        break;
    case PROGRAM_PWM_FREQ_8000HZ:
        bsp_pwm_set_freq(bsp_pwm_outs_group_123, bsp_pwm_freq_8000_hz, 1);
        bsp_pwm_set_freq(bsp_pwm_outs_group_456, bsp_pwm_freq_8000_hz, 1);
        break; 
    default:
        break;
    }

    for (uint8_t i = 0; i < 6; i++)
    {
        SET_PWM(i, 500.0f);
    }

    bsp_pwm_start_IRQ_123_IRQ_456();
}
 
#define PROGRAM_PERIOD_1K_CALBACK (0.001f)
__STATIC_INLINE void Program_regulatorInit()
{
    for (uint8_t phase = 0; phase < program.setup.phaseCount; phase++)
    {
        program.control.sau.voltageRegulator[phase].k_P   = program.setup.RegU_kp[phase];
        program.control.sau.voltageRegulator[phase].k_Int = program.setup.RegU_ki[phase];

        switch (program.setup.PWM_freq)
        {
        case PROGRAM_PWM_FREQ_4000HZ:
            program.control.sau.voltageRegulator[phase].period = 0.00025f;
            break;
        case PROGRAM_PWM_FREQ_4800HZ:
            program.control.sau.voltageRegulator[phase].period = 0.0002083f;
            break;
        case PROGRAM_PWM_FREQ_5600HZ:
            program.control.sau.voltageRegulator[phase].period = 0.0001786f;
            break;
        case PROGRAM_PWM_FREQ_6000HZ:
            program.control.sau.voltageRegulator[phase].period = 0.0001667f;
            break;
        case PROGRAM_PWM_FREQ_6400HZ:
            program.control.sau.voltageRegulator[phase].period = 0.00015625;
            break;
        case PROGRAM_PWM_FREQ_8000HZ:
            program.control.sau.voltageRegulator[phase].period = 0.000125f;
            break;
        default:
            program.control.sau.voltageRegulator[phase].period = 0.00025f;
            break;
        }

        program.control.sau.voltageRegulator[phase].IntMin = 0.0f;
        program.control.sau.voltageRegulator[phase].IntMax = program.setup.RegU_max[phase];
        program.control.sau.voltageRegulator[phase].OutMin = 0.0f;
        program.control.sau.voltageRegulator[phase].OutMax = program.setup.RegU_max[phase];

        program.control.sau.voltageRegulator[phase].In = program.setup.U_out;
        
    }

    dsp_intensSetterSetup(&program.control.sau.ZI, program.setup.U_out, PROGRAM_PERIOD_1K_CALBACK);
}

uint8_t Program_set_pwmOuts_debug(bsp_pwm_outs_group_typedef group, uint8_t onOff)
{
    if ( program.control.step != step_debug )
    {
        return 0;
    }

    if (group == bsp_pwm_outs_group_123)
    {
        program.control.remote.pwmEnable123 = onOff;
    }
    else if (group == bsp_pwm_outs_group_456)
    {
        program.control.remote.pwmEnable456 = onOff;
    }
    else
    {
        return 0;
    }
    return 1;
}

#define PROGRAM_SETUP_KMOD_MAX (1.0f)
void bsp_pwm_123_callback()
{
    float currentVal = 0.0f;
    float kMod = 0.0f;

    if (program.control.step == step_debug)
    {
        if (program.control.remote.choise_kPWM_or_kMod == REMOTE_KMOD)
        {
            for (int phase = 0; phase < program.setup.phaseCount; phase++)
            {
                kMod = saturate(program.control.remote.k_modIn, 0.0f, 1.0f);
                // float tmp = rateLimitter(kMod, 1.0f, 0);
                currentVal = *(&program.sin.sinBuf[phase][0] + program.sin.currentIdx);
                currentVal *= kMod;
                setPhasePWM(&program.phase[phase], currentVal);
            }
            if (++program.sin.currentIdx == program.sin.bufLen)
            {
                program.sin.currentIdx = 0;
            }
        }
        else if (program.control.remote.choise_kPWM_or_kMod == REMOTE_KPWM)
        {
            for (uint8_t i = 0; i < COUNT_CHANNEL_PWM; i++)
            {
                SET_PWM(i, program.control.remote.kPWM[i]);
            }
        }
        else
        {
            // Ошибка !!!
            asm("Nop");
        }
        return;
    }

    for (uint8_t phase = 0; phase < program.setup.phaseCount; phase++)
    {
        kMod = saturate(program.phase[phase].k_modOut, 0.0f, PROGRAM_SETUP_KMOD_MAX);
        currentVal = *(&program.sin.sinBuf[phase][0] + program.sin.currentIdx);
        currentVal *= kMod;
        setPhasePWM(&program.phase[phase], currentVal);
    }
    if (++program.sin.currentIdx == program.sin.bufLen)
    {
        asm("Nop");
        program.sin.currentIdx = 0;
    }

    return;
}
//----------------------- PWM END----------------------

//------------   АЦП   ------------//
Program_AIN_typedef* Program_analogGetByIdx(Program_ANALOG_ENUM_typedef idx){

    if(idx >= PRG_ANALOG_COUNT){
        return NULL;
    }

    return &program.analog.aIn[idx];
}

uint8_t Program_analogSetZero(Program_ANALOG_ENUM_typedef idx){
    
    if(program.control.step != step_debug)
    {
        return 0;
    }
    
    if (idx >= PRG_ANALOG_COUNT)
    {
        return 0;
    }

    program.analog.aIn[idx].shift = program.analog.aIn[idx].valueRaw;
    program.setup.analog_shift[idx] = program.analog.aIn[idx].shift;
    return 1;
}

uint8_t Program_analogCalibKMul(Program_ANALOG_ENUM_typedef idx, float value){

    if(program.control.step != step_debug)
    {
        return 0;
    }

    if (idx >= PRG_ANALOG_COUNT)
    {
        return 0;
    }

    float kMul = program.analog.aIn[idx].kMul;
    if(kMul == 0.0f) return 0;
    float currentVal = program.analog.aIn[idx].value / kMul;

    program.analog.aIn[idx].kMul = value / currentVal;
    program.setup.analog_kMul[idx] = program.analog.aIn[idx].kMul;
    return 1;
}

uint8_t Program_analogSetShift(Program_ANALOG_ENUM_typedef idx, float value){

    if (program.control.step != step_debug)
    {
        return 0;
    }

    if (idx >= PRG_ANALOG_COUNT)
    {
        return 0;
    }

    program.setup.analog_shift[idx] = value;
    return 1;
}

uint8_t Program_analogSetKMul(Program_ANALOG_ENUM_typedef idx, float value){

    if (program.control.step != step_debug)
    {
        return 0;
    }

    if (idx >= PRG_ANALOG_COUNT)
    {
        return 0;
    }

    program.setup.analog_kMul[idx] = value;
    return 1;
}

uint8_t Program_analogSetAvOrder(Program_ANALOG_ENUM_typedef idx, uint8_t order){

    if (program.control.step != step_debug)
    {
        return 0;
    }

    if (idx >= PRG_ANALOG_COUNT)
    {
        return 0;
    }

    if(order < 1 ) order = 1;
    else if (order > PROGRAM_ADC_MAX_FILTER_ORDER) order = PROGRAM_ADC_MAX_FILTER_ORDER;
    program.setup.analog_av_order[idx] = order;
    return 1;
}

uint8_t Program_analogSetFilterN(Program_ANALOG_ENUM_typedef idx, uint16_t filterN){

    if (program.control.step != step_debug)
    {
        return 0;
    }

    if (idx >= PRG_ANALOG_COUNT)
    {
        return 0;
    }

    if(filterN < 1 )
    {
        filterN = 1;
    } 
    else if (filterN > PROGRAM_ADC_MAX_FILTER_N)
    {
        filterN = PROGRAM_ADC_MAX_FILTER_N;
    }

    program.setup.analog_filter_N[idx] = filterN;
    return 1;
}

__STATIC_INLINE uint8_t Program_analogInit()
{
    for (uint8_t i = 0; i < PRG_ANALOG_COUNT; i++)
    {
        program.analog.aIn[i].bspIdx        = -1;
        program.analog.aIn[i].order         = program.setup.analog_av_order[i];
        program.analog.aIn[i].analogFilterN = program.setup.analog_filter_N[i];
        program.analog.aIn[i].kMul          = program.setup.analog_kMul[i];
        program.analog.aIn[i].shift         = program.setup.analog_shift[i];
    }

    program.analog.aIn[prg_analog_I_u].bspIdx      = 0;
    program.analog.aIn[prg_analog_I_v].bspIdx      = 1;
    program.analog.aIn[prg_analog_I_w].bspIdx      = 2;
    program.analog.aIn[prg_analog_U_u].bspIdx      = 3;
    program.analog.aIn[prg_analog_U_v].bspIdx      = 4;
    program.analog.aIn[prg_analog_U_w].bspIdx      = 5;
    program.analog.aIn[prg_analog_Udc_invL].bspIdx = 6;
    program.analog.aIn[prg_analog_Udc_invR].bspIdx = 7;
    program.analog.aIn[prg_analog_AI8].bspIdx      = 8;
    program.analog.aIn[prg_analog_AI9].bspIdx      = 9;
    program.analog.aIn[prg_analog_AI10].bspIdx     = 10;

    bsp_analogIn_start();

    return 1;
}

__STATIC_INLINE float saturate(float in, float min, float max)
{
    if (in < min)
    {
        return min;
    }  
    else if (in > max)
    {
        return max;
    }
    return in;
}

#define PROGRAM_SETUP_PWM_MIN_VAL (250)
__STATIC_INLINE void setPhasePWM(Program_PHASE_typedef *phase, float value)
{
    uint16_t Arr = LL_HRTIM_TIM_GetPeriod(HRTIM1, BSP_PWM_TIM_PWM_1);
    uint16_t halfArr = Arr / 2;

    float tmp = value * (halfArr - PROGRAM_SETUP_PWM_MIN_VAL);   // CCR регистр должен быть >= 2 & <= arr - 2

    uint32_t current_val_invNO1 = halfArr - (int16_t)tmp;
    uint32_t current_val_invNO2 = halfArr + (int16_t)tmp;

    bsp_pwm_set_ccr(phase->channelNo1, current_val_invNO1);
    bsp_pwm_set_ccr(phase->channelNo2, current_val_invNO2);
    return;
}

#define FFT_0HZ    (0)
#define FFT_50HZ   (1)
#define FFT_400HZ  (2)
void bsp_analogIn_ready_callback()
{
    uint8_t count = PRG_ANALOG_COUNT;
    float value = 0.0f;
    float valueLast = 0.0f;
    float kFilter = 0.0f;
    float data = 0.0f;

    asm("NOP");

    for (uint8_t ch = 0; ch < count; ch++)
    {
        if (program.analog.aIn[ch].bspIdx == -1)
            continue;

        uint8_t bspIdx = program.analog.aIn[ch].bspIdx;

        if ((bspIdx >= 0) && (bspIdx <= 5))
        {
            data = bsp_analogIn_struct.rawDataUI_FFT[bspIdx][FFT_50HZ];
        }
        if ((bspIdx >= 6) && (bspIdx <= 7))
        {
            data = bsp_analogIn_struct.rawDataUI_FFT[bspIdx][FFT_50HZ];
        }
        if ((bspIdx >= 8) && (bspIdx <= 11))
        {
            data = bsp_analogIn_struct.rawDataUI_FFT[bspIdx][FFT_50HZ];
        }

        program.analog.aIn[ch].buf[program.analog.aIn[ch].bufIdx++] = data;

        if (program.analog.aIn[ch].bufIdx == program.analog.aIn[ch].order)
        {
            program.analog.aIn[ch].bufIdx = 0;
        }

        float sum = 0.0f;
        for (uint8_t idx = 0; idx < program.analog.aIn[ch].order; idx++)
        {
            sum += program.analog.aIn[ch].buf[idx];
        }
        program.analog.aIn[ch].valueRaw = (sum / program.analog.aIn[ch].order);

        value = (program.analog.aIn[ch].valueRaw - program.analog.aIn[ch].shift) *
                program.analog.aIn[ch].kMul;

        valueLast = program.analog.aIn[ch].valueLast;
        // Формула: Yavg(i) = Yavg(i-1) + a * ( X(i) - Yavg(i-1) );
        // a = 2/(N + 1) -> Коэффициент фильтра;
        // N -> количество точек для усреднения, N >= 1;
        // N = 1 -> фильтр отключен;
        kFilter = 2.0f / ((float)program.analog.aIn[ch].analogFilterN + 1.0f);
        value = valueLast + kFilter * (value - valueLast);

        program.analog.aIn[ch].value = value;
        program.analog.aIn[ch].valueLast = value;
    }
}
//------------   АЦП End  ------------//

