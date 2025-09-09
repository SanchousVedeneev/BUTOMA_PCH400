

#include "Program.h"
#include "ProtocolMbRtuSlaveCtrl.h"
#include "FlashWorker.h"

Program_typedef programStruct;

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
#define PRG_LED_FAULT_BLINK_PERIOD (200)
__STATIC_INLINE void __stepWaitInit()
{

//---------------- переключатель
    switch (programStruct.control.target)
    {
    case target_waitOp:
        programStruct.control.step = step_init; // ------->
        break;
    
    default:
        break;
    }
//---------------- переключатель конец
}
__STATIC_INLINE void __stepInit()
{
static uint8_t ledCounter = 1;
static uint16_t timer = 0;
//------------------  LED BLINK ---------------
if (((timer++) % 30) == 0)
{
    if (ledCounter < 24)
    {
        bsp_dInOut_setDout(bsp_dInOut_led_a1_y + ledCounter);
        bsp_dInOut_resetDout(bsp_dInOut_led_a1_y + ledCounter-1);
    }
    ledCounter++;
}

if(ledCounter < 25) return;

for (uint8_t i = 0; i < 24; i++)
{
    bsp_dInOut_resetDout(bsp_dInOut_led_a1_y + i);
}
//------------------  LED BLINK END  ---------------

//---------------- переключатель
    switch (programStruct.control.target)
    {
    case target_waitOp:
        programStruct.control.step = step_wait_op; // ------->
        break;
    
    default:
        break;
    }
//---------------- переключатель конец
}
__STATIC_INLINE void __stepDebug()
{
    static uint16_t cnt_led = 0;

//-----------------------  LED BLINK -----------------------
    if(((cnt_led++)%100)==0){
        BSP_LED_TOGGLE(BSP_LED_RDY);
    }
//-----------------------  LED BLINK END-----------------------

//------------------------   DOUTS ------------------------------
    bsp_dInOut_setDouts1_10(programStruct.control.remote.dout.w16);
//------------------------   DOUTS END------------------------------

    Program_pwmOutsControl(bsp_pwm_outs_group_123, programStruct.control.remote.pwmEnable123);
    Program_pwmOutsControl(bsp_pwm_outs_group_456, programStruct.control.remote.pwmEnable456);
//---------------- переключатель
    switch (programStruct.control.target)
    {
    case target_reset:
        programStruct.control.step = step_reset; // ------->
        break;
    
    default:
        break;
    }
//---------------- переключатель конец

}
__STATIC_INLINE void __stepReset()
{
    //HAL_UART_Transmit(&huart,buf,5,100);
    bsp_sys_reset();

}
__STATIC_INLINE void __stepError()
{
    static uint16_t timerFaultBlink = 0;

    Program_fastStop();

    if(((timerFaultBlink++)%PRG_LED_FAULT_BLINK_PERIOD)==0){
        BSP_LED_TOGGLE(BSP_LED_FAULT);
    }

    //---------------- переключатель
    switch (programStruct.control.target)
    {
    case target_debug:
        programStruct.control.step = step_debug; // ------->
        break;

    default:
        break;
    }
    //---------------- переключатель конец
}
__STATIC_INLINE void __stepWaitOp()
{
    static uint16_t cnt = 0;


    if(((cnt++)%1000)==0){
        BSP_LED_TOGGLE(BSP_LED_RDY);
    }

//---------------- переключатель
    switch (programStruct.control.target)
    {
    case target_debug:
        programStruct.control.step = step_debug; // ------->
        break;
    default:
        break;
    }
//---------------- переключатель конец

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

    bsp_sys_tick1k_start();
    bsp_fft_init();
    bsp_cordic_init();
    Program_sinBuf_init();
    Program_phase_init();
    
    asm("NOP");
    Program_switchTarget(target_waitOp);
}

extern float sinBuf[3][320];
void Program_sinBuf_init()
{
    programStruct.sin.f_pwm  = programStruct.setup.PWM_freq;
    programStruct.sin.f_out = programStruct.setup.f_out;
    programStruct.sin.bufLen = bsp_writeSinBuf(programStruct.sin.f_pwm, programStruct.sin.f_out);
    programStruct.sin.currentIdx = 0;
    for (uint8_t i = 0; i < 3; i++)
    {
        for (uint16_t j = 0; j < programStruct.sin.bufLen; j++)
        {
            programStruct.sin.sinBuf[i][j] = sinBuf[i][j];
        }
    }
    return;
}

void Program_phase_init()
{
    programStruct.phase[PROGRAM_FHASE_U].invNo1 = PROGRAM_INV_L;
    programStruct.phase[PROGRAM_FHASE_U].channelNo1 = PROGRAM_OUT_U_INV_L;
    programStruct.phase[PROGRAM_FHASE_U].invNo2 = PROGRAM_INV_R;
    programStruct.phase[PROGRAM_FHASE_U].channelNo2 = PROGRAM_OUT_U_INV_R;
    programStruct.phase[PROGRAM_FHASE_U].k_modOut = 0.0f;
    programStruct.phase[PROGRAM_FHASE_U].voltageFb = &programStruct.analog.aIn[prg_analog_U_u].value;

    programStruct.phase[PROGRAM_FHASE_V].invNo1 = PROGRAM_INV_L;
    programStruct.phase[PROGRAM_FHASE_V].channelNo1 = PROGRAM_OUT_V_INV_L;
    programStruct.phase[PROGRAM_FHASE_V].invNo2 = PROGRAM_INV_R;
    programStruct.phase[PROGRAM_FHASE_V].channelNo2 = PROGRAM_OUT_V_INV_R;
    programStruct.phase[PROGRAM_FHASE_V].k_modOut = 0.0f;
    programStruct.phase[PROGRAM_FHASE_V].voltageFb = &programStruct.analog.aIn[prg_analog_U_v].value;

    programStruct.phase[PROGRAM_FHASE_W].invNo1 = PROGRAM_INV_L;
    programStruct.phase[PROGRAM_FHASE_W].channelNo1 = PROGRAM_OUT_W_INV_L;
    programStruct.phase[PROGRAM_FHASE_W].invNo2 = PROGRAM_INV_R;
    programStruct.phase[PROGRAM_FHASE_W].channelNo2 = PROGRAM_OUT_W_INV_R;
    programStruct.phase[PROGRAM_FHASE_W].k_modOut = 0.0f;
    programStruct.phase[PROGRAM_FHASE_W].voltageFb = &programStruct.analog.aIn[prg_analog_U_w].value;

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
__STATIC_INLINE void Program_fastStop(){
    /* снять все импульсы, выключить все контакторы*/
    Program_pwmOutsControl(bsp_pwm_outs_group_123, 0);
    Program_pwmOutsControl(bsp_pwm_outs_group_456, 0);
    bsp_dInOut_setDouts1_10(0);
}

/* ЗАДАТЬ НАСТРОЙКИ ПО УМОЛЧАНИЮ */
#define PROGRAM_FHASE_COUNT_1 (1)
#define PROGRAM_FHASE_COUNT_3 (3)

#define PROGRAM_F_OUT_50HZ    (50)
#define PROGRAM_F_OUT_400HZ   (400)

void Program_ParamSetToDefault()
{
    for (uint8_t i = 0; i < PRG_ANALOG_COUNT; i++)
    {
        programStruct.setup.analog_av_order[i] = 12;  // 1 - фильтр отключен
        programStruct.setup.analog_filter_N[i] = 1;   // 1 - Фильтр отключен

        programStruct.setup.analog_kMul[i] = 1.0f;
        programStruct.setup.analog_shift[i] = 0;
    }
    programStruct.setup.phaseCount = 3;
    programStruct.setup.f_out = 50;
    programStruct.setup.U_out = 120;
    programStruct.setup.PWM_freq = 8000;

    return;
}

#define PROGRAM_PARAM_SIZE_BYTE sizeof(Program_PARAM_typedef)
uint8_t Program_ParamLoad()
{
   programStruct.sys.flash_counter = FlashWorker_getAvailableRecords(PROGRAM_PARAM_SIZE_BYTE);
   return FlashWorker_load((void*)(&programStruct.setup),PROGRAM_PARAM_SIZE_BYTE);
}

#define SAVE_COUNT_MAX_PER_SESSION (10)
uint8_t Program_ParamSave()
{
    static uint8_t saveCounter = 0;

    if(++saveCounter >= SAVE_COUNT_MAX_PER_SESSION ){
        return 0;
    }

    if (programStruct.control.step == step_debug)
    {
        FlashWorker_save((void *)(&programStruct.setup), PROGRAM_PARAM_SIZE_BYTE);
        programStruct.sys.flash_counter = FlashWorker_getAvailableRecords(PROGRAM_PARAM_SIZE_BYTE);
        return 1;
    }
    return 0;
}

__INLINE uint8_t Program_GoDebug()
{
    if ((programStruct.control.step != step_wait_op) && (programStruct.control.step != step_error))
    {
        return 0;
    }
    /*
        Перед отладкой обнулить REMOTE структуру!
    */
    programStruct.control.remote.dout.w16 = 0;
    for (uint8_t i = 0; i < 6; i++)
    {
        //programStruct.control.remote.pwmArray[i] = 0;
    }

    programStruct.control.remote.pwmEnable123 = 0;
    programStruct.control.remote.pwmEnable456 = 0;

   /*
        Перед отладкой обнулить REMOTE структуру!
    */
    Program_switchTarget(target_debug);
    return 1;
}

__INLINE uint8_t Program_set_dout_debug(uint16_t douts)
{
    if(programStruct.control.step == step_debug){

        programStruct.control.remote.dout.w16 = douts;
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
    if (programStruct.control.step != step_debug)
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
        programStruct.control.remote.kPWM[channel_IDx] = pwm1000Perc;
    }
    return 1;
}

#define PROGRAM_KMOD_MDB_MAX (990)
#define PROGRAM_KMOD_MDB_MIN (0)
__INLINE uint8_t Program_set_k_mod_debug (uint16_t kMod_mdb)
{
    if (programStruct.control.step != step_debug)
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
    
    for (int phase = 0; phase < programStruct.setup.phaseCount; phase++)
    {
        programStruct.control.remote.k_modIn = ((float)kMod_mdb)/1000.0f;
    }
    return 1;
}

__INLINE uint8_t Program_сhoice_kPWM_or_kMod_debug(uint16_t choise)
{
    if (programStruct.control.step != step_debug)
    {
        return 0;
    }

    if (choise == REMOTE_KMOD)
    {
        programStruct.control.remote.choise_kPWM_or_kMod = REMOTE_KMOD;
    }
    else if (choise == REMOTE_KPWM)
    {
        programStruct.control.remote.choise_kPWM_or_kMod = REMOTE_KPWM;
    }

    return 1;
}

__INLINE uint8_t Program_set_phaseCount_debug(uint16_t phaseCount)
{
    if (programStruct.control.step != step_debug)
    {
        return 0;
    }

    if (phaseCount == PROGRAM_FHASE_COUNT_1)
    {
        programStruct.setup.phaseCount = PROGRAM_FHASE_COUNT_1;
    }
    else if (phaseCount == PROGRAM_FHASE_COUNT_3)
    {
        programStruct.setup.phaseCount = PROGRAM_FHASE_COUNT_3;
    }
    else 
    {
        programStruct.setup.phaseCount = PROGRAM_FHASE_COUNT_1;
        return 0;
    }
    return 1;
}

__INLINE uint8_t Program_set_fOut_debug(uint16_t fOut)
{
    if (programStruct.control.step != step_debug)
    {
        return 0;
    }

    if (fOut == PROGRAM_F_OUT_50HZ)
    {
        programStruct.setup.f_out = PROGRAM_F_OUT_50HZ;
    }
    else if (fOut == PROGRAM_F_OUT_400HZ)
    {
        programStruct.setup.f_out = PROGRAM_F_OUT_400HZ;
    }
    else 
    {
        programStruct.setup.f_out = PROGRAM_F_OUT_50HZ;
        return 0;
    }
    return 1; 
}

#define PROGRAM_U_OUT_MIN (0)
#define PROGRAM_U_OUT_MAX (230)
__INLINE uint8_t Program_set_uOut_debug(uint16_t uOut)
{
    if (programStruct.control.step != step_debug)
    {
        return 0;
    }

    if (uOut <= PROGRAM_U_OUT_MIN)
    {
        uOut = PROGRAM_U_OUT_MIN;
        return 0;
    }
    else if (uOut >= PROGRAM_U_OUT_MAX)
    {
        uOut = PROGRAM_U_OUT_MAX;
        return 0;
    }
    programStruct.setup.U_out = uOut;
    return 1;
}

#define PROGRAM_PWM_FREQ_4000HZ (4000)
#define PROGRAM_PWM_FREQ_4800HZ (4800)
#define PROGRAM_PWM_FREQ_5600HZ (5600)
#define PROGRAM_PWM_FREQ_6000HZ (6000)
#define PROGRAM_PWM_FREQ_6400HZ (6400)
#define PROGRAM_PWM_FREQ_8000HZ (8000)
__INLINE uint8_t Program_set_PWM_freq_debug(uint16_t PWM_freq)
{
    if (programStruct.control.step != step_debug)
    {
        return 0;
    }

    switch (PWM_freq)
    {
    case PROGRAM_PWM_FREQ_4000HZ:
        programStruct.setup.PWM_freq = PROGRAM_PWM_FREQ_4000HZ;
        break;
    case PROGRAM_PWM_FREQ_4800HZ:
        programStruct.setup.PWM_freq = PROGRAM_PWM_FREQ_4800HZ;
        break; 
    case PROGRAM_PWM_FREQ_5600HZ:
        programStruct.setup.PWM_freq = PROGRAM_PWM_FREQ_5600HZ;
        break;
    case PROGRAM_PWM_FREQ_6000HZ:
        programStruct.setup.PWM_freq = PROGRAM_PWM_FREQ_6000HZ;
        break;
    case PROGRAM_PWM_FREQ_6400HZ:
        programStruct.setup.PWM_freq = PROGRAM_PWM_FREQ_6400HZ;
        break;
    case PROGRAM_PWM_FREQ_8000HZ:
        programStruct.setup.PWM_freq = PROGRAM_PWM_FREQ_8000HZ;
        break; 
    default:
        programStruct.setup.PWM_freq = PROGRAM_PWM_FREQ_4000HZ;
        return 0;
        break;
    }

    return 1;
}

__INLINE uint8_t Program_GoReset()
{
    if(programStruct.control.step == step_debug){

        Program_switchTarget(target_reset);
        return 1;
    }
    return 0;
}

__INLINE uint8_t Program_LoadDefaultParam_debug()
{
    if (programStruct.control.step == step_debug)
    {
        Program_ParamSetToDefault();
        return 1;
    }
    return 0;
}

__STATIC_INLINE uint8_t Program_setError(Program_ERROR_typedef error)
{
    if (error == error_noError)
        return 0;

    if (programStruct.setup.protect_control & (uint64_t)(1 << (error - 1)))
    {
        return 0;
    }
    programStruct.control.errorCode = error;
    Program_switchTarget(target_error);
    return 1;
}

__INLINE void Program_switchTarget(Program_TARGET_typedef newTarget)
{

    programStruct.control.target = newTarget;
}
//------------  ФУНКЦИИ КОНЕЦ ------------//

//------------   Задача 1 кГц   ------------//
void bsp_sys_tick_1k_callback()
{
    // Проверка нажатия Аварийного стопа
    // В отладочном режиме аварийный стоп не работает!
    // if ((Program_checkDin(prg_din3_ALARM_STOP) == PRG_DIN_ALARM_STOP_VAL) &&
    //     (programStruct.control.target != target_debug))
    // {
    //     if (Program_setError(error_fastStop))
    //     {
    //         programStruct.control.step = step_error;
    //     }
    // }

    // Пуск водорода от кнопки Start, только из target_waitOp и step_wait_op 
    // if ((Program_checkDin(prg_din1_PUSK) == PRG_DIN_PUSK_VAL) &&
    //     (programStruct.control.target == target_waitOp) && 
    //     (programStruct.control.step == step_wait_op))
    // {
    //     Program_switchTarget(target_vodorodWork);
    // }

    // обновить состояние буферов Modbus Slave
    protocolMbRtuSlaveCtrl_update_tables();
    
    asm("NOP");
    switch (programStruct.control.step)
    {
    case step_waitInit:
        __stepWaitInit();
        break;
    case step_init:
        __stepInit();
        break;
    case step_wait_op:
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


    switch (programStruct.setup.PWM_freq)
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

__STATIC_INLINE void Program_regulatorInit()
{
    for (uint8_t phase = 0; phase < programStruct.setup.phaseCount; phase++)
    {
        programStruct.control.sau.voltageRegulator[phase].IntMin = 0.0f;
        programStruct.control.sau.voltageRegulator[phase].IntMax = 1.0f;
        programStruct.control.sau.voltageRegulator[phase].OutMin = 0.0f;
        programStruct.control.sau.voltageRegulator[phase].OutMax = 1.0f;
        
    }
}

uint8_t Program_set_pwmOuts_debug(bsp_pwm_outs_group_typedef group, uint8_t onOff)
{
    if ( programStruct.control.step != step_debug )
    {
        return 0;
    }

    if (group == bsp_pwm_outs_group_123)
    {
        programStruct.control.remote.pwmEnable123 = onOff;
    }
    else if (group == bsp_pwm_outs_group_456)
    {
        programStruct.control.remote.pwmEnable456 = onOff;
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

    if (programStruct.control.step == step_debug)
    {
        if (programStruct.control.remote.choise_kPWM_or_kMod == REMOTE_KMOD)
        {
            for (int phase = 0; phase < programStruct.setup.phaseCount; phase++)
            {
                kMod = saturate(programStruct.control.remote.k_modIn, 0.0f, 1.0f);
                // float tmp = rateLimitter(kMod, 1.0f, 0);
                currentVal = *(&programStruct.sin.sinBuf[phase][0] + programStruct.sin.currentIdx);
                currentVal *= kMod;
                setPhasePWM(&programStruct.phase[phase], currentVal);
            }
            if (++programStruct.sin.currentIdx == programStruct.sin.bufLen)
            {
                programStruct.sin.currentIdx = 0;
            }
        }
        else if (programStruct.control.remote.choise_kPWM_or_kMod == REMOTE_KPWM)
        {
            for (uint8_t i = 0; i < COUNT_CHANNEL_PWM; i++)
            {
                SET_PWM(i, programStruct.control.remote.kPWM[i]);
            }
        }
        else
        {
            // Ошибка !!!
            asm("Nop");
        }
        return;
    }

    for (uint8_t phase = 0; phase < programStruct.setup.phaseCount; phase++)
    {
        kMod = saturate(programStruct.phase[phase].k_modOut, 0.0f, PROGRAM_SETUP_KMOD_MAX);
        currentVal = *(&programStruct.sin.sinBuf[phase][0] + programStruct.sin.currentIdx);
        currentVal *= kMod;
        setPhasePWM(&programStruct.phase[phase], currentVal);
    }
    if (++programStruct.sin.currentIdx == programStruct.sin.bufLen)
    {
        asm("Nop");
        programStruct.sin.currentIdx = 0;
    }

    return;
}
//----------------------- PWM END----------------------

//------------   АЦП   ------------//
Program_AIN_typedef* Program_analogGetByIdx(Program_ANALOG_ENUM_typedef idx){

    if(idx >= PRG_ANALOG_COUNT){
        return NULL;
    }

    return &programStruct.analog.aIn[idx];
}

uint8_t Program_analogSetZero(Program_ANALOG_ENUM_typedef idx){
    
    if(programStruct.control.step != step_debug){
        return 0;
    }
    
    if (idx >= PRG_ANALOG_COUNT)
    {
        return 0;
    }

    programStruct.analog.aIn[idx].shift = programStruct.analog.aIn[idx].valueRaw;
    programStruct.setup.analog_shift[idx] = programStruct.analog.aIn[idx].shift;
    return 1;
}

uint8_t Program_analogCalibKMul(Program_ANALOG_ENUM_typedef idx, float value){

    if(programStruct.control.step != step_debug){
        return 0;
    }

    if (idx >= PRG_ANALOG_COUNT)
    {
        return 0;
    }

    float kMul = programStruct.analog.aIn[idx].kMul;
    if(kMul == 0.0f) return 0;
    float currentVal = programStruct.analog.aIn[idx].value / kMul;

    programStruct.analog.aIn[idx].kMul = value / currentVal;
    programStruct.setup.analog_kMul[idx] = programStruct.analog.aIn[idx].kMul;
    return 1;
}

uint8_t Program_analogSetShift(Program_ANALOG_ENUM_typedef idx, float value){

    if (programStruct.control.step != step_debug)
    {
        return 0;
    }

    if (idx >= PRG_ANALOG_COUNT)
    {
        return 0;
    }

    programStruct.setup.analog_shift[idx] = value;
    return 1;
}

uint8_t Program_analogSetKMul(Program_ANALOG_ENUM_typedef idx, float value){

    if (programStruct.control.step != step_debug)
    {
        return 0;
    }

    if (idx >= PRG_ANALOG_COUNT)
    {
        return 0;
    }

    programStruct.setup.analog_kMul[idx] = value;
    return 1;
}

uint8_t Program_analogSetAvOrder(Program_ANALOG_ENUM_typedef idx, uint8_t order){

    if (programStruct.control.step != step_debug)
    {
        return 0;
    }

    if (idx >= PRG_ANALOG_COUNT)
    {
        return 0;
    }

    if(order < 1 ) order = 1;
    else if (order > PROGRAM_ADC_MAX_FILTER_ORDER) order = PROGRAM_ADC_MAX_FILTER_ORDER;
    programStruct.setup.analog_av_order[idx] = order;
    return 1;
}

uint8_t Program_analogSetFilterN(Program_ANALOG_ENUM_typedef idx, uint16_t filterN){

    if (programStruct.control.step != step_debug)
    {
        return 0;
    }

    if (idx >= PRG_ANALOG_COUNT)
    {
        return 0;
    }

    if(filterN < 1 ) filterN = 1;
    else if (filterN > 350) filterN = 350;
    programStruct.setup.analog_filter_N[idx] = filterN;
    return 1;
}

__STATIC_INLINE uint8_t Program_analogInit()
{
    for (uint8_t i = 0; i < PRG_ANALOG_COUNT; i++)
    {
        programStruct.analog.aIn[i].bspIdx        = -1;
        programStruct.analog.aIn[i].order         = programStruct.setup.analog_av_order[i];
        programStruct.analog.aIn[i].analogFilterN = programStruct.setup.analog_filter_N[i];
        programStruct.analog.aIn[i].kMul          = programStruct.setup.analog_kMul[i];
        programStruct.analog.aIn[i].shift         = programStruct.setup.analog_shift[i];
    }

    programStruct.analog.aIn[prg_analog_I_u].bspIdx = 0;
    programStruct.analog.aIn[prg_analog_I_v].bspIdx = 1;
    programStruct.analog.aIn[prg_analog_I_w].bspIdx = 2;
    programStruct.analog.aIn[prg_analog_U_u].bspIdx = 3;
    programStruct.analog.aIn[prg_analog_U_v].bspIdx = 4;
    programStruct.analog.aIn[prg_analog_U_w].bspIdx = 5;
    programStruct.analog.aIn[prg_analog_Uzpt_inv_L].bspIdx = 6;
    programStruct.analog.aIn[prg_analog_Uzpt_inv_R].bspIdx = 7;
    programStruct.analog.aIn[prg_analog_AI8].bspIdx = 8;
    programStruct.analog.aIn[prg_analog_AI9].bspIdx = 9;
    programStruct.analog.aIn[prg_analog_AI10].bspIdx = 10;

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
        if (programStruct.analog.aIn[ch].bspIdx == -1)
            continue;

        uint8_t bspIdx = programStruct.analog.aIn[ch].bspIdx;

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

        programStruct.analog.aIn[ch].buf[programStruct.analog.aIn[ch].bufIdx++] = data;

        if (programStruct.analog.aIn[ch].bufIdx == programStruct.analog.aIn[ch].order)
        {
            programStruct.analog.aIn[ch].bufIdx = 0;
        }

        float sum = 0.0f;
        for (uint8_t idx = 0; idx < programStruct.analog.aIn[ch].order; idx++)
        {
            sum += programStruct.analog.aIn[ch].buf[idx];
        }
        programStruct.analog.aIn[ch].valueRaw = (sum / programStruct.analog.aIn[ch].order);

        value = (programStruct.analog.aIn[ch].valueRaw - programStruct.analog.aIn[ch].shift) *
                programStruct.analog.aIn[ch].kMul;

        valueLast = programStruct.analog.aIn[ch].valueLast;
        // Формула: Yavg(i) = Yavg(i-1) + a * ( X(i) - Yavg(i-1) );
        // a = 2/(N + 1) -> Коэффициент фильтра;
        // N -> количество точек для усреднения, N >= 1;
        // N = 1 -> фильтр отключен;
        kFilter = 2.0f / ((float)programStruct.analog.aIn[ch].analogFilterN + 1.0f);
        value = valueLast + kFilter * (value - valueLast);

        programStruct.analog.aIn[ch].value = value;
        programStruct.analog.aIn[ch].valueLast = value;
    }
}
//------------   АЦП End  ------------//

