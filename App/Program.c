

#include "Program.h"

#include "ProtocolMbRtuSlaveCtrl.h"
#include "FlashWorker.h"

//------------  Главная структура   ------------//
Program_typedef programStruct;
//------------  Главная структура Конец  ------------//

#define LOW_CHARGE_SOC (10.0f) //%
#define HIGH_CHARGE_SOC (90.0f)

// Выбор параметра по которому определяем сигнал аварийный "высокий/низкий заряд батареи"
#define CONROL_CHARGE_BAT_OF_U
//#define CONROL_CHARGE_BAT_OF_SOC

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

#define SET_PWM_STEP_UP(IDX,VALUE_1000) bsp_pwm_set_ccrPercentX10((IDX), 1000.0f - (VALUE_1000))
#define SET_PWM_STEP_DOWN(IDX,VALUE_1000) bsp_pwm_set_ccrPercentX10((IDX), (VALUE_1000))

/*----------------------------- PRIVATE FCN MACRO END ---------------------------------*/

// --------------------- EXTERN ---------------------//
extern bsp_analogIn_typedef bsp_analogIn_struct;
// --------------------- EXTERN END---------------------//

/*----------------------------- STEPS ---------------------------------*/
/*----------------------------- STEPS ---------------------------------*/
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

    SET_PWM_STEP_UP(0,programStruct.control.remote.pwmArray[0]);
    SET_PWM_STEP_UP(1,programStruct.control.remote.pwmArray[1]);
    SET_PWM_STEP_UP(2,programStruct.control.remote.pwmArray[2]);
    SET_PWM_STEP_DOWN(3,programStruct.control.remote.pwmArray[3]);
    SET_PWM_STEP_UP(4,programStruct.control.remote.pwmArray[4]);
    SET_PWM_STEP_DOWN(5,programStruct.control.remote.pwmArray[5]);

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


/*----------------------------- STEPS END ---------------------------------*/
/*----------------------------- STEPS END ---------------------------------*/
/*----------------------------- STEPS END ---------------------------------*/

//------------  ФУНКЦИИ   ------------//
void Program_start()
{
    protocolMbRtuSlaveCtrl_init(1);
    protocolMbRtuSlaveCtrl_init(2);

    // Загрузка параметров
    Program_ParamSetToDefault();
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
    bsp_writeSinBuf(programStruct.sin.f_pwm, programStruct.sin.f_out, programStruct.sin.sinBuf);

    asm("NOP");
    Program_switchTarget(target_waitOp);
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
void Program_ParamSetToDefault()
{
    for (uint8_t i = 0; i < PRG_ANALOG_COUNT; i++)
    {
        programStruct.setupParam.analog_av_order[i] = 12;  // 1 - фильтр отключен
        programStruct.setupParam.analog_filter_N[i] = 1; // 1 - Фильтр отключен

        programStruct.setupParam.analog_kMul[i] = 1.0f;
        programStruct.setupParam.analog_shift[i] = 0;
    }

    return;
}

#define PROGRAM_PARAM_SIZE_BYTE sizeof(Program_PARAM_typedef)
uint8_t Program_ParamLoad()
{
   programStruct.sys.flash_counter = FlashWorker_getAvailableRecords(PROGRAM_PARAM_SIZE_BYTE);
   return FlashWorker_load((void*)(&programStruct.setupParam),PROGRAM_PARAM_SIZE_BYTE);
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
        FlashWorker_save((void *)(&programStruct.setupParam), PROGRAM_PARAM_SIZE_BYTE);
        programStruct.sys.flash_counter = FlashWorker_getAvailableRecords(PROGRAM_PARAM_SIZE_BYTE);
        return 1;
    }
    return 0;
}

__INLINE uint8_t Program_GoDebug()
{
    if (
        (programStruct.control.step != step_wait_op) &&
        (programStruct.control.step != step_error))
    {
        return 0;
    }
    /*
        Перед отладкой обнулить REMOTE структуру!
    */
    programStruct.control.remote.dout.w16 = 0;
    for (uint8_t i = 0; i < 6; i++)
    {
        programStruct.control.remote.pwmArray[i] = 0;
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

#define PWM_REMOTE_MAX_PERCENT_STEP_UP_X10 (600)
#define PWM_REMOTE_MAX_PERCENT (1000)
__INLINE uint8_t Program_set_pwm_debug(uint8_t channel_IDx, uint16_t pwm1000Perc)
{

    if (channel_IDx > 5)
        return 0;

    if (programStruct.control.step == step_debug)
    {

        if (channel_IDx < 3)
        {
            if (pwm1000Perc > PWM_REMOTE_MAX_PERCENT_STEP_UP_X10)
                pwm1000Perc = PWM_REMOTE_MAX_PERCENT_STEP_UP_X10;
        }
        else
        {
            if (pwm1000Perc > PWM_REMOTE_MAX_PERCENT)
                pwm1000Perc = PWM_REMOTE_MAX_PERCENT;
        }
        programStruct.control.remote.pwmArray[channel_IDx] = pwm1000Perc;
        return 1;
    }
    return 0;
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

    if (programStruct.setupParam.protect_control & (uint64_t)(1 << (error - 1)))
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
        bsp_pwm_enable_out_VT_CONCEPT(0, bsp_pwm_outs_type_low);
        bsp_pwm_enable_out_VT_CONCEPT(1, bsp_pwm_outs_type_low);
        bsp_pwm_enable_out_VT_CONCEPT(2, bsp_pwm_outs_type_low);
    }
    else
    {
        bsp_pwm_enable_out_VT_CONCEPT(3, bsp_pwm_outs_type_high);
        bsp_pwm_enable_out_VT_CONCEPT(4, bsp_pwm_outs_type_low);
        bsp_pwm_enable_out_VT_CONCEPT(5, bsp_pwm_outs_type_high);
    }
}

__STATIC_INLINE void Program_pwmInit()
{
  bsp_pwm_set_tim(bsp_pwm_tim_mode_up, 300, BSP_PWM_POSITIVE_POLARITY);

  bsp_pwm_set_freq(bsp_pwm_outs_group_123,bsp_pwm_freq_4000_hz, 1);
  bsp_pwm_set_freq(bsp_pwm_outs_group_456,bsp_pwm_freq_4000_hz, 1);

  SET_PWM_STEP_UP(0,0.0f);
  SET_PWM_STEP_UP(1,0.0f);
  SET_PWM_STEP_UP(2,0.0f);

  SET_PWM_STEP_DOWN(3,0.0f);
  SET_PWM_STEP_UP(4,0.0f);
  SET_PWM_STEP_DOWN(5,0.0f);

  bsp_pwm_start_IRQ_123_IRQ_456();
}

__STATIC_INLINE void Program_regulatorInit(){

//--------------------  VODOROD -----------------------//
//--------------------  REG U -----------------------//
    // programStruct.control.sau.vodorod_RegU.k_Int = programStruct.setupParam.vodorod_RegU_ki;
    // programStruct.control.sau.vodorod_RegU.k_P = programStruct.setupParam.vodorod_RegU_kp;
    // programStruct.control.sau.vodorod_RegU.period = 0.00025f;

    // programStruct.control.sau.vodorod_RegU.IntMin = 0.0f;
    // programStruct.control.sau.vodorod_RegU.IntMax = programStruct.setupParam.vodorod_RegU_MAX;
    // programStruct.control.sau.vodorod_RegU.OutMin = 0.0f;
    // programStruct.control.sau.vodorod_RegU.OutMax = programStruct.setupParam.vodorod_RegU_MAX;
    // dsp_intensSetterSetup(&programStruct.control.sau.vodorod_ZI, programStruct.setupParam.vodorod_ZI, 0.00025f);
//--------------------  REG U END-----------------------//

//--------------------  REG I -----------------------//
    // for (uint8_t i = 0; i < 3; i++)
    // {
    //     programStruct.control.sau.vodorod_RegI[i].k_Int = programStruct.setupParam.vodorod_RegI_ki;
    //     programStruct.control.sau.vodorod_RegI[i].k_P = programStruct.setupParam.vodorod_RegI_kp;
    //     programStruct.control.sau.vodorod_RegI[i].period = 0.00025f;

    //     programStruct.control.sau.vodorod_RegI[i].IntMin = 0.0f;
    //     programStruct.control.sau.vodorod_RegI[i].IntMax = programStruct.setupParam.vodorod_RegI_MAX;
    //     programStruct.control.sau.vodorod_RegI[i].OutMin = 0.0f;
    //     programStruct.control.sau.vodorod_RegI[i].OutMax = programStruct.setupParam.vodorod_RegI_MAX;
    // }
//--------------------  REG I END-----------------------//
//--------------------  VODOROD END-----------------------//


//--------------------  ZU -----------------------//

//--------------------  REG U -----------------------//
//     programStruct.control.sau.ZU_RegU.In = programStruct.setupParam.ZU_RegU_in;
//     programStruct.control.sau.ZU_RegU.k_Int = programStruct.setupParam.ZU_RegU_ki;
//     programStruct.control.sau.ZU_RegU.k_P = programStruct.setupParam.ZU_RegU_kp;
//     programStruct.control.sau.ZU_RegU.period = 0.00025f;

//     programStruct.control.sau.ZU_RegU.IntMin = 0.0f;
//     programStruct.control.sau.ZU_RegU.IntMax = programStruct.setupParam.ZU_RegU_MAX;
//     programStruct.control.sau.ZU_RegU.OutMin = 0.0f;
//     programStruct.control.sau.ZU_RegU.OutMax = programStruct.setupParam.ZU_RegU_MAX;
// //--------------------  REG U END-----------------------//

// //--------------------  REG I -----------------------//
//     programStruct.control.sau.ZU_RegI.In = programStruct.setupParam.ZU_RegU_MAX;
//     programStruct.control.sau.ZU_RegI.k_Int = programStruct.setupParam.ZU_RegI_ki;
//     programStruct.control.sau.ZU_RegI.k_P = programStruct.setupParam.ZU_RegI_kp;
//     programStruct.control.sau.ZU_RegI.period = 0.00025f;

//     programStruct.control.sau.ZU_RegI.IntMin = 0.0f;
//     programStruct.control.sau.ZU_RegI.IntMax = programStruct.setupParam.ZU_RegI_MAX;
//     programStruct.control.sau.ZU_RegI.OutMin = 0.0f;
//     programStruct.control.sau.ZU_RegI.OutMax = programStruct.setupParam.ZU_RegI_MAX;
//--------------------  REG I END-----------------------//
//--------------------  ZU END-----------------------//

}

uint8_t Program_set_pwmOuts_debug(bsp_pwm_outs_group_typedef group, uint8_t onOff){

    if ( programStruct.control.step != step_debug )
    {
        return 0;
    }

    if(group == bsp_pwm_outs_group_123){
            programStruct.control.remote.pwmEnable123 = onOff;
    }else if(group == bsp_pwm_outs_group_456){
            programStruct.control.remote.pwmEnable456 = onOff;
    }else{
        return 0;
    }
    return 1;
}

// Работа повышающего преобразователя
#define PRG_STEP_UP_OUT_1 (0)
#define PRG_VODOROD_CH1_UP (0)
#define PRG_VODOROD_CH2_UP (1)
#define PRG_VODOROD_CH3_UP (2)
#define PRG_K_POWER (1000.0f)

// Нужны были для отладки (не раскоменчивать!!!)
//#define PRG_TEST_PP_ZAMKN_U
//#define PRG_TEST_PP_ZAMKN_I

void bsp_pwm_123_callback()
{

    if (programStruct.control.step == step_debug)
    {
        return;
    }

    return;
}

// Работа зарядника
#define PRG_STEP_DOWN_OUT_1 (3)
#define PRG_ZU_CH1_DOWN (3)
#define PRG_ZU_CH2_UP (4)

void bsp_pwm_456_callback(){

    if (programStruct.control.step == step_debug)
    {
        return;
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
    programStruct.setupParam.analog_shift[idx] = programStruct.analog.aIn[idx].shift;
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
    programStruct.setupParam.analog_kMul[idx] = programStruct.analog.aIn[idx].kMul;
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

    programStruct.setupParam.analog_shift[idx] = value;
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

    programStruct.setupParam.analog_kMul[idx] = value;
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
    programStruct.setupParam.analog_av_order[idx] = order;
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
    programStruct.setupParam.analog_filter_N[idx] = filterN;
    return 1;
}

__STATIC_INLINE uint8_t Program_analogInit(){

    for (uint8_t i = 0; i < PRG_ANALOG_COUNT; i++)
    {
        programStruct.analog.aIn[i].bspIdx = -1;
        programStruct.analog.aIn[i].order = programStruct.setupParam.analog_av_order[i];
        programStruct.analog.aIn[i].analogFilterN = programStruct.setupParam.analog_filter_N[i];
        programStruct.analog.aIn[i].kMul = programStruct.setupParam.analog_kMul[i];
        programStruct.analog.aIn[i].shift = programStruct.setupParam.analog_shift[i];
    }

    programStruct.analog.aIn[prg_analog_AI0].bspIdx = 0;
    programStruct.analog.aIn[prg_analog_AI1].bspIdx = 1;
    programStruct.analog.aIn[prg_analog_AI2].bspIdx = 2;
    programStruct.analog.aIn[prg_analog_AI3].bspIdx = 3;
    programStruct.analog.aIn[prg_analog_AI4].bspIdx = 4;
    programStruct.analog.aIn[prg_analog_AI5].bspIdx = 5;
    programStruct.analog.aIn[prg_analog_AI6].bspIdx = 6;
    programStruct.analog.aIn[prg_analog_AI7].bspIdx = 7;
    programStruct.analog.aIn[prg_analog_AI8].bspIdx = 8;
    programStruct.analog.aIn[prg_analog_AI9].bspIdx = 9;
    programStruct.analog.aIn[prg_analog_AI10].bspIdx = 10;

    bsp_analogIn_start();

    return 1;
}

#define FFT_0HZ    (0)
#define FFT_50HZ   (1)
#define FFT_400HZ  (2)
void bsp_analogIn_ready_callback(){

    uint8_t count = PRG_ANALOG_COUNT;
    float value = 0.0f;
    float valueLast = 0.0f;
    float kFilter = 0.0f;
    float data = 0.0f;

    asm("NOP");

        for (uint8_t ch = 0; ch < count; ch++)
        {
            if(programStruct.analog.aIn[ch].bspIdx == -1 ) continue;

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

            value = (programStruct.analog.aIn[ch].valueRaw  - programStruct.analog.aIn[ch].shift) *\
                                                 programStruct.analog.aIn[ch].kMul;

            valueLast = programStruct.analog.aIn[ch].valueLast;
            // Формула: Yavg(i) = Yavg(i-1) + a * ( X(i) - Yavg(i-1) );
            // a = 2/(N + 1) -> Коэффициент фильтра;
            // N -> количество точек для усреднения, N >= 1;
            // N = 1 -> фильтр отключен;
            kFilter = 2.0f / ((float)programStruct.analog.aIn[ch].analogFilterN + 1.0f);
            value = valueLast + kFilter*(value - valueLast);

            programStruct.analog.aIn[ch].value = value;
            programStruct.analog.aIn[ch].valueLast = value;
        }
}
//------------   АЦП End  ------------//

