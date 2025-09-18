
#ifndef __PROGRAM__H__
#define __PROGRAM__H__

#include "main.h"
#include "DSP.h"
#include "BSP.h"

//Steps
typedef enum{
    step_waitInit = 0,
    step_init,
    step_debug,
    step_reset,
    step_error,
    step_waitOp,
    step_Check_Udc,
    step_Voltage_Up,
    step_Work,
    step_Stop
}Program_STEP_typedef;

//Targets
typedef enum{
    target_debug = 1,
    target_waitOp,
    target_reset,
    target_error,
    target_Work
}Program_TARGET_typedef;

//Errors
typedef enum{
    error_noError = 0,
    error_Udc_L_low,
    error_Udc_L_high,
    error_Udc_R_low,
    error_Udc_R_high,
    error_Uac_U_no_ok,
    error_Uac_V_no_ok,
    error_Uac_W_no_ok,
    error_Uac_voltageUp,
    error_Iac_U_overload,
    error_Iac_V_overload,
    error_Iac_W_overload,
    error_Iac_U_KZ,
    error_Iac_V_KZ,
    error_Iac_W_KZ,
    error_Ttv_high,
    error_Tradiator_high
}Program_ERROR_typedef;


#define PRG_DIN_PUSK_VAL (1)
#define PRG_DIN_STOP_VAL (0)
#define PRG_DIN_KT_ON (0)
typedef enum
{
	prg_din1_Pusk,         // 0
	prg_din2_Stop,         // 1
	prg_din3_TV_KT1,       // 2
	prg_din4_TV_KT2,       // 3
	prg_din5_UNUSED,       // 4
	prg_din6_UNUSED,       // 5
	prg_din7_UNUSED,       // 6
	prg_din8_UNUSED,       // 7
	prg_din9_UNUSED,       // 8
	prg_din10_UNUSED       // 9
} Program_din_typedef;

typedef enum
{
	prg_dout1_LampReady,     // 0
	prg_dout2_LampWork,      // 1
	prg_dout3_LampOverheat,  // 2
	prg_dout4_LampFaulth,    // 3
	prg_dout5_KM1,           // 4
	prg_dout6_KM2,           // 5
	prg_dout7_FanPower,      // 6
	prg_dout8_UNUSED,        // 7
	prg_dout9_UNUSED,        // 8
	prg_dout10_UNUSED        // 9
} Program_dout_typedef;


#define PROGRAM_INV_L (0)
#define PROGRAM_INV_R (1)

#define PROGRAM_OUT_U_INV_L (0)
#define PROGRAM_OUT_V_INV_L (1)
#define PROGRAM_OUT_W_INV_L (2)

#define PROGRAM_OUT_U_INV_R (3)
#define PROGRAM_OUT_V_INV_R (4)
#define PROGRAM_OUT_W_INV_R (5)

#define PROGRAM_FHASE_U (0)
#define PROGRAM_FHASE_V (1)
#define PROGRAM_FHASE_W (2)

#define PROGRAM_FHASE_COUNT (3)
typedef struct           // U1 - U2
{
    uint8_t invNo1;      // 0 -> L ; 1 -> R
    uint8_t channelNo1;  // [0 1 2] -> [U V W]
    uint8_t invNo2;      // 0 -> L ; 1 -> R
    uint8_t channelNo2;  // [3 4 5] -> [U V W]
    float k_modOut;
    float *voltageFb;
}Program_PHASE_typedef;

#define COUNT_CHANNEL_PWM (6)
#define REMOTE_KMOD (0)
#define REMOTE_KPWM (1)
typedef struct
{
    union
    {
        uint16_t w16;
        struct
        {
            uint8_t dout1 : 1;
            uint8_t dout2 : 1;
            uint8_t dout3 : 1;
            uint8_t dout4 : 1;
            uint8_t dout5 : 1;
            uint8_t dout6 : 1;
            uint8_t dout7 : 1;
            uint8_t dout8 : 1;
            uint8_t dout9 : 1;
            uint8_t dout10 : 1;
            uint8_t        : 6;
        }bits; 
    } dout;
    uint8_t pwmEnable123;
    uint8_t pwmEnable456;
    uint8_t choise_kPWM_or_kMod;
    float kPWM[COUNT_CHANNEL_PWM];
    float k_modIn;  // 0..1
    // float voltageIn;
    // float currentIn;
} Program_REMOTE_typedef;

typedef struct
{
    dsp_regulator_typedef voltageRegulator[3];
    float voltageIn;
    dsp_intensSetter_typedef ZI;
}Program_SAU_typedef;

#define PROGRAM_ADC_MAX_FILTER_ORDER (12)
#define PROGRAM_ADC_MAX_FILTER_N     (255)
typedef struct
{
       float value;
       float valueLast;
       float valueRaw;
       float buf[PROGRAM_ADC_MAX_FILTER_ORDER];
       uint8_t bufIdx;
       float shift;
       float kMul;
       uint8_t order;
       uint16_t analogFilterN;
       int8_t bspIdx;
}Program_AIN_typedef;

typedef enum{
    prg_analog_I_u,
    prg_analog_I_v,
    prg_analog_I_w,
    prg_analog_U_u,
    prg_analog_U_v,
    prg_analog_U_w,
    prg_analog_Udc_invL,
    prg_analog_Udc_invR,
    prg_analog_AI8,
    prg_analog_AI9,
    prg_analog_AI10
}Program_ANALOG_ENUM_typedef;

#define PRG_ANALOG_COUNT (prg_analog_AI10 + 1)

typedef struct
{
    Program_AIN_typedef aIn[PRG_ANALOG_COUNT];
}Program_ANALOG_typedef;

typedef struct
{
    uint16_t f_pwm;
    uint16_t f_out;
    uint16_t bufLen;
    float sinBuf[3][320];
    uint16_t currentIdx;
} Program_SIN_typedef;

typedef struct
{
    Program_STEP_typedef step; // Current step (dc charge, work ...)
    Program_TARGET_typedef target; // Current target (start, stop ...)
    Program_ERROR_typedef errorCode;
    Program_REMOTE_typedef remote;
    Program_SAU_typedef sau;
}Program_CONTROL_typedef;

typedef struct
{
       int16_t flash_counter;
}Program_SYS_typedef;

/*
    Структура сохраняется во Flash память 
    с помощью FlashSaver после команды SaveParam
    в отладочном режиме
*/
typedef struct
{
    uint16_t analog_shift   [PRG_ANALOG_COUNT]; 
    float    analog_kMul    [PRG_ANALOG_COUNT]; 
    uint8_t  analog_av_order[PRG_ANALOG_COUNT]; 
    uint16_t analog_filter_N[PRG_ANALOG_COUNT];

    uint16_t protect_control; // по умолчанию =0
    uint8_t  phaseCount;
    uint16_t f_out;
    uint16_t U_out;
    uint16_t PWM_freq;

    float    RegU_kp[PROGRAM_FHASE_COUNT];
    float    RegU_ki[PROGRAM_FHASE_COUNT];
    float    RegU_max[PROGRAM_FHASE_COUNT];

    uint16_t Udc_low;           // V
    uint16_t Udc_high;          // V
    float    Uac_no_ok_percent; // %
    uint16_t Iac_nominal;       // A
    uint16_t Tradiator_high;    // C
} Program_PARAM_typedef;

typedef struct
{
    Program_CONTROL_typedef control;
    Program_PARAM_typedef setup;
    Program_SYS_typedef sys;
    Program_ANALOG_typedef analog;
    Program_SIN_typedef sin;
    Program_PHASE_typedef phase[PROGRAM_FHASE_COUNT];
}Program_typedef;

void Program_start();

Program_AIN_typedef* Program_analogGetByIdx(Program_ANALOG_ENUM_typedef idx);
uint8_t Program_analogSetZero(Program_ANALOG_ENUM_typedef idx);
uint8_t Program_analogCalibKMul(Program_ANALOG_ENUM_typedef idx, float value);
uint8_t Program_analogSetShift(Program_ANALOG_ENUM_typedef idx, float value);
uint8_t Program_analogSetKMul(Program_ANALOG_ENUM_typedef idx, float value);
uint8_t Program_analogSetAvOrder(Program_ANALOG_ENUM_typedef idx, uint8_t order);
uint8_t Program_analogSetFilterN(Program_ANALOG_ENUM_typedef idx, uint16_t filterN);

void Program_ParamSetToDefault();
uint8_t Program_ParamLoad();
uint8_t Program_ParamSave();
void Program_switchTarget(Program_TARGET_typedef newTarget);
uint8_t Program_GoDebug();

uint8_t Program_set_dout_debug(uint16_t douts);
uint8_t Program_set_pwm_debug(uint8_t channel_IDx, uint16_t pwm1000Perc);
uint8_t Program_set_pwmOuts_debug(bsp_pwm_outs_group_typedef group, uint8_t onOff);
uint8_t Program_LoadDefaultParam_debug();

uint8_t Program_set_k_mod_debug (uint16_t kMod);
uint8_t Program_сhoice_kPWM_or_kMod_debug (uint16_t choise);
uint8_t Program_set_phaseCount_debug (uint16_t phaseCount);
uint8_t Program_set_fOut_debug (uint16_t fOut);
uint8_t Program_set_uOut_debug (uint16_t uOut);
uint8_t Program_set_PWM_freq_debug (uint16_t PWM_freq);

uint8_t Program_set_regul_kp (uint8_t phase_idx, float kp);
uint8_t Program_set_regul_ki (uint8_t phase_idx, float ki);
uint8_t Program_set_regul_uOut_max (uint8_t phase_idx, float uOut_max);
uint8_t Program_set_ZI (uint8_t ZI);

uint8_t Program_set_Uac_no_ok_percent (float Uac_no_ok);
uint8_t Program_set_Iac_nominal (uint16_t Iac_nominal);

void Program_sinBuf_init();
void Program_phase_init();

uint8_t Program_GoReset();


#define setBit  (reg, bit)   (reg |=  (1 << bit))
#define resetBit(reg, bit)   (reg &= ~(1 << bit))

#endif