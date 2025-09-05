#ifndef __BSP_PWM__H__
#define __BSP_PWM__H__


#include "main.h"

//-------------------------------------------------------------------------
// В соответствии со схемой
#define BSP_PWM_TIM_PWM_1 (LL_HRTIM_TIMER_F)
#define BSP_PWM_TIM_PWM_2 (LL_HRTIM_TIMER_C)
#define BSP_PWM_TIM_PWM_3 (LL_HRTIM_TIMER_D)
#define BSP_PWM_TIM_PWM_4 (LL_HRTIM_TIMER_B)
#define BSP_PWM_TIM_PWM_5 (LL_HRTIM_TIMER_E)
#define BSP_PWM_TIM_PWM_6 (LL_HRTIM_TIMER_A)


#define BSP_PWM_TIM_OUTS_123_HIGH (LL_HRTIM_OUTPUT_TF1 | LL_HRTIM_OUTPUT_TC1 | LL_HRTIM_OUTPUT_TD1)
#define BSP_PWM_TIM_OUTS_123_LOW  (LL_HRTIM_OUTPUT_TF2 | LL_HRTIM_OUTPUT_TC2 | LL_HRTIM_OUTPUT_TD2)

#define BSP_PWM_TIM_OUTS_456_HIGH (LL_HRTIM_OUTPUT_TB1 | LL_HRTIM_OUTPUT_TE1 | LL_HRTIM_OUTPUT_TA1)
#define BSP_PWM_TIM_OUTS_456_LOW  (LL_HRTIM_OUTPUT_TB2 | LL_HRTIM_OUTPUT_TE2 | LL_HRTIM_OUTPUT_TA2)

//-------------------------------------------------------------------------

//-------------------------------------------------------------------------
#define BSP_PWM_SET_PERIOD(TIM, ARR) LL_HRTIM_TIM_SetPeriod(HRTIM1, TIM, ARR);

#define BSP_PWM_SET_PERIOD_VT1(ARR) BSP_PWM_SET_PERIOD(BSP_PWM_TIM_PWM_1, ARR);
#define BSP_PWM_SET_PERIOD_VT2(ARR) BSP_PWM_SET_PERIOD(BSP_PWM_TIM_PWM_2, ARR);
#define BSP_PWM_SET_PERIOD_VT3(ARR) BSP_PWM_SET_PERIOD(BSP_PWM_TIM_PWM_3, ARR);
#define BSP_PWM_SET_PERIOD_VT4(ARR) BSP_PWM_SET_PERIOD(BSP_PWM_TIM_PWM_4, ARR);
#define BSP_PWM_SET_PERIOD_VT5(ARR) BSP_PWM_SET_PERIOD(BSP_PWM_TIM_PWM_5, ARR);
#define BSP_PWM_SET_PERIOD_VT6(ARR) BSP_PWM_SET_PERIOD(BSP_PWM_TIM_PWM_6, ARR);
//-------------------------------------------------------------------------

//-------------------------------------------------------------------------
#define BSP_PWM_SET_CNT(TIM, CNT) LL_HRTIM_TIM_SetCounter(HRTIM1, TIM, (CNT));

#define BSP_PWM_SET_CNT_VT1(CNT) BSP_PWM_SET_CNT(BSP_PWM_TIM_PWM_1, (CNT));
#define BSP_PWM_SET_CNT_VT2(CNT) BSP_PWM_SET_CNT(BSP_PWM_TIM_PWM_2, (CNT));
#define BSP_PWM_SET_CNT_VT3(CNT) BSP_PWM_SET_CNT(BSP_PWM_TIM_PWM_3, (CNT));
#define BSP_PWM_SET_CNT_VT4(CNT) BSP_PWM_SET_CNT(BSP_PWM_TIM_PWM_4, (CNT));
#define BSP_PWM_SET_CNT_VT5(CNT) BSP_PWM_SET_CNT(BSP_PWM_TIM_PWM_5, (CNT));
#define BSP_PWM_SET_CNT_VT6(CNT) BSP_PWM_SET_CNT(BSP_PWM_TIM_PWM_6, (CNT));
//-------------------------------------------------------------------------

#define BSP_PWM_START_ALL_CNT()  LL_HRTIM_TIM_CounterEnable(HRTIM1,	\
														  LL_HRTIM_TIMER_A \
														| LL_HRTIM_TIMER_B \
														| LL_HRTIM_TIMER_C \
														| LL_HRTIM_TIMER_D \
														| LL_HRTIM_TIMER_E \
														| LL_HRTIM_TIMER_F \
														);

#define BSP_PWM_STOP_ALL_CNT()  LL_HRTIM_TIM_CounterDisable(HRTIM1,	\
														  LL_HRTIM_TIMER_A \
														| LL_HRTIM_TIMER_B \
														| LL_HRTIM_TIMER_C \
														| LL_HRTIM_TIMER_D \
														| LL_HRTIM_TIMER_E \
														| LL_HRTIM_TIMER_F \
														);

#define BSP_PWM_SET_CCR(TIM,PWM)	(LL_HRTIM_TIM_SetCompare1(HRTIM1, TIM, PWM))

#define BSP_PWM_SET_PWM_1(PWM)	BSP_PWM_SET_CCR(BSP_PWM_TIM_PWM_1, PWM)
#define BSP_PWM_SET_PWM_2(PWM)	BSP_PWM_SET_CCR(BSP_PWM_TIM_PWM_2, PWM)
#define BSP_PWM_SET_PWM_3(PWM)	BSP_PWM_SET_CCR(BSP_PWM_TIM_PWM_3, PWM)
#define BSP_PWM_SET_PWM_4(PWM)	BSP_PWM_SET_CCR(BSP_PWM_TIM_PWM_4, PWM)
#define BSP_PWM_SET_PWM_5(PWM)	BSP_PWM_SET_CCR(BSP_PWM_TIM_PWM_5, PWM)
#define BSP_PWM_SET_PWM_6(PWM)	BSP_PWM_SET_CCR(BSP_PWM_TIM_PWM_6, PWM)

//---------------------------- IRQ HANDLERS ----------------------------
#define BSP_PWM_IRQ_HANDLER_123         HRTIM1_TIMD_IRQHandler
#define BSP_PWM_IRQ_CLEAR_FLAG_123()                  \
                                        LL_HRTIM_ClearFlag_REP(HRTIM1, LL_HRTIM_TIMER_D); \
                                        LL_HRTIM_ClearFlag_UPDATE(HRTIM1, LL_HRTIM_TIMER_D);



#define BSP_PWM_IRQ_HANDLER_456         HRTIM1_TIMA_IRQHandler
#define BSP_PWM_IRQ_CLEAR_FLAG_456()                  \
                                        LL_HRTIM_ClearFlag_REP(HRTIM1, LL_HRTIM_TIMER_A); \
                                        LL_HRTIM_ClearFlag_UPDATE(HRTIM1, LL_HRTIM_TIMER_A);

void bsp_pwm_123_callback();
void bsp_pwm_456_callback();

//---------------------------- IRQ HANDLERS END----------------------------


// polarity POSITIVE PWM -> LL_HRTIM_OUT_POSITIVE_POLARITY
// polarity NEGATIVE PWM -> LL_HRTIM_OUT_NEGATIVE_POLARITY
#define BSP_PWM_POSITIVE_POLARITY LL_HRTIM_OUT_POSITIVE_POLARITY
#define BSP_PWM_NEGATIVE_POLARITY LL_HRTIM_OUT_NEGATIVE_POLARITY
typedef enum{
    bsp_pwm_tim_mode_up_down,
    bsp_pwm_tim_mode_up
} bsp_pwm_tim_mode_typedef;
void bsp_pwm_set_tim(bsp_pwm_tim_mode_typedef mode, uint16_t DT, uint32_t polarity);
void bsp_pwm_start_IRQ_123_IRQ_456();

typedef enum{
    bsp_pwm_outs_group_123,
    bsp_pwm_outs_group_456
} bsp_pwm_outs_group_typedef;

typedef enum{
    bsp_pwm_outs_type_high,
    bsp_pwm_outs_type_low,
    bsp_pwm_outs_type_all
}bsp_pwm_outs_type_typedef;
void bsp_pwm_enable_out_VT_CONCEPT(uint8_t outIdx, bsp_pwm_outs_type_typedef type);
void bsp_pwm_enable_outs_VT_CONCEPT(bsp_pwm_outs_group_typedef group, bsp_pwm_outs_type_typedef type);
void bsp_pwm_disable_outs_VT(bsp_pwm_outs_group_typedef group);

typedef enum{              //up      //up dowm
    bsp_pwm_freq_4000_hz,  //42000   //21000    +
    bsp_pwm_freq_4800_hz,  //35000   //17500    +
    bsp_pwm_freq_5600_hz,  //30000   //15000    +
    bsp_pwm_freq_6000_hz,  //28000   //14000    +
    bsp_pwm_freq_6400_hz,  //26250   //13125     +
    bsp_pwm_freq_8000_hz   //21000   //10500     +
}bsp_pwm_freq_typedef;

uint8_t bsp_pwm_set_freq(bsp_pwm_outs_group_typedef group, bsp_pwm_freq_typedef freq, uint8_t phaseShift);
uint8_t bsp_pwm_set_ccrPercentX10(uint8_t ccrIdx, float valuePercentX10);
uint8_t bsp_pwm_set_ccr(uint8_t ccrIdx, uint32_t ccr);

#endif