#ifndef __BSP_ANALOG_IN__H__
#define __BSP_ANALOG_IN__H__

#include "main.h"

typedef struct
{
    int16_t rawDataUI[12];
    float   rawDataUI_FFT[12][3];
    uint16_t rawDataTemp[2];
    float currentTemp[2];
    uint16_t rawDataCpuTemp;
}bsp_analogIn_typedef;


void bsp_analogIn_start();

float bsp_analogIn_getTemp(uint8_t channel);

#define BSP_ANALOG_IRQ_HANDLER          ADC3_IRQHandler
#define BSP_ANALOG_CLEAR_FLAG()         LL_ADC_ClearFlag_JEOS(ADC3)

#define BSP_ANALOG_TIM                  TIM3
#define BSP_ANALOG_IRQ_HANDLER_TIM      TIM3_IRQHandler

void bsp_analogIn_ready_callback();


#endif