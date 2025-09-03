#ifndef __BSP__H__
#define __BSP__H__

#include "main.h"
#include "BSP_dInOut.h"
#include "BSP_analogIn.h"
#include "BSP_PWM.h"

#include "DSP_K.h"
#include "Buf2.h"



//-------------------- LEDS GPIO-------------
#define BSP_LED_LINK        	O_D_Led_Link_GPIO_Port, O_D_Led_Link_Pin
#define BSP_LED_RDY       		O_D_Led_Ready_GPIO_Port, O_D_Led_Ready_Pin
#define BSP_LED_FAULT       	O_D_Led_Fault_GPIO_Port, O_D_Led_Fault_Pin

#define BSP_LED_ON(LED)			LL_GPIO_SetOutputPin(LED)
#define BSP_LED_OFF(LED)		LL_GPIO_ResetOutputPin(LED)
#define BSP_LED_TOGGLE(LED)		LL_GPIO_TogglePin(LED)
//-------------------- LEDS GPIO END-------------


// ----------------------------- RS-485 -----------------------------

void bsp_rs485_setPortToModbusRtu(uint8_t portNo, uint8_t *bufRxTX, uint16_t bufSizeByte);

void bsp_rs485_sendBlock(uint8_t portNo, uint8_t *buf, uint8_t bufSizeByte);
void bsp_rs485_sendTestBlock(uint8_t portNo);

void bsp_rs485_callback_rxBlockReady(uint8_t portNo);
void bsp_rs485_callback_rxTimeout(uint8_t portNo);

#define BSP_RS485_1_IRQ_HANDLER_RTOF 			USART1_IRQHandler
#define BSP_RS485_1_IRQ_HANDLER_DMA_RX 			DMA1_Channel3_IRQHandler
#define BSP_RS485_1_IRQ_HANDLER_DMA_TX 			DMA1_Channel4_IRQHandler

#define BSP_RS485_2_IRQ_HANDLER_RTOF 			USART3_IRQHandler
#define BSP_RS485_2_IRQ_HANDLER_DMA_RX 			DMA1_Channel1_IRQHandler
#define BSP_RS485_2_IRQ_HANDLER_DMA_TX 			DMA1_Channel2_IRQHandler

// ----------------------------- RS-485 END-----------------------------

// ----------------------------- TIMING -----------------------------
#define BSP_SYS_TICK_1K_TIMER 		TIM17
#define BSP_SYS_TICK_1K_IRQ_HANDLER TIM1_TRG_COM_TIM17_IRQHandler
void bsp_sys_tick1k_start();
void bsp_sys_tick_1k_callback();
// ----------------------------- TIMING END-----------------------------

// ----------------------------- FFT -----------------------------
void bsp_fft_init();
void bsp_fft_start_conv();
void bsp_get_fft();
void bsp_dsp_wait_bufs_complete();
void bsp_dsp_release_bufs();
// --------------------------- FFT END ---------------------------

// ----------------------------- CORDIC -----------------------------
int32_t bsp_float_to_q1_31(float in);
float bsp_q1_31_to_float(int32_t in);
uint16_t bsp_writeSinBuf(uint16_t f_pwm, uint16_t f_out);
void bsp_cordic_init();


// --------------------------- CORDIC END ---------------------------

// ----------------------------- SYS --------------------------------
void bsp_sys_reset();

// ----------------------------- SYS END-----------------------------

#endif