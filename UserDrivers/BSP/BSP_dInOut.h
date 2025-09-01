#ifndef __BSP_DINOUT__H__
#define __BSP_DINOUT__H__

#include "main.h"

// ------------------------------ SPI DIN DOUT ------------------------------
#define BSP_DINOUT_O_D_SOEN O_T_SPI_SOEN_GPIO_Port, O_T_SPI_SOEN_Pin
#define BSP_DINOUT_O_D_IN_E O_T_SPI_IN_E_GPIO_Port, O_T_SPI_IN_E_Pin

typedef struct
{
	uint8_t bufTX[6];
	uint8_t bufRX[6];
	union
	{
		uint16_t w16;
		uint8_t w8[2];
		struct
		{
			//---------- 0 -----------//
			uint8_t vts1 : 1; // 0
			uint8_t vts2 : 1; // 1
			uint8_t vts3 : 1; // 2
			uint8_t vts4 : 1; // 3
			uint8_t vts5 : 1; // 4
			uint8_t vts6 : 1; // 5
			uint8_t in1 : 1;  // 6
			uint8_t in2 : 1;  // 7
			//----------- 1 ----------//
			uint8_t in3 : 1;  // 0
			uint8_t in4 : 1;  // 1
			uint8_t in5 : 1;  // 2
			uint8_t in6 : 1;  // 3
			uint8_t in7 : 1;  // 4
			uint8_t in8 : 1;  // 5
			uint8_t in9 : 1;  // 6
			uint8_t in10 : 1; // 7
		} bits;
	} in;

	union
	{
		uint16_t w16[3];
		uint8_t w8[6];
		struct outBits
		{
			//---------- byte 0 word 0	-----------//
			uint8_t led_a1_y : 1; // 0
			uint8_t led_a1_g : 1; // 1
			uint8_t led_a2_y : 1; // 2
			uint8_t led_a2_g : 1; // 3
			uint8_t led_a3_y : 1; // 4
			uint8_t led_a3_g : 1; // 5
			uint8_t led_a4_y : 1; // 6
			uint8_t led_a4_g : 1; // 7
			//---------- byte 1 word 0	-----------//
			uint8_t led_a5_y : 1; // 0
			uint8_t led_a5_g : 1; // 1
			uint8_t led_a6_y : 1; // 2
			uint8_t led_a6_g : 1; // 3
			uint8_t led_a7_y : 1; // 4
			uint8_t led_a7_g : 1; // 5
			uint8_t led_a8_y : 1; // 6
			uint8_t led_a8_g : 1; // 7
			//---------- byte 2 word 1	-----------//
			uint8_t led_a9_y : 1;  // 0
			uint8_t led_a9_g : 1;  // 1
			uint8_t led_a10_y : 1; // 2
			uint8_t led_a10_g : 1; // 3
			uint8_t led_a11_y : 1; // 4
			uint8_t led_a11_g : 1; // 5
			uint8_t led_a12_y : 1; // 6
			uint8_t led_a12_g : 1; // 7
			//---------- byte 3 word 1	-----------//
			uint8_t led_rs485_1_g : 1; // 0
			uint8_t led_rs485_1_y : 1; // 1
			uint8_t led_rs485_2_g : 1; // 2
			uint8_t led_rs485_2_y : 1; // 3
			uint8_t led_can_1_g : 1;   // 4
			uint8_t led_can_1_y : 1;   // 5
			uint8_t led_can_2_g : 1;   // 6
			uint8_t led_can_2_y : 1;   // 7
			//---------- byte 4 word 2	-----------//
			uint8_t out1 : 1; // 0
			uint8_t out2 : 1; // 1
			uint8_t out3 : 1; // 2
			uint8_t out4 : 1; // 3
			uint8_t out5 : 1; // 4
			uint8_t out6 : 1; // 5
			uint8_t out7 : 1; // 6
			uint8_t out8 : 1; // 7
			//---------- byte 5 word 2	-----------//
			uint8_t out9 : 1;  // 0
			uint8_t out10 : 1; // 1
			uint8_t : 6;
		} bits;
	} out;
} bsp_dInOut_typedef;

typedef enum
{
	//---------- 0 -----------//
	bsp_dInOut_led_a1_y = 0, // 0
	bsp_dInOut_led_a1_g,	  // 1
	bsp_dInOut_led_a2_y,	  // 2
	bsp_dInOut_led_a2_g,	  // 3
	bsp_dInOut_led_a3_y,	  // 4
	bsp_dInOut_led_a3_g,	  // 5
	bsp_dInOut_led_a4_y,	  // 6
	bsp_dInOut_led_a4_g,	  // 7
	//---------- 1 -----------//
	bsp_dInOut_led_a5_y, // 0
	bsp_dInOut_led_a5_g, // 1
	bsp_dInOut_led_a6_y, // 2
	bsp_dInOut_led_a6_g, // 3
	bsp_dInOut_led_a7_y, // 4
	bsp_dInOut_led_a7_g, // 5
	bsp_dInOut_led_a8_y, // 6
	bsp_dInOut_led_a8_g, // 7
	//---------- 2 -----------//
	bsp_dInOut_led_a9_y,  // 0
	bsp_dInOut_led_a9_g,  // 1
	bsp_dInOut_led_a10_y, // 2
	bsp_dInOut_led_a10_g, // 3
	bsp_dInOut_led_a11_y, // 4
	bsp_dInOut_led_a11_g, // 5
	bsp_dInOut_led_a12_y, // 6
	bsp_dInOut_led_a12_g, // 7
	//---------- 3 -----------//
	bsp_dInOut_led_rs485_1_g, // 0
	bsp_dInOut_led_rs485_1_y, // 1
	bsp_dInOut_led_rs485_2_g, // 2
	bsp_dInOut_led_rs485_2_y, // 3
	bsp_dInOut_led_can_1_g,   // 4
	bsp_dInOut_led_can_1_y,   // 5
	bsp_dInOut_led_can_2_g,   // 6
	bsp_dInOut_led_can_2_y,   // 7
	//---------- 4 -----------//
	bsp_dInOut_out1, // 0
	bsp_dInOut_out2, // 1
	bsp_dInOut_out3, // 2
	bsp_dInOut_out4, // 3
	bsp_dInOut_out5, // 4
	bsp_dInOut_out6, // 5
	bsp_dInOut_out7, // 6
	bsp_dInOut_out8, // 7
	//---------- 5 -----------//
	bsp_dInOut_out9, // 0
	bsp_dInOut_out10 // 1
} bsp_dInOut_dout_typedef;

#define BSP_DINOUT_DOUT_COUNT (10)

typedef enum
{
	//---------- 0 -----------//
	bsp_dInOut_vts1 = 0, // 0
	bsp_dInOut_vts2,	 // 1
	bsp_dInOut_vts3,	 // 2
	bsp_dInOut_vts4,	 // 3
	bsp_dInOut_vts5,	 // 4
	bsp_dInOut_vts6,	 // 5
	bsp_dInOut_in1,		 // 6
	bsp_dInOut_in2,		 // 7
	//----------- 1 ----------//
	bsp_dInOut_in3, // 0
	bsp_dInOut_in4, // 1
	bsp_dInOut_in5, // 2
	bsp_dInOut_in6, // 3
	bsp_dInOut_in7, // 4
	bsp_dInOut_in8, // 5
	bsp_dInOut_in9, // 6
	bsp_dInOut_in10 // 7
} bsp_dInOut_din_typedef;

uint8_t bsp_dInOut_readDin(bsp_dInOut_din_typedef din);
void bsp_dInOut_setDout(bsp_dInOut_dout_typedef dout);
void bsp_dInOut_resetDout(bsp_dInOut_dout_typedef dout);
void bsp_dInOut_toggleDout(bsp_dInOut_dout_typedef dout);

void bsp_dInOut_setDouts1_10(uint16_t word);

void bsp_dInOut_readWrite_poll();
void bsp_dInOut_resetStruct();
void bsp_dInOut_readWrite_dma(bsp_dInOut_typedef *bsp_dInOut);
void bsp_dInOut_readWrite_hw_start();

#define BSP_dInOut_TIM_SPI TIM2
#define BSP_dInOut_TIM_SPI_START_IRQ_HANDLER TIM2_IRQHandler
// ------------------------------ SPI DIN DOUT END ------------------------------

#endif