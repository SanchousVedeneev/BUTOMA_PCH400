

#include "App.h"

#include "BSP.h"
#include "Program.h"

extern bsp_dInOut_typedef bsp_dInOut_struct;

static void app_hardware_init(void);

void app_main(void)
{

  app_hardware_init();

  Program_start();


  while (1) // основной цикл
  {


    // bsp_rs485_sendTestBlock(1);
    // bsp_rs485_sendTestBlock(2);
    //bsp_dInOut_readWrite_dma(&bsp_dInOut_struct);
    
    // protocolMbRtuSlaveCtrl_update_tables();

    // BSP_LED_TOGGLE(BSP_LED_RDY);
    bsp_fft_start_conv();
    //HAL_Delay(500);
  }
}

static void app_hardware_init(void)
{
  bsp_dInOut_readWrite_hw_start();
  bsp_fft_init();
  bsp_cordic_init();
  HAL_Delay(10);
  return;
}









