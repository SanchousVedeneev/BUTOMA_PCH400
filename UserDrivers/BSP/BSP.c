
#include "BSP.h"

//-------------------------------
#include "tim.h"
#include "spi.h"
#include "usart.h"
#include "cordic.h"

#include "DSP.h"
//-------------------------------


// ------------------------------ DBG MCU ------------------------------

// ------------------------------ DBG MCU END ------------------------------

// ------------------------------ SPI DIN DOUT ------------------------------
/*
 Настройки SPI PSC и время посылки->
 PSC 32     64      128     256
 t   12us   20us    43us    87
*/
bsp_dInOut_typedef bsp_dInOut_struct;
void bsp_dInOut_readWrite_poll()
{

  // HAL_GPIO_WritePin(BSP_DINOUT_O_D_IN_E,GPIO_PIN_SET);
  // HAL_Delay(50);
  // HAL_GPIO_WritePin(BSP_DINOUT_O_D_IN_E,GPIO_PIN_RESET);

  static uint8_t bufRX[6];
  static uint8_t bufTX[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  // spi устройства
  // in_e_strob(); // старт строб
  HAL_GPIO_WritePin(BSP_DINOUT_O_D_IN_E, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(BSP_DINOUT_O_D_IN_E, GPIO_PIN_SET);
  HAL_SPI_TransmitReceive(&hspi3, bufTX, bufRX, 6, 5000);
  HAL_Delay(1);
  asm("NOP");
  // s_oen_strob(); // стоп строб для записи в регистр вывода
  HAL_GPIO_WritePin(BSP_DINOUT_O_D_SOEN, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(BSP_DINOUT_O_D_SOEN, GPIO_PIN_RESET);
  // HAL_Delay(100);
  asm("NOP");
}
void bsp_dInOut_readWrite_dma(bsp_dInOut_typedef *bsp_dInOut)
{

  static uint8_t bufRX[6];
  // static uint8_t bufTX[6];// = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
  static uint8_t bufTX[6];
  for (uint8_t i = 0; i < 6; i++)
  {
    bufTX[i] = bsp_dInOut->out.w8[5 - i];
  }

  // spi устройства
  HAL_GPIO_WritePin(BSP_DINOUT_O_D_IN_E, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(BSP_DINOUT_O_D_IN_E, GPIO_PIN_SET);
  HAL_SPI_TransmitReceive_DMA(&hspi3, bufTX, bufRX, 6);
  HAL_Delay(1);
  asm("NOP");
  HAL_GPIO_WritePin(BSP_DINOUT_O_D_SOEN, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(BSP_DINOUT_O_D_SOEN, GPIO_PIN_RESET);

  bsp_dInOut->in.w8[0] = ~bufRX[1];
  bsp_dInOut->in.w8[1] = ~bufRX[0];

  asm("NOP");
}
void bsp_dInOut_readWrite_hw_start()
{

  /*

    Не работает подход по автоматическим транзакциям DMA

   //static uint8_t bufRX[6];
   //static uint8_t bufTX[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

    // HAL_DMA_Start(hspi3.hdmarx, (uint32_t)&hspi3.Instance->DR, (uint32_t)bufRX, 1);
    // SET_BIT(hspi3.Instance->CR2, SPI_CR2_RXDMAEN);
    // __HAL_SPI_ENABLE(&hspi3);

    //__HAL_TIM_ENABLE_DMA(&htim2,TIM_DMA_CC3);

    //HAL_DMA_Start(htim2.hdma[TIM_DMA_ID_CC3],(uint32_t)bufTX,(uint32_t)&hspi3.Instance->DR,1);
    //HAL_SPI_TransmitReceive_DMA(&hspi3, bufTX, bufRX, 6);
  */

  LL_GPIO_SetOutputPin(O_D_SPI_SR_GPIO_Port, O_D_SPI_SR_Pin);
  HAL_Delay(1);
  bsp_dInOut_resetStruct();

  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3);
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_2);
}
__STATIC_INLINE void bsp_dInOut_parseDataBuf()
{
  for (uint8_t i = 0; i < 6; i++)
  {
    bsp_dInOut_struct.bufTX[i] = bsp_dInOut_struct.out.w8[5 - i];
  }
  bsp_dInOut_struct.in.w8[0] = ~bsp_dInOut_struct.bufRX[1];
  bsp_dInOut_struct.in.w8[1] = ~bsp_dInOut_struct.bufRX[0];
}
void BSP_dInOut_TIM_SPI_START_IRQ_HANDLER()
{

  BSP_dInOut_TIM_SPI->SR &= ~TIM_SR_UIF;
  BSP_dInOut_TIM_SPI->SR &= ~TIM_SR_CC1IF;
  BSP_dInOut_TIM_SPI->SR &= ~TIM_SR_CC2IF;
  BSP_dInOut_TIM_SPI->SR &= ~TIM_SR_CC3IF;
  BSP_dInOut_TIM_SPI->SR &= ~TIM_SR_CC4IF;

  bsp_dInOut_parseDataBuf();
  HAL_SPI_TransmitReceive_DMA(&hspi3, bsp_dInOut_struct.bufTX, bsp_dInOut_struct.bufRX, 6);

  // BSP_LED_TOGGLE(BSP_LED_FAULT);
}
__INLINE void bsp_dInOut_resetStruct()
{

  for (uint8_t i = 0; i < 3; i++)
  {
    bsp_dInOut_struct.out.w16[i] = 0;
  }
}
__INLINE void bsp_dInOut_setDouts1_10(uint16_t word)
{
  bsp_dInOut_struct.out.w16[2] = word;
}
__INLINE void bsp_dInOut_setDout(bsp_dInOut_dout_typedef dout)
{
  uint8_t byte = dout / 8;
  uint8_t bit = dout % 8;

  bsp_dInOut_struct.out.w8[byte] |= (1 << bit);
}
__INLINE void bsp_dInOut_resetDout(bsp_dInOut_dout_typedef dout)
{
  uint8_t byte = dout / 8;
  uint8_t bit = dout % 8;

  bsp_dInOut_struct.out.w8[byte] &= ~(1 << bit);
}
__INLINE void bsp_dInOut_toggleDout(bsp_dInOut_dout_typedef dout)
{
  uint8_t byte = dout / 8;
  uint8_t bit = dout % 8;

  if (bsp_dInOut_struct.out.w8[byte] & ((1 << bit)))
  {
    bsp_dInOut_struct.out.w8[byte] &= ~(1 << bit);
  }
  else
  {
    bsp_dInOut_struct.out.w8[byte] |= (1 << bit);
  }
}
uint8_t bsp_dInOut_readDin(bsp_dInOut_din_typedef din)
{
  uint8_t byte = din / 8;
  uint8_t bit = din % 8;

  return (bsp_dInOut_struct.in.w8[byte] & (1 << bit)) > 0;
}
// ------------------------------ SPI DIN DOUT END ------------------------------

// ------------------------------ RS485 ------------------------------
#define BSP_RS485_1 huart1
#define BSP_RS485_2 huart3
#define BSP_RS_485_RX_TIMEOUT (2000)
static uint16_t timer_rs485_timeout[2] = {BSP_RS_485_RX_TIMEOUT, BSP_RS_485_RX_TIMEOUT};
void bsp_rs485_setPortToModbusRtu(uint8_t portNo, uint8_t *bufRxTX, uint16_t bufSizeByte)
{

  UART_HandleTypeDef *port = NULL;
  HAL_Delay(100);
  if (portNo == 1)
    port = &BSP_RS485_1;
  else
    port = &BSP_RS485_2;
  port->Instance->RTOR |= 0xfff << USART_RTOR_RTO_Pos;
  port->Instance->CR1 |= USART_CR1_RTOIE;
  port->Instance->CR2 |= USART_CR2_RTOEN;
  //__HAL_UART_ENABLE_IT(&huart3, UART_IT_TC);
  port->NbRxDataToProcess = bufSizeByte;
  // port->RxXferSize = bufSizeByte;

  HAL_UART_Abort(port);
  HAL_DMA_Start(port->hdmarx, (uint32_t)&port->Instance->RDR, (uint32_t)bufRxTX, bufSizeByte);
  port->Instance->CR3 |= USART_CR3_DMAR;
}
void bsp_rs485_sendBlock(uint8_t portNo, uint8_t *buf, uint8_t bufSizeByte)
{
  UART_HandleTypeDef *port = NULL;
  if (portNo == 1)
    port = &BSP_RS485_1;
  else
    port = &BSP_RS485_2;
  // HAL_UART_Abort(port);
  port->hdmatx->State = HAL_DMA_STATE_READY;
  __HAL_UNLOCK(port->hdmatx);
  HAL_DMA_Start(port->hdmatx, (uint32_t)buf, (uint32_t)&port->Instance->TDR, bufSizeByte);
  port->Instance->CR3 |= USART_CR3_DMAT;
}
void bsp_rs485_sendTestBlock(uint8_t portNo)
{
  static uint8_t testBuf[5] = {0xAA, 1, 2, 3, 4};
  bsp_rs485_sendBlock(portNo, testBuf, 5);
}
__weak void bsp_rs485_callback_rxBlockReady(uint8_t portNo)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(portNo);
  /*
      Если callback нужен его необходимо переопределить.
      void bsp_rs485_callback_rxBlockReady(UART_HandleTypeDef* port){

      }
      Удалять функцию не надо.
  */
}
__weak void bsp_rs485_callback_rxTimeout(uint8_t portNo)
{
}
void BSP_RS485_1_IRQ_HANDLER_RTOF(void)
{
  // BLOCK RX READY
  if (__HAL_UART_GET_FLAG(&BSP_RS485_1, UART_FLAG_RTOF))
  {
    __HAL_UART_CLEAR_FLAG(&BSP_RS485_1, UART_CLEAR_RTOF);

    __HAL_DMA_DISABLE(BSP_RS485_1.hdmarx);
    __HAL_UART_CLEAR_FLAG(&BSP_RS485_1, UART_CLEAR_OREF);

    // MAIN_LED1_TOGGLE();

    if (BSP_RS485_1.NbRxDataToProcess != BSP_RS485_1.hdmarx->Instance->CNDTR)
    {
      bsp_rs485_callback_rxBlockReady(1);
      bsp_dInOut_toggleDout(bsp_dInOut_led_rs485_1_g);
      bsp_dInOut_resetDout(bsp_dInOut_led_rs485_1_y);
      timer_rs485_timeout[0] = BSP_RS_485_RX_TIMEOUT;
    }

    BSP_RS485_1.hdmarx->Instance->CNDTR = BSP_RS485_1.NbRxDataToProcess;
    __HAL_DMA_ENABLE(BSP_RS485_1.hdmarx);
    BSP_RS485_1.Instance->CR3 |= USART_CR3_DMAR;
  }
  // TRANSFER COMPLETE
  else if (__HAL_UART_GET_FLAG(&BSP_RS485_1, UART_FLAG_TC))
  {
    __HAL_UART_CLEAR_FLAG(&BSP_RS485_1, UART_FLAG_TC);
    asm("NOP");
    if (BSP_RS485_1.hdmatx->Instance->CNDTR == 0)
    {
      // LL_GPIO_ResetOutputPin(O_D_RS485_DE_GPIO_Port,O_D_RS485_DE_Pin);
    }
  }
  else
  {
    Error_Handler();
    NVIC_SystemReset();
  }
}
void BSP_RS485_2_IRQ_HANDLER_RTOF(void)
{
  // BLOCK RX READY
  if (__HAL_UART_GET_FLAG(&BSP_RS485_2, UART_FLAG_RTOF))
  {
    __HAL_UART_CLEAR_FLAG(&BSP_RS485_2, UART_CLEAR_RTOF);

    __HAL_DMA_DISABLE(BSP_RS485_2.hdmarx);
    __HAL_UART_CLEAR_FLAG(&BSP_RS485_2, UART_CLEAR_OREF);
    // MAIN_LED1_TOGGLE();
    if (BSP_RS485_2.NbRxDataToProcess != BSP_RS485_2.hdmarx->Instance->CNDTR)
    {
      bsp_rs485_callback_rxBlockReady(2);
      bsp_dInOut_toggleDout(bsp_dInOut_led_rs485_2_g);
      bsp_dInOut_resetDout(bsp_dInOut_led_rs485_2_y);
      timer_rs485_timeout[1] = BSP_RS_485_RX_TIMEOUT;
    }
    BSP_RS485_2.hdmarx->Instance->CNDTR = BSP_RS485_2.NbRxDataToProcess;
    __HAL_DMA_ENABLE(BSP_RS485_2.hdmarx);
    BSP_RS485_2.Instance->CR3 |= USART_CR3_DMAR;
  }
  // TRANSFER COMPLETE
  else if (__HAL_UART_GET_FLAG(&BSP_RS485_2, UART_FLAG_TC))
  {
    __HAL_UART_CLEAR_FLAG(&BSP_RS485_2, UART_FLAG_TC);
    asm("NOP");
    if (BSP_RS485_2.hdmatx->Instance->CNDTR == 0)
    {
      // LL_GPIO_ResetOutputPin(O_D_RS485_DE_GPIO_Port,O_D_RS485_DE_Pin);
    }
  }
  else
  {
    Error_Handler();
    NVIC_SystemReset();
  }
}
// ------------------------------ RS485 END ------------------------------

// ------------------------------ ANALOG IN ------------------------------
void bsp_analogIn_start()
{

  ADC_TypeDef *adc_list[] = {ADC1, ADC2, ADC3, ADC4, ADC5};
  for (int i = 0; i < 5; i++)
  {
    // АЦП синхронизировано с таймером TIM 3 TRGO
    HAL_Delay(10);
    LL_ADC_StartCalibration(adc_list[i], LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(adc_list[i]) != 0)
    {
    }
    LL_ADC_Enable(adc_list[i]);
    HAL_Delay(10);
    LL_ADC_INJ_StartConversion(adc_list[i]);
    HAL_Delay(10);
  }
  LL_ADC_EnableIT_JEOS(ADC3); // прерывание конеца последовательности
  htim3.Instance->DIER |= TIM_DIER_UIE;
  htim3.Instance->ARR = 13124;
  HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);
}
typedef struct
{
  ADC_TypeDef *adc;
  uint32_t channel;
} bsp_adc_struct;
#define BSP_ANALOG_IN_COUNT (12 + 2 + 1)
#define BSP_ANALOG_TIM_FREQ (12800)
#define MAIN_ADC_BUF_COUNT (12)
Buf_16_typedef dsp_adc_buf[MAIN_ADC_BUF_COUNT];

bsp_adc_struct adcStruct[BSP_ANALOG_IN_COUNT] = {
    {.adc = ADC1, .channel = LL_ADC_INJ_RANK_1}, // A1
    {.adc = ADC1, .channel = LL_ADC_INJ_RANK_3}, // A2
    {.adc = ADC2, .channel = LL_ADC_INJ_RANK_1}, // A3
    {.adc = ADC2, .channel = LL_ADC_INJ_RANK_3}, // A4
    {.adc = ADC3, .channel = LL_ADC_INJ_RANK_2}, // A5
    {.adc = ADC3, .channel = LL_ADC_INJ_RANK_1}, // A6
    {.adc = ADC1, .channel = LL_ADC_INJ_RANK_2}, // A7
    {.adc = ADC4, .channel = LL_ADC_INJ_RANK_3}, // A8 EXT
    {.adc = ADC2, .channel = LL_ADC_INJ_RANK_2}, // A9
    {.adc = ADC3, .channel = LL_ADC_INJ_RANK_3}, // A10
    {.adc = ADC4, .channel = LL_ADC_INJ_RANK_1}, // A11 EXT
    {.adc = ADC4, .channel = LL_ADC_INJ_RANK_2}, // A12 EXT
    {.adc = ADC5, .channel = LL_ADC_INJ_RANK_2}, // TEMP1
    {.adc = ADC5, .channel = LL_ADC_INJ_RANK_3}, // TEMP2
    {.adc = ADC5, .channel = LL_ADC_INJ_RANK_1}  // Cpu temp
};
bsp_analogIn_typedef bsp_analogIn_struct;
__INLINE float bsp_analogIn_getTemp(uint8_t channel)
{

  const dsp_point_typedef tempPoints[] = {
      {.x = 7500.0f, .y = 100.0f},
      {.x = 10000.0f, .y = 85.0f},
      {.x = 12500.0f, .y = 78.0f},
      {.x = 20000.0f, .y = 62.0f},
      {.x = 30000.0f, .y = 46.0f},
      {.x = 35000.0f, .y = 40.0f},
      {.x = 65535.0f, .y = 5.0f}};

  const uint8_t tempPointsCount = sizeof(tempPoints) / sizeof(tempPoints[0]);

  float input = 0.0f;

  if (channel == 1)
  {
    input = bsp_analogIn_struct.rawDataTemp[0];
  }
  else
  {
    input = bsp_analogIn_struct.rawDataTemp[1];
  }

  return dsp_lineApprox(tempPoints, tempPointsCount, input);
}
void BSP_ANALOG_IRQ_HANDLER()
{
  BSP_ANALOG_CLEAR_FLAG();

  // BSP_LED_OFF(BSP_LED_LINK);

  uint16_t shift = 32767;
  static DSP_T rawBuf[MAIN_ADC_BUF_COUNT];

  for (uint8_t i = 0; i < MAIN_ADC_BUF_COUNT; i++)
  {
    bsp_analogIn_struct.rawDataUI[i] = LL_ADC_INJ_ReadConversionData32(adcStruct[i].adc, adcStruct[i].channel) - shift;
    rawBuf[i] = bsp_analogIn_struct.rawDataUI[i];
    Buf_16_push(&dsp_adc_buf[i], &rawBuf[i]);
  }

  static uint16_t temp_cnt = 0;

  bsp_analogIn_struct.rawDataTemp[0] = LL_ADC_INJ_ReadConversionData32(adcStruct[12].adc, adcStruct[12].channel);
  bsp_analogIn_struct.rawDataTemp[1] = LL_ADC_INJ_ReadConversionData32(adcStruct[13].adc, adcStruct[13].channel);
  bsp_analogIn_struct.rawDataCpuTemp = LL_ADC_INJ_ReadConversionData32(adcStruct[14].adc, adcStruct[14].channel);

  if (((temp_cnt++) % BSP_ANALOG_TIM_FREQ) == 0)
  {
    bsp_analogIn_struct.currentTemp[0] = bsp_analogIn_getTemp(1);
    bsp_analogIn_struct.currentTemp[1] = bsp_analogIn_getTemp(2);
  }

  asm("NOP");
  //bsp_analogIn_ready_callback();
}
void BSP_ANALOG_IRQ_HANDLER_TIM()
{

  BSP_ANALOG_TIM->SR &= ~TIM_SR_UIF;

  // BSP_LED_ON(BSP_LED_LINK);
}

__weak void bsp_analogIn_ready_callback(){

 asm("NOP");
}
// ------------------------------ ANALOG IN END ------------------------------



// ------------------------------ SYS ------------------------------
__INLINE void bsp_sys_tick1k_start()
{

  BSP_SYS_TICK_1K_TIMER->DIER |= TIM_DIER_UIE;
  BSP_SYS_TICK_1K_TIMER->CR1 |= TIM_CR1_CEN;
}
__weak void bsp_sys_tick_1k_callback()
{

  asm("NOP");
}
void BSP_SYS_TICK_1K_IRQ_HANDLER()
{
  BSP_SYS_TICK_1K_TIMER->SR &= ~TIM_SR_UIF;

  asm("NOP");

  // ------------------------- RS 485 -------------------------
  for (uint8_t i = 0; i < 2; i++)
  {
    if (timer_rs485_timeout[i] > 0)
    {
      if (--timer_rs485_timeout[i] == 0)
      {
        bsp_rs485_callback_rxTimeout(i + 1);
        bsp_dInOut_resetDout(bsp_dInOut_led_rs485_1_g + i * 2);
        bsp_dInOut_setDout(bsp_dInOut_led_rs485_1_y + i * 2);
      }
    }
  }
  // ------------------------- RS 485 END-------------------------

  bsp_sys_tick_1k_callback();
}
void bsp_sys_reset()
{
  bsp_dInOut_resetStruct();

  // HAL_Delay(50);

  NVIC_SystemReset();
}
// ------------------------------ SYS END------------------------------

// ------------------------------ PWM ------------------------------
// DT 0..511
// polarity POSITIVE PWM -> LL_HRTIM_OUT_POSITIVE_POLARITY
// polarity NEGATIVE PWM -> LL_HRTIM_OUT_NEGATIVE_POLARITY
void bsp_pwm_set_tim(bsp_pwm_tim_mode_typedef mode, uint16_t DT, uint32_t polarity)
{
  uint32_t tim = 0;
  uint32_t TxOut1 = 0;
  uint32_t TxOut2 = 0;

  for (size_t i = 0; i < 6; i++)
  {
    tim = (1 << (i + 17));  //LL_HRTIM_TIMER_A + i
    TxOut1 = (1 << (i * 2));
    TxOut2 = (1 << (i * 2 + 1));
    //LL_HRTIM_EnableSwapOutputs(HRTIM1, tim);

    LL_HRTIM_TIM_SetPrescaler(HRTIM1, tim, LL_HRTIM_PRESCALERRATIO_DIV1);
    LL_HRTIM_TIM_SetCounterMode(HRTIM1, tim, LL_HRTIM_MODE_CONTINUOUS);
    LL_HRTIM_TIM_SetPeriod(HRTIM1, tim, _HRTIM_PERIOD);
    LL_HRTIM_TIM_SetRepetition(HRTIM1, tim, 0x00);
    LL_HRTIM_TIM_SetUpdateGating(HRTIM1, tim, LL_HRTIM_UPDATEGATING_INDEPENDENT);
    if(mode == bsp_pwm_tim_mode_up)  LL_HRTIM_TIM_SetCountingMode(HRTIM1, tim, LL_HRTIM_COUNTING_MODE_UP);
    if(mode == bsp_pwm_tim_mode_up_down)  LL_HRTIM_TIM_SetCountingMode(HRTIM1, tim, LL_HRTIM_COUNTING_MODE_UP_DOWN);
    LL_HRTIM_TIM_SetComp1Mode(HRTIM1, tim, LL_HRTIM_GTCMP1_EQUAL);
    LL_HRTIM_TIM_SetRollOverMode(HRTIM1, tim, LL_HRTIM_ROLLOVER_MODE_BOTH);
    LL_HRTIM_TIM_SetFaultEventRollOverMode(HRTIM1, tim, LL_HRTIM_ROLLOVER_MODE_BOTH);
    LL_HRTIM_TIM_SetBMRollOverMode(HRTIM1, tim, LL_HRTIM_ROLLOVER_MODE_BOTH);
    LL_HRTIM_TIM_SetADCRollOverMode(HRTIM1, tim, LL_HRTIM_ROLLOVER_MODE_BOTH);
    LL_HRTIM_TIM_SetOutputRollOverMode(HRTIM1, tim, LL_HRTIM_ROLLOVER_MODE_BOTH);
    LL_HRTIM_TIM_SetDACTrig(HRTIM1, tim, LL_HRTIM_DACTRIG_NONE);
    LL_HRTIM_TIM_DisableHalfMode(HRTIM1, tim);
    LL_HRTIM_TIM_SetInterleavedMode(HRTIM1, tim, LL_HRTIM_INTERLEAVED_MODE_DISABLED);
    LL_HRTIM_TIM_DisableStartOnSync(HRTIM1, tim);
    LL_HRTIM_TIM_DisableResetOnSync(HRTIM1, tim);
    LL_HRTIM_TIM_EnablePreload(HRTIM1, tim);
    LL_HRTIM_TIM_SetUpdateTrig(HRTIM1, tim, LL_HRTIM_UPDATETRIG_NONE | LL_HRTIM_UPDATETRIG_NONE | LL_HRTIM_UPDATETRIG_RESET);
    LL_HRTIM_TIM_SetResetTrig(HRTIM1, tim, LL_HRTIM_RESETTRIG_NONE);
    LL_HRTIM_TIM_DisablePushPullMode(HRTIM1, tim);
    LL_HRTIM_TIM_EnableDeadTime(HRTIM1, tim);
    LL_HRTIM_TIM_SetBurstModeOption(HRTIM1, tim, LL_HRTIM_BURSTMODE_MAINTAINCLOCK);
    LL_HRTIM_ForceUpdate(HRTIM1, tim);
    LL_HRTIM_TIM_DisableResyncUpdate(HRTIM1, tim);
    LL_HRTIM_DT_SetPrescaler(HRTIM1, tim, LL_HRTIM_DT_PRESCALER_DIV1);
    LL_HRTIM_DT_SetRisingValue(HRTIM1, tim, DT);
    LL_HRTIM_DT_SetRisingSign(HRTIM1, tim, LL_HRTIM_DT_RISING_POSITIVE);
    LL_HRTIM_DT_SetFallingValue(HRTIM1, tim, DT);
    LL_HRTIM_DT_SetFallingSign(HRTIM1, tim, LL_HRTIM_DT_FALLING_POSITIVE);
    LL_HRTIM_OUT_SetPolarity(HRTIM1, TxOut1, polarity);
    LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, TxOut1, LL_HRTIM_OUTPUTSET_TIMCMP1);
    if(mode == bsp_pwm_tim_mode_up) LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, TxOut1, LL_HRTIM_OUTPUTRESET_TIMPER);
    if(mode == bsp_pwm_tim_mode_up_down) LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, TxOut1, LL_HRTIM_OUTPUTRESET_NONE);
    LL_HRTIM_OUT_SetIdleMode(HRTIM1, TxOut1, LL_HRTIM_OUT_NO_IDLE);
    LL_HRTIM_OUT_SetIdleLevel(HRTIM1, TxOut1, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
    LL_HRTIM_OUT_SetFaultState(HRTIM1, TxOut1, LL_HRTIM_OUT_FAULTSTATE_NO_ACTION);
    LL_HRTIM_OUT_SetChopperMode(HRTIM1, TxOut1, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);
    LL_HRTIM_OUT_SetPolarity(HRTIM1, TxOut2, polarity);
    LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, TxOut2, LL_HRTIM_OUTPUTSET_NONE);
    LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, TxOut2, LL_HRTIM_OUTPUTRESET_TIMPER);
    if(mode == bsp_pwm_tim_mode_up) LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, TxOut2, LL_HRTIM_OUTPUTRESET_TIMPER);
    if(mode == bsp_pwm_tim_mode_up_down) LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, TxOut2, LL_HRTIM_OUTPUTRESET_NONE);
    LL_HRTIM_OUT_SetIdleMode(HRTIM1, TxOut2, LL_HRTIM_OUT_NO_IDLE);
    LL_HRTIM_OUT_SetIdleLevel(HRTIM1, TxOut2, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
    LL_HRTIM_OUT_SetFaultState(HRTIM1, TxOut2, LL_HRTIM_OUT_FAULTSTATE_NO_ACTION);
    LL_HRTIM_OUT_SetChopperMode(HRTIM1, TxOut2, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);


    while (LL_HRTIM_IsActiveFlag_DLLRDY(HRTIM1) == RESET)
    {
      asm("NOP");
    }
  }
}

static const uint32_t out1_array[] = {
  LL_HRTIM_OUTPUT_TF1,
  LL_HRTIM_OUTPUT_TC1,
  LL_HRTIM_OUTPUT_TD1,
  LL_HRTIM_OUTPUT_TB1,
  LL_HRTIM_OUTPUT_TE1,
  LL_HRTIM_OUTPUT_TA1
};
static const uint32_t out2_array[] = {
  LL_HRTIM_OUTPUT_TF2,
  LL_HRTIM_OUTPUT_TC2,
  LL_HRTIM_OUTPUT_TD2,
  LL_HRTIM_OUTPUT_TB2,
  LL_HRTIM_OUTPUT_TE2,
  LL_HRTIM_OUTPUT_TA2
};

void bsp_pwm_enable_out_VT_CONCEPT(uint8_t outIdx, bsp_pwm_outs_type_typedef type)
{
  uint32_t controlWord = 0;

  if(outIdx > 5) return;

  /*
    На плате перепутаны верхний и нижний ключ
    Канал А - нижний
    Канал B - верхний
  */
  switch (type)
  {
  case bsp_pwm_outs_type_high:
    controlWord = out2_array[outIdx];
    break;
  case bsp_pwm_outs_type_low:
    controlWord = out1_array[outIdx];
    break;
  case bsp_pwm_outs_type_all:
    controlWord = out2_array[outIdx] | out1_array[outIdx];
    break;
  default:
    break;
  }
  LL_HRTIM_EnableOutput(HRTIM1, controlWord);
}

void bsp_pwm_enable_outs_VT_CONCEPT(bsp_pwm_outs_group_typedef group, bsp_pwm_outs_type_typedef type)
{
  uint32_t controlWord = 0;
  uint8_t lowSide = 0;
  uint8_t highSide = 0;

  uint32_t group123_high =  BSP_PWM_TIM_OUTS_123_HIGH;
  uint32_t group123_low =   BSP_PWM_TIM_OUTS_123_LOW;
  uint32_t group456_high =  BSP_PWM_TIM_OUTS_456_HIGH;
  uint32_t group456_low =   BSP_PWM_TIM_OUTS_456_LOW;

  /*
    На плате перепутаны верхний и нижний ключ
    Канал А - нижний
    Канал B - верхний
  */
  switch (type)
  {
  case bsp_pwm_outs_type_high:
    lowSide = 1;
    break;
  case bsp_pwm_outs_type_low:
    highSide = 1;
    break;
  case bsp_pwm_outs_type_all:
    lowSide = 1;
    highSide = 1;
    break;
  default:
    break;
    }

    if (group == bsp_pwm_outs_group_123)
    {
      controlWord = group123_high * highSide + group123_low * lowSide;
    }
    else if (group == bsp_pwm_outs_group_456)
    {
      controlWord = group456_high * highSide + group456_low * lowSide;
    }

    LL_HRTIM_EnableOutput(HRTIM1, controlWord);
}

void bsp_pwm_disable_outs_VT(bsp_pwm_outs_group_typedef group){

  uint32_t controlWord = 0;

  uint32_t group123 =  BSP_PWM_TIM_OUTS_123_HIGH | BSP_PWM_TIM_OUTS_123_LOW;
  uint32_t group456 =  BSP_PWM_TIM_OUTS_456_HIGH | BSP_PWM_TIM_OUTS_456_LOW;

    if(group == bsp_pwm_outs_group_123)
    {
          controlWord = group123;
    }
    else if (group == bsp_pwm_outs_group_456)
    {
          controlWord = group456;
    }
    LL_HRTIM_DisableOutput(HRTIM1,controlWord);
}

void bsp_pwm_disable_all_outs_VT(){

  LL_HRTIM_DisableOutput(HRTIM1,  BSP_PWM_TIM_OUTS_123_HIGH | 
                                  BSP_PWM_TIM_OUTS_123_LOW  |
                                  BSP_PWM_TIM_OUTS_456_HIGH |
                                  BSP_PWM_TIM_OUTS_456_LOW  );
}

static uint16_t freqArrayArr[] = 
{
  24000,  //  bsp_pwm_freq_3500_hz,     //48000 - up  24000 - upDown
  21000,  //   bsp_pwm_freq_4000_hz,    //42000 - up  21000 - upDown
  15000,  //  bsp_pwm_freq_5600_hz,     //30000 - up  15000 - upDown 
  12000,  //  bsp_pwm_freq_7000_hz,     //24000 - up  12000 - upDown   
  9600,   //  bsp_pwm_freq_8750_hz,     //19200 - up  9600  - upDown  
  8400    //  bsp_pwm_freq_10000_hz      //16800 - up  8400  - upDown 
};

uint8_t bsp_pwm_set_freq(bsp_pwm_outs_group_typedef group, bsp_pwm_freq_typedef freq, uint8_t phaseShift){

  uint16_t _freq = freqArrayArr[freq];

  while (LL_HRTIM_IsActiveFlag_DLLRDY(HRTIM1) == RESET)
  {
    asm("NOP");
  }

  if(freq > bsp_pwm_freq_10000_hz)
  {
    return 0;
  }

  if(group == bsp_pwm_outs_group_123)
  {
    BSP_PWM_SET_PERIOD_VT1(_freq);
    BSP_PWM_SET_PERIOD_VT2(_freq);
    BSP_PWM_SET_PERIOD_VT3(_freq);
    if(phaseShift)
    {
        BSP_PWM_SET_CNT_VT2(_freq/3);
        BSP_PWM_SET_CNT_VT3(2*(_freq/3));
    }
    LL_HRTIM_ForceUpdate(HRTIM1, BSP_PWM_TIM_PWM_1 | BSP_PWM_TIM_PWM_2 | BSP_PWM_TIM_PWM_3);
  }
  else
  {
    BSP_PWM_SET_PERIOD_VT4(_freq);
    BSP_PWM_SET_PERIOD_VT5(_freq);
    BSP_PWM_SET_PERIOD_VT6(_freq);
    if(phaseShift)
    {
        BSP_PWM_SET_CNT_VT5(_freq/3);
        BSP_PWM_SET_CNT_VT6(2*(_freq/3));
    }
    LL_HRTIM_ForceUpdate(HRTIM1, BSP_PWM_TIM_PWM_4 | BSP_PWM_TIM_PWM_5 | BSP_PWM_TIM_PWM_6);
  }

  return 1;
}

static const uint32_t timer_array[] = 
{
  BSP_PWM_TIM_PWM_1,
  BSP_PWM_TIM_PWM_2,
  BSP_PWM_TIM_PWM_3,
  BSP_PWM_TIM_PWM_4,
  BSP_PWM_TIM_PWM_5,
  BSP_PWM_TIM_PWM_6
};

uint8_t bsp_pwm_set_ccrPercentX10(uint8_t ccrIdx, float valuePercentX10){

    if(ccrIdx > 5) return 0;

    uint16_t arr = LL_HRTIM_TIM_GetPeriod(HRTIM1,timer_array[ccrIdx]);
    float ccr = arr * valuePercentX10;
    ccr /= 1000.0f;


    if(ccr<0.0f) ccr = 0.0f;
    else if(ccr > (arr - 1)) ccr = arr + 1;

    /*
      Если ccr == arr, происходит сбой
    */

    BSP_PWM_SET_CCR(timer_array[ccrIdx], ccr);
    return 1;
}


uint8_t bsp_pwm_set_ccr(uint8_t ccrIdx, uint32_t ccr)
{
    if(ccrIdx > 5)
    {
      return 0;
    }

    BSP_PWM_SET_CCR(timer_array[ccrIdx], ccr);
    return 1;
}


void bsp_pwm_start_IRQ_123_IRQ_456()
{
  while (LL_HRTIM_IsActiveFlag_DLLRDY(HRTIM1) == RESET)
  {
    asm("NOP");
  }

  HAL_Delay(1);
  //LL_HRTIM_EnableIT_UPDATE(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_EnableIT_UPDATE(HRTIM1, LL_HRTIM_TIMER_D);



  // BSP_PWM_ENABLE_OUTS_123();
  // BSP_PWM_ENABLE_OUTS_456();

  BSP_PWM_START_ALL_CNT();
}

// __INLINE void bsp_pwm_start_IRQ_group123(){

// }

void BSP_PWM_IRQ_HANDLER_123()
{

  BSP_PWM_IRQ_CLEAR_FLAG_123();

  //BSP_LED_TOGGLE(BSP_LED_LINK);
  bsp_pwm_123_callback();
}

void BSP_PWM_IRQ_HANDLER_456()
{

  BSP_PWM_IRQ_CLEAR_FLAG_456();

  //BSP_LED_TOGGLE(BSP_LED_FAULT);
  bsp_pwm_456_callback();
}


__weak void bsp_pwm_123_callback(){
    asm("NOP");
}
__weak void bsp_pwm_456_callback(){
    asm("NOP");
}

// ------------------------------ PWM END------------------------------

// ------------------------------ FFT ------------------------------

#define ADC_FS (12800) // Частота семплирования 
#define DSP_FFT_SIZE (256)	// Размер FFT
#define DSP_FFT_SIZEF (256.0f) // Размер FFT float

#define DSP_FFT_0_INDEX (0)	//�?ндекс постонянной составляющей в результате FFT
#define DSP_FFT_50_INDEX (50*DSP_FFT_SIZE/ADC_FS)//�?ндекс 50 Hz в результате FFT
#define DSP_FFT_400_INDEX (400*DSP_FFT_SIZE/ADC_FS)//�?ндекс 400 Hz в результате FFT

#define ADC_I_INVERTOR_INDEX  (0)
#define ADC_I_INVERTOR_COUNT  (3)

#define ADC_U_OUT_INDEX       (3)
#define ADC_U_OUT_COUNT       (3)

#define ADC_U_ZPT_INDEX       (6)
#define ADC_U_ZPT_COUNT       (2)

DSP_T adc_raw_buf[MAIN_ADC_BUF_COUNT][256];

DSP_T dsp_fft_buf[512];

float var_fft_out[MAIN_ADC_BUF_COUNT][3];

float var_fft_out_i_cal[3][3];
float var_fft_out_u_cal[3][3];
float var_fft_out_uzpt_cal[2][3];

void bsp_fft_init()
{
  asm("NOP");
  for (int i = 0; i < MAIN_ADC_BUF_COUNT; i++)
  {
    dsp_adc_buf[i].dataBuf = &adc_raw_buf[i][0];
    dsp_adc_buf[i].size = 256;;
  }
}

void bsp_fft_start_conv()
{
  bsp_dsp_wait_bufs_complete();
  bsp_get_fft();
  bsp_dsp_release_bufs();
  bsp_analogIn_ready_callback();
}

void bsp_get_fft()
{
  static uint32_t tick = 0;
  tick = HAL_GetTick();

  uint8_t logN = 0;

  switch (DSP_FFT_SIZE)
  {
  case 1024:
  logN = 10;
  break;
  case 512:
  logN = 9;
  break;
  case 256:
  logN = 8;
  break;
  case 128:
  logN = 7;
  break;
  case 64:
  logN = 6;
  break;
  default:
    Error_Handler();
  break;
  }

  for (int chIdx = 0; chIdx < MAIN_ADC_BUF_COUNT; chIdx++)
  {
    for (int i = 0; i < DSP_FFT_SIZE; i++)
    {
      dsp_fft_buf[i] = *(dsp_adc_buf[chIdx].dataBuf + i);
      dsp_fft_buf[i + DSP_FFT_SIZE] = 0.0f;
    }

#ifndef USE_ARM_MATH
    if (FFTx(dsp_fft_buf, (dsp_fft_buf + DSP_FFT_SIZE), DSP_FFT_SIZE, logN, -1) == 0)
    {
      Error_Handler();
    }
#endif
    float underRoot = 0.0f;
    float magnitude = 0.0f;
    float re = 0.0f, im = 0.0f;
    for (int i = 0; i < DSP_FFT_SIZE / 2; i++)
    {

      re = dsp_fft_buf[i];
      im = dsp_fft_buf[i + DSP_FFT_SIZE];
      underRoot = re * re + im * im;

      __ASM("VSQRT.F32 %0,%1"
            : "=t"(magnitude)
            : "t"(underRoot));
      magnitude /= DSP_FFT_SIZEF;
      if (i)
        magnitude *= 2.0f;

      dsp_fft_buf[i] = magnitude;
    }

    var_fft_out[chIdx][0] = dsp_fft_buf[DSP_FFT_0_INDEX];
    var_fft_out[chIdx][1] = dsp_fft_buf[DSP_FFT_50_INDEX];
    var_fft_out[chIdx][2] = dsp_fft_buf[DSP_FFT_400_INDEX];
  }

  // Вывод значений
  for (int chIdx = 0; chIdx < MAIN_ADC_BUF_COUNT; chIdx++)
  {
    bsp_analogIn_struct.rawDataUI_FFT[chIdx][0] = var_fft_out[chIdx][0];
    bsp_analogIn_struct.rawDataUI_FFT[chIdx][1] = var_fft_out[chIdx][1];
    bsp_analogIn_struct.rawDataUI_FFT[chIdx][2] = var_fft_out[chIdx][2];
  }
  
  tick = HAL_GetTick() - tick;
  return;
}

__INLINE void bsp_dsp_wait_bufs_complete()
{
  for (int i = 0; i < MAIN_ADC_BUF_COUNT; i++)
  {
    while (dsp_adc_buf[i]._block == 0) /* wait until buf is not blocked */
    {
      asm("NOP");
    }
  }
  return;
}

__INLINE void bsp_dsp_release_bufs()
{
  for (int i = 0; i < MAIN_ADC_BUF_COUNT; i++)
  {
    Buf_16_release_buf(&dsp_adc_buf[i]);
  }
  return;
}
// ---------------------------- FFT END ----------------------------


// ------------------------------ CORDIC ------------------------------
float sinBuf[3][320];

void bsp_cordic_init()
{
    LL_CORDIC_Config(CORDIC,
     LL_CORDIC_FUNCTION_COSINE,   /* cosine function */ \
		LL_CORDIC_PRECISION_4CYCLES, /* max precision for q1.31 cosine */\
		LL_CORDIC_SCALE_0,           /* no scale */\
		LL_CORDIC_NBWRITE_1,         /* One input data: angle. Second input data (modulus) is 1 after cordic reset */\
		LL_CORDIC_NBREAD_2,          /* Two output data: cosine, then sine */\
		LL_CORDIC_INSIZE_32BITS,     /* q1.31 format for input data */\
		LL_CORDIC_OUTSIZE_32BITS);   /* q1.31 format for output data */
}

int32_t bsp_float_to_q1_31(float in)
{
  if (in < INT32_MIN)
    in = INT32_MIN;
  else if (in > INT32_MAX)
    in = INT32_MAX;
  return (int32_t)(in * ((float)(INT32_MAX)));
}

float bsp_q1_31_to_float(int32_t in)
{
  float out = ((float)(in) / (float)(INT32_MAX));
  if (out < -1.0f)
    out = -1.0f;
  else if (out > 1.0f)
    out = 1.0f;
  return out;
}

uint16_t bsp_writeSinBuf(uint16_t f_pwm, uint16_t f_out)
{
  uint16_t bufLen = (f_pwm / f_out) * 2;

  float angle[] = {0.0f, 2.0f / 3.0f, -2.0f / 3.0f}; // PI rad => 1.0 ; -PI => -1.0
  for (int phase = 0; phase < 3; phase++)
  {
    for (int i = 0; i < bufLen; i++)
    {
      int32_t Q131 = bsp_float_to_q1_31(angle[phase]);
      /* Write angle */
      LL_CORDIC_WriteData(CORDIC, Q131);
      /* Read cosine */
      LL_CORDIC_ReadData(CORDIC);
      /* Read sine */
      Q131 = (int32_t)LL_CORDIC_ReadData(CORDIC);
      sinBuf[phase][i] = bsp_q1_31_to_float(Q131);
      angle[phase] += 2.0f / ((float)bufLen);
      if (angle[phase] > 1.0f)
      {
        angle[phase] -= 2.0f;
      }
    }
  }
  return bufLen;
}

// ---------------------------- CORDIC END ----------------------------