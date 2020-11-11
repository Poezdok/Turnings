#include "ws2812b.h"


uint16_t MatrixSource [ARR_LEN] = {0};


void setPixel(uint8_t Rpixel , uint8_t Gpixel, uint8_t Bpixel, uint16_t posX)
{
  volatile uint16_t i;
  for(i=0;i<8;i++)
  {
    if (BitIsSet(Rpixel,(7-i)) == 1)
    {
      MatrixSource[DELAY_LEN+posX*24+i+8] = HIGH_Lev;
    }else
    {
      MatrixSource[DELAY_LEN+posX*24+i+8] = LOW_Lev;
    }
    if (BitIsSet(Gpixel,(7-i)) == 1)
    {
      MatrixSource[DELAY_LEN+posX*24+i+0] = HIGH_Lev;
    }else
    {
      MatrixSource[DELAY_LEN+posX*24+i+0] = LOW_Lev;
    }
    if (BitIsSet(Bpixel,(7-i)) == 1)
    {
      MatrixSource[DELAY_LEN+posX*24+i+16] = HIGH_Lev;
    }else
    {
      MatrixSource[DELAY_LEN+posX*24+i+16] = LOW_Lev;
    }
  }
}

void RefreshMatrix(void){	
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, ARR_LEN);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
}

void WS2812Init(void){

    LL_TIM_DisableCounter(TIM3);
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, ARR_LEN);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, (uint32_t) &MatrixSource, (uint32_t) &TIM3->CCR4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_CIRCULAR);
	
	LL_TIM_EnableDMAReq_CC4(TIM3);

	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
	LL_TIM_EnableCounter(TIM3);
	
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
//	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
	
	LL_mDelay(50);
//	setColor(200,100,100,1);
	for (uint16_t Pix = 0; Pix < Led_Count; Pix ++){
		setPixel(0,0,0,Pix);
	}
	
	RefreshMatrix();
	
}
