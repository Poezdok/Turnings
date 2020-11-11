#include "main.h"
#include "string.h"

#define HIGH_Lev 61
#define LOW_Lev 27
#define DELAY_LEN 47

#define Led_Count 24

#define ARR_LEN Led_Count*24 + DELAY_LEN


#define BitIsSet(reg, bit) ((reg & (1<<bit)) != 0)



void setPixel(uint8_t Rpixel, uint8_t Gpixel, uint8_t Bpixel, uint16_t posX);

void RefreshMatrix(void);

void WS2812Init(void);

