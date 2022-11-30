#ifndef __ADC_H_
#define __ADC_H_
// #include "config.h"
#include "main.h"

#define DMABufferSize 6  // DMA»º´æÊý
#define ADC1_DR_Address ((uint32_t)ADC1 + 0x4C)

// EXT u16 ADC_DMA_Buff[DMABufferSize];  // DAM»º´æ
// EXT u16 CURBuff;

void Adc_Init(void);
void AdcGpioInit(void);
void DMAInit(void);
// u16 ADC1_SingleChannel_Get(void);
// unsigned int ADC_Sample_Arcforce(unsigned int ad_res);
// unsigned int ADC_Sample_Current(unsigned int ad_res);

#endif
