/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/
/**
 *\*\file main.c
 *\*\author Nations
 *\*\version v1.0.0
 *\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 **/
#include "main.h"

#include "ADTIM_common.h"
#include "timer_common.h"

static OCInitType TIM_OCInitStructure;

static TIM_BDTRInitType TIM_BDTRInitStructure;

uint16_t TimerPeriod = 0;
uint16_t Channel1Pulse = 0, Channel2Pulse = 0, Channel3Pulse = 0, channel4Pulse = 0;

uint32_t ADTIMClockFrequency = 0;
;

void DelayUs(uint32_t count)
{
    uint32_t temp;

    SysTick->LOAD = SystemClockFrequency / 1000000 * count + 1;           /* set reload register */
    SysTick->VAL = 0UL;                                                   /* Load the SysTick Counter Value */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; /* Enable SysTick Timer */

    do {
        temp = SysTick->CTRL;
    } while (temp & 0x01 && !(temp & (1 << 16)));

    SysTick->CTRL = 0;
}

int main(void)
{
    GPIO_InitType GPIO_InitStructure = {0};
    RCC_ClocksType RCC_Clocks = {0};

    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOA | RCC_AHB_PERIPH_GPIOB | RCC_AHB_PERIPH_GPIOC | RCC_AHB_PERIPH_GPIOD);
    RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_AFIO);
    RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_TIM1);

    RCC_Pclk2_Config(RCC_HCLK_DIV2);
    RCC_Clocks_Frequencies_Value_Get(&RCC_Clocks);
    if (RCC_Clocks.HclkFreq > RCC_Clocks.Pclk2Freq) {
        ADTIMClockFrequency = RCC_Clocks.Pclk2Freq * 2;
    } else {
        ADTIMClockFrequency = RCC_Clocks.Pclk2Freq;
    }

    GPIO_Structure_Initialize(&GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUT_PP;
    GPIO_InitStructure.GPIO_Current = GPIO_DS_4MA;
    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);
    GPIO_Pins_Set(GPIOA, GPIO_PIN_8);
    DelayUs(100);
    GPIO_Pins_Reset(GPIOA, GPIO_PIN_8);

    GPIO_InitStructure.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF3_TIM1;
    GPIO_InitStructure.GPIO_Pull = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Current = GPIO_DS_4MA;
    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);

    DelayUs(1000);

    /* Compute the value to be set in AR register to generate signal frequency at 17.57 Khz */
    TimerPeriod = (ADTIMClockFrequency / 17570) - 1;
    /* Compute CCDAT1 value to generate a duty cycle at 50% for channel 1 */
    Channel1Pulse = (uint16_t)(((uint32_t)5 * (TimerPeriod - 1)) / 10);
    /* Compute CCDAT2 value to generate a duty cycle at 25%  for channel 2 */
    Channel2Pulse = (uint16_t)(((uint32_t)25 * (TimerPeriod - 1)) / 100);
    /* Compute CCDAT3 value to generate a duty cycle at 12.5%  for channel 3 */
    Channel3Pulse = (uint16_t)(((uint32_t)125 * (TimerPeriod - 1)) / 1000);
    /* Compute CCDAT4 value to generate a duty cycle at 12.5%  for channel 3 */
    channel4Pulse = (uint16_t)(((uint32_t)125 * (TimerPeriod - 1)) / 1000);

    /* TIM Base Init, Period = TimerPeriod, Prescaler = 0 */
    Common_TIM_Base_Initialize(ADTIM, TimerPeriod, 0);

    TIM_Output_Channel_Struct_Initialize(&TIM_OCInitStructure);

    /* Channel 1, 2 and 3 Configuration in PWM mode */
    TIM_OCInitStructure.OcMode = TIM_OCMODE_PWM2;
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.OutputNState = TIM_OUTPUT_NSTATE_ENABLE;
    TIM_OCInitStructure.Pulse = Channel2Pulse;
    TIM_OCInitStructure.OcPolarity = TIM_OC_POLARITY_LOW;
    TIM_OCInitStructure.OcNPolarity = TIM_OCN_POLARITY_LOW;
    TIM_OCInitStructure.OcIdleState = TIM_OC_IDLE_STATE_SET;
    TIM_OCInitStructure.OcNIdleState = TIM_OCN_IDLE_STATE_RESET;
    TIM_Output_Channel2_Initialize(ADTIM, &TIM_OCInitStructure);

    TIM_OCInitStructure.Pulse = Channel3Pulse;
    TIM_Output_Channel3_Initialize(ADTIM, &TIM_OCInitStructure);

    TIM_On(ADTIM);
    TIM_PWM_Output_Enable(ADTIM);

    while (1) {
    }
}
