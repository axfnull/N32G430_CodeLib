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

#include "adc.h"

__IO uint16_t ADCConvertedValue;

void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void Tim_Init(void);
/**
 *\*\name    Main program.
 *\*\return  none
 **/
int main(void)
{
    /* System clocks configuration ---------------------------------------------*/
    RCC_Configuration();
    /* GPIO configuration ------------------------------------------------------*/
    GPIO_Configuration();

    Adc_Init();
    DMAInit();
	Tim_Init();


    while (1) {
    }
}


void Tim_Init(void)
{
	TIM_TimeBaseInitType TIM_TimeBaseStructure = {0};
	OCInitType TIM_OCInitStructure = {0};
	
    /* Time Base configuration */
    TIM_Base_Initialize(TIM1, &TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.Period    = 0x8FF; // 0xFF;
    TIM_TimeBaseStructure.Prescaler = 0x4;
    TIM_TimeBaseStructure.ClkDiv    = 0x0;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;
    TIM_Base_Initialize(TIM1, &TIM_TimeBaseStructure);
    /* TIM1 channel1 configuration in PWM mode */
    TIM_OCInitStructure.OcMode      = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = 0x7F;
    TIM_OCInitStructure.OcPolarity  = TIM_OC_POLARITY_LOW;
    TIM_Output_Channel2_Initialize(TIM1, &TIM_OCInitStructure);

    /* TIM1 counter enable */
    TIM_On(TIM1);
    /* TIM1 main Output Enable */
    TIM_PWM_Output_Enable(TIM1);
}

/**
 *\*\name    RCC_Configuration.
 *\*\fun     Configures the different system clocks.
 *\*\return  none
 **/
void RCC_Configuration(void)
{
    /* Enable peripheral clocks ------------------------------------------------*/

    RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_TIM1);
	
    /* Enable DMA GPIO ADC clocks */
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_DMA | RCC_AHB_PERIPH_GPIOA | RCC_AHB_PERIPH_GPIOB | RCC_AHB_PERIPH_ADC);
    /* RCC_ADCHCLK_DIV16*/
//    ADC_Clock_Mode_Config(ADC_CKMOD_AHB, RCC_ADCHCLK_DIV16);
//    RCC_ADC_1M_Clock_Config(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8);  // selsect HSE as RCC ADC1M CLK Source
}

/**
 *\*\name    GPIO_Configuration.
 *\*\fun     Configures the different GPIO ports.
 *\*\return  none
 **/
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_Structure_Initialize(&GPIO_InitStructure);
    /* Configure adc input as analog input -------------------------*/
    GPIO_InitStructure.Pin = GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_ANALOG;
    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_14 | GPIO_PIN_13 | GPIO_PIN_12 | GPIO_PIN_11 | GPIO_PIN_10 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_ANALOG;
    GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);
}

#ifdef USE_FULL_ASSERT

/**
 *\*\name    assert_failed.
 *\*\fun     Reports the name of the source file and the source line number
 *\*\        where the assert_param error has occurred.
 *\*\param   file pointer to the source file name
 *\*\param   line assert_param error line source number
 *\*\return  none
 **/
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1) {
    }
}

#endif
