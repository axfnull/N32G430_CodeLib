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
#include "log.h"
#include <stdio.h>

/**
**  Cortex-M4F MPU
**/

#define ACCESS_PERMISSION

#define ARRAY_ADDRESS_START (0x20002000UL)
#define ARRAY_SIZE          (0x09UL << 0UL)
#define ARRAY_REGION_NUMBER (0x03UL << MPU_RNR_REGION_Pos)

/** Public define **/
#define RAM_ADDRESS_START                    (0x20000000UL)
#define RAM_SIZE                             (0x19UL << 0UL)
#define PERIPH_ADDRESS_START                 (0x40000000)
#define PERIPH_SIZE                          (0x39UL << 0UL)
#define FLASH_ADDRESS_START                  (0x08000000)
#define FLASH_SIZE                           (0x27UL << 0UL)
#define portMPU_REGION_READ_WRITE            (0x03UL << MPU_RASR_AP_Pos)
#define portMPU_REGION_PRIVILEGED_READ_ONLY  (0x05UL << MPU_RASR_AP_Pos)
#define portMPU_REGION_READ_ONLY             (0x06UL << MPU_RASR_AP_Pos)
#define portMPU_REGION_PRIVILEGED_READ_WRITE (0x01UL << MPU_RASR_AP_Pos)
#define RAM_REGION_NUMBER                    (0x00UL << MPU_RNR_REGION_Pos)
#define FLASH_REGION_NUMBER                  (0x01UL << MPU_RNR_REGION_Pos)
#define PERIPH_REGION_NUMBER                 (0x02UL << MPU_RNR_REGION_Pos)

#if defined(__CC_ARM)
uint8_t privilegedreadonlyarray[32] __attribute__((at(0x20002000)));
#elif defined(__ICCARM__)
#pragma location = 0x20002000
__no_init uint8_t privilegedreadonlyarray[32];
#elif defined(__GNUC__)
uint8_t privilegedreadonlyarray[32] __attribute__((section(".ROarraySection")));
#elif defined(__TASKING__)
uint8_t privilegedreadonlyarray[32] __at(0x20002000);
#endif


/**
*\*\name   main
*\*\fun    Main program.
*\*\return none
**/
int main(void)
{    
    /* USART Init */
    log_init();
    
    printf("Cortex-M4F MPU \r\n");

    /* Set MPU regions */
    MPU_SETUP();

#ifdef ACCESS_PERMISSION
    accesspermission();
#endif

    while (1)
    {
        printf("The access is permitted. \r\n");
    }
  
}


void MPU_SETUP(void)
{
    /* Disable MPU */
    MPU->CTRL &= ~MPU_CTRL_ENABLE_Msk;

    /* Configure RAM region as Region NÝ0, 8kB of size and R/W region */
    MPU->RNR  = RAM_REGION_NUMBER;
    MPU->RBAR = RAM_ADDRESS_START;
    MPU->RASR = RAM_SIZE | portMPU_REGION_READ_WRITE;

    /* Configure FLASH region as REGION NÝ1, 1MB of size and R/W region */
    MPU->RNR  = FLASH_REGION_NUMBER;
    MPU->RBAR = FLASH_ADDRESS_START;
    MPU->RASR = FLASH_SIZE | portMPU_REGION_READ_WRITE;

    /* Configure Peripheral region as REGION NÝ2, 0.5GB of size, R/W and Execute
    Never region */
    MPU->RNR  = PERIPH_REGION_NUMBER;
    MPU->RBAR = PERIPH_ADDRESS_START;
    MPU->RASR = PERIPH_SIZE | portMPU_REGION_READ_WRITE | MPU_RASR_XN_Msk;

    /* Enable the memory fault exception */
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;

    /* Enable MPU */
    MPU->CTRL |= MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_ENABLE_Msk;
}


void accesspermission(void)
{
    volatile uint8_t a;

    /* Configure region for privilegedreadonlyarray as REGION NÝ3, 32byte and R
       only in privileged mode */
    MPU->RNR = ARRAY_REGION_NUMBER;
    MPU->RBAR |= ARRAY_ADDRESS_START;
    MPU->RASR |= ARRAY_SIZE | portMPU_REGION_PRIVILEGED_READ_ONLY;

    /* Read from privilegedreadonlyarray. This will not generate error */
    a = privilegedreadonlyarray[0];

    /* Uncomment the following line to write to privilegedreadonlyarray. This will
       generate error */
    // privilegedreadonlyarray[0] = 'e';
}
