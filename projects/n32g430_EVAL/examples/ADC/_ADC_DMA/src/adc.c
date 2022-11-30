#include "adc.h"

u16 ADC_DMA_Buff[DMABufferSize];  // DAM缓存
u16 CURBuff, MaxFeedback;

// void AdcGpioInit(void)
// {
//     GPIO_InitType GPIO_InitStructure;

//     RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOA);  //开启GPIO时钟
//     GPIO_Structure_Initialize(&GPIO_InitStructure);
//     GPIO_InitStructure.Pin = GPIO_PIN_0    //电流采样
//                              | GPIO_PIN_1  //电压采样
//                              | GPIO_PIN_2  //温控
//                              | GPIO_PIN_3  //驱动15V电压
//                              | GPIO_PIN_4  //电流
//                              | GPIO_PIN_5  //推力
//         ;
//     GPIO_InitStructure.GPIO_Mode = GPIO_MODE_ANALOG;  //模拟输入
//     GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);
// }

void Adc_Init(void)
{
    ADC_InitType ADC_InitStructure;

    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_DMA | RCC_AHB_PERIPH_ADC);

    ADC_Clock_Mode_Config(ADC_CKMOD_AHB, RCC_ADCHCLK_DIV2);            // 128/4 = 64M
    RCC_ADC_1M_Clock_Config(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8);  // selsect HSE as RCC ADC1M CLK Source

    ADC_Initializes_Structure(&ADC_InitStructure);
    ADC_InitStructure.MultiChEn = ENABLE;
    ADC_InitStructure.ContinueConvEn = DISABLE;
    ADC_InitStructure.ExtTrigSelect = ADC_EXT_TRIGCONV_REGULAR_T1_CC2;  //外部触发通道
    ADC_InitStructure.DatAlign = ADC_DAT_ALIGN_R;                       //数据右对齐
    ADC_InitStructure.ChsNumber = ADC_REGULAR_LEN_6;                    // ADC通道的数量
    ADC_Initializes(&ADC_InitStructure);

    // 20us脉冲周期 10us内完成所有通道转换，第一通道最短脉冲宽度0.4us
    // ADC_Channel_Sample_Time_Config(ADC_Channel_01_PA0 ,ADC_SAMP_TIME_1CYCLES5);		// 电流反馈		64M/14	0.2us
    ADC_Channel_Sample_Time_Config(ADC_Channel_01_PA0, ADC_SAMP_TIME_7CYCLES5);  // 电流反馈		64M/20	0.3us
    // ADC_Channel_Sample_Time_Config(ADC_Channel_01_PA0 ,ADC_SAMP_TIME_13CYCLES5);		// 电流反馈		64M/25	0.4us
    ADC_Channel_Sample_Time_Config(ADC_Channel_02_PA1, ADC_SAMP_TIME_28CYCLES5);  // 电压反馈
    ADC_Channel_Sample_Time_Config(ADC_Channel_03_PA2, ADC_SAMP_TIME_28CYCLES5);  // 温控
    ADC_Channel_Sample_Time_Config(ADC_Channel_04_PA3, ADC_SAMP_TIME_28CYCLES5);  // 驱动电压
    ADC_Channel_Sample_Time_Config(ADC_Channel_05_PA4, ADC_SAMP_TIME_28CYCLES5);  // 电流
    ADC_Channel_Sample_Time_Config(ADC_Channel_06_PA5, ADC_SAMP_TIME_28CYCLES5);  // 推力

    ADC_Regular_Sequence_Conversion_Number_Config(ADC_Channel_01_PA0, ADC_REGULAR_NUMBER_1);
    ADC_Regular_Sequence_Conversion_Number_Config(ADC_Channel_02_PA1, ADC_REGULAR_NUMBER_2);
    ADC_Regular_Sequence_Conversion_Number_Config(ADC_Channel_03_PA2, ADC_REGULAR_NUMBER_3);
    ADC_Regular_Sequence_Conversion_Number_Config(ADC_Channel_04_PA3, ADC_REGULAR_NUMBER_4);
    ADC_Regular_Sequence_Conversion_Number_Config(ADC_Channel_05_PA4, ADC_REGULAR_NUMBER_5);
    ADC_Regular_Sequence_Conversion_Number_Config(ADC_Channel_06_PA5, ADC_REGULAR_NUMBER_6);

    ADC_External_Trigger_Conversion_Config(ADC_EXTTRIGCONV_REGULAR_ENABLE);  //使用外部触发启动ADC1
   
   
    ADC_DMA_Transfer_Enable();  // ADC1的DMA使能

	ADC_ON();

    while (ADC_Flag_Status_Get(ADC_RD_FLAG, ADC_FLAG_JENDCA, ADC_FLAG_RDY) == RESET)
        ;                                               // Check ADC Ready
    ADC_Calibration_Operation(ADC_CALIBRATION_ENABLE);  // ADC校准
    while (ADC_Calibration_Operation(ADC_CALIBRATION_STS))
        ;  // Check the end of ADC1 calibration

}
void DMAInit(void)
{
    DMA_InitType DMA_InitStructure;
    NVIC_InitType NVIC_InitStruct;

    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_DMA);

    /* DMA channel1 configuration ----------------------------------------------*/
    DMA_Reset(DMA_CH1);
    DMA_InitStructure.PeriphAddr = (uint32_t)&ADC->DAT;                 // DMA传输外设地址
    DMA_InitStructure.MemAddr = (uint32_t)&ADC_DMA_Buff;                // DMA传输内存地址
    DMA_InitStructure.Direction = DMA_DIR_PERIPH_SRC;                   //从外设到内存的DMA传输方向
    DMA_InitStructure.BufSize = DMABufferSize;                          // DMA缓存数量
    DMA_InitStructure.PeriphInc = DMA_PERIPH_INC_MODE_DISABLE;          //接收到数据后，外设地址不增加
    DMA_InitStructure.MemoryInc = DMA_MEM_INC_MODE_ENABLE;              //在接收到这些数据后，内存地址自加1
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_WIDTH_HALFWORD;  //将外围数据的宽度定义为16位
    DMA_InitStructure.MemDataSize = DMA_MEM_DATA_WIDTH_HALFWORD;        //定义内存数据宽度为16位
    DMA_InitStructure.CircularMode = DMA_CIRCULAR_MODE_ENABLE;          //循环转换模式
    DMA_InitStructure.Priority = DMA_CH_PRIORITY_HIGH;                  // DMA的优先级高
    DMA_InitStructure.Mem2Mem = DMA_MEM2MEM_DISABLE;                    // M2M mode is disabled
    DMA_Initializes(DMA_CH1, &DMA_InitStructure);
    DMA_Channel_Request_Remap(DMA_CH1, DMA_REMAP_ADC);

    DMA_Flag_Status_Clear(DMA, DMA_CH1_TXCF);
    DMA_Interrupt_Status_Clear(DMA, DMA_CH1_INT_TXC);
    DMA_Interrupts_Enable(DMA_CH1, DMA_INT_TXC);
    DMA_Channel_Enable(DMA_CH1);

    NVIC_InitStruct.NVIC_IRQChannel = DMA_Channel1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;         //从优先级1
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Initializes(&NVIC_InitStruct);
}

void DMA_Channel1_IRQHandler(void)
{
    if (DMA_Interrupt_Status_Get(DMA, DMA_CH1_INT_TXC)) DMA_Flag_Status_Clear(DMA, DMA_CH1_TXCF);
    ADC_External_Trigger_Conversion_Config(ADC_EXTTRIGCONV_REGULAR_DISABLE);

    // CurrentGivPID = CurrentGiv * CurCoef / 100;                 //给定电流值
    // CurrentSampling = ADC_DMA_Buff[0];                          //实际电流值
    // PWM_Out = PID_Incremental(CurrentSampling, CurrentGivPID);  //增量式PID,参数：电流采样，电流设定

    // // if (CurrentFlag = FALSE)		//空载占空比为45%
    // // PWM_Out=0.45*PWM_DUTY/2;

    // MaxFeedback = 4095 * CurCoef * 1.3f / 100;
    // if (CurrentSampling >= MaxFeedback)  //最大电流限制
    // {
    //     OverCurrentCount++;
    //     if (OverCurrentCount >= 2) {
    //         PWM_Out = 0.04 * PWM_DUTY;
    //     }
    // } else
    //     OverCurrentCount = 0;
    // // 故障,关闭PWM
    // if ((PowreFlag == DISABLE)        // 上电EN,延时发波
    //     || (OC_Flag == ENABLE)        // 温度超温
    //     || (PowerEN == DISABLE)       // 系统主动EN
    //     || (DriveVolFlag == DISABLE)  // 驱动欠压
    //     || (AntiStick_Fg == ENABLE)   //防粘
    // ) {
    //     PWM_Out = PWM_Preinstall_OFF;
    // }
    // TIM_Compare2_Set(TIM1, PWM_Out);
    // TIM_Compare3_Set(TIM1, PWM_DUTY - PWM_Out);

    // FgSystemTime = TRUE;
    ADC_External_Trigger_Conversion_Config(ADC_EXTTRIGCONV_REGULAR_ENABLE);  //使用外部触发启动ADC1
}

// unsigned int ADC_Sample_Current(unsigned int ad_res)
// {
//     static u16 ad_value, ad_value1, count;
//     static u32 adsum = 0;
//     static u16 adtimes = 0;
//     volatile u16 ad_temp;

//     ad_temp = ad_res;
//     adsum += ad_temp;
//     if (++adtimes >= 1000) {
//         ad_value = adsum / 1000;  // n次平均值作为最终结果
//         adsum = 0;
//         adtimes = 0;
//     }
//     if (ad_value1 != ad_value) {
//         count++;
//         if (count >= 1000) ad_value1 = ad_value;
//     } else
//         count = 0;

//     return ad_value1;
// }
// unsigned int ADC_Sample_Arcforce(unsigned int ad_res)
// {
//     static u16 ad_value, ad_value1, count;
//     static u32 adsum = 0;
//     static u16 adtimes = 0;
//     volatile u16 ad_temp;

//     ad_temp = ad_res;
//     adsum += ad_temp;
//     if (++adtimes >= 1000) {
//         ad_value = adsum / 1000;  // n次平均值作为最终结果
//         adsum = 0;
//         adtimes = 0;
//     }
//     if (ad_value1 != ad_value) {
//         count++;
//         if (count >= 1000) ad_value1 = ad_value;
//     } else
//         count = 0;

//     return ad_value1;
// }
