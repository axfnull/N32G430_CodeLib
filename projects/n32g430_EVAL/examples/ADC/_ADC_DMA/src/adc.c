#include "adc.h"

u16 ADC_DMA_Buff[DMABufferSize];  // DAM����
u16 CURBuff, MaxFeedback;

// void AdcGpioInit(void)
// {
//     GPIO_InitType GPIO_InitStructure;

//     RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOA);  //����GPIOʱ��
//     GPIO_Structure_Initialize(&GPIO_InitStructure);
//     GPIO_InitStructure.Pin = GPIO_PIN_0    //��������
//                              | GPIO_PIN_1  //��ѹ����
//                              | GPIO_PIN_2  //�¿�
//                              | GPIO_PIN_3  //����15V��ѹ
//                              | GPIO_PIN_4  //����
//                              | GPIO_PIN_5  //����
//         ;
//     GPIO_InitStructure.GPIO_Mode = GPIO_MODE_ANALOG;  //ģ������
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
    ADC_InitStructure.ExtTrigSelect = ADC_EXT_TRIGCONV_REGULAR_T1_CC2;  //�ⲿ����ͨ��
    ADC_InitStructure.DatAlign = ADC_DAT_ALIGN_R;                       //�����Ҷ���
    ADC_InitStructure.ChsNumber = ADC_REGULAR_LEN_6;                    // ADCͨ��������
    ADC_Initializes(&ADC_InitStructure);

    // 20us�������� 10us���������ͨ��ת������һͨ�����������0.4us
    // ADC_Channel_Sample_Time_Config(ADC_Channel_01_PA0 ,ADC_SAMP_TIME_1CYCLES5);		// ��������		64M/14	0.2us
    ADC_Channel_Sample_Time_Config(ADC_Channel_01_PA0, ADC_SAMP_TIME_7CYCLES5);  // ��������		64M/20	0.3us
    // ADC_Channel_Sample_Time_Config(ADC_Channel_01_PA0 ,ADC_SAMP_TIME_13CYCLES5);		// ��������		64M/25	0.4us
    ADC_Channel_Sample_Time_Config(ADC_Channel_02_PA1, ADC_SAMP_TIME_28CYCLES5);  // ��ѹ����
    ADC_Channel_Sample_Time_Config(ADC_Channel_03_PA2, ADC_SAMP_TIME_28CYCLES5);  // �¿�
    ADC_Channel_Sample_Time_Config(ADC_Channel_04_PA3, ADC_SAMP_TIME_28CYCLES5);  // ������ѹ
    ADC_Channel_Sample_Time_Config(ADC_Channel_05_PA4, ADC_SAMP_TIME_28CYCLES5);  // ����
    ADC_Channel_Sample_Time_Config(ADC_Channel_06_PA5, ADC_SAMP_TIME_28CYCLES5);  // ����

    ADC_Regular_Sequence_Conversion_Number_Config(ADC_Channel_01_PA0, ADC_REGULAR_NUMBER_1);
    ADC_Regular_Sequence_Conversion_Number_Config(ADC_Channel_02_PA1, ADC_REGULAR_NUMBER_2);
    ADC_Regular_Sequence_Conversion_Number_Config(ADC_Channel_03_PA2, ADC_REGULAR_NUMBER_3);
    ADC_Regular_Sequence_Conversion_Number_Config(ADC_Channel_04_PA3, ADC_REGULAR_NUMBER_4);
    ADC_Regular_Sequence_Conversion_Number_Config(ADC_Channel_05_PA4, ADC_REGULAR_NUMBER_5);
    ADC_Regular_Sequence_Conversion_Number_Config(ADC_Channel_06_PA5, ADC_REGULAR_NUMBER_6);

    ADC_External_Trigger_Conversion_Config(ADC_EXTTRIGCONV_REGULAR_ENABLE);  //ʹ���ⲿ��������ADC1
   
   
    ADC_DMA_Transfer_Enable();  // ADC1��DMAʹ��

	ADC_ON();

    while (ADC_Flag_Status_Get(ADC_RD_FLAG, ADC_FLAG_JENDCA, ADC_FLAG_RDY) == RESET)
        ;                                               // Check ADC Ready
    ADC_Calibration_Operation(ADC_CALIBRATION_ENABLE);  // ADCУ׼
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
    DMA_InitStructure.PeriphAddr = (uint32_t)&ADC->DAT;                 // DMA���������ַ
    DMA_InitStructure.MemAddr = (uint32_t)&ADC_DMA_Buff;                // DMA�����ڴ��ַ
    DMA_InitStructure.Direction = DMA_DIR_PERIPH_SRC;                   //�����赽�ڴ��DMA���䷽��
    DMA_InitStructure.BufSize = DMABufferSize;                          // DMA��������
    DMA_InitStructure.PeriphInc = DMA_PERIPH_INC_MODE_DISABLE;          //���յ����ݺ������ַ������
    DMA_InitStructure.MemoryInc = DMA_MEM_INC_MODE_ENABLE;              //�ڽ��յ���Щ���ݺ��ڴ��ַ�Լ�1
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_WIDTH_HALFWORD;  //����Χ���ݵĿ�ȶ���Ϊ16λ
    DMA_InitStructure.MemDataSize = DMA_MEM_DATA_WIDTH_HALFWORD;        //�����ڴ����ݿ��Ϊ16λ
    DMA_InitStructure.CircularMode = DMA_CIRCULAR_MODE_ENABLE;          //ѭ��ת��ģʽ
    DMA_InitStructure.Priority = DMA_CH_PRIORITY_HIGH;                  // DMA�����ȼ���
    DMA_InitStructure.Mem2Mem = DMA_MEM2MEM_DISABLE;                    // M2M mode is disabled
    DMA_Initializes(DMA_CH1, &DMA_InitStructure);
    DMA_Channel_Request_Remap(DMA_CH1, DMA_REMAP_ADC);

    DMA_Flag_Status_Clear(DMA, DMA_CH1_TXCF);
    DMA_Interrupt_Status_Clear(DMA, DMA_CH1_INT_TXC);
    DMA_Interrupts_Enable(DMA_CH1, DMA_INT_TXC);
    DMA_Channel_Enable(DMA_CH1);

    NVIC_InitStruct.NVIC_IRQChannel = DMA_Channel1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;         //�����ȼ�1
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Initializes(&NVIC_InitStruct);
}

void DMA_Channel1_IRQHandler(void)
{
    if (DMA_Interrupt_Status_Get(DMA, DMA_CH1_INT_TXC)) DMA_Flag_Status_Clear(DMA, DMA_CH1_TXCF);
    ADC_External_Trigger_Conversion_Config(ADC_EXTTRIGCONV_REGULAR_DISABLE);

    // CurrentGivPID = CurrentGiv * CurCoef / 100;                 //��������ֵ
    // CurrentSampling = ADC_DMA_Buff[0];                          //ʵ�ʵ���ֵ
    // PWM_Out = PID_Incremental(CurrentSampling, CurrentGivPID);  //����ʽPID,���������������������趨

    // // if (CurrentFlag = FALSE)		//����ռ�ձ�Ϊ45%
    // // PWM_Out=0.45*PWM_DUTY/2;

    // MaxFeedback = 4095 * CurCoef * 1.3f / 100;
    // if (CurrentSampling >= MaxFeedback)  //����������
    // {
    //     OverCurrentCount++;
    //     if (OverCurrentCount >= 2) {
    //         PWM_Out = 0.04 * PWM_DUTY;
    //     }
    // } else
    //     OverCurrentCount = 0;
    // // ����,�ر�PWM
    // if ((PowreFlag == DISABLE)        // �ϵ�EN,��ʱ����
    //     || (OC_Flag == ENABLE)        // �¶ȳ���
    //     || (PowerEN == DISABLE)       // ϵͳ����EN
    //     || (DriveVolFlag == DISABLE)  // ����Ƿѹ
    //     || (AntiStick_Fg == ENABLE)   //��ճ
    // ) {
    //     PWM_Out = PWM_Preinstall_OFF;
    // }
    // TIM_Compare2_Set(TIM1, PWM_Out);
    // TIM_Compare3_Set(TIM1, PWM_DUTY - PWM_Out);

    // FgSystemTime = TRUE;
    ADC_External_Trigger_Conversion_Config(ADC_EXTTRIGCONV_REGULAR_ENABLE);  //ʹ���ⲿ��������ADC1
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
//         ad_value = adsum / 1000;  // n��ƽ��ֵ��Ϊ���ս��
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
//         ad_value = adsum / 1000;  // n��ƽ��ֵ��Ϊ���ս��
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
