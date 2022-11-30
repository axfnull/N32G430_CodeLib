#include "adc.h"

ADC_InitType ADC_InitStructure;
DMA_InitType DMA_InitStructure;
__IO uint16_t ADCConvertedValue[6]={0,0,0};

void ADC_RCC_Configuration(void);
void ADC_GPIO_Configuration(void);
void NVIC_Configuration(void);


void ADC_Init(void)
{
    /* System clocks configuration ---------------------------------------------*/
    ADC_RCC_Configuration();
    /* GPIO configuration ------------------------------------------------------*/
    ADC_GPIO_Configuration();
    DMA_Structure_Initializes(&DMA_InitStructure);
    /* DMA channel1 configuration    DMA ͨ�� 1 ���� ----------------------------------------------*/
    DMA_Reset(DMA_CH1);
	// ���� DMA ��ʼ���ṹ��
	// �����ַΪ��ADC ���ݼĴ�����ַ
    DMA_InitStructure.PeriphAddr     = (uint32_t)&ADC->DAT;
	// �洢����ַ��ʵ���Ͼ���һ���ڲ�SRAM�ı���
    DMA_InitStructure.MemAddr        = (uint32_t)ADCConvertedValue;
	// //���ݴ��䷽�򣬴������ȡ���͵��ڴ�
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_SRC;
	// ��������СΪ2���������Ĵ�СӦ�õ��ڴ洢���Ĵ�С
    DMA_InitStructure.BufSize        = 6;
	// �洢����ַ�̶�
    DMA_InitStructure.MemoryInc      = DMA_MEM_INC_MODE_ENABLE;	
		// ѭ������ģʽ
    DMA_InitStructure.CircularMode   = DMA_CIRCULAR_MODE_ENABLE;	
		// ��ֹ�洢�����洢��ģʽ����Ϊ�Ǵ����赽�洢��
    DMA_InitStructure.Mem2Mem        = DMA_MEM2MEM_DISABLE;
		// �洢�����ݴ�СҲΪ���֣����������ݴ�С��ͬ
    DMA_InitStructure.MemDataSize    = DMA_MEM_DATA_WIDTH_HALFWORD;
	// ����Ĵ���ֻ��һ������ַ���õ���
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_MODE_DISABLE;
		// DMA ����ͨ�����ȼ�Ϊ�ߣ���ʹ��һ��DMAͨ��ʱ�����ȼ����ò�Ӱ��
    DMA_InitStructure.Priority       = DMA_CH_PRIORITY_HIGH;
		// �������ݴ�СΪ���֣��������ֽ�
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_WIDTH_HALFWORD;
		// ��ʼ��DMA
    DMA_Initializes(DMA_CH1, &DMA_InitStructure);
    DMA_Channel_Request_Remap(DMA_CH1, DMA_REMAP_ADC);
    /* Enable DMA channel1// ʹ�� DMA ͨ�� */
    DMA_Channel_Enable(DMA_CH1);

    ADC_Initializes_Structure(&ADC_InitStructure);
    ADC_InitStructure.MultiChEn      = ENABLE;
    ADC_InitStructure.ContinueConvEn = ENABLE;
    ADC_InitStructure.DatAlign       = ADC_DAT_ALIGN_R;
    ADC_InitStructure.ExtTrigSelect  = ADC_EXT_TRIGCONV_REGULAR_SWSTRRCH;
    ADC_InitStructure.ChsNumber      = ADC_REGULAR_LEN_6 ;
    ADC_Initializes(&ADC_InitStructure);

		ADC_Channel_Sample_Time_Config(ADC_Channel_03_PA2 ,ADC_SAMP_TIME_55CYCLES5);
    ADC_Regular_Sequence_Conversion_Number_Config(ADC_Channel_03_PA2 , ADC_REGULAR_NUMBER_1);

    ADC_Channel_Sample_Time_Config(ADC_Channel_07_PA6 ,ADC_SAMP_TIME_55CYCLES5);
    ADC_Regular_Sequence_Conversion_Number_Config(ADC_Channel_07_PA6 , ADC_REGULAR_NUMBER_3);
		ADC_Channel_Sample_Time_Config(ADC_Channel_08_PA7 ,ADC_SAMP_TIME_55CYCLES5);
    ADC_Regular_Sequence_Conversion_Number_Config(ADC_Channel_08_PA7 , ADC_REGULAR_NUMBER_5);

    /* ADC channel sampletime configuration */
    
		
    /* ADC regular channel configuration */

    
    /* Enable ADC DMA */
    ADC_DMA_Transfer_Enable();

    /* Enable ADC */
    ADC_ON();
    
    /* Check ADC Ready */
    while(ADC_Flag_Status_Get(ADC_RD_FLAG, ADC_FLAG_JENDCA, ADC_FLAG_RDY) == RESET)
        ;
    
    /* Start ADC1 calibration// ��ʼ��ADC У׼�Ĵ��� */
    ADC_Calibration_Operation(ADC_CALIBRATION_ENABLE);
    /* Check the end of ADC1 calibration// �ȴ�У׼��� */
    while (ADC_Calibration_Operation(ADC_CALIBRATION_STS))
        ;
    /* Start ADC Software Conversion// ����û�в����ⲿ����������ʹ���������ADCת��  */
    ADC_Regular_Channels_Software_Conversion_Operation(ADC_EXTRTRIG_SWSTRRCH_ENABLE);
}

void ADC_RCC_Configuration(void)
{
    /* Enable peripheral clocks ------------------------------------------------*/
    
    /* Enable DMA GPIO ADC clocks */
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_DMA|RCC_AHB_PERIPH_GPIOA|RCC_AHB_PERIPH_GPIOB|RCC_AHB_PERIPH_ADC);
    /* RCC_ADCHCLK_DIV16*/
    ADC_Clock_Mode_Config(ADC_CKMOD_AHB, RCC_ADCHCLK_DIV16);
    RCC_ADC_1M_Clock_Config(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8);  //selsect HSE as RCC ADC1M CLK Source
}


void ADC_GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_Structure_Initialize(&GPIO_InitStructure);
    /* Configure adc input as analog input -------------------------*/
    GPIO_InitStructure.Pin       = GPIO_PIN_2|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_ANALOG;
    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);
}

