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
    /* DMA channel1 configuration    DMA 通道 1 配置 ----------------------------------------------*/
    DMA_Reset(DMA_CH1);
	// 配置 DMA 初始化结构体
	// 外设基址为：ADC 数据寄存器地址
    DMA_InitStructure.PeriphAddr     = (uint32_t)&ADC->DAT;
	// 存储器地址，实际上就是一个内部SRAM的变量
    DMA_InitStructure.MemAddr        = (uint32_t)ADCConvertedValue;
	// //数据传输方向，从外设读取发送到内存
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_SRC;
	// 缓冲区大小为2，缓冲区的大小应该等于存储器的大小
    DMA_InitStructure.BufSize        = 6;
	// 存储器地址固定
    DMA_InitStructure.MemoryInc      = DMA_MEM_INC_MODE_ENABLE;	
		// 循环传输模式
    DMA_InitStructure.CircularMode   = DMA_CIRCULAR_MODE_ENABLE;	
		// 禁止存储器到存储器模式，因为是从外设到存储器
    DMA_InitStructure.Mem2Mem        = DMA_MEM2MEM_DISABLE;
		// 存储器数据大小也为半字，跟外设数据大小相同
    DMA_InitStructure.MemDataSize    = DMA_MEM_DATA_WIDTH_HALFWORD;
	// 外设寄存器只有一个，地址不用递增
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_MODE_DISABLE;
		// DMA 传输通道优先级为高，当使用一个DMA通道时，优先级设置不影响
    DMA_InitStructure.Priority       = DMA_CH_PRIORITY_HIGH;
		// 外设数据大小为半字，即两个字节
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_WIDTH_HALFWORD;
		// 初始化DMA
    DMA_Initializes(DMA_CH1, &DMA_InitStructure);
    DMA_Channel_Request_Remap(DMA_CH1, DMA_REMAP_ADC);
    /* Enable DMA channel1// 使能 DMA 通道 */
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
    
    /* Start ADC1 calibration// 初始化ADC 校准寄存器 */
    ADC_Calibration_Operation(ADC_CALIBRATION_ENABLE);
    /* Check the end of ADC1 calibration// 等待校准完成 */
    while (ADC_Calibration_Operation(ADC_CALIBRATION_STS))
        ;
    /* Start ADC Software Conversion// 由于没有采用外部触发，所以使用软件触发ADC转换  */
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

