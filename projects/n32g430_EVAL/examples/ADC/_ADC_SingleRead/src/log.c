#include "log.h"

#define LOG_USARTx USART1
#define LOG_PERIPH RCC_APB2_PERIPH_USART1
#define LOG_GPIO GPIOA
#define LOG_PERIPH_GPIO RCC_AHB_PERIPH_GPIOA
#define LOG_TX_PIN GPIO_PIN_9
#define LOG_RX_PIN GPIO_PIN_10

void log_init(void)
{
    GPIO_InitType GPIO_InitStructure;
    USART_InitType USART_InitStructure;

    RCC_AHB_Peripheral_Clock_Enable(LOG_PERIPH_GPIO);
    RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_AFIO | LOG_PERIPH);

    GPIO_Structure_Initialize(&GPIO_InitStructure);
    GPIO_InitStructure.Pin = LOG_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF5_USART1;
    GPIO_Peripheral_Initialize(LOG_GPIO, &GPIO_InitStructure);

    //    GPIO_InitStructure.Pin             = LOG_RX_PIN;
    //    GPIO_InitStructure.GPIO_Alternate  = GPIO_AF5_USART1;
    //    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.BaudRate = 115200;
    USART_InitStructure.WordLength = USART_WL_8B;
    USART_InitStructure.StopBits = USART_STPB_1;
    USART_InitStructure.Parity = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode = USART_MODE_TX;

    /* init uart */
    USART_Initializes(LOG_USARTx, &USART_InitStructure);

    /* enable uart */
    USART_Enable(LOG_USARTx);
}

static int is_lr_sent = 0;

int fputc(int ch, FILE* f)
{
    if (ch == '\r') {
        is_lr_sent = 1;
    } else if (ch == '\n') {
        if (!is_lr_sent) {
            USART_Data_Send(LOG_USARTx, (uint8_t)'\r');
            /* Loop until the end of transmission */
            while (USART_Flag_Status_Get(LOG_USARTx, USART_FLAG_TXC) == RESET) {
            }
        }
        is_lr_sent = 0;
    } else {
        is_lr_sent = 0;
    }
    USART_Data_Send(LOG_USARTx, (uint8_t)ch);
    /* Loop until the end of transmission */
    while (USART_Flag_Status_Get(LOG_USARTx, USART_FLAG_TXDE) == RESET) {
    }
    return ch;
}

//=============================================================
//=============================================================
//=============================================================
//=============================================================

void PrintDat(uint8_t* datAddr, uint32_t num)
{
    for (int i = 0; i < num; i++) {
        USART_Data_Send(LOG_USARTx, datAddr[i]);
        while (USART_Flag_Status_Get(LOG_USARTx, USART_FLAG_TXDE) == RESET)
            ;
    }
}

void PrintfDat(uint8_t* datAddr, uint32_t num, const char* prefix)
{
    for (int i = 0; i < num; i++) {
        printf("%s%02X", prefix, datAddr[i]);
    }
}

void PrintfDat16(uint16_t* datAddr, uint32_t num, const char* prefix)
{
    for (int i = 0; i < num; i++) {
        printf("%s%04X", prefix, datAddr[i]);
    }
}

void PrintfDat32(uint32_t* datAddr, uint32_t num, const char* prefix)
{
    for (int i = 0; i < num; i++) {
        printf("%s%08X", prefix, datAddr[i]);
    }
}
