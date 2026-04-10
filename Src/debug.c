#include "debug.h"

/* print via SWO
#define PRINT_BUF_SIZE 128
volatile uint8_t uartTxBusy = 0;   // 1 = busying, 0 = standby
char uartBuf[PRINT_BUF_SIZE];

int fputc(int ch, FILE *f)
{
    if(ch == '\n') {
        uint8_t cr = '\r';
        while(uartTxBusy);
        uartTxBusy = 1;
        HAL_UART_Transmit_IT(&huart1, &cr, 1);
    }

    uint8_t c = (uint8_t)ch;
    while(uartTxBusy);
    uartTxBusy = 1;
    HAL_UART_Transmit_IT(&huart1, &c, 1);

    return ch;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1) {
        uartTxBusy = 0;
    }
}*/
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

//Debug Unfreeze
void Debug_Unfreeze()
{
	__HAL_DBGMCU_UNFREEZE_TIM8();
	__HAL_DBGMCU_UNFREEZE_TIM6(); 
	//DBGMCU->CR &= ~DBGMCU_CR_DBG_TIM1_STOP; //clear Tim1 Stop
}

void Set_Run_On_Ram()
{
	SCB->VTOR = 0x20000000;
}

//block function for Keil MDS UART
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
