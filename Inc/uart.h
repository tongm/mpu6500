#ifndef _UART_H
#define _UART_H

#include "stm32f0xx_hal.h"
#include "matrix.h"


void MX_USART1_UART_Init(void);
void printf_matrix(_Matrix *A);

#endif

