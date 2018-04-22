#include "uart.h"


UART_HandleTypeDef huart1;


/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
//串口1使用printf函数
int fputc(int ch, FILE *f)
{ 	
	while((USART1->ISR&0X40)==0);//循环发送,直到发送完毕   
	USART1->TDR = (u8) ch;      
	return ch;
}

float test_print[6][6] ={0};

//打印2维矩阵  
void printf_matrix(_Matrix *A)  
{  
    int i = 0;  
    int j = 0;  
    int m = 0;  
    int n = 0;  
    
    m = A->m;  
    n = A->n;  
    for (i = 0;i < m;i++)  
    {  
        for (j = 0;j < n;j++)  
        {  
			test_print[i][j]=matrix_read(A,i,j);
			printf("%f\t",test_print[i][j]);  
//            printf("%f\t",matrix_read(A,i,j));  
        }  
        printf("\n");  
    }  
}



