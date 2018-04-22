/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "mpu6500.h"
#include <stdio.h>  
#include "matrix.h"  
#include "uart.h"
#include "mpu6500_sys.h"
#include "mpu6500_bsp.h"
/* USER CODE END Includes */

float test_value=0;

int main(void)
{
	u8 mpu6500data[6]={0,0,0,0,0,0};
	int i = 0;  
	int j = 0;  
	int k = 0;  
	_Matrix m1;  
	_Matrix m2;  
	_Matrix m3;  
	
	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	
	//初始化内存  
	matrix_set_m(&m1,3);  
	matrix_set_n(&m1,3);  
	matrix_init(&m1);  
	matrix_set_m(&m2,3);  
	matrix_set_n(&m2,3);  
	matrix_init(&m2);  
	matrix_set_m(&m3,3);  
	matrix_set_n(&m3,3);  
	matrix_init(&m3); 

	//初始化数据  
	k = 1;  
//	for (i = 0;i < m1.m;i++)  
//	{  
//		for (j = 0;j < m1.n;j++)  
//		{  
//			matrix_write(&m1,i,j,k++);  
//		}  
//	}  
	matrix_write(&m1,0,0,1);
	matrix_write(&m1,0,1,0);
	matrix_write(&m1,0,2,2);
	matrix_write(&m1,1,0,-1);
	matrix_write(&m1,1,1,5);
	matrix_write(&m1,1,2,9);
	matrix_write(&m1,2,0,0);
	matrix_write(&m1,2,1,3);
	matrix_write(&m1,2,2,-9);
	for (i = 0;i < m2.m;i++)  
	{  
		for (j = 0;j < m2.n;j++)  
		{  
			matrix_write(&m2,i,j,k++);  
		}  
	} 

	 //原数据  
	printf("A:\n");  
	printf_matrix(&m1);  
	printf("B:\n");  
	printf_matrix(&m2);  

	test_value=matrix_det(&m1);
	printf("A:行列式的值%f\n",test_value);  
	  
	//C = A + B  
	if (matrix_add(&m1,&m2,&m3) > 0)  
	{  
		printf("C = A + B:\n");  
		printf_matrix(&m3);  
	}  
	  
	//C = A - B  
	if (matrix_subtract(&m1,&m2,&m3) > 0)  
	{  
		printf("C = A - B:\n");  
		printf_matrix(&m3);  
	}  
	  
	//C = A * B  
	if (matrix_multiply(&m1,&m2,&m3) > 0)  
	{  
		printf("C = A * B:\n");  
		printf_matrix(&m3);  
	}  
	  
	//C = AT  
	if (matrix_transpos(&m1,&m3) > 0)  
	{  
		printf("C = AT:\n");  
		printf_matrix(&m3);  
	}  
	  
	if (matrix_inverse(&m1,&m3) > 0)  
	{  
		printf("C = A^(-1):\n");  
		printf_matrix(&m3);  
	}  

	WritMPU6500_Reg(0x00,0x01);
	WritMPU6500_Reg(0x01,0x02);
	WritMPU6500_Reg(0x02,0x03);
	WritMPU6500_Reg(0x0D,0x04);
	WritMPU6500_Reg(0x0E,0x05);
	WritMPU6500_Reg(0x0F,0x06);
	while (1)
	{
		mpu6500data[0]=ReadMPU6500_Reg(WHO_AM_I);  //who I am
		mpu6500data[1]=ReadMPU6500_Reg(0x01);  //test reg
		mpu6500data[2]=ReadMPU6500_Reg(0x02);
		mpu6500data[3]=ReadMPU6500_Reg(0x0D);
		mpu6500data[4]=ReadMPU6500_Reg(0x0E);
		mpu6500data[5]=ReadMPU6500_Reg(0x0F);
		for(i=0;i<4800000;i++);
	}

}


void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
