#include "mpu6500.h"

SPI_HandleTypeDef hspi1;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
	__HAL_SPI_ENABLE(&hspi1);
}


u8 ReadMPU6500_Reg(u8 reg)
{
	u8 reg_addr[3];
	u8 reg_data[3];
	reg_addr[0]=reg_addr[1]=reg_addr[2]=reg|0x80;
	MPU6500_CS;
	HAL_SPI_TransmitReceive(&hspi1,reg_addr,reg_data,3,1000);
	MPU6500_NCS;
	return reg_data[1];
}

void WritMPU6500_Reg(u8 reg, u8 data)
{
	u8 send_data[5];
	send_data[0]=reg;
	send_data[1]=data;
	MPU6500_CS;
	HAL_SPI_Transmit(&hspi1,send_data,2,100);
	MPU6500_NCS;
}
