#include "as5048.h"
extern SPI_HandleTypeDef hspi1;

	
float get_angle(void) {
	float  angle_raw;
	uint8_t Rx[2] = {0};
	uint8_t Tx[2] = {0};
	uint16_t angle_raw_16 = 0;
	static uint8_t pr = 0;
	Tx[0]	= 0xff;
	Tx[1]	= 0x3f;
	if(!pr) {		
		HAL_GPIO_WritePin(GPIOA, CS_A4_Pin, GPIO_PIN_RESET);  // CS on				
		HAL_SPI_TransmitReceive(&hspi1, Tx, Rx, 3, 0x1000);			
		HAL_GPIO_WritePin(GPIOA, CS_A4_Pin, GPIO_PIN_SET);  // CS off	
		pr=1;
	}
	HAL_GPIO_WritePin(GPIOA, CS_A4_Pin, GPIO_PIN_RESET);  // CS on				
	HAL_SPI_TransmitReceive(&hspi1, Tx, Rx, 1, 0x1000);
	HAL_GPIO_WritePin(GPIOA, CS_A4_Pin, GPIO_PIN_SET);  // CS off	
			
	angle_raw_16 = ( (Rx[1] << 8 ) | Rx[0] ) & 0x3FFF;
	angle_raw = (float)(angle_raw_16)*0.021973997;
	return 	angle_raw;
}	
