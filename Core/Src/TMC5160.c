#include "TMC5160.h"


HAL_StatusTypeDef TMC5160_WriteRegister(TMC5160_HandleTypeDef *htmc, TMC5160_Regs reg_addr, uint8_t data[]){
	  //reg_addr - т.к. это на write, первый бит -> 1
	  reg_addr |= 0b10000000;
	  uint8_t buff[5];
	  buff[0] = reg_addr;
	  buff[1] = data[0];
	  buff[2] = data[1];
	  buff[3] = data[2];
	  buff[4] = data[3];

	  //Receive
	  HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_RESET);
	  HAL_StatusTypeDef result = HAL_SPI_Transmit(htmc->spi, buff, 5, 100);
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_SET);
	  return result;
}

HAL_StatusTypeDef TMC5160_ReadRegister(TMC5160_HandleTypeDef *htmc, TMC5160_Regs reg_addr, uint8_t data[]){

	  uint8_t buff[5];
	  buff[0] = reg_addr;
	  buff[1] = data[0];
	  buff[2] = data[1];
	  buff[3] = data[2];
	  buff[4] = data[3];
	  // sending buffer
	  HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(htmc->spi, buff, 5, 100);
	  HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(htmc->spi, buff, data, 5, 100);
	  HAL_GPIO_WritePin(htmc->GPIOx,htmc->CS, GPIO_PIN_SET);
	  return result;
}

//  Configuration


HAL_StatusTypeDef TMC5160_Configuration(TMC5160_HandleTypeDef *htmc){

	uint8_t cmd_init[5][4]= {
			{0x00, 0x01, 0x00, 0xC3},
			{0x00, 0x06, 0x1F, 0x0A},
			{0x00, 0x00, 0x00, 0x0A},
			{0x00, 0x00, 0x00, 0x04},
			{0x00, 0x00, 0x01, 0xF4},
	};
	// Костыль, который отправляет первую посылку данных, так как первая  посылка с одного раза не отправляется
	HAL_StatusTypeDef result = TMC5160_WriteRegister(htmc, CHOPCONF, cmd_init[0]);

	if(result != HAL_OK){
				return result;
		}

	result = TMC5160_WriteRegister(htmc, CHOPCONF, cmd_init[0]);

	if(result != HAL_OK){
			return result;
	}

	result = TMC5160_WriteRegister(htmc, IHOLD_IRUN, cmd_init[1]);

	if(result != HAL_OK){
				return result;
	}

	result = TMC5160_WriteRegister(htmc, TPOWERDOWN, cmd_init[2]);

	if(result != HAL_OK){
		return result;
	}

	result = TMC5160_WriteRegister(htmc, GCONF, cmd_init[3]);

	if(result != HAL_OK){
				return result;
	}

	result = TMC5160_WriteRegister(htmc,  TPWMTHRS, cmd_init[4]);

	if(result != HAL_OK){
				return result;
	}
	return result ;
}



// Transform data
void  divide_uint32_t(uint32_t value, uint8_t *data){
    data[0]  = (uint8_t)(value >> 24);
    data[1]  = (uint8_t)(value >> 16);
    data[2]  = (uint8_t)(value >> 8);
    data[3]  = (uint8_t)(value >> 0);
}

// Drive functions
HAL_StatusTypeDef TMC5160_setFirstAcceleration(TMC5160_HandleTypeDef *htmc, uint16_t value){
if((value > TMC5160_FIRST_ACCELERATION_LIMIT) || (value < 0) ){
		return HAL_ERROR;
	}
	divide_uint32_t(value, htmc->first_acceleration);
	return HAL_OK;
}


HAL_StatusTypeDef TMC5160_setMaxAcceleration(TMC5160_HandleTypeDef *htmc, uint16_t value){
  if((value > TMC5160_MAX_ACCELERATION_LIMIT) || (value < 0) ){
		return HAL_ERROR;
	}

	divide_uint32_t(value, htmc->max_acceleration );
	return HAL_OK;
}
//
HAL_StatusTypeDef TMC5160_setMaxDeceleration(TMC5160_HandleTypeDef *htmc, uint16_t value){
  if((value > TMC5160_MAX_DECELERATION_LIMIT) || (value < 0) ){
		return HAL_ERROR;
	}
	divide_uint32_t(value, htmc->max_deceleration  );
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_setSecondDeceleration(TMC5160_HandleTypeDef *htmc, uint16_t value){
  if((value > TMC5160_SECOND_DECELERATION_LIMIT) || (value < 0) ){
		return HAL_ERROR;
	}
	divide_uint32_t(value, htmc->second_deceleration);
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_setStopVelocity(TMC5160_HandleTypeDef *htmc, uint32_t value){
  if((value > TMC5160_STOP_VELOCITY_LIMIT) || (value < 0) ){
		return HAL_ERROR;
	}
	divide_uint32_t(value, htmc->stop_speed);
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_setFirstVelocity(TMC5160_HandleTypeDef *htmc, uint32_t value){
  if((value > TMC5160_FIRST_VELOCITY_LIMIT) || (value < 0) ){
		return HAL_ERROR;
	}
	divide_uint32_t(value, htmc->first_speed);
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_setMaxVelocity(TMC5160_HandleTypeDef *htmc, uint32_t value){
  if((value > TMC5160_MAX_VELOCITY_LIMIT) || (value < 0) ){
		return HAL_ERROR;
	}
	divide_uint32_t(value, htmc->max_speed);
	return HAL_OK;
}

HAL_StatusTypeDef TMC5160_setRampMode(TMC5160_HandleTypeDef *htmc, RampModes mode){
 	divide_uint32_t(mode, htmc->ramp_mode);
	return HAL_OK;
}
// Configuration Drive
HAL_StatusTypeDef TMC5160_Configuration_Drive(TMC5160_HandleTypeDef *htmc){
	TMC5160_setFirstAcceleration(htmc,1000);
	TMC5160_setFirstVelocity(htmc,50000);
	TMC5160_setMaxAcceleration(htmc,500);
	TMC5160_setMaxVelocity(htmc,200000);
	TMC5160_setMaxDeceleration(htmc,700);
	TMC5160_setSecondDeceleration(htmc,1400);
	TMC5160_setStopVelocity(htmc,10);
	TMC5160_setRampMode(htmc, Positioning);
	return HAL_OK;
}

// Default Configurations
HAL_StatusTypeDef TMC5160_default_init(TMC5160_HandleTypeDef *htmc){

	HAL_StatusTypeDef result  =   TMC5160_Configuration_Drive(htmc);
	if(result != HAL_OK){
				return result;
			}
	result = TMC5160_Configuration(htmc);
	if(result != HAL_OK){
			return result;
		}
	result = TMC5160_WriteRegister(htmc, A1, htmc->first_acceleration);
	if(result != HAL_OK){
				return result;
			}
	result = TMC5160_WriteRegister(htmc, V1, htmc->first_speed);
	if(result != HAL_OK){
				return result;
			}
	result = TMC5160_WriteRegister(htmc, AMAX, htmc->max_acceleration);
	if(result != HAL_OK){
				return result;
			}
	result = TMC5160_WriteRegister(htmc, VMAX, htmc->max_speed);
	if(result != HAL_OK){
				return result;
			}
	result = TMC5160_WriteRegister(htmc, DMAX, htmc->max_deceleration);
	if(result != HAL_OK){
				return result;
			}
	result = TMC5160_WriteRegister(htmc, D1, htmc->second_deceleration);
	if(result != HAL_OK){
				return result;
			}
	result = TMC5160_WriteRegister(htmc, VSTOP, htmc->stop_speed);
	if(result != HAL_OK){
				return result;
			}
	result = TMC5160_WriteRegister(htmc, RAMPMODE, htmc->ramp_mode);
	if(result != HAL_OK){
				return result;
			}
	return HAL_OK;
}

