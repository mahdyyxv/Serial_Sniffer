#include "stm32f4xx_hal.h"
#include "DataTypes.h"
#include "eeprom.h"

extern I2C_HandleTypeDef hi2c1;

/**
 * @brief 
 * 
 * @param Address 
 * @param ByteAddress 
 * @param Data 
 * @return eReturnStatus 
 */
eReturnStatus udtEEprom_Write(uint8_t Address, uint8_t ByteAddress, const uint8_t *Data, uint8_t size){
    Address &= ~(1<<0);
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(&hi2c1, Address, ByteAddress, size, Data, size, 100);
    if (ret != HAL_OK){
    	return STATE_ERROR;
    }
    return STATE_OK;
}

/**
 * @brief 
 * 
 * @param Address 
 * @param ByteAddress 
 * @param ptrData 
 * @return eReturnStatus 
 */
eReturnStatus udtEEprom_Read(uint8_t Address, uint8_t ByteAddress, uint8_t* ptrData, uint8_t size){
	//Address &= ~(1<<0);
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, Address, ByteAddress, size, ptrData, size, 100);
	if (ret != HAL_OK){
		return STATE_ERROR;
	}
	return STATE_OK;
}
