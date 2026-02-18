/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

int8_t VL53L1_WriteMulti( VL53L1_Dev_t* dev, uint16_t index, uint8_t* pdata, uint32_t count) {
	uint8_t status = 255;
	uint8_t txbuff[6];
	uint16_t* txIndex = (uint16_t*) txbuff ;
	uint16_t bindex = __bswap16(index);
	*txIndex=bindex; 
	for (int i = 0; i<count; i++){
		txbuff[2 + i] = pdata[i];
	}

	if(i2c_master_transmit(dev->dev_handle, txbuff, count + 2, 100) == ESP_OK){
		status = 0; //si la fonction s'éxecute correctement
	} 
	else{
		status = 1; //si la fonction ne s'éxecute pas correctement
	} 

	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	return status;
}

int8_t VL53L1_ReadMulti(VL53L1_Dev_t* dev, uint16_t index, uint8_t *pdata, uint32_t count){
	uint8_t status = 255;
	uint16_t bindex = __bswap16(index);
	if(i2c_master_transmit_receive(dev->dev_handle, (uint8_t*) &bindex, 2, pdata, count, 100) == ESP_OK){
		status = 0;
	}
	else{
		status = 1;
	} 

	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	return status;
}

int8_t VL53L1_WrByte(VL53L1_Dev_t* dev, uint16_t index, uint8_t data) { //data n'est pas un pointeur?
	return VL53L1_WriteMulti(dev, index, &data, 1); //à vérifier

	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
}

int8_t VL53L1_WrWord(VL53L1_Dev_t* dev, uint16_t index, uint16_t data) {
	return VL53L1_WriteMulti(dev, index, (uint8_t*) &data, 2);
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
}

int8_t VL53L1_WrDWord(VL53L1_Dev_t* dev, uint16_t index, uint32_t data) {
	return VL53L1_WriteMulti(dev, index, (uint8_t*) &data, 4);
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
}

int8_t VL53L1_RdByte(VL53L1_Dev_t* dev, uint16_t index, uint8_t *data) {
	return VL53L1_ReadMulti(dev, index, data, 1);
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */

}

int8_t VL53L1_RdWord(VL53L1_Dev_t* dev, uint16_t index, uint16_t *data) {
	return VL53L1_ReadMulti(dev, index, (uint8_t*) data, 2);
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
}

int8_t VL53L1_RdDWord(VL53L1_Dev_t* dev, uint16_t index, uint32_t *data) {
	return VL53L1_ReadMulti(dev, index, (uint8_t*) data, 4);
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
}

int8_t VL53L1_WaitMs(VL53L1_Dev_t* dev, int32_t wait_ms){
	vTaskDelay(wait_ms / portTICK_PERIOD_MS);
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	return 0;
}
