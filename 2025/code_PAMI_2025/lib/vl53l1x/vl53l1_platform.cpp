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
#include <Arduino.h>
#include <FreeRTOS.h>
#include "task.h"

int8_t VL53L1_WriteMulti( VL53L1_DEV dev, uint16_t index, uint8_t *pdata, uint32_t count) {
  dev->wire->beginTransmission(dev->addr);
  uint16_t index_be = __builtin_bswap16(index);
  dev->wire->write((uint8_t*)&index_be, 2);
  dev->wire->write(pdata, count);
  uint8_t status = dev->wire->endTransmission(true);

	return status;
}

int8_t VL53L1_ReadMulti(VL53L1_DEV dev, uint16_t index, uint8_t *pdata, uint32_t count){
  dev->wire->beginTransmission(dev->addr);
  uint16_t index_be = __builtin_bswap16(index);
  dev->wire->write((uint8_t*)&index_be, 2);
  Wire.endTransmission();
  size_t nb = dev->wire->requestFrom(dev->addr, count);
  if(nb < count) {
    dev->wire->flush();
    return VL53L1_READ_ERROR;
  }
  nb = Wire.readBytes(pdata, count);
  if(nb != count) {
    return VL53L1_READ_ERROR;
  }

	return 0;
}

int8_t VL53L1_WrByte(VL53L1_DEV dev, uint16_t index, uint8_t data) {
  return VL53L1_WriteMulti(dev, index, &data, 1);
}

int8_t VL53L1_WrWord(VL53L1_DEV dev, uint16_t index, uint16_t data) {
  uint16_t data_be = __builtin_bswap16(data);
  return VL53L1_WriteMulti(dev, index,(uint8_t*) &data_be, 2);
}

int8_t VL53L1_WrDWord(VL53L1_DEV dev, uint16_t index, uint32_t data) {
  uint32_t data_be = __builtin_bswap32(data);
	return VL53L1_WriteMulti(dev, index,(uint8_t*) &data_be, 4);
}

int8_t VL53L1_RdByte(VL53L1_DEV dev, uint16_t index, uint8_t *data) {
  return VL53L1_ReadMulti(dev, index, data, 1);
}

int8_t VL53L1_RdWord(VL53L1_DEV dev, uint16_t index, uint16_t *data) {
  uint16_t data_be;
  int8_t ret =  VL53L1_ReadMulti(dev, index, (uint8_t*)&data_be, 2);
  *data = __builtin_bswap16(data_be);
	return ret;
}

int8_t VL53L1_RdDWord(VL53L1_DEV dev, uint16_t index, uint32_t *data) {
	uint32_t data_be;
  int8_t ret =  VL53L1_ReadMulti(dev, index, (uint8_t*)&data_be, 4);
  *data = __builtin_bswap32(data_be);
	return ret;
}

int8_t VL53L1_WaitMs(VL53L1_DEV dev, int32_t wait_ms){
  vTaskDelay(pdMS_TO_TICKS(wait_ms));
	return 0;
}
