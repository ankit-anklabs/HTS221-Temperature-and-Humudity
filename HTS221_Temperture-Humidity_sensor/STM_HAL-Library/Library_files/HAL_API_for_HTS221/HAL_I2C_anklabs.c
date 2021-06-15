/**
 ******************************************************************************
 * @file    HAL_I2C_anklabs.h
 * @author  R. Ankit singh
 * @brief   HAL_I2C_anklabs header driver file
 * @Version 1.0
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 ANKLABS.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ANKLAB under MIT,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/MIT
 *
 ******************************************************************************
 */
/*
 * HAL_I2C_anklabs.c
 *
 *  Created on: Apr 17, 2021
 *      Author: R. Ankit singh
 */

#include "HAL_I2C_anklabs.h"

I2C_HandleTypeDef hi2c1;
uint16_t wait = 100;

void HTS221_Write_i2c(uint8_t Dev_address, uint8_t R_Address, uint8_t Data){
	uint8_t buff[2] = {0,0};

	buff[0] = (R_Address | 0x80);
	buff[1] = Data;

	HAL_I2C_Master_Transmit(&hi2c1, Dev_address, buff, 2, wait);
}

uint8_t HTS221_Read_i2c(uint8_t Dev_address, uint8_t Dev_addressR, uint8_t R_Address){
	uint8_t temp = 5;

	HAL_I2C_Master_Transmit(&hi2c1, Dev_address, &R_Address, 1, wait);
	HAL_I2C_Master_Receive(&hi2c1, Dev_addressR, &temp, 1, wait);
	return temp;
}

HTS221_Read_multi_i2c(uint8_t Dev_address, uint8_t Dev_addressR, uint8_t R_Address, uint8_t size){
	R_Address |= 0x80;
	uint8_t buff_arry[size];
	HAL_I2C_Master_Transmit(&hi2c1, Dev_address, &R_Address, 1, wait);
	HAL_I2C_Master_Receive(&hi2c1, Dev_addressR, buff_arry, size, wait);
	for(int i =0;i<size;i++){
		a1.arr[i] = buff_arry[i];
	}
}

