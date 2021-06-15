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
 * HAL_I2C_anklabs.h
 *
 *  Created on: Apr 15, 2021
 *      Author: R. Ankit singh
 */

#include <stdio.h>
#include "stm32l1xx_hal.h"

#ifndef INC_HAL_I2C_ANKLABS_H_
#define INC_HAL_I2C_ANKLABS_H_

#endif /* INC_HAL_I2C_ANKLABS_H_ */

struct array
{
    uint8_t arr[8];
}a1;

uint8_t HTS221_Read_i2c(uint8_t Dev_address, uint8_t Dev_addressR, uint8_t R_Address);
void HTS221_Write_i2c(uint8_t Dev_address, uint8_t R_Address, uint8_t Data);
uint8_t HTS221_Read_multi_i2c(uint8_t Dev_address, uint8_t Dev_addressR, uint8_t R_Address, uint8_t size);

