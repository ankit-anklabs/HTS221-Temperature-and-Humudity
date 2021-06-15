/**
 ******************************************************************************
 * @file    HTS221_anklabs.h
 * @author  R. Ankit singh
 * @brief   HTS221 header driver file
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
 * HTS221_anklabs.h
 *
 *  Created on: Apr 15, 2021
 *      Author: R. Ankit singh
 */
#include<stdio.h>

#ifndef __HTS221_ANKLABS_H_
#define __HTS221_ANKLABS_H_
#endif /* ANKLABS_INC_HTS221_ANKLABS_H_ */

//I2C Bus Device Address
#define HTS221_W_A  (uint8_t)0xBE
#define HTS221_R_A  (uint8_t)0xBF
//Device ID
#define HTS221_ID_R	(uint8_t)0x0F  //Device ID Register Address
#define HTS221_ID_D (uint8_t)0xBC  //Device ID Stored in Register
///////////////Configuration Registers
//HTS221_AV_CONF_R Default value: 0x1B (16 Temp Average, 32 Humidity Average)
#define HTS221_AV_CONF_R  (uint8_t)0x10 //Internal Average for sensor value
#define HTS221_AVG_T_B    (uint8_t)3
#define HTS221_AVG_H_B    (uint8_t)0
#define HTS221_AVG_H_MASK (uint8_t)0x07 //[2:0](0x00 - 0x07)
#define HTS221_AVG_T_MASK (uint8_t)0x38 //[5:3](0x00 - 0x07)
//Control Register 1
#define HTS221_CTRL_R1  (uint8_t)0x20 //Control Register 1 Address
#define HTS221_PD_B     (uint8_t)7
#define HTS221_BDU_B    (uint8_t)2
#define HTS221_ODR_B    (uint8_t)0
#define HTS221_PD_MASK  (uint8_t)0x80 //bit 7 - power on/off(1,0)
#define HTS221_BDU_MASK (uint8_t)0x04 //bit 2 data update - continuous/after read action on lsb/msb(0,1)
#define HTS221_ODR_MASK (uint8_t)0x03 //data rate[1:0]
//Control Register 2
#define HTS221_CTRL_R2      (uint8_t)0x21
#define HTS221_BOOT_B       (uint8_t)7
#define HTS221_HEATHER_B    (uint8_t)1
#define HTS221_ONESHOT_B    (uint8_t)0
#define HTS221_BOOT_MASK    (uint8_t)0x80 //bit 7 normal/reset(0,1)
#define HTS221_HEATHER_MASK (uint8_t)0x02 //bit 1
#define HTS221_ONESHOT_MASK (uint8_t)0x01 //bit 0
//Control Register 3
#define HTS221_CTRL_R3      (uint8_t)0x22
#define HTS221_DRDY_B       (uint8_t)7
#define HTS221_PP_OD_B      (uint8_t)6
#define HTS221_DRDY_EN_B    (uint8_t)2
#define HTS221_DRDY_MASK    (uint8_t)0x80
#define HTS221_PP_OD_MASK   (uint8_t)0x40
#define HTS221_DRDY_EN_MASK (uint8_t)0x04
//Status Register
#define HTS221_STATUS_R (uint8_t)0x27
#define HTS221_H_B      (uint8_t)1
#define HTS221_T_B      (uint8_t)0
#define HTS221_H_MASK   (uint8_t)0x02
#define HTS221_T_MASK   (uint8_t)0x01
//Calibration Register
#define HTS221_H0_RH_X2      (uint8_t)0x30
#define HTS221_H1_RH_X2      (uint8_t)0x31
#define HTS221_T0_DEGC_X8    (uint8_t)0x32
#define HTS221_T1_DEGC_X8    (uint8_t)0x33

#define HTS221_T0_T1_DEGC_H2 (uint8_t)0x35
#define HTS221_H0_T0_OUT_L   (uint8_t)0x36
#define HTS221_H0_T0_OUT_H   (uint8_t)0x37

#define HTS221_H1_T0_OUT_L   (uint8_t)0x3A
#define HTS221_H1_T0_OUT_H   (uint8_t)0x3B
#define HTS221_T0_OUT_L      (uint8_t)0x3C
#define HTS221_T0_OUT_H      (uint8_t)0x3D
#define HTS221_T1_OUT_L      (uint8_t)0x3E
#define HTS221_T1_OUT_H      (uint8_t)0x3F
//Output Register
#define HTS221_H_OUT_L_R (uint8_t)0x28 //Humidity LSB
#define HTS221_H_OUT_H_R (uint8_t)0x29 //Humidity MSB-Signed
#define HTS221_T_OUT_L_R (uint8_t)0x2A //Temperature LSB
#define HTS221_T_OUT_H_R (uint8_t)0x2B //Temperature MSB-Signed

uint8_t HTS221_Dev_ID(void);
uint8_t HTS221_Detected(void);
void HTS221_I(void);
void HTS221_Average_func(void);
void HTS221_Start_conversion(void);
uint8_t HTS221_Data_Ready_T(void);
uint8_t HTS221_Data_Ready_H(void);
uint8_t HTS221_Data_Ready(void);
int32_t HTS221_Read_temp(void);
int32_t HTS221_Read_humi(void);

