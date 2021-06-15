/*
 * HTS221_anklabs.c
 *
 *  Created on: Apr 15, 2021
 *      Author: Ankit singh
 */
#include "HTS221_anklabs.h"
#include <stdio.h>

#include "HAL_I2C_anklabs.h"

uint8_t HTS221_Dev_ID()
{
  uint8_t ID = 0x00;
  //Read device ID from Register
  ID = HTS221_Read_i2c(HTS221_W_A, HTS221_R_A, HTS221_ID_R);
  return ID;
}
uint8_t HTS221_Detected()
{
	uint8_t ID = 0x00;
	ID = HTS221_Read_i2c(HTS221_W_A, HTS221_R_A, HTS221_ID_R);
	if(ID == HTS221_ID_D){
		return 1;
	}
	else
		return 0;
}
void HTS221_I()
{
  uint8_t temp = 0;
  /* Control Register 1 */
  temp &= ~HTS221_BDU_MASK;
  temp |= (1 << HTS221_BDU_B); //Enable BDU

  temp &= ~HTS221_ODR_MASK;
  temp |= (uint8_t)0x01;       //Set ODR to 1Hz
  //temp |= (uint8_t)0x02;     //Set ODR to 7Hz
  //temp |= (uint8_t)0x03;     //Set ODR to 12.5Hz

  temp |= HTS221_PD_MASK;      //Activate the device(Power ON)
  /* Apply settings to CTRL_REG1 */
  HTS221_Write_i2c(HTS221_W_A, HTS221_CTRL_R1, temp);
}
void HTS221_Average_func()
{
	uint8_t temp = 0;
	temp &= ~HTS221_AVG_T_MASK;
	//temp |= ((uint8_t)0x00 << HTS221_AVG_T_B); //2 Average(0.8 uA)
	//temp |= ((uint8_t)0x01 << HTS221_AVG_T_B); //4 Average(1.05 uA)
	//temp |= ((uint8_t)0x02 << HTS221_AVG_T_B); //8 Average(1.40 uA)
	temp |= ((uint8_t)0x03 << HTS221_AVG_T_B); //16 Average(default)(2.10 uA)
	//temp |= ((uint8_t)0x04 << HTS221_AVG_T_B); //32 Average(3.43 uA)
	//temp |= ((uint8_t)0x05 << HTS221_AVG_T_B); //64 Average(6.15 uA)
	//temp |= ((uint8_t)0x06 << HTS221_AVG_T_B); //128 Average(11.6 uA)
	//temp |= ((uint8_t)0x07 << HTS221_AVG_T_B); //256 Average(22.5 uA)

	temp &= ~HTS221_AVG_H_MASK;
	//temp |= ((uint8_t)0x00); //4 Average(0.8 uA)
	//temp |= ((uint8_t)0x01); //8 Average(1.05 uA)
	//temp |= ((uint8_t)0x02); //16 Average(1.40 uA)
	temp |= ((uint8_t)0x03); //32 Average(default)(2.10 uA)
	//temp |= ((uint8_t)0x04); //64 Average(3.43 uA)
	//temp |= ((uint8_t)0x05); //128 Average(6.15 uA)
	//temp |= ((uint8_t)0x06); //256 Average(11.6 uA)
	//temp |= ((uint8_t)0x07); //512 Average(22.5 uA)

	HTS221_Write_i2c(HTS221_W_A, HTS221_AV_CONF_R, temp);
}
void HTS221_Start_conversion()
{
	HTS221_Write_i2c(HTS221_W_A, HTS221_CTRL_R2, HTS221_ONESHOT_MASK);
	//bit 0 sets to 0 after conversion complete hence we have to start the conversion manually.
}
uint8_t HTS221_Data_Ready_T()
{
	uint8_t temp = 0;

	temp = HTS221_Read_i2c(HTS221_W_A, HTS221_R_A, HTS221_STATUS_R);
	temp &= ~HTS221_H_MASK;
	if(temp){
		return 1;
	}
	else{
		return 0;
	}
}
uint8_t HTS221_Data_Ready_H()
{
	uint8_t temp = 0;

	temp = HTS221_Read_i2c(HTS221_W_A, HTS221_R_A, HTS221_STATUS_R);
	temp &= ~HTS221_T_MASK;
	if(temp){
		return 1;
	}
	else{
		return 0;
	}
}
uint8_t HTS221_Data_Ready()
{
	uint8_t temp = 0;

	temp = HTS221_Read_i2c(HTS221_W_A, HTS221_R_A, HTS221_STATUS_R);
	if(temp == 0x03){
		return 1;
	}
	else{
		return 0;
	}
}
int32_t HTS221_Read_temp(){
	uint8_t buff[4];
	uint8_t temp = 0;
	int16_t T0_OUT_LH, T1_OUT_LH, T0_degC_x8_LH, T1_degC_x8_LH, T_out;
	int32_t tmp32;

	HTS221_Read_multi_i2c(HTS221_W_A, HTS221_R_A, HTS221_T0_OUT_L, 4);
	for(int i =0;i<4;i++){
		buff[i] = a1.arr[i];
	}

	T0_OUT_LH = (((uint16_t)buff[1]<<8) | ((uint16_t)buff[0]));
	T1_OUT_LH = (((uint16_t)buff[3]<<8) | ((uint16_t)buff[2]));

	HTS221_Read_multi_i2c(HTS221_W_A, HTS221_R_A, HTS221_T0_DEGC_X8, 2);
	for(int i =0;i<2;i++){
		buff[i] = a1.arr[i];
	}
	temp = HTS221_Read_i2c(HTS221_W_A, HTS221_R_A, HTS221_T0_T1_DEGC_H2);
	T0_degC_x8_LH = ((((uint16_t)(temp & 0x03)) << 8) | ((uint16_t)buff[0]));
	T1_degC_x8_LH = ((((uint16_t)(temp & 0x0C)) << 6) | ((uint16_t)buff[1]));
	T0_degC_x8_LH = T0_degC_x8_LH >> 3;
	T1_degC_x8_LH = T1_degC_x8_LH >> 3;

	HTS221_Read_multi_i2c(HTS221_W_A, HTS221_R_A, HTS221_T_OUT_L_R, 2);
	for(int i =0;i<2;i++){
		buff[i] = a1.arr[i];
	}
	T_out = (((uint16_t)buff[1]<<8) | ((uint16_t)buff[0]));

	tmp32 = (((((uint32_t)(T_out - T0_OUT_LH)) * ((uint32_t)(T1_degC_x8_LH - T0_degC_x8_LH) * 10)) / ((uint32_t)(T1_OUT_LH - T0_OUT_LH)))  +  (uint32_t)(T0_degC_x8_LH * 10));
	return tmp32 *10;
}

int32_t HTS221_Read_humi(){
	uint8_t buff[2];
	int16_t H0_T0_OUT_LH, H1_T0_OUT_LH, H0_rh_x2_LH, H1_rh_x2_LH, H_out;
	int32_t tmp32;
	int32_t Temp;

	HTS221_Read_multi_i2c(HTS221_W_A, HTS221_R_A, HTS221_H0_T0_OUT_L, 2);
	for(int i =0;i<2;i++){
		buff[i] = a1.arr[i];
	}
	H0_T0_OUT_LH = (((uint16_t)buff[1]<<8) | ((uint16_t)buff[0]));
    HTS221_Read_multi_i2c(HTS221_W_A, HTS221_R_A, HTS221_H1_T0_OUT_L, 2);
	for(int i =0;i<2;i++){
		buff[i] = a1.arr[i];
	}
    H1_T0_OUT_LH = (((uint16_t)buff[1]<<8) | ((uint16_t)buff[0]));

	HTS221_Read_multi_i2c(HTS221_W_A, HTS221_R_A, HTS221_H0_RH_X2, 2);
	for(int i =0;i<2;i++){
		buff[i] = a1.arr[i];
	}
	H0_rh_x2_LH = buff[0]>>1;
	H1_rh_x2_LH = buff[1]>>1;

	HTS221_Read_multi_i2c(HTS221_W_A, HTS221_R_A, HTS221_H_OUT_L_R, 2);
	for(int i =0;i<2;i++){
		buff[i] = a1.arr[i];
	}
	H_out = (((uint16_t)buff[1]<<8) | ((uint16_t)buff[0]));
	//tmp32 = (((((uint32_t)(H_out - H0_T0_OUT_LH)) * ((uint32_t)(H1_rh_x2_LH - H0_rh_x2_LH) * 10)) / ((uint32_t)(H1_T0_OUT_LH - H0_T0_OUT_LH)))  +  (uint32_t)(H0_rh_x2_LH * 10));
	Temp = ((uint32_t)(H_out - H0_T0_OUT_LH)) * ((uint32_t)(H1_rh_x2_LH - H0_rh_x2_LH) * 10);
	tmp32 = (uint16_t)(Temp / (uint32_t)(H1_T0_OUT_LH - H0_T0_OUT_LH))  +  (uint32_t)(H0_rh_x2_LH * 10);
	return tmp32*10;
}

