/*
Library:					Accelerometer - LIS3DSH
Written by:				Mohamed Yaqoob (MYaqoobEmbedded YouTube Channel)
Date Written:			12/12/2018
Last modified:		-/-
Description:			This is an STM32 device driver library for the LIS3DSH Accelerometer, using STM HAL libraries

References:
			1) STMicroelectronics LIS3DSH datasheet
				 https://www.st.com/resource/en/datasheet/lis3dsh.pdf
			2) ST opensource LIS3DSH accelerometer dsp drivers.
										
* Copyright (C) 2018 - M. Yaqoob
   This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
   of the GNU General Public Licenseversion 3 as published by the Free Software Foundation.
	
   This software library is shared with puplic for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
   or indirectly by this software, read more about this on the GNU General Public License.
*/

//Header files
#include "MY_LIS3DSH.h"
#include <math.h>
//SPI Chip Select
#define _LIS3DHS_CS_ENABLE		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
#define _LIS3DHS_CS_DISABLE		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

//Library variables
//1. SPI handle
static SPI_HandleTypeDef accSPI_Handle;

//2. Sensitivity value
static float lis3dsh_Sensitivity = LIS3DSH_SENSITIVITY_0_06G;
//3. bias variables
//static float __X_Bias = 0.0f;
//static float __Y_Bias = 0.0f;
//static float __Z_Bias = 0.0f;
//4. scaling variables
//static float __X_Scale = 1.0f;
//static float __Y_Scale = 1.0f;
//static float __Z_Scale = 1.0f;

//Functions definitions
//Private functions
//1. Write IO
void LIS3DSH_WriteIO(uint8_t reg, uint8_t *dataW, uint8_t size)
{
	uint8_t spiReg = reg;
	//Enable CS
	_LIS3DHS_CS_ENABLE;
	//set register value
	HAL_SPI_Transmit(&accSPI_Handle, &spiReg, 1, 10);
	//Transmit data
	HAL_SPI_Transmit(&accSPI_Handle, dataW, size, 10);
	//Disable CS
	_LIS3DHS_CS_DISABLE;
}
//2. Read IO
uint8_t LIS3DSH_ReadIO(uint8_t reg, uint8_t *dataR, uint8_t size)
{
	uint8_t spiBuf[4];
	spiBuf[0] = reg | 0x80;
	//Enable CS
	_LIS3DHS_CS_ENABLE;
	//set register value
	HAL_SPI_Transmit(&accSPI_Handle, spiBuf, 1, 10);
	//Transmit data
	HAL_SPI_Receive(&accSPI_Handle, spiBuf, size, 10);
	//Disable CS
	_LIS3DHS_CS_DISABLE;
	
	for(uint8_t i=0; i<(size&0x3); i++)
	{
		dataR[i] = spiBuf[i];
	}
	return dataR[0];
}


//1. Accelerometer initialise function
void LIS3DSH_Init(SPI_HandleTypeDef *accSPI, LIS3DSH_InitTypeDef *accInitDef)
{
	uint8_t spiData = 0;
	
	memcpy(&accSPI_Handle, accSPI, sizeof(*accSPI));
	//** 1. Enable Axes and Output Data Rate **//
	//Set CTRL REG4 settings value
	spiData |= (accInitDef->enableAxes & 0x07);		//Enable Axes
	spiData |= (accInitDef->dataRate & 0xF0);			//Output Data Rate
	//Write to accelerometer
	LIS3DSH_WriteIO(LIS3DSH_CTRL_REG4_ADDR, &spiData, 1);
	
	//** 2. Full-Scale selection, Anti-aliasing BW, self test and 4-wire SPI **//
	spiData = 0;
	spiData |= (accInitDef->antiAliasingBW & 0xC0);		//Anti-aliasing BW
	spiData |= (accInitDef->fullScale & 0x38);				//Full-Scale
	//Write to accelerometer
	LIS3DSH_WriteIO(LIS3DSH_CTRL_REG5_ADDR, &spiData, 1);
	
	//** 3. Interrupt Configuration **//
	if(accInitDef->interruptEnable)
	{
		spiData = 0x88;
		//Write to accelerometer
		LIS3DSH_WriteIO(LIS3DSH_CTRL_REG3_ADDR, &spiData, 1);
	}
	
	//Assign sensor sensitivity (based on Full-Scale)
	switch(accInitDef->fullScale)
	{
		case LIS3DSH_FULLSCALE_2:
			lis3dsh_Sensitivity = LIS3DSH_SENSITIVITY_0_06G;
			break;
		
		case LIS3DSH_FULLSCALE_4:
			lis3dsh_Sensitivity = LIS3DSH_SENSITIVITY_0_12G;
			break;
		
		case LIS3DSH_FULLSCALE_6:
			lis3dsh_Sensitivity = LIS3DSH_SENSITIVITY_0_18G;
			break;
		
		case LIS3DSH_FULLSCALE_8:
			lis3dsh_Sensitivity = LIS3DSH_SENSITIVITY_0_24G;
			break;
		
		case LIS3DSH_FULLSCALE_16:
			lis3dsh_Sensitivity = LIS3DSH_SENSITIVITY_0_73G;
			break;
	}
	_LIS3DHS_CS_DISABLE;
}
//2. Get Accelerometer raw data
LIS3DSH_DataRaw LIS3DSH_GetDataRaw(void)
{
	uint8_t spiBuf[4];
	LIS3DSH_DataRaw tempDataRaw_L;
	LIS3DSH_DataRaw tempDataRaw_H;
	LIS3DSH_DataRaw tempDataRaw;
	//Read X data
	tempDataRaw_L.x = LIS3DSH_ReadIO(LIS3DSH_OUT_X_L_ADDR, spiBuf, 2);
	tempDataRaw_H.x = LIS3DSH_ReadIO(LIS3DSH_OUT_X_H_ADDR, spiBuf+2, 2);
	
	//Read Y data
	tempDataRaw_L.y = LIS3DSH_ReadIO(LIS3DSH_OUT_Y_L_ADDR, spiBuf, 2);
	tempDataRaw_H.y = LIS3DSH_ReadIO(LIS3DSH_OUT_Y_H_ADDR, spiBuf+2, 2);
	
	//Read Z data
	tempDataRaw_L.z = LIS3DSH_ReadIO(LIS3DSH_OUT_Z_L_ADDR, spiBuf, 2);
	tempDataRaw_H.z = LIS3DSH_ReadIO(LIS3DSH_OUT_Z_H_ADDR, spiBuf+2, 2);
	
	tempDataRaw.x = (tempDataRaw_H.x << 8) | (tempDataRaw_L.x);
	tempDataRaw.y = (tempDataRaw_H.y << 8) | (tempDataRaw_L.y);
	tempDataRaw.z = (tempDataRaw_H.z << 8) | (tempDataRaw_L.z);
	
	return tempDataRaw;
}

float CalcAngle(float V, float H)
{
	uint16_t orientation = 0;
	float retVal;
	if(H == 0) H = 0.001;
	if(V == 0) V = 0.001;
	if((H>0) && (V>0)) orientation = 0;
	if((H<0) && (V>0)) orientation = 90;
	if((H<0) && (V<0)) orientation = 180;
	if((H>0) && (V<0)) orientation = 270;
	
	retVal = (atan((double)V/(double)H/3.14159)*180.0);
	if((double)retVal < 0)
			retVal = (double)retVal + 90.0;
		retVal = fabs(retVal) + orientation;
	return retVal;
}
//3. Get Accelerometer mg data
LIS3DSH_DataScaled LIS3DSH_GetDataScaled(void)
{
	//Read raw data
	LIS3DSH_DataRaw tempRawData = LIS3DSH_GetDataRaw();
	//Scale data and return 
	LIS3DSH_DataScaled tempScaledData;
	tempScaledData.x = (float)(tempRawData.x / -141);
	tempScaledData.y = (float)(tempRawData.y / -141);
	tempScaledData.z = (float)(tempRawData.z / -141);
	
	return tempScaledData;
}
//4. Poll for Data Ready
bool LIS3DSH_PollDRDY(uint32_t msTimeout)
{
	uint8_t Acc_status;
	uint32_t startTick = HAL_GetTick();
	do
	{
		//Read status register with a timeout
		LIS3DSH_ReadIO(0x27, &Acc_status, 1);
		if(Acc_status & 0x07)break;
		
	}while((Acc_status & 0x07)==0 && (HAL_GetTick() - startTick) < msTimeout);
	if(Acc_status & 0x07)
	{
		return true;
	}
	return false;
	
}

//** Calibration functions **//
//1. Set X-Axis calibrate
//void LIS3DSH_X_calibrate(float x_min, float x_max)
//{
//	__X_Bias = (x_max+x_min)/2.0f;
//	__X_Scale = (2*1000)/(x_max - x_min);
//}
////2. Set Y-Axis calibrate
//void LIS3DSH_Y_calibrate(float y_min, float y_max)
//{
//	__Y_Bias = (y_max+y_min)/2.0f;
//	__Y_Scale = (2*1000)/(y_max - y_min);
//}
////3. Set Z-Axis calibrate
//void LIS3DSH_Z_calibrate(float z_min, float z_max)
//{
//	__Z_Bias = (z_max+z_min)/2.0f;
//	__Z_Scale = (2*1000)/(z_max - z_min);
//}








































































