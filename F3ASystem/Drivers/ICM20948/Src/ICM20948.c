/*
 * ICM20948.c
 *
 *  Created on: Apr 9, 2021
 *      Author: dlago
 */
//#include "ICM20948_SPIconfig.h"
#include "ICM20948.h"

#define ICM20948_ReadBit	0x80
#define ICM20948_WriteBit 	0x00

// Private function prototypes --------------------------------------------------------------------
void ICM20948_Write_Byte(ICM20948_DEVICE *pdev, uint8_t regaddr, uint8_t *pdata);
void ICM20948_Read_Byte(ICM20948_DEVICE *pdev, uint8_t regaddr, uint8_t *pdata);
void ICM20948_Write_Burst(ICM20948_DEVICE *pdev, uint8_t regaddr, uint8_t *pdata, uint8_t len);
void ICM20948_Read_Burst(ICM20948_DEVICE *pdev, uint8_t regaddr, uint8_t *pdata, uint8_t len);
void ICM20948_SetUserBank(ICM20948_DEVICE *dev, ICM20948_ADDR_REG_BANK_SEL_VAL userBank);
ICM20948_ADDR_REG_BANK_SEL_VAL ICM20948_GetUserBank(ICM20948_DEVICE *pdev); //
void ICM20948_Write_RegBits(ICM20948_DEVICE *pdev, ICM20948_ADDR_REG_BANK_SEL_VAL bank, uint8_t regAddress, uint8_t LSBpos, uint8_t val);
void ICM20948_Write_FullReg(ICM20948_DEVICE *pdev, ICM20948_ADDR_REG_BANK_SEL_VAL bank, uint8_t regAddress, uint8_t val);
void ICM20948_Read_RegBits(ICM20948_DEVICE *pdev, ICM20948_ADDR_REG_BANK_SEL_VAL bank, uint8_t regAddress, uint8_t *pdata, uint8_t LSBpos, uint8_t nBits);
void ICM20948_Read_FullReg(ICM20948_DEVICE *pdev, ICM20948_ADDR_REG_BANK_SEL_VAL bank, uint8_t regAddress, uint8_t *pdata);
ICM20948_REG_COMP_RESULT ICM20948_Compare_RegVal(ICM20948_DEVICE *pdev, ICM20948_ADDR_REG_BANK_SEL_VAL bank, uint8_t regAddress, uint8_t val);

// Private function definitions--------------------------------------------------------------------
void ICM20948_Write_Byte(ICM20948_DEVICE *pdev, uint8_t regaddr, uint8_t *pdata)
{
	uint8_t tx_buffer[2];
	tx_buffer[0] = ICM20948_WriteBit | regaddr;
	tx_buffer[1] = *pdata;
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT, pdev->serif.CS_PIN_NUMBER, GPIO_PIN_RESET);
	HAL_SPI_Transmit(pdev->serif.SPI_HANDLE, &tx_buffer[0], 2, 10);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT, pdev->serif.CS_PIN_NUMBER, GPIO_PIN_SET);
}
void ICM20948_Read_Byte(ICM20948_DEVICE *pdev, uint8_t regaddr, uint8_t *pdata)
{
	uint8_t tx_buffer = ICM20948_ReadBit | regaddr;
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT, pdev->serif.CS_PIN_NUMBER, GPIO_PIN_RESET);
	HAL_SPI_Transmit(pdev->serif.SPI_HANDLE, &tx_buffer, 1, 10);
	HAL_SPI_Receive(pdev->serif.SPI_HANDLE, pdata, 1, 10);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT, pdev->serif.CS_PIN_NUMBER, GPIO_PIN_SET);
}
void ICM20948_Write_Burst(ICM20948_DEVICE *pdev, uint8_t regaddr, uint8_t *pdata, uint8_t len)
{
	uint8_t tx_buffer[1+len];
	tx_buffer[0] = ICM20948_WriteBit | regaddr;
	memcpy(&tx_buffer[1],pdata,len);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT, pdev->serif.CS_PIN_NUMBER, GPIO_PIN_RESET);
	HAL_SPI_Transmit(pdev->serif.SPI_HANDLE, &tx_buffer[0], 1+len, 100);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT, pdev->serif.CS_PIN_NUMBER, GPIO_PIN_SET);
}
void ICM20948_Read_Burst(ICM20948_DEVICE *pdev, uint8_t regaddr, uint8_t *pdata, uint8_t len)
{
	uint8_t tx_buffer;
	tx_buffer = ICM20948_ReadBit | regaddr;
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT, pdev->serif.CS_PIN_NUMBER, GPIO_PIN_RESET);
	HAL_SPI_Transmit(pdev->serif.SPI_HANDLE, &tx_buffer, 1, 10);
	HAL_SPI_Receive(pdev->serif.SPI_HANDLE, pdata, len, 100);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT, pdev->serif.CS_PIN_NUMBER, GPIO_PIN_SET);
}
void ICM20948_SetUserBank(ICM20948_DEVICE *pdev, ICM20948_ADDR_REG_BANK_SEL_VAL userBank)
{
	if (pdev->currentBank != userBank)
	{
		ICM20948_Write_Byte(pdev,ICM20948_ADDR_REG_BANK_SEL, &userBank);
		pdev->currentBank = userBank;
	}
}
ICM20948_ADDR_REG_BANK_SEL_VAL ICM20948_GetUserBank(ICM20948_DEVICE *pdev)
{
	ICM20948_ADDR_REG_BANK_SEL_VAL bank;
	ICM20948_Read_Byte(pdev,ICM20948_ADDR_REG_BANK_SEL, &bank);
	return bank;
}
void ICM20948_Write_RegBits(ICM20948_DEVICE *pdev, ICM20948_ADDR_REG_BANK_SEL_VAL bank, uint8_t regAddress, uint8_t LSBpos, uint8_t val)
{
	ICM20948_SetUserBank(pdev, bank);
	uint8_t oldRegVal;
	ICM20948_Read_Byte(pdev, regAddress, &oldRegVal);
	uint8_t newRegVal = modifyBits(oldRegVal, LSBpos, val);
	ICM20948_Write_Byte(pdev, regAddress, &newRegVal);
	//incompleta
	//ICM20948_REG_COMP_RESULT result = ICM20948_Compare_RegVal(pdev, bank, regAddress, val);//Check if register has changed correctly
}
void ICM20948_Write_FullReg(ICM20948_DEVICE *pdev, ICM20948_ADDR_REG_BANK_SEL_VAL bank, uint8_t regAddress, uint8_t val)
{
	ICM20948_SetUserBank(pdev, bank);
	ICM20948_Write_Byte(pdev, regAddress, &val);
	//incompleta, faltaria comprobacion de que se ha escrito bien el registro
	//ICM20948_REG_COMP_RESULT result = ICM20948_Compare_RegVal(pdev, bank, regAddress, val);//Check if register has changed correctly
}
void ICM20948_Read_RegBits(ICM20948_DEVICE *pdev, ICM20948_ADDR_REG_BANK_SEL_VAL bank, uint8_t regAddress, uint8_t *pdata, uint8_t LSBpos, uint8_t nBits)
{
	uint8_t regData;
	ICM20948_Read_FullReg(pdev, bank, regAddress, &regData);
	*pdata = (regData<<(8-LSBpos-nBits))>>(8-nBits);
}
void ICM20948_Read_FullReg(ICM20948_DEVICE *pdev, ICM20948_ADDR_REG_BANK_SEL_VAL bank, uint8_t regAddress, uint8_t *pdata)
{
	ICM20948_SetUserBank(pdev, bank);
	ICM20948_Read_Byte(pdev, regAddress, pdata);
}
ICM20948_REG_COMP_RESULT ICM20948_Compare_RegVal(ICM20948_DEVICE *pdev, ICM20948_ADDR_REG_BANK_SEL_VAL bank, uint8_t regAddress, uint8_t val)
{
	ICM20948_REG_COMP_RESULT result;
	uint8_t regVal;
	ICM20948_Read_FullReg(pdev, bank, regAddress, &regVal);
	if (regVal == val)
	{
		result = ICM20948_REG_COMP_Match;
	}
	else
	{
		result = ICM20948_REG_COMP_Mismatch;
	}
	return result;
}

//Public function definitions----------------------------------------------------------------------

//High level functions
void ICM20948_Init_Device(ICM20948_DEVICE *pdev, SPI_HandleTypeDef *SPI_HANDLE, GPIO_TypeDef *CS_PIN_PORT, uint16_t CS_PIN_NUMBER)
{
	HAL_Delay(100);
	pdev->serif.SPI_HANDLE = SPI_HANDLE;
	pdev->serif.CS_PIN_PORT = CS_PIN_PORT;
	pdev->serif.CS_PIN_NUMBER = CS_PIN_NUMBER;
	pdev->currentBank = BANK_3; //to produce a bank0 set the first time SetUserBank is entered
	ICM20948_Device_Reset(pdev);
	HAL_Delay(100);
	ICM20948_SlaveI2C_Reset(pdev);
	ICM20948_Get_ICM20948_Id(pdev);
	ICM20948_Sleep(pdev, ICM20948_False);
	HAL_Delay(35);
//	ICM20948_LowPower_Digital(pdev, ICM20948_False);
//	ICM20948_Disable_Temp(pdev, ICM20948_False);
	ICM20948_Set_I2C_MST_CLK(pdev, 7); //to guarantee max I2C clock frequency of 400kHz
	ICM20948_Init_AK09916(pdev);
	ICM20948_Config_I2C_SLV(pdev, 0, AK09916_I2C_ADDR, AK09916_ADDR_ST1, 9); //configures master I2C slave 0 to read 9 AK09916 registers
}
void ICM20948_Init_AK09916(ICM20948_DEVICE *pdev)
{
	ICM20948_MasterI2C_Enable(pdev, ICM20948_True); //enable masteri2c module
	uint8_t id = 0;
	while (id != 0x9) //repeat master i2c reset until we are able to read magnetometer id
	{
		ICM20948_MasterI2C_Reset(pdev);
		id = ICM20948_Get_AK09916_Id(pdev);
	}
	ICM20948_AK09916_Reset(pdev); //magnetometer software reset
	pdev->idMag = ICM20948_Get_AK09916_Id(pdev); //store magnetometer id in the general struct
}
void ICM20948_Set_Sensors_Default_Config(ICM20948_DEVICE *pdev)
{
	//GYRO config
	ICM20948_GYRO_CONFIG gyroConfig;
	gyroConfig.PowerMode = ICM20948_SENSOR_PM_On;
	gyroConfig.DLPF_State = ICM20948_DLPF_Disabled;
	gyroConfig.FS = ICM20948_Gyro_1000dps;
	ICM20948_Set_Gyro_Config(pdev, gyroConfig);

	//ACCEL config
	ICM20948_ACCEL_CONFIG accelConfig;
	accelConfig.PowerMode = ICM20948_SENSOR_PM_On;
	accelConfig.DLPF_State = ICM20948_DLPF_Disabled;
	accelConfig.FS = ICM20948_Accel_4g;
	ICM20948_Set_Accel_Config(pdev, accelConfig);

	//TEMP config
	ICM20948_TEMP_CONFIG tempConfig;
	tempConfig.PowerMode = ICM20948_SENSOR_PM_On;
	tempConfig.DLPF_Val = ICM20948_Temp_DLPCFG_7932Hz;
	ICM20948_Set_Temp_Config(pdev, tempConfig);

	//MAGNETO config
	ICM20948_MAGNETO_CONFIG magnetoConfig;
	magnetoConfig.Mode = Continuous_Measurement_Mode4_100Hz;
	ICM20948_Set_Magneto_Config(pdev, magnetoConfig);
}
void ICM20948_Get_AGMT_RawData(ICM20948_DEVICE *pdev)
{
	uint8_t data[21];
	ICM20948_SetUserBank(pdev, BANK_0);
	ICM20948_Read_Burst(pdev, ICM20948_ADDR_ACCEL_XOUT_H, &data[0], 21);
	pdev->sensorsData.accelData.x = (data[0]<<8)|data[1];
	pdev->sensorsData.accelData.y = (data[2]<<8)|data[3];
	pdev->sensorsData.accelData.z = (data[4]<<8)|data[5];
	pdev->sensorsData.gyroData.x = ((data[6]<<8)|data[7]);
	pdev->sensorsData.gyroData.y = ((data[8]<<8)|data[9]);
	pdev->sensorsData.gyroData.z = ((data[10]<<8)|data[11]);
	pdev->sensorsData.tempData = ((data[12]<<8)|data[13]);
	pdev->sensorsData.magnetoData.x = (data[16]<<8)|data[15]; //little endian
	pdev->sensorsData.magnetoData.y = (data[18]<<8)|data[17]; //little endian
	pdev->sensorsData.magnetoData.z = (data[20]<<8)|data[19]; //little endian
	pdev->sensorsDataProc.accelData.x = pdev->sensorsData.accelData.x/pdev->accelConfig.SensitivityF;
	pdev->sensorsDataProc.accelData.y = pdev->sensorsData.accelData.y/pdev->accelConfig.SensitivityF;
	pdev->sensorsDataProc.accelData.z = pdev->sensorsData.accelData.z/pdev->accelConfig.SensitivityF;
	pdev->sensorsDataProc.gyroData.x = pdev->sensorsData.gyroData.x/pdev->gyroConfig.SensitivityF;
	pdev->sensorsDataProc.gyroData.y = pdev->sensorsData.gyroData.y/pdev->gyroConfig.SensitivityF;
	pdev->sensorsDataProc.gyroData.z = pdev->sensorsData.gyroData.z/pdev->gyroConfig.SensitivityF;
	pdev->sensorsDataProc.tempData = ((pdev->sensorsData.tempData-pdev->sensorsOffsetProc.tempRT21Offset)/pdev->tempConfig.SensitivityF)+21;
	pdev->sensorsDataProc.magnetoData.x = pdev->sensorsData.magnetoData.x * pdev->magnetoConfig.SensitivityF;
	pdev->sensorsDataProc.magnetoData.y = pdev->sensorsData.magnetoData.y * pdev->magnetoConfig.SensitivityF;
	pdev->sensorsDataProc.magnetoData.z = pdev->sensorsData.magnetoData.z * pdev->magnetoConfig.SensitivityF;
}
void ICM20948_Get_Sensors_Offset(ICM20948_DEVICE *pdev)
{
	ICM20948_Get_Accel_Offset(pdev);
	ICM20948_Get_Gyro_Offset(pdev);
}
void ICM20948_Set_Accel_Config(ICM20948_DEVICE *pdev, ICM20948_ACCEL_CONFIG accelConfig)
{
	// POWER CONFIGURATION
	ICM20948_PWR_MGMT_2_VAL power;
	ICM20948_BOOL_REG_VAL powerLP;
	switch (accelConfig.PowerMode)
	{
		case ICM20948_SENSOR_PM_On:
			power = ICM20948_SENSOR_ENABLE;
			powerLP = ICM20948_False;
			break;
		case ICM20948_SENSOR_PM_Off:
			power = ICM20948_SENSOR_DISABLE;
			powerLP = ICM20948_False;
			break;
		case ICM20948_SENSOR_PM_LP:
			power = ICM20948_SENSOR_ENABLE;
			powerLP = ICM20948_True;
			break;
	}
	ICM20948_Accel_Power(pdev, power);
	ICM20948_ACCEL_LP(pdev, powerLP);

	//DIGITAL LOW PASS FILTER CONFIGURATION
	ICM20948_Accel_DLPF_Enable(pdev, accelConfig.DLPF_State);
	ICM20948_Accel_DLPF_SelectFc(pdev, accelConfig.DLPF_Val);

	//AVERAGING FILTER CONFIGURATION (only useful if DLPF enabled and LP mode)
	ICM20948_Accel_Averaging_Select(pdev, accelConfig.AVGF_Val);

	//SAMPLE RATE DIVIDER CONFIG (only useful if DLPF is enabled)
	ICM20948_Accel_SMPLRTDiv_Set(pdev, accelConfig.SMPLRT_Div);

	//SCALE CONFIGURATION
	ICM20948_Accel_FS_Select(pdev, accelConfig.FS);

	//Update device struct------------------------------------

	//STORE MAIN CONFIG SETTINGS
	pdev->accelConfig = accelConfig;

	//CALCULATE OUTPUT DATA RATE
	if (accelConfig.DLPF_State == ICM20948_DLPF_Enabled) //if low pass filter enabled
	{
		pdev->accelConfig.ODR = 1125/(1+accelConfig.SMPLRT_Div);
	}
	else
	{
		pdev->accelConfig.ODR = 4500;
	}

	//CALCULATE SENSITIVITY
	switch (accelConfig.FS)
	{
		case ICM20948_Accel_2g:
			pdev->accelConfig.SensitivityF = 16384;
			break;
		case ICM20948_Accel_4g:
			pdev->accelConfig.SensitivityF = 8192;
			break;
		case ICM20948_Accel_8g:
			pdev->accelConfig.SensitivityF = 4096;
			break;
		case ICM20948_Accel_16g:
			pdev->accelConfig.SensitivityF = 2048;
			break;
	}
}
void ICM20948_Set_Gyro_Config(ICM20948_DEVICE *pdev, ICM20948_GYRO_CONFIG gyroConfig)
{
	// POWER CONFIGURATION
	ICM20948_PWR_MGMT_2_VAL power;
	ICM20948_BOOL_REG_VAL powerLP;
	switch (gyroConfig.PowerMode)
	{
		case ICM20948_SENSOR_PM_On:
			power = ICM20948_SENSOR_ENABLE;
			powerLP = ICM20948_False;
			break;
		case ICM20948_SENSOR_PM_Off:
			power = ICM20948_SENSOR_DISABLE;
			powerLP = ICM20948_False;
			break;
		case ICM20948_SENSOR_PM_LP:
			power = ICM20948_SENSOR_ENABLE;
			powerLP = ICM20948_True;
			break;
	}
	ICM20948_Gyro_Power(pdev, power);
	ICM20948_GYRO_LP(pdev, powerLP);

	//DIGITAL LOW PASS FILTER CONFIGURATION
	ICM20948_Gyro_DLPF_Enable(pdev, gyroConfig.DLPF_State);
	ICM20948_Gyro_DLPF_SelectFc(pdev, gyroConfig.DLPF_Val);

	//AVERAGING FILTER CONFIGURATION (only useful if DLPF enabled and LP mode)
	ICM20948_Gyro_Averaging_Select(pdev, gyroConfig.AVGF_Val);

	//SAMPLE RATE DIVIDER CONFIG (only useful if DLPF is enabled)
	ICM20948_Gyro_SMPLRTDiv_Set(pdev, gyroConfig.SMPLRT_Div);

	//SCALE CONFIGURATION
	ICM20948_Gyro_FS_Select(pdev, gyroConfig.FS);

	//Update device struct------------------------------------

	//STORE MAIN CONFIG SETTINGS
	pdev->gyroConfig = gyroConfig;

	//CALCULATE OUTPUT DATA RATE
	if (gyroConfig.DLPF_State == ICM20948_DLPF_Enabled) //if low pass filter enabled
	{
		pdev->gyroConfig.ODR = 1125/(1+gyroConfig.SMPLRT_Div);
	}
	else
	{
		pdev->gyroConfig.ODR = 9000;
	}

	//CALCULATE SENSITIVITY
	switch (gyroConfig.FS)
	{
		case ICM20948_Gyro_250dps:
			pdev->gyroConfig.SensitivityF = 131;
			break;
		case ICM20948_Gyro_500dps:
			pdev->gyroConfig.SensitivityF = 65.5;
			break;
		case ICM20948_Gyro_1000dps:
			pdev->gyroConfig.SensitivityF = 32.8;
			break;
		case ICM20948_Gyro_2000dps:
			pdev->gyroConfig.SensitivityF = 16.4;
			break;
	}
}
void ICM20948_Set_Temp_Config(ICM20948_DEVICE *pdev, ICM20948_TEMP_CONFIG tempConfig)
{
	//POWER CONFIGURATION
	ICM20948_BOOL_REG_VAL power;
	switch (tempConfig.PowerMode)
	{
		case ICM20948_SENSOR_PM_On:
			power = ICM20948_False;
			break;
		case ICM20948_SENSOR_PM_Off:
			power = ICM20948_True;
			break;
		case ICM20948_SENSOR_PM_LP:
			power = ICM20948_False;
			break;
	}
	ICM20948_Disable_Temp(pdev, power);

	//DIGITAL LOW PASS FILTER CONFIGURATION
	ICM20948_Temp_DLPF_SelectFc(pdev, tempConfig.DLPF_Val);

	//STORE MAIN CONFIG SETTINGS
	pdev->tempConfig = tempConfig;

	//CALCULATE OUTPUT DATA RATE
	switch (tempConfig.DLPF_Val)
	{
		case 0:
			pdev->tempConfig.ODR = 9000;
			break;
		case 1:
			pdev->tempConfig.ODR = 1125;
			break;
		case 2:
			pdev->tempConfig.ODR = 1125;
			break;
		case 3:
			pdev->tempConfig.ODR = 1125;
			break;
		case 4:
			pdev->tempConfig.ODR = 1125;
			break;
		case 5:
			pdev->tempConfig.ODR = 1125;
			break;
		case 6:
			pdev->tempConfig.ODR = 1125;
			break;
	}

	//SET SENSITIVITY
	pdev->tempConfig.SensitivityF = 333.87;

	//offset temperatura a 21 grados (esto hay que moverlo de sitio)
	pdev->sensorsOffset.tempRT21Offset = 0;
	pdev->sensorsOffsetProc.tempRT21Offset = 0;
}
void ICM20948_Set_Magneto_Config(ICM20948_DEVICE *pdev, ICM20948_MAGNETO_CONFIG magnetoConfig)
{
	ICM20948_Set_AK09916_Mode(pdev, magnetoConfig.Mode); //set sampling mode of magnetometer
	HAL_Delay(50);
	//AK09916_CNTL2_MODE_VAL md = ICM20948_Get_AK09916_Mode(pdev);

	//STORE config in the main struct
	pdev->magnetoConfig = magnetoConfig;

	//SET SENSITIVITY
	pdev->magnetoConfig.SensitivityF = 0.15;
}
void ICM20948_Calibrate_Accel(ICM20948_DEVICE *pdev, uint8_t seconds)
{
	ICM20948_Get_Accel_Offset(pdev);
	uint16_t i, j = 0; //loops index
	const float period = 0.1; //in seconds
	const uint32_t delayTime = period*1000; //time to wait between readings
	uint16_t readings = seconds/period; //number of readings to do
	uint8_t data[6]; //buffer to hold sensor acceleration reading
	int16_t accelData16[3][readings]; //buffer to hold all 3 axes readings in signed format
	float avgAccel[3]; //average acceleration measured
	ICM20948_3AXES_DATA_INT16 newOffset; //new offset to update
	ICM20948_SetUserBank(pdev, BANK_0);
	for (i=0; i<readings; i++)
	{
		ICM20948_Read_Burst(pdev, ICM20948_ADDR_ACCEL_XOUT_H, &data[0], 6);
		accelData16[0][i] = (data[0]<<8)|data[1];
		accelData16[1][i] = (data[2]<<8)|data[3];
		accelData16[2][i] = (data[4]<<8)|data[5];
		HAL_Delay(delayTime);
	}

	for (j=0; j<3; j++)
	{
		for (i=0; i<readings; i++)
		{
			avgAccel[j] = avgAccel[j] + accelData16[j][i];
		}
	}
	for (j=0; j<3; j++)
	{
		avgAccel[j] = avgAccel[j]/readings;
	}
	newOffset.x = pdev->sensorsOffset.accelOffset.x + (int16_t)avgAccel[0];
	newOffset.y = pdev->sensorsOffset.accelOffset.y + (int16_t)avgAccel[1];
	newOffset.z = pdev->sensorsOffset.accelOffset.z + (int16_t)avgAccel[2];
	ICM20948_Set_Accel_Offset(pdev, newOffset);
}
void ICM20948_Calibrate_Gyro(ICM20948_DEVICE *pdev, uint8_t seconds)
{
	ICM20948_Get_Gyro_Offset(pdev);
	uint16_t i, j = 0; //loops index
	const float period = 0.1; //in seconds
	const uint32_t delayTime = period*1000; //time to wait between readings
	uint16_t readings = seconds/period; //number of readings to do
	uint8_t data[6]; //buffer to hold sensor acceleration reading
	int16_t gyroData16[3][readings]; //buffer to hold all 3 axes readings in signed format
	float avgGyro[3]; //average gyro measured
	ICM20948_3AXES_DATA_INT16 newOffset; //new offset to update
	ICM20948_SetUserBank(pdev, BANK_0);
	for (i=0; i<readings; i++)
	{
		ICM20948_Read_Burst(pdev, ICM20948_ADDR_GYRO_XOUT_H, &data[0], 6);
		gyroData16[0][i] = (data[0]<<8)|data[1];
		gyroData16[1][i] = (data[2]<<8)|data[3];
		gyroData16[2][i] = (data[4]<<8)|data[5];
		HAL_Delay(delayTime);
	}

	for (j=0; j<3; j++)
	{
		for (i=0; i<readings; i++)
		{
			avgGyro[j] = avgGyro[j] + gyroData16[j][i];
		}
	}
	for (j=0; j<3; j++)
	{
		avgGyro[j] = avgGyro[j]/readings;
	}
	newOffset.x = pdev->sensorsOffset.gyroOffset.x + (int16_t)avgGyro[0];
	newOffset.y = pdev->sensorsOffset.gyroOffset.y + (int16_t)avgGyro[1];
	newOffset.z = pdev->sensorsOffset.gyroOffset.z + (int16_t)avgGyro[2];
	ICM20948_Set_Gyro_Offset(pdev, newOffset);
}

//Debug functions
void ICM20948_Get_BANK0_Regs(ICM20948_DEVICE *pdev, uint8_t *pdata)
{
	ICM20948_SetUserBank(pdev, BANK_0);
	ICM20948_Read_Burst(pdev, 0, pdata, 128);
}
void ICM20948_Get_BANK1_Regs(ICM20948_DEVICE *pdev, uint8_t *pdata)
{
	ICM20948_SetUserBank(pdev, BANK_1);
	ICM20948_Read_Burst(pdev, 0, pdata, 128);
}
void ICM20948_Get_BANK2_Regs(ICM20948_DEVICE *pdev, uint8_t *pdata)
{
	ICM20948_SetUserBank(pdev, BANK_2);
	ICM20948_Read_Burst(pdev, 0, pdata, 128);
}
void ICM20948_Get_BANK3_Regs(ICM20948_DEVICE *pdev, uint8_t *pdata)
{
	ICM20948_SetUserBank(pdev, BANK_3);
	ICM20948_Read_Burst(pdev, 0, pdata, 128);
}

//BANK 0--------------------------------------------------------------------
//Lower level functions
//WHO_AM_I
uint8_t ICM20948_Get_ICM20948_Id(ICM20948_DEVICE *pdev)
{
	ICM20948_SetUserBank(pdev,BANK_0);
	uint8_t ICM20948_id;
	ICM20948_Read_Byte(pdev,ICM20948_ADDR_WHO_AM_I, &ICM20948_id);
	pdev->id = ICM20948_id;
	return ICM20948_id;
}
//USER_CTRL
void ICM20948_MasterI2C_Reset(ICM20948_DEVICE *pdev)
{
	ICM20948_Write_RegBits(pdev, BANK_0, ICM20948_ADDR_USER_CTRL, ICM20948_USER_CTRL_I2C_MST_RST, 1);
}
void ICM20948_SlaveI2C_Reset(ICM20948_DEVICE *pdev)
{
	ICM20948_Write_RegBits(pdev, BANK_0, ICM20948_ADDR_USER_CTRL, ICM20948_USER_CTRL_I2C_IF_DIS, 1);
}
void ICM20948_MasterI2C_Enable(ICM20948_DEVICE *pdev, ICM20948_BOOL_REG_VAL val)
{
	ICM20948_Write_RegBits(pdev, BANK_0, ICM20948_ADDR_USER_CTRL, ICM20948_USER_CTRL_I2C_MST_EN, val);
}
void ICM20948_FIFO_Enable(ICM20948_DEVICE *pdev, ICM20948_BOOL_REG_VAL val)
{
	ICM20948_Write_RegBits(pdev, BANK_0, ICM20948_ADDR_USER_CTRL, ICM20948_USER_CTRL_FIFO_EN, val);
}
//LP_CONFIG
void ICM20948_GYRO_LP(ICM20948_DEVICE *pdev, ICM20948_BOOL_REG_VAL val)
{
	ICM20948_Write_RegBits(pdev, BANK_0, ICM20948_ADDR_LP_CONFIG, ICM20948_LP_CONFIG_GYRO_CYCLE, val);
}
void ICM20948_ACCEL_LP(ICM20948_DEVICE *pdev, ICM20948_BOOL_REG_VAL val)
{
	ICM20948_Write_RegBits(pdev, BANK_0, ICM20948_ADDR_LP_CONFIG, ICM20948_LP_CONFIG_ACCEL_CYCLE, val);
}
void ICM20948_I2CMST_LP(ICM20948_DEVICE *pdev, ICM20948_BOOL_REG_VAL val)
{
	ICM20948_Write_RegBits(pdev, BANK_0, ICM20948_ADDR_LP_CONFIG, ICM20948_LP_CONFIG_I2CMST_CYCLE, val);
}
//PWR_MGMT_1
void ICM20948_Device_Reset(ICM20948_DEVICE *pdev)
{
	ICM20948_SetUserBank(pdev, BANK_0);
	uint8_t val = ICM20948_PWR_MGMT_1_DEVICE_RESET;
	ICM20948_Write_Byte(pdev, ICM20948_ADDR_PWR_MGMT1, &val);
}
void ICM20948_Sleep(ICM20948_DEVICE *pdev, ICM20948_PWR_MGMT_1_SLEEP_VAL val)
{
	ICM20948_Write_RegBits(pdev, BANK_0, ICM20948_ADDR_PWR_MGMT1, ICM20948_PWR_MGMT_1_SLEEP, val);
}
void ICM20948_LowPower_Digital(ICM20948_DEVICE *pdev, ICM20948_BOOL_REG_VAL val)
{
	ICM20948_Write_RegBits(pdev, BANK_0, ICM20948_ADDR_PWR_MGMT1, ICM20948_PWR_MGMT_1_LP_EN, val);
}
void ICM20948_Disable_Temp(ICM20948_DEVICE *pdev, ICM20948_BOOL_REG_VAL val)
{
	ICM20948_Write_RegBits(pdev, BANK_0, ICM20948_ADDR_PWR_MGMT1, ICM20948_PWR_MGMT_1_TEMP_DIS, val);

}
void ICM20948_Select_ClockSRC(ICM20948_DEVICE *pdev, ICM20948_PWR_MGMT_1_CLKSEL_VAL val)
{
	ICM20948_Write_RegBits(pdev, BANK_0, ICM20948_ADDR_PWR_MGMT1, ICM20948_PWR_MGMT_1_CLKSEL, val);
}
//PWR_MGMT_2
void ICM20948_Gyro_Power(ICM20948_DEVICE *pdev, ICM20948_PWR_MGMT_2_VAL val)
{
	ICM20948_Write_RegBits(pdev, BANK_0, ICM20948_ADDR_PWR_MGMT2, ICM20948_PWR_MGMT2_DISABLE_GYRO, val);
}
void ICM20948_Accel_Power(ICM20948_DEVICE *pdev, ICM20948_PWR_MGMT_2_VAL val)
{
	ICM20948_Write_RegBits(pdev, BANK_0, ICM20948_ADDR_PWR_MGMT2, ICM20948_PWR_MGMT2_DISABLE_ACCEL, val);
}
//I2C_MST_STATUS
ICM20948_BOOL_REG_VAL ICM20948_Get_I2C_SLV4_Done(ICM20948_DEVICE *pdev)
{
	ICM20948_BOOL_REG_VAL result;
	ICM20948_Read_RegBits(pdev, BANK_0, ICM20948_ADDR_I2C_MSC_STATUS, &result, ICM20948_I2C_MSC_STATUS_I2C_SLV4_DONE, 1);
	return result;
}
//INT_STATUS_1
ICM20948_BOOL_REG_VAL ICM20948_IsRawDataReady(ICM20948_DEVICE *pdev)
{
	ICM20948_SetUserBank(pdev, BANK_0);
	ICM20948_BOOL_REG_VAL val;
	ICM20948_Read_Byte(pdev, ICM20948_ADDR_INT_STATUS_1, &val);
	return val;
}
//ACCEL_OUT
void ICM20948_Get_Accel_RawData(ICM20948_DEVICE *pdev)
{
	uint8_t data[6];
	ICM20948_SetUserBank(pdev, BANK_0);
	ICM20948_Read_Burst(pdev, ICM20948_ADDR_ACCEL_XOUT_H, &data[0], 6);
	pdev->sensorsData.accelData.x = (data[0]<<8)|data[1];
	pdev->sensorsData.accelData.y = (data[2]<<8)|data[3];
	pdev->sensorsData.accelData.z = (data[4]<<8)|data[5];
	pdev->sensorsDataProc.accelData.x = pdev->sensorsData.accelData.x/pdev->accelConfig.SensitivityF;
	pdev->sensorsDataProc.accelData.y = pdev->sensorsData.accelData.y/pdev->accelConfig.SensitivityF;
	pdev->sensorsDataProc.accelData.z = pdev->sensorsData.accelData.z/pdev->accelConfig.SensitivityF;
}
//GYRO_OUT
void ICM20948_Get_Gyro_RawData(ICM20948_DEVICE *pdev)
{
	uint8_t data[6];
	ICM20948_SetUserBank(pdev, BANK_0);
	ICM20948_Read_Burst(pdev, ICM20948_ADDR_GYRO_XOUT_H, &data[0], 6);
	pdev->sensorsData.gyroData.x = ((data[0]<<8)|data[1]);
	pdev->sensorsData.gyroData.y = ((data[2]<<8)|data[3]);
	pdev->sensorsData.gyroData.z = ((data[4]<<8)|data[5]);
	pdev->sensorsDataProc.gyroData.x = pdev->sensorsData.gyroData.x/pdev->gyroConfig.SensitivityF;
	pdev->sensorsDataProc.gyroData.y = pdev->sensorsData.gyroData.y/pdev->gyroConfig.SensitivityF;
	pdev->sensorsDataProc.gyroData.z = pdev->sensorsData.gyroData.z/pdev->gyroConfig.SensitivityF;
}
//TEMP_OUT
void ICM20948_Get_Temp_RawData(ICM20948_DEVICE *pdev)
{
	uint8_t data[2];
	ICM20948_SetUserBank(pdev, BANK_0);
	ICM20948_Read_Burst(pdev, ICM20948_ADDR_TEMP_OUT_H, &data[0], 2);
	pdev->sensorsData.tempData = ((data[0]<<8)|data[1]);
	pdev->sensorsDataProc.tempData = ((pdev->sensorsData.tempData-pdev->sensorsOffsetProc.tempRT21Offset)/pdev->tempConfig.SensitivityF)+21;
}
//ACCEL_OUT + GYRO_OUT
void ICM20948_Get_AccelGyro_RawData(ICM20948_DEVICE *pdev)
{
	uint8_t data[12];
	ICM20948_SetUserBank(pdev, BANK_0);
	ICM20948_Read_Burst(pdev, ICM20948_ADDR_ACCEL_XOUT_H, &data[0], 12);
	pdev->sensorsData.accelData.x = (data[0]<<8)|data[1];
	pdev->sensorsData.accelData.y = (data[2]<<8)|data[3];
	pdev->sensorsData.accelData.z = (data[4]<<8)|data[5];
	pdev->sensorsData.gyroData.x = ((data[6]<<8)|data[7]);
	pdev->sensorsData.gyroData.y = ((data[8]<<8)|data[9]);
	pdev->sensorsData.gyroData.z = ((data[10]<<8)|data[11]);
	pdev->sensorsDataProc.accelData.x = pdev->sensorsData.accelData.x/pdev->accelConfig.SensitivityF;
	pdev->sensorsDataProc.accelData.y = pdev->sensorsData.accelData.y/pdev->accelConfig.SensitivityF;
	pdev->sensorsDataProc.accelData.z = pdev->sensorsData.accelData.z/pdev->accelConfig.SensitivityF;
	pdev->sensorsDataProc.gyroData.x = pdev->sensorsData.gyroData.x/pdev->gyroConfig.SensitivityF;
	pdev->sensorsDataProc.gyroData.y = pdev->sensorsData.gyroData.y/pdev->gyroConfig.SensitivityF;
	pdev->sensorsDataProc.gyroData.z = pdev->sensorsData.gyroData.z/pdev->gyroConfig.SensitivityF;
}
//MAGNETOMETER_OUT (EXT_SLV_SENS_DATA_00 TO 08)
void ICM20948_Get_Magneto_RawData(ICM20948_DEVICE *pdev)
{
	uint8_t data[6];
	ICM20948_SetUserBank(pdev, BANK_0);
	ICM20948_Read_Burst(pdev, ICM20948_ADDR_EXT_SLV_SENS_DATA_01, &data[0], 6);
	pdev->sensorsData.magnetoData.x = (data[1]<<8)|data[0]; //little endian
	pdev->sensorsData.magnetoData.y = (data[3]<<8)|data[2]; //little endian
	pdev->sensorsData.magnetoData.z = (data[5]<<8)|data[4]; //little endian
	pdev->sensorsDataProc.magnetoData.x = pdev->sensorsData.magnetoData.x * pdev->magnetoConfig.SensitivityF;
	pdev->sensorsDataProc.magnetoData.y = pdev->sensorsData.magnetoData.y * pdev->magnetoConfig.SensitivityF;
	pdev->sensorsDataProc.magnetoData.z = pdev->sensorsData.magnetoData.z * pdev->magnetoConfig.SensitivityF;
}

//BANK 1--------------------------------------------------------------------
//ACCELEROMETER OFFSET
void ICM20948_Get_Accel_Offset(ICM20948_DEVICE *pdev)
{
	uint8_t data[6];
	ICM20948_SetUserBank(pdev, BANK_1);
	ICM20948_Read_Burst(pdev, ICM20948_ADDR_XA_OFFS_H, &data[0], 6);
	pdev->sensorsOffset.accelOffset.x = (data[0]<<8)|data[1];
	pdev->sensorsOffset.accelOffset.y = (data[2]<<8)|data[3];
	pdev->sensorsOffset.accelOffset.z = (data[4]<<8)|data[5];
	pdev->sensorsOffsetProc.accelOffset.x = pdev->sensorsOffset.accelOffset.x/16;
	pdev->sensorsOffsetProc.accelOffset.y = pdev->sensorsOffset.accelOffset.y/16;
	pdev->sensorsOffsetProc.accelOffset.z = pdev->sensorsOffset.accelOffset.z/16;
}
void ICM20948_Set_Accel_Offset(ICM20948_DEVICE *pdev, ICM20948_3AXES_DATA_INT16 offset)
{
	uint8_t data[6];
	data[0] = offset.x>>8;
	data[1] = offset.x;
	data[2] = offset.y>>8;
	data[3] = offset.y;
	data[4] = offset.z>>8;
	data[5] = offset.z;
	ICM20948_SetUserBank(pdev, BANK_1);
	ICM20948_Write_Burst(pdev, ICM20948_ADDR_XA_OFFS_H, &data[0], 6);
	ICM20948_Get_Accel_Offset(pdev); //update pdev struct with the new values
}
//BANK 2--------------------------------------------------------------------
//GYRO_SMPLRT_DIV
void ICM20948_Gyro_SMPLRTDiv_Set(ICM20948_DEVICE *pdev, uint8_t val)
{
	ICM20948_Write_FullReg(pdev, BANK_2, ICM20948_ADDR_GYRO_SMPLRT_DIV, val);
}
//GYRO_CONFIG_1
void ICM20948_Gyro_DLPF_Enable(ICM20948_DEVICE *pdev, ICM20948_SENSOR_FCHOICE_VAL val)
{
	ICM20948_Write_RegBits(pdev, BANK_2, ICM20948_ADDR_GYRO_CONFIG_1, ICM20948_GYRO_FCHOICE, val);
}
void ICM20948_Gyro_FS_Select(ICM20948_DEVICE *pdev, ICM20948_GYRO_FS_SEL_VAL val)
{
	ICM20948_Write_RegBits(pdev, BANK_2, ICM20948_ADDR_GYRO_CONFIG_1, ICM20948_GYRO_FS_SEL, val);
}
void ICM20948_Gyro_DLPF_SelectFc(ICM20948_DEVICE *pdev, ICM20948_GYRO_DLPFCFG_VAL val)
{
	ICM20948_Write_RegBits(pdev, BANK_2, ICM20948_ADDR_GYRO_CONFIG_1, ICM20948_GYRO_DLPFCFG, val);
}
//GYRO_CONFIG_2
void ICM20948_Gyro_Averaging_Select(ICM20948_DEVICE *pdev, ICM20948_GYRO_CONFIG_2_GYRO_AVGCFG_VAL val)
{
	ICM20948_Write_RegBits(pdev, BANK_2, ICM20948_ADDR_GYRO_CONFIG_2, ICM20948_GYRO_CONFIG_2_GYRO_AVGCFG, val);
}
//GYRO OFFSET
void ICM20948_Get_Gyro_Offset(ICM20948_DEVICE *pdev)
{
	uint8_t data[6];
	ICM20948_SetUserBank(pdev, BANK_2);
	ICM20948_Read_Burst(pdev, ICM20948_ADDR_XG_OFFS_USRH, &data[0], 6);
	pdev->sensorsOffset.gyroOffset.x = (data[0]<<8)|data[1];
	pdev->sensorsOffset.gyroOffset.y = (data[2]<<8)|data[3];
	pdev->sensorsOffset.gyroOffset.z = (data[4]<<8)|data[5];
	pdev->sensorsOffsetProc.gyroOffset.x = pdev->sensorsOffset.gyroOffset.x/2000;
	pdev->sensorsOffsetProc.gyroOffset.y = pdev->sensorsOffset.gyroOffset.y/2000;
	pdev->sensorsOffsetProc.gyroOffset.z = pdev->sensorsOffset.gyroOffset.z/2000;
}
void ICM20948_Set_Gyro_Offset(ICM20948_DEVICE *pdev, ICM20948_3AXES_DATA_INT16 offset)
{
	uint8_t data[6];
	data[0] = offset.x>>8;
	data[1] = offset.x;
	data[2] = offset.y>>8;
	data[3] = offset.y;
	data[4] = offset.z>>8;
	data[5] = offset.z;
	ICM20948_SetUserBank(pdev, BANK_2);
	ICM20948_Write_Burst(pdev, ICM20948_ADDR_XG_OFFS_USRH, &data[0], 6);
	ICM20948_Get_Gyro_Offset(pdev); //update pdev struct with the new values
}

//ACCEL_SMPLRT_DIV_1 & ACCEL_SMPLRT_DIV_2
void ICM20948_Accel_SMPLRTDiv_Set(ICM20948_DEVICE *pdev, uint16_t val)
{
	uint8_t MSBval = val>>8;
	uint8_t LSBval = val;
	ICM20948_Write_FullReg(pdev, BANK_2, ICM20948_ADDR_ACCEL_SMPLRT_DIV_1, MSBval);
	ICM20948_Write_FullReg(pdev, BANK_2, ICM20948_ADDR_ACCEL_SMPLRT_DIV_2, LSBval);
}
//ACCEL_CONFIG
void ICM20948_Accel_DLPF_Enable(ICM20948_DEVICE *pdev, ICM20948_SENSOR_FCHOICE_VAL val)
{
	ICM20948_Write_RegBits(pdev, BANK_2, ICM20948_ADDR_ACCEL_CONFIG, ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE, val);
}
void ICM20948_Accel_FS_Select(ICM20948_DEVICE *pdev, ICM20948_ACCEL_FS_SEL_VAL val)
{
	ICM20948_Write_RegBits(pdev, BANK_2, ICM20948_ADDR_ACCEL_CONFIG, ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL, val);
}
void ICM20948_Accel_DLPF_SelectFc(ICM20948_DEVICE *pdev, ICM20948_ACCEL_DLPFCFG_VAL val)
{
	ICM20948_Write_RegBits(pdev, BANK_2, ICM20948_ADDR_ACCEL_CONFIG, ICM20948_ACCEL_CONFIG_ACCEL_DLPFCFG, val);
}
//ACCEL_CONFIG_2
void ICM20948_Accel_Averaging_Select(ICM20948_DEVICE *pdev, ICM20948_ACCEL_CONFIG_2_DEC3_CFG_VAL val)
{
	ICM20948_Write_RegBits(pdev, BANK_2, ICM20948_ADDR_ACCEL_CONFIG_2, ICM20948_ACCEL_CONFIG_2_DEC3_CFG, val);
}
//TEMP_CONFIG
void ICM20948_Temp_DLPF_SelectFc(ICM20948_DEVICE *pdev, ICM20948_TEMP_CONFIG_TEMP_DLPFCFG_VAL val)
{
	ICM20948_Write_FullReg(pdev, BANK_2, ICM20948_ADDR_TEMP_CONFIG, val);
}

//BANK 3--------------------------------------------------------------------
//I2C_MST_CTRL
void ICM20948_Set_I2C_MST_CLK(ICM20948_DEVICE *pdev, uint8_t clk)
{
	ICM20948_Write_RegBits(pdev, BANK_3, ICM20948_ADDR_I2C_MST_CTRL, ICM20948_I2C_MST_CTRL_I2C_MST_CLK, clk);
}
//SLV0, SLV1, SLV2, SLV3
void ICM20948_Config_I2C_SLV(ICM20948_DEVICE *pdev, uint8_t SLV, uint8_t dev_I2C_Address, uint8_t firstRegister, uint8_t nBytes)
{
	dev_I2C_Address = (1<<7)|dev_I2C_Address; //we are configuring I2C sensor reads
	uint8_t I2C_SLV_ADDR_Reg;
	uint8_t I2C_SLV_REG_Reg;
	uint8_t I2C_SLV_CTRL_Reg;
	switch (SLV)
	{
		case 0: //SLV0
			I2C_SLV_ADDR_Reg = ICM20948_ADDR_I2C_SLV0_ADDR;
			I2C_SLV_REG_Reg = ICM20948_ADDR_I2C_SLV0_REG;
			I2C_SLV_CTRL_Reg = ICM20948_ADDR_I2C_SLV0_CTRL;
			break;
		case 1: //SLV1
			I2C_SLV_ADDR_Reg = ICM20948_ADDR_I2C_SLV1_ADDR;
			I2C_SLV_REG_Reg = ICM20948_ADDR_I2C_SLV1_REG;
			I2C_SLV_CTRL_Reg = ICM20948_ADDR_I2C_SLV1_CTRL;
			break;
		case 2: //SLV2
			I2C_SLV_ADDR_Reg = ICM20948_ADDR_I2C_SLV2_ADDR;
			I2C_SLV_REG_Reg = ICM20948_ADDR_I2C_SLV2_REG;
			I2C_SLV_CTRL_Reg = ICM20948_ADDR_I2C_SLV2_CTRL;
			break;
		case 3: //SLV3
			I2C_SLV_ADDR_Reg = ICM20948_ADDR_I2C_SLV3_ADDR;
			I2C_SLV_REG_Reg = ICM20948_ADDR_I2C_SLV3_REG;
			I2C_SLV_CTRL_Reg = ICM20948_ADDR_I2C_SLV3_CTRL;
			break;
	}
	uint8_t CTRLRegVal = 0x80 | nBytes; //0x80: EN=1; BYTE_SW=0; REG_DIS=0; GRP=0;
	ICM20948_Write_FullReg(pdev, BANK_3, I2C_SLV_ADDR_Reg, dev_I2C_Address);
	ICM20948_Write_FullReg(pdev, BANK_3, I2C_SLV_REG_Reg, firstRegister);
	ICM20948_Write_FullReg(pdev, BANK_3, I2C_SLV_CTRL_Reg, CTRLRegVal);
}
//SLV4
void ICM20948_I2C_SLV4_TransferBytes(ICM20948_DEVICE *pdev, uint8_t dev_I2C_Address, uint8_t reg, uint8_t *pdata, uint8_t nBytes, uint8_t rw)
{
	dev_I2C_Address = (rw<<7)|dev_I2C_Address;
	ICM20948_Write_FullReg(pdev, BANK_3, ICM20948_ADDR_I2C_SLV4_ADDR, dev_I2C_Address);
	ICM20948_Write_FullReg(pdev, BANK_3, ICM20948_ADDR_I2C_SLV4_REG, reg);
	uint8_t SLV4_CTRL_val = 0x80; //other config will not be used...
	uint8_t i;
	for (i=0; i<nBytes; i++)
	{
		if (!rw) //if writing registers to device
		{
			ICM20948_Write_FullReg(pdev, BANK_3, ICM20948_ADDR_I2C_SLV4_DO, *(pdata+i));
		}
		ICM20948_Write_FullReg(pdev, BANK_3, ICM20948_ADDR_I2C_SLV4_CTRL, SLV4_CTRL_val);
		ICM20948_BOOL_REG_VAL isDone = ICM20948_False;
		while (isDone == ICM20948_False) //wait while the transfer is not completed
		{
			isDone = ICM20948_Get_I2C_SLV4_Done(pdev);
		}
		if (rw) //if reading registers from device
		{
			ICM20948_Read_FullReg(pdev, BANK_3, ICM20948_ADDR_I2C_SLV4_DI, pdata+i);
		}
	}
}

//MAGNETOMETER FUNCTIONS--------------------------------------------------------------------
uint8_t ICM20948_Get_AK09916_Id(ICM20948_DEVICE *pdev)
{
	uint8_t Id;
	ICM20948_Read_AK09916_Byte(pdev, AK09916_ADDR_WIA2, &Id);
	return Id;
}
void ICM20948_AK09916_Reset(ICM20948_DEVICE *pdev)
{
	uint8_t data = 1;
	ICM20948_Write_AK09916_Byte(pdev, AK09916_ADDR_CNTL3, &data);
}
void ICM20948_Set_AK09916_Mode(ICM20948_DEVICE *pdev, AK09916_CNTL2_MODE_VAL mode)
{
	ICM20948_Write_AK09916_Byte(pdev, AK09916_ADDR_CNTL2, &mode);
}
AK09916_CNTL2_MODE_VAL ICM20948_Get_AK09916_Mode(ICM20948_DEVICE *pdev)
{
	AK09916_CNTL2_MODE_VAL mode;
	ICM20948_Read_AK09916_Byte(pdev, AK09916_ADDR_CNTL2, &mode);
	return mode;
}

void ICM20948_Read_AK09916_Byte(ICM20948_DEVICE *pdev, uint8_t regaddr, uint8_t *pdata)
{
	ICM20948_I2C_SLV4_TransferBytes(pdev, AK09916_I2C_ADDR, regaddr, pdata, 1, 1);
}
void ICM20948_Write_AK09916_Byte(ICM20948_DEVICE *pdev, uint8_t regaddr, uint8_t *pdata)
{
	ICM20948_I2C_SLV4_TransferBytes(pdev, AK09916_I2C_ADDR, regaddr, pdata, 1, 0);
}








