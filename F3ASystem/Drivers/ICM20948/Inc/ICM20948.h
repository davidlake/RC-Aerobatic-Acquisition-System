/*
 * ICM20948.h
 *
 *  Created on: Apr 9, 2021
 *      Author: dlago
 */

#ifndef ICM20948_INC_ICM20948_H_
#define ICM20948_INC_ICM20948_H_

#include <AK09916_Registers.h>
#include "main.h"
#include "HelperFcns.h"
#include <string.h>
#include "ICM20948_Registers.h"

typedef enum _ICM20948_SENSOR_POWER_MODE
{
	ICM20948_SENSOR_PM_Off = 0,
	ICM20948_SENSOR_PM_On = 1,
	ICM20948_SENSOR_PM_LP = 2,
}ICM20948_SENSOR_POWER_MODE;
typedef struct _ICM20948_GYRO_CONFIG
{
	ICM20948_SENSOR_POWER_MODE PowerMode;
	ICM20948_SENSOR_FCHOICE_VAL DLPF_State;
	ICM20948_GYRO_DLPFCFG_VAL DLPF_Val;
	uint8_t SMPLRT_Div;
	ICM20948_GYRO_CONFIG_2_GYRO_AVGCFG_VAL AVGF_Val;
	ICM20948_GYRO_FS_SEL_VAL FS;
	float ODR; //dependent, do not set by user
	float SensitivityF; //dependent, do not set by user
}ICM20948_GYRO_CONFIG;
typedef struct _ICM20948_ACCEL_CONFIG
{
	ICM20948_SENSOR_POWER_MODE PowerMode;
	ICM20948_SENSOR_FCHOICE_VAL DLPF_State;
	ICM20948_ACCEL_DLPFCFG_VAL DLPF_Val;
	uint16_t SMPLRT_Div;
	ICM20948_ACCEL_CONFIG_2_DEC3_CFG_VAL AVGF_Val;
	ICM20948_ACCEL_FS_SEL_VAL FS;
	float ODR; //dependent, do not set by user
	float SensitivityF; //dependent, do not set by user
}ICM20948_ACCEL_CONFIG;
typedef struct _ICM20948_TEMP_CONFIG
{
	ICM20948_SENSOR_POWER_MODE PowerMode;
	ICM20948_TEMP_CONFIG_TEMP_DLPFCFG_VAL DLPF_Val;
	float ODR;
	float SensitivityF;
}ICM20948_TEMP_CONFIG;
typedef struct _ICM20948_MAGNETO_CONFIG
{
	AK09916_CNTL2_MODE_VAL Mode;
	float SensitivityF;
}ICM20948_MAGNETO_CONFIG;
typedef struct _ICM20948_3AXES_DATA_INT16
{
	int16_t x;
	int16_t y;
	int16_t z;
} ICM20948_3AXES_DATA_INT16;
typedef struct _ICM20948_3AXES_DATA_FLOAT
{
	float x;
	float y;
	float z;
} ICM20948_3AXES_DATA_FLOAT;
typedef struct _ICM20948_SENSORS_RAW_DATA
{
	ICM20948_3AXES_DATA_INT16 accelData;
	ICM20948_3AXES_DATA_INT16 gyroData;
	ICM20948_3AXES_DATA_INT16 magnetoData;
	int16_t tempData;
}ICM20948_SENSORS_RAW_DATA;
typedef struct _ICM20948_SENSORS_RAW_DATA_PROC
{
	ICM20948_3AXES_DATA_FLOAT accelData;
	ICM20948_3AXES_DATA_FLOAT gyroData;
	ICM20948_3AXES_DATA_FLOAT magnetoData;
	float tempData;
}ICM20948_SENSORS_RAW_DATA_PROC;
typedef struct _ICM20948_SENSORS_OFFSET
{
	ICM20948_3AXES_DATA_INT16 accelOffset;
	ICM20948_3AXES_DATA_INT16 gyroOffset;
	ICM20948_3AXES_DATA_INT16 magnetoOffset;
	int16_t tempRT21Offset;
}ICM20948_SENSORS_OFFSET;
typedef struct _ICM20948_SENSORS_OFFSET_PROC
{
	ICM20948_3AXES_DATA_FLOAT accelOffset;
	ICM20948_3AXES_DATA_FLOAT gyroOffset;
	ICM20948_3AXES_DATA_FLOAT magnetoOffset;
	float tempRT21Offset;
}ICM20948_SENSORS_OFFSET_PROC;
typedef enum _ICM20948_REG_COMP_RESULT
{
	ICM20948_REG_COMP_Match = 1,
	ICM20948_REG_COMP_Mismatch = 0,
}ICM20948_REG_COMP_RESULT;
typedef struct _ICM20948_HAL_SERIF //saves device serial interface settings
{
	SPI_HandleTypeDef *SPI_HANDLE;
	GPIO_TypeDef *CS_PIN_PORT;
	uint16_t CS_PIN_NUMBER;
} ICM20948_HAL_SERIF;
typedef struct _ICM20948_DEVICE //main data struct to store some device settings
{
	uint8_t id;
	uint8_t idMag;
	ICM20948_HAL_SERIF serif;
	ICM20948_ADDR_REG_BANK_SEL_VAL currentBank;
	ICM20948_GYRO_CONFIG gyroConfig;
	ICM20948_ACCEL_CONFIG accelConfig;
	ICM20948_TEMP_CONFIG tempConfig;
	ICM20948_MAGNETO_CONFIG magnetoConfig;
	ICM20948_SENSORS_OFFSET sensorsOffset;
	ICM20948_SENSORS_OFFSET_PROC sensorsOffsetProc;
	ICM20948_SENSORS_RAW_DATA sensorsData;
	ICM20948_SENSORS_RAW_DATA_PROC sensorsDataProc;

} ICM20948_DEVICE;

//High level functions
void ICM20948_Init_Device(ICM20948_DEVICE *pdev, SPI_HandleTypeDef *SPI_HANDLE, GPIO_TypeDef *CS_PIN_PORT, uint16_t CS_PIN_NUMBER);
void ICM20948_Init_AK09916(ICM20948_DEVICE *pdev);
void ICM20948_Set_Sensors_Default_Config(ICM20948_DEVICE *pdev);
void ICM20948_Get_AGMT_RawData(ICM20948_DEVICE *pdev); //get all sensors raw data (no DMP processed). Accelerometer, gyro, magnetometer and temperature
void ICM20948_Get_Sensors_Offset(ICM20948_DEVICE *pdev); //no hace falta
void ICM20948_Set_Accel_Config(ICM20948_DEVICE *pdev, ICM20948_ACCEL_CONFIG accelConfig);
void ICM20948_Set_Gyro_Config(ICM20948_DEVICE *pdev, ICM20948_GYRO_CONFIG gyroConfig);
void ICM20948_Set_Temp_Config(ICM20948_DEVICE *pdev, ICM20948_TEMP_CONFIG tempConfig);
void ICM20948_Set_Magneto_Config(ICM20948_DEVICE *pdev, ICM20948_MAGNETO_CONFIG magnetoConfig);
void ICM20948_Calibrate_Accel(ICM20948_DEVICE *pdev, uint8_t seconds); //esta funcion esta mal (el primer bit es reservado)
void ICM20948_Calibrate_Gyro(ICM20948_DEVICE *pdev, uint8_t seconds);

//Debug functions
void ICM20948_Get_BANK0_Regs(ICM20948_DEVICE *pdev, uint8_t *pdata);
void ICM20948_Get_BANK1_Regs(ICM20948_DEVICE *pdev, uint8_t *pdata);
void ICM20948_Get_BANK2_Regs(ICM20948_DEVICE *pdev, uint8_t *pdata);
void ICM20948_Get_BANK3_Regs(ICM20948_DEVICE *pdev, uint8_t *pdata);

//Lower level functions

//BANK 0---------------------------------------------------------------------------------------------------------------------
//WHO_AM_I
uint8_t ICM20948_Get_ICM20948_Id(ICM20948_DEVICE *pdev);
//USER_CTRL
void ICM20948_MasterI2C_Reset(ICM20948_DEVICE *pdev); //resets I2C master module if it hangs
void ICM20948_SlaveI2C_Reset(ICM20948_DEVICE *pdev); //resets I2C slave module. SPI mode only communication with external MCU
void ICM20948_MasterI2C_Enable(ICM20948_DEVICE *pdev, ICM20948_BOOL_REG_VAL val); //enable/disable I2C master module
void ICM20948_FIFO_Enable(ICM20948_DEVICE *pdev, ICM20948_BOOL_REG_VAL val); //enable/disable FIFO
//LP_CONFIG (sensor power modes)
void ICM20948_GYRO_LP(ICM20948_DEVICE *pdev, ICM20948_BOOL_REG_VAL val); //enable disable gyro low power mode
void ICM20948_ACCEL_LP(ICM20948_DEVICE *pdev, ICM20948_BOOL_REG_VAL val); //enable disable accelerometer low power mode
void ICM20948_I2CMST_LP(ICM20948_DEVICE *pdev, ICM20948_BOOL_REG_VAL val); //enable disable I2C master low power mode
//PWR_MGMT_1
void ICM20948_Device_Reset(ICM20948_DEVICE *pdev); //software reset of ICM20948
void ICM20948_Sleep(ICM20948_DEVICE *pdev, ICM20948_PWR_MGMT_1_SLEEP_VAL val); //sensors and DMP off
void ICM20948_LowPower_Digital(ICM20948_DEVICE *pdev, ICM20948_BOOL_REG_VAL val); //digital circuits in low power on or off
void ICM20948_Disable_Temp(ICM20948_DEVICE *pdev, ICM20948_BOOL_REG_VAL val); //enable/disable temperature sensor
void ICM20948_Select_ClockSRC(ICM20948_DEVICE *pdev, ICM20948_PWR_MGMT_1_CLKSEL_VAL val); //select clock source
//PWR_MGMT_2
void ICM20948_Gyro_Power(ICM20948_DEVICE *pdev, ICM20948_PWR_MGMT_2_VAL val); //turn gyro on/off
void ICM20948_Accel_Power(ICM20948_DEVICE *pdev, ICM20948_PWR_MGMT_2_VAL val); //turn accelerometer on/off
//I2C_MST_STATUS
ICM20948_BOOL_REG_VAL ICM20948_Get_I2C_SLV4_Done(ICM20948_DEVICE *pdev);
//INT_STATUS_1
ICM20948_BOOL_REG_VAL ICM20948_IsRawDataReady(ICM20948_DEVICE *pdev); //check if new raw data from sensors is ready to be read
//ACCEL_OUT
void ICM20948_Get_Accel_RawData(ICM20948_DEVICE *pdev);
//GYRO_OUT
void ICM20948_Get_Gyro_RawData(ICM20948_DEVICE *pdev);
//TEMP_OUT
void ICM20948_Get_Temp_RawData(ICM20948_DEVICE *pdev);
//ACCEL_OUT + GYRO_OUT
void ICM20948_Get_AccelGyro_RawData(ICM20948_DEVICE *pdev);
//MAGNETOMETER_OUT (EXT_SLV_SENS_DATA_00 TO 08)
void ICM20948_Get_Magneto_RawData(ICM20948_DEVICE *pdev);


//BANK 1---------------------------------------------------------------------------------------------------------------------
//ACCELEROMETER OFFSET
void ICM20948_Get_Accel_Offset(ICM20948_DEVICE *pdev);
void ICM20948_Set_Accel_Offset(ICM20948_DEVICE *pdev, ICM20948_3AXES_DATA_INT16 offset);

//BANK 2---------------------------------------------------------------------------------------------------------------------
//GYRO_SMPLRT_DIV
void ICM20948_Gyro_SMPLRTDiv_Set(ICM20948_DEVICE *pdev, uint8_t val);//sets sample rate divider for gyro (only used if DLPF activated)
//GYRO_CONFIG_1
void ICM20948_Gyro_DLPF_Enable(ICM20948_DEVICE *pdev, ICM20948_SENSOR_FCHOICE_VAL val);//enable/disable gyro DLPF
void ICM20948_Gyro_FS_Select(ICM20948_DEVICE *pdev, ICM20948_GYRO_FS_SEL_VAL val);//sets gyro full scale value (dps)
void ICM20948_Gyro_DLPF_SelectFc(ICM20948_DEVICE *pdev, ICM20948_GYRO_DLPFCFG_VAL val);//sets gyro DLPF fc
//GYRO_CONFIG_2
void ICM20948_Gyro_Averaging_Select(ICM20948_DEVICE *pdev, ICM20948_GYRO_CONFIG_2_GYRO_AVGCFG_VAL val);//sets gyro averaging filter (only works in low power mode)
//GYRO OFFSET
void ICM20948_Get_Gyro_Offset(ICM20948_DEVICE *pdev);
void ICM20948_Set_Gyro_Offset(ICM20948_DEVICE *pdev, ICM20948_3AXES_DATA_INT16 offset);
//ACCEL_SMPLRT_DIV_1 & ACCEL_SMPLRT_DIV_2
void ICM20948_Accel_SMPLRTDiv_Set(ICM20948_DEVICE *pdev, uint16_t val);//sets sample rate divider for accelerometer
//ACCEL_CONFIG
void ICM20948_Accel_DLPF_Enable(ICM20948_DEVICE *pdev, ICM20948_SENSOR_FCHOICE_VAL val);//enable/disable accelerometer DLPF
void ICM20948_Accel_FS_Select(ICM20948_DEVICE *pdev, ICM20948_ACCEL_FS_SEL_VAL val);//sets accelerometer full scale value (Gs)
void ICM20948_Accel_DLPF_SelectFc(ICM20948_DEVICE *pdev, ICM20948_ACCEL_DLPFCFG_VAL val);//sets accelerometer DLPF fc
//ACCEL_CONFIG_2
void ICM20948_Accel_Averaging_Select(ICM20948_DEVICE *pdev, ICM20948_ACCEL_CONFIG_2_DEC3_CFG_VAL val);//sets accelerometer averaging filter (effect depends on fchoice val)
//TEMP_CONFIG
void ICM20948_Temp_DLPF_SelectFc(ICM20948_DEVICE *pdev, ICM20948_TEMP_CONFIG_TEMP_DLPFCFG_VAL val);//sets gyro DLPF fc

//BANK 3---------------------------------------------------------------------------------------------------------------------
//I2C_MST_ODR_CONFIG
//To be implemented --> I2C master ODR when gyro and accel are off
//I2C_MST_CTRL
void ICM20948_Set_I2C_MST_CLK(ICM20948_DEVICE *pdev, uint8_t clk); //sets i2c master clock frequency. clk value: 0 to 15
//SLV0, SLV1, SLV2, SLV3
void ICM20948_Config_I2C_SLV(ICM20948_DEVICE *pdev, uint8_t SLV, uint8_t dev_I2C_Address, uint8_t firstRegister, uint8_t nBytes);
//SLV4
void ICM20948_I2C_SLV4_TransferBytes(ICM20948_DEVICE *pdev, uint8_t dev_I2C_Address, uint8_t reg, uint8_t *pdata, uint8_t nBytes, uint8_t rw); //rw=1: read, rw=0: write

//MAGNETOMETER FUNCTIONS--------------------------------------------------------------------
uint8_t ICM20948_Get_AK09916_Id(ICM20948_DEVICE *pdev);
void ICM20948_AK09916_Reset(ICM20948_DEVICE *pdev); //AK09916 software reset
void ICM20948_Set_AK09916_Mode(ICM20948_DEVICE *pdev, AK09916_CNTL2_MODE_VAL mode); //sets AK09916 sampling mode
AK09916_CNTL2_MODE_VAL ICM20948_Get_AK09916_Mode(ICM20948_DEVICE *pdev); //gets AK09916 sampling mode

void ICM20948_Read_AK09916_Byte(ICM20948_DEVICE *pdev, uint8_t regaddr, uint8_t *pdata);
void ICM20948_Write_AK09916_Byte(ICM20948_DEVICE *pdev, uint8_t regaddr, uint8_t *pdata);


//EXAMPLES
//  ICM20948_Init_Device(&ICM20948_Dev, &hspi3, IMU_SPI3_CS_GPIO_Port ,IMU_SPI3_CS_Pin);
//  ICM20948_Set_Sensors_Default_Config(&ICM20948_Dev);
//  int i;
//  for (i=0; i<50; i++)
//  {
//    if (ICM20948_IsRawDataReady(&ICM20948_Dev))
//		{
//			ICM20948_Get_AGMT_RawData(&ICM20948_Dev);
//		}
//	  HAL_Delay(500);
//  }
//  ICM20948_BOOL_REG_VAL ready = ICM20948_IsRawDataReady(&ICM20948_Dev);


#endif /* ICM20948_INC_ICM20948_H_ */
