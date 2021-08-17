/*
 * AK09913_Registers.h
 *
 *  Created on: 24 abr. 2021
 *      Author: dlago
 */

#ifndef ICM20948_INC_AK09916_REGISTERS_H_
#define ICM20948_INC_AK09916_REGISTERS_H_

#define AK09916_I2C_ADDR 				0x0C

#define AK09916_ADDR_WIA2				0x01
#define AK09916_ADDR_ST1				0x10
#define AK09916_ADDR_HXL				0x11
#define AK09916_ADDR_HXH				0x12
#define AK09916_ADDR_HYL				0x13
#define AK09916_ADDR_HYH				0x14
#define AK09916_ADDR_HZL				0x15
#define AK09916_ADDR_HZH				0x16
#define AK09916_ADDR_ST2				0x18
#define AK09916_ADDR_CNTL2				0x31
typedef enum _AK09916_CNTL2_MODE_VAL
{
	Power_Down_Mode = 0x0,
	Single_Measurement_Mode = 0x1,
	Continuous_Measurement_Mode1_10Hz = 0x2,
	Continuous_Measurement_Mode2_20Hz = 0x4,
	Continuous_Measurement_Mode3_50Hz = 0x6,
	Continuous_Measurement_Mode4_100Hz = 0x8,
	Self_Test_Mode = 0x10,
}AK09916_CNTL2_MODE_VAL;
#define AK09916_ADDR_CNTL3				0x32

#endif /* ICM20948_INC_AK09916_REGISTERS_H_ */
