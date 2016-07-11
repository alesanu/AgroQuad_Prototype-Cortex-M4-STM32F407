/*
 * Read_Process_Params.h
 *
 *  Created on: Jun 28, 2016
 *      Author: Siddhant Gangapurwala
 */

#ifndef READ_PROCESS_PARAMS_H_
#define READ_PROCESS_PARAMS_H_

#include "I2C_SensorCom.h"

/*
void Read_RawGyroParams(int16_t *, int16_t *, int16_t *, int16_t *, int16_t *, int16_t *);
void Read_RawAccelerometerParams(int16_t *, int16_t *, int16_t *, int16_t *, int16_t *, int16_t *);
void Read_RawBaroParams(int16_t *, int16_t *);
void Read_RawMagnetometerParams(int16_t *, int16_t *, int16_t *, int16_t *, int16_t *, int16_t *);
*/

void Read_RawGyroParams(int16_t *Gyro_X, int16_t *Gyro_Y, int16_t *Gyro_Z, int16_t *Cal_Gyro_X, int16_t *Cal_Gyro_Y, int16_t *Cal_Gyro_Z)
{
	uint8_t Gyro_X_H;
	uint8_t Gyro_X_L;
	uint8_t Gyro_Y_H;
	uint8_t Gyro_Y_L;
	uint8_t Gyro_Z_H;
	uint8_t Gyro_Z_L;

	Gyro_X_H = I2C_Initialize_ReadfromReg_NAck(I2C1, MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_H);
	Gyro_X_L = I2C_Initialize_ReadfromReg_NAck(I2C1, MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_L);
	Gyro_Y_H = I2C_Initialize_ReadfromReg_NAck(I2C1, MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_H);
	Gyro_Y_L = I2C_Initialize_ReadfromReg_NAck(I2C1, MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_L);
	Gyro_Z_H = I2C_Initialize_ReadfromReg_NAck(I2C1, MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_H);
	Gyro_Z_L = I2C_Initialize_ReadfromReg_NAck(I2C1, MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_L);

	*Gyro_X = ((Gyro_X_H<<8)|Gyro_X_L) - *Cal_Gyro_X;
	*Gyro_Y = ((Gyro_Y_H<<8)|Gyro_Y_L) - *Cal_Gyro_Y;
	*Gyro_Z = ((Gyro_Z_H<<8)|Gyro_Z_L) - *Cal_Gyro_Z;

	*Gyro_X = *Gyro_X; //&& 0b1111111111111000;
	*Gyro_Y = *Gyro_Y; // && 0b1111111111111000;
	*Gyro_Z = *Gyro_Z; // && 0b1111111111111000;
}


void Read_RawAccelerometerParams(int16_t *Acc_X, int16_t *Acc_Y, int16_t *Acc_Z, int16_t *Cal_Acc_X, int16_t *Cal_Acc_Y, int16_t *Cal_Acc_Z)
{
	uint8_t Acc_X_H;
	uint8_t Acc_X_L;
	uint8_t Acc_Y_H;
	uint8_t Acc_Y_L;
	uint8_t Acc_Z_H;
	uint8_t Acc_Z_L;

	Acc_X_H = I2C_Initialize_ReadfromReg_NAck(I2C1, MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H);
	Acc_X_L = I2C_Initialize_ReadfromReg_NAck(I2C1, MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_L);
	Acc_Y_H = I2C_Initialize_ReadfromReg_NAck(I2C1, MPU6050_ADDRESS, MPU6050_RA_ACCEL_YOUT_H);
	Acc_Y_L = I2C_Initialize_ReadfromReg_NAck(I2C1, MPU6050_ADDRESS, MPU6050_RA_ACCEL_YOUT_L);
	Acc_Z_H = I2C_Initialize_ReadfromReg_NAck(I2C1, MPU6050_ADDRESS, MPU6050_RA_ACCEL_ZOUT_H);
	Acc_Z_L = I2C_Initialize_ReadfromReg_NAck(I2C1, MPU6050_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L);

	*Acc_X = ((Acc_X_H<<8)|Acc_X_L);
	*Acc_Y = ((Acc_Y_H<<8)|Acc_Y_L);
	*Acc_Z = ((Acc_Z_H<<8)|Acc_Z_L);
}


void Read_RawBaroParams(int16_t *Baro_Val, int16_t *Cal_Baro_Val)
{
	uint8_t Baro_Val_H;
	uint8_t Baro_Val_L;

	Baro_Val_H = I2C_Initialize_ReadfromReg_NAck(I2C1, BMP085_ADDRESS, BMP085_Val_MSB);
	Baro_Val_L = I2C_Initialize_ReadfromReg_NAck(I2C1, BMP085_ADDRESS, BMP085_Val_LSB);

	*Baro_Val = ((Baro_Val_H<<8)|Baro_Val_L) - *Cal_Baro_Val;
}


void Read_RawMagnetometerParams(int16_t *Mag_X, int16_t *Mag_Y, int16_t *Mag_Z, int16_t *Cal_Mag_X, int16_t *Cal_Mag_Y, int16_t *Cal_Mag_Z)
{
	uint8_t Mag_X_H;
	uint8_t Mag_X_L;
	uint8_t Mag_Y_H;
	uint8_t Mag_Y_L;
	uint8_t Mag_Z_H;
	uint8_t Mag_Z_L;

	Mag_X_H = I2C_Initialize_ReadfromReg_NAck(I2C1, HMC5883L_ADDRESS, HMC5883L_X_MSB);
	Mag_X_L = I2C_Initialize_ReadfromReg_NAck(I2C1, HMC5883L_ADDRESS, HMC5883L_X_LSB);
	Mag_Y_H = I2C_Initialize_ReadfromReg_NAck(I2C1, HMC5883L_ADDRESS, HMC5883L_Y_MSB);
	Mag_Y_L = I2C_Initialize_ReadfromReg_NAck(I2C1, HMC5883L_ADDRESS, HMC5883L_Y_LSB);
	Mag_Z_H = I2C_Initialize_ReadfromReg_NAck(I2C1, HMC5883L_ADDRESS, HMC5883L_Z_MSB);
	Mag_Z_L = I2C_Initialize_ReadfromReg_NAck(I2C1, HMC5883L_ADDRESS, HMC5883L_Z_LSB);

	*Mag_X = ((Mag_X_H<<8)|Mag_X_L) - *Cal_Mag_X;
	*Mag_Y = ((Mag_Y_H<<8)|Mag_Y_L) - *Cal_Mag_Y;
	*Mag_Z = ((Mag_Z_H<<8)|Mag_Z_L) - *Cal_Mag_Z;

	//AcMag_X = (*Mag_X )/16384;
	//AcMag_Y	= (*Mag_Y )/16384;
	//AcMag_Z = (*Mag_Z )/16384;
}

#endif /* READ_PROCESS_PARAMS_H_ */
