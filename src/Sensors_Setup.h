/*
 * Sensors_Setup.h
 *
 *  Created on: Jun 28, 2016
 *      Author: Siddhant
 */

#ifndef SENSORS_SETUP_H_
#define SENSORS_SETUP_H_

#include "I2C_SensorCom.h"

void Setup_MPU6050()
{
    //Sets sample rate to 8000/1+7 = 1000Hz
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x07);
    //Disable FSync, 256Hz DLPF
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_CONFIG, 0x00);
    //Disable gyro self tests, scale of 500 degrees/s
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0b00001000);  //0b00001000 for 500 deg/s
    //Disable accel self tests, scale of +-2g, no DHPF
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x00);
    //Freefall threshold of |0mg|
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_FF_THR, 0x00);
    //Freefall duration limit of 0
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_FF_DUR, 0x00);
    //Motion threshold of 0mg
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_MOT_THR, 0x00);
    //Motion duration of 0s
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_MOT_DUR, 0x00);
    //Zero motion threshold
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_ZRMOT_THR, 0x00);
    //Zero motion duration threshold
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_ZRMOT_DUR, 0x00);
    //Disable sensor output to FIFO buffer
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_FIFO_EN, 0x00);

    //AUX I2C setup
    //Sets AUX I2C to single master control, plus other config
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_I2C_MST_CTRL, 0x00);
    //Setup AUX I2C slaves
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_ADDR, 0x00);
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_REG, 0x00);
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_CTRL, 0x00);
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_ADDR, 0x00);
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_REG, 0x00);
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_CTRL, 0x00);
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_ADDR, 0x00);
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_REG, 0x00);
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_CTRL, 0x00);
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_ADDR, 0x00);
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_REG, 0x00);
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_CTRL, 0x00);
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_ADDR, 0x00);
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_REG, 0x00);
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_DO, 0x00);
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_CTRL, 0x00);
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_DI, 0x00);

    //MPU6050_RA_I2C_MST_STATUS //Read-only
    //Setup INT pin and AUX I2C pass through
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 0x00);
    //Enable data ready interrupt
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 0x00);

    //MPU6050_RA_DMP_INT_STATUS        //Read-only
    //MPU6050_RA_INT_STATUS 3A        //Read-only
    //MPU6050_RA_ACCEL_XOUT_H         //Read-only
    //MPU6050_RA_ACCEL_XOUT_L         //Read-only
    //MPU6050_RA_ACCEL_YOUT_H         //Read-only
    //MPU6050_RA_ACCEL_YOUT_L         //Read-only
    //MPU6050_RA_ACCEL_ZOUT_H         //Read-only
    //MPU6050_RA_ACCEL_ZOUT_L         //Read-only
    //MPU6050_RA_TEMP_OUT_H         //Read-only
    //MPU6050_RA_TEMP_OUT_L         //Read-only
    //MPU6050_RA_GYRO_XOUT_H         //Read-only
    //MPU6050_RA_GYRO_XOUT_L         //Read-only
    //MPU6050_RA_GYRO_YOUT_H         //Read-only
    //MPU6050_RA_GYRO_YOUT_L         //Read-only
    //MPU6050_RA_GYRO_ZOUT_H         //Read-only
    //MPU6050_RA_GYRO_ZOUT_L         //Read-only
    //MPU6050_RA_EXT_SENS_DATA_00     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_01     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_02     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_03     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_04     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_05     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_06     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_07     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_08     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_09     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_10     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_11     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_12     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_13     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_14     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_15     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_16     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_17     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_18     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_19     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_20     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_21     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_22     //Read-only
    //MPU6050_RA_EXT_SENS_DATA_23     //Read-only
    //MPU6050_RA_MOT_DETECT_STATUS     //Read-only

    //Slave out, dont care
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_DO, 0x00);
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_DO, 0x00);
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_DO, 0x00);
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_DO, 0x00);
    //More slave config
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_I2C_MST_DELAY_CTRL, 0x00);
    //Reset sensor signal paths
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_SIGNAL_PATH_RESET, 0x00);
    //Motion detection control
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_MOT_DETECT_CTRL, 0x00);
    //Disables FIFO, AUX I2C, FIFO and I2C reset bits to 0
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, 0x00);
    //Sets clock source to gyro reference w/ PLL
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0b00000010);
    //Controls frequency of wakeups in accel low power mode plus the sensor standby modes
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 0x00);
    //MPU6050_RA_BANK_SEL            //Not in datasheet
    //MPU6050_RA_MEM_START_ADDR        //Not in datasheet
    //MPU6050_RA_MEM_R_W            //Not in datasheet
    //MPU6050_RA_DMP_CFG_1            //Not in datasheet
    //MPU6050_RA_DMP_CFG_2            //Not in datasheet
    //MPU6050_RA_FIFO_COUNTH        //Read-only
    //MPU6050_RA_FIFO_COUNTL        //Read-only
    //Data transfer to and from the FIFO buffer
    I2C_Initialize_WritetoReg(I2C1, MPU6050_ADDRESS, MPU6050_RA_FIFO_R_W, 0x00);
    //MPU6050_RA_WHO_AM_I             //Read-only, I2C address

    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

}

void Setup_HMC5883L()
{
	I2C_Initialize_WritetoReg(I2C1, HMC5883L_ADDRESS, HMC5883L_Reg_A, 0b01010100);
	I2C_Initialize_WritetoReg(I2C1, HMC5883L_ADDRESS, HMC5883L_Reg_B, 0b00100000);
	I2C_Initialize_WritetoReg(I2C1, HMC5883L_ADDRESS, HMC5883L_Mode, 0b00000000);
}


void Setup_BMP085()
{
	I2C_Initialize_WritetoReg(I2C1, BMP085_ADDRESS, BMP085_Control, BMP085_PressureMode1);
}



#endif /* SENSORS_SETUP_H_ */
