/*
 * I2C_SensorCom.h
 *
 *  Created on: Jun 28, 2016
 *      Author: Siddhant Gangapurwala
 */

#ifndef I2C_SENSORCOM_H_
#define I2C_SENSORCOM_H_

uint8_t returnVal = 0;
uint8_t data = 0;

void init_I2C1(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // PB6 and PB7
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);	// SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA

	I2C_StructInit(&I2C_InitStruct);
	I2C_InitStruct.I2C_ClockSpeed = 400000;
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C1, &I2C_InitStruct);

	I2C_Cmd(I2C1, ENABLE);
}


int I2C_Start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction)
{
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2Cx, ENABLE);

	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2Cx, address, direction);

	if(direction == I2C_Direction_Transmitter){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}

	else if(direction == I2C_Direction_Receiver){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}

	return 1;
}


void I2C_Write(I2C_TypeDef* I2Cx, uint8_t data)
{
	I2C_SendData(I2Cx, data);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}


void I2C_WritetoReg(I2C_TypeDef* I2Cx, uint8_t reg_address, uint8_t data)
{
	I2C_Write(I2Cx, reg_address);
	I2C_Write(I2Cx, data);
}


uint8_t I2C_Read_Ack(I2C_TypeDef* I2Cx)
{
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	data = I2C_ReceiveData(I2Cx);
	return data;
}


uint8_t I2C_Read_NAck(I2C_TypeDef* I2Cx)
{
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	data = I2C_ReceiveData(I2Cx);
	return data;
}


void I2C_Stop(I2C_TypeDef* I2Cx)
{
	I2C_GenerateSTOP(I2Cx, ENABLE);
}


void I2C_Initialize_WritetoReg(I2C_TypeDef* I2Cx, uint8_t device_address, uint8_t reg_address, uint8_t data)
{
	if(I2C_Start(I2Cx, device_address, I2C_Direction_Transmitter) == 1)
	{
		I2C_WritetoReg(I2Cx, reg_address, data);
		I2C_Stop(I2Cx);
	}
}


uint8_t I2C_Initialize_ReadfromReg_Ack(I2C_TypeDef* I2Cx, uint8_t device_address, uint8_t reg_address)
{
	I2C_GenerateSTART(I2Cx, ENABLE);
	uint8_t returnVal;
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
	I2C_Send7bitAddress(I2Cx, device_address, I2C_Direction_Transmitter);

	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_Write(I2Cx, reg_address);
	I2C_GenerateSTART(I2Cx, ENABLE);

	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2Cx, device_address, I2C_Direction_Receiver);

	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	returnVal = I2C_Read_Ack(I2Cx);
	I2C_Stop(I2Cx);
	return returnVal;
}

uint8_t I2C_Initialize_ReadfromReg_NAck(I2C_TypeDef* I2Cx, uint8_t device_address, uint8_t reg_address)
{
	I2C_GenerateSTART(I2Cx, ENABLE);
	uint8_t returnVal;
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2Cx, device_address, I2C_Direction_Transmitter);

	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_Write(I2Cx, reg_address);
	I2C_GenerateSTART(I2Cx, ENABLE);

	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2Cx, device_address, I2C_Direction_Receiver);

	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	returnVal = I2C_Read_NAck(I2Cx);

	I2C_Stop(I2Cx);
	return returnVal;
}


#endif /* I2C_SENSORCOM_H_ */
