#include "stm32f4xx.h"
#include "MPU6050_Address_Ref.h"
#include "HMC5883L_Address_Ref.h"
#include "BMP085_Address_Ref.h"
#include "Read_Process_Params.h"
#include "stdio.h"
#include "arm_math.h"
#include "defines.h"
#include "Motors_Setup_Control.h"
#include "AQDelay.h"
#include "USART_COmmunication.h"
#include "UtilMisc.h"
#include "Sensors_Setup.h"


/* Used for Communication with the Ground Station */
//#include "CommandControl.h"

extern __IO uint32_t ConState;

#define MAX_STRLEN 12
volatile char received_string[MAX_STRLEN+1];

#ifndef M_PI
#define M_PI 3.14159265359
#endif

#define dt 0.004		//Experimentally Obtained. Corresponds to 4 ms of Control Loop Interval.

#define ACCELEROMETER_SENSITIVITY 16384.0
#define GYROSCOPE_SENSITIVITY 65.536

float pitchGyro = 0, rollGyro = 0;

/* For Simple PID Loop. These values need to be fine tuned, especially when using the Complex PID Control Loop */
float kp_roll = 6.2;
float ki_roll = 0.015;
float kd_roll = 18;

float kp_pitch = 6.0;
float ki_pitch = 0.017;
float kd_pitch = 20;

float kp_yaw = 0;
float ki_yaw = 0;
float kd_yaw = 0;

int del = 0;

float prevPitch = 0;
float prevRoll = 0;

float p_prevPitch = 0;
float p_prevRoll = 0;

float last_pitchAcc = 0;
float last_rollAcc = 0;

float d_roll = 0;
float d_pitch = 0;
float d_yaw = 0;
float Cal_pitch = 0, Cal_roll = 0, Cal_yaw = 0;

int16_t CalAcc_X[25];
int16_t CalAcc_Y[25];
int16_t CalAcc_Z[25];

int throttle = 0;

//float GPS_longitude = 0, GPS_lattitude = 0;

float error_sum_roll = 0;
float error_sum_pitch = 0;
float error_sum_yaw = 0;

float diff_error_roll = 0;
float diff_error_pitch = 0;
float diff_error_yaw = 0;

float diff_diff_error_pitch = 0;
float diff_error_last_pitch = 0;

float roll_pid = 0;
float pitch_pid = 0;
float yaw_pid = 0;

float error_last_roll = 0;
float error_last_pitch = 0;
float error_last_yaw = 0;

float roll = 0, pitch = 0, yaw = 0;
int Lroll = 0, Lpitch = 0, Lyaw = 0;

int Throttle_FL;
int Throttle_FR;
int Throttle_BL;
int Throttle_BR;

/* -------- Used to Hardcode the Differences in Two Opposite Motors ------- */
int Delta_FLBR = 0;
int Delta_FRBL = 0;

int16_t Acc_X = 0, Acc_Y = 0, Acc_Z = 0;
int16_t Gyro_X = 0, Gyro_Y = 0, Gyro_Z = 0;
int16_t Mag_X = 0, Mag_Y = 0, Mag_Z = 0;
int16_t Baro_Val = 0;

/*
int GPS_Lat_Int = 0, GPS_Lat_Dec = 0;
int GPS_Long_Int = 0, GPS_Long_Dec = 0;
*/

int16_t Cal_Acc_X = 0, Cal_Acc_Y = 0, Cal_Acc_Z = 0;
int16_t Cal_Gyro_X = 0, Cal_Gyro_Y = 0, Cal_Gyro_Z = 0;
int16_t Cal_Mag_X = 0, Cal_Mag_Y = 0, Cal_Mag_Z = 0;
int16_t Cal_Baro_Val = 0;

float AcAcc_X, AcAcc_Y, AcAcc_Z;
float Gyro_Roll, Gyro_Pitch, Gyro_Yaw;

void ComplementaryFilter(int16_t, int16_t, int16_t, int16_t, int16_t, int16_t, int16_t, int16_t, int16_t, float *, float *, float *, float, float, float);
void ComplexComplementaryFilter(int16_t, int16_t, int16_t, int16_t, int16_t, int16_t, int16_t, int16_t, int16_t, float *, float *, float *, float, float, float);

void PID_Control(float roll, float pitch, float *roll_pid, float *pitch_pid,float d_roll, float d_pitch);
void PID_ComplexControl(float roll, float pitch, float *roll_pid, float *pitch_pid,float d_roll, float d_pitch);

void PID_Yaw(float yaw, float *yaw_pid, float d_yaw);

void Calibrate_AllParams(int16_t *, int16_t *, int16_t *, int16_t *, int16_t *, int16_t *, int16_t *, int16_t *, int16_t *, int16_t *, int16_t *, int16_t *, int16_t *, int16_t *, int16_t *, int16_t *, int16_t *, int16_t *, int16_t *, int16_t *);

void Initialize_Timer2()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM_Prescaler = 2500;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 500;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &timerInitStructure);
	TIM_Cmd(TIM2, ENABLE);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}


void EnableTimer2_Interrupt()
{
    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    nvicStructure.NVIC_IRQChannelSubPriority = 0;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);
}


void init_Discovery_LEDS()
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* Configure PC13, PC14 and PC15 in output pushpull mode */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Configure PE6 in output pushpull mode */

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);


}


void init_MotorPortPins()
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2| GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}


int main(void)
{

	int i = 0;

	ConState = 4000;
	SysTick_Config(SystemCoreClock/1000000);

	for(i = 0; i <25; i++)
	{
		CalAcc_X[i] = 0;
		CalAcc_Y[i] = 0;
		CalAcc_Z[i] = 0;
	}

    init_USART1(115200);

	init_Discovery_LEDS();

	GPIO_ResetBits(GPIOE, GPIO_Pin_6);
	GPIO_SetBits(GPIOC, GPIO_Pin_14);
	GPIO_SetBits(GPIOC, GPIO_Pin_15);

	init_MotorPortPins();

	GPIO_SetBits(GPIOA, GPIO_Pin_0);
	GPIO_SetBits(GPIOA, GPIO_Pin_1);
	GPIO_SetBits(GPIOA, GPIO_Pin_2);
	GPIO_SetBits(GPIOA, GPIO_Pin_3);

	GPIO_ResetBits(GPIOC, GPIO_Pin_13);

	Delay(2000);

	init_I2C1();

	Setup_MPU6050();
	Setup_BMP085();
	Setup_HMC5883L();

	GPIO_ResetBits(GPIOC, GPIO_Pin_14);

	char c[15];

	Init_Motors(); //---------------------> Careful Here!!!

	GPIO_ResetBits(GPIOC, GPIO_Pin_15);

	throttle = 1010;

	sprintf(c, "Calibrating Values");
	USART_puts(USART1, c);

	float new_roll = 0;
	float new_pitch = 0;
	float new_yaw = 0;

	for(i = 0; i < 500; i++)
	{
		while(ConState % 8000 != 0);
		Read_RawAccelerometerParams(&Acc_X, &Acc_Y, &Acc_Z, &Cal_Acc_X, &Cal_Acc_Y, &Cal_Acc_Z);
		Read_RawGyroParams(&Gyro_X, &Gyro_Y, &Gyro_Z, &Cal_Gyro_X, &Cal_Gyro_Y, &Cal_Gyro_Z);
		ComplementaryFilter(Acc_X, Acc_Y, Acc_Z, Gyro_X, Gyro_Y, Gyro_Z, Mag_X, Mag_Y, Mag_Z, &roll, &pitch, &yaw, Cal_roll, Cal_pitch, Cal_yaw);

		new_roll += roll;
		new_pitch += pitch;
		new_yaw += yaw;
	}

	Cal_roll = new_roll/500;
	Cal_pitch = new_pitch/500;
	Cal_yaw = new_yaw/500;

	d_roll = 0;
	d_pitch = 0;
	d_yaw = 0;

	sprintf(c, "Done. Going for it!");
	USART_puts(USART1, c);


	while (1)
	{
		if(ConState % 4000 == 0)
		{
			GPIO_SetBits(GPIOC, GPIO_Pin_14);

			Read_RawAccelerometerParams(&Acc_X, &Acc_Y, &Acc_Z, &Cal_Acc_X, &Cal_Acc_Y, &Cal_Acc_Z);
			Read_RawGyroParams(&Gyro_X, &Gyro_Y, &Gyro_Z, &Cal_Gyro_X, &Cal_Gyro_Y, &Cal_Gyro_Z);
			Read_RawBaroParams(&Baro_Val, &Cal_Baro_Val);
			Read_RawMagnetometerParams(&Mag_X, &Mag_Y, &Mag_Z, &Cal_Mag_X, &Cal_Mag_Y, &Cal_Mag_Z);

			GPIO_ResetBits(GPIOC, GPIO_Pin_14);

			ComplementaryFilter(Acc_X, Acc_Y, Acc_Z, Gyro_X, Gyro_Y, Gyro_Z, Mag_X, Mag_Y, Mag_Z, &roll, &pitch, &yaw, Cal_roll, Cal_pitch, Cal_yaw);

			new_roll = roll - Cal_roll;
			new_pitch = pitch - Cal_pitch;
			new_yaw = yaw - Cal_yaw;

			PID_Control(new_roll, new_pitch, &roll_pid, &pitch_pid, d_roll, d_pitch);
			PID_Yaw(new_yaw, &yaw_pid, d_yaw);

			Throttle_FL = (int)(throttle + Delta_FLBR - pitch_pid + yaw_pid);
			Throttle_BR = (int)(throttle - Delta_FLBR + pitch_pid + yaw_pid);

			Throttle_FR = (int)(throttle + Delta_FRBL + roll_pid - yaw_pid);
			Throttle_BL = (int)(throttle - Delta_FRBL - roll_pid - yaw_pid);

			/*

			----- Status of the Device ---

			del++;
			if(del > 150)
			{
				del = 0;
				char c[40];
				sprintf(c, "Roll: %d Pitch: %d\n", (int)new_roll, (int)new_pitch);
				USART_puts(USART1, c);
			}

			 */

			if(Throttle_FL < 1010)
			{
				Throttle_FL = 1010;
			}

			if(Throttle_FR < 1010)
			{
				Throttle_FR = 1010;
			}

			if(Throttle_BL < 1010)
			{
				Throttle_BL = 1010;
			}

			if(Throttle_BR < 1010)
			{
				Throttle_BR = 1010;
			}

			if(Throttle_FL > 1950)
			{
				Throttle_FL = 1950;
			}

			if(Throttle_FR > 1950)
			{
				Throttle_FR = 1950;
			}

			if(Throttle_BL > 1950)
			{
				Throttle_BL = 1950;
			}

			if(Throttle_BR > 1950)
			{
				Throttle_BR = 1950;
			}

			Motors_write(Throttle_FL, Throttle_FR, Throttle_BL, Throttle_BR);
		}
	}
}


void USART2_IRQHandler(void)
{

}

/*	----------------------------------------------------------------------------
	----------------------------------------------------------------------------
	----------------------------------------------------------------------------

		USED FOR COMMUNICATION WITH A TERMINAL PROGRAM. FOR EXPERIMENTAL PURPOSES
		AN ANDROID APP WAS DEVELOPED AND INTERFACED WITH THE XBEE S2 ZIGBEE MODULE.

	----------------------------------------------------------------------------
	----------------------------------------------------------------------------
	----------------------------------------------------------------------------*/

void USART1_IRQHandler(void)
{
	static uint8_t cnt = 0; // this counter is used to determine the string length
	char t;
	char c[25];

	if( USART_GetITStatus(USART1, USART_IT_RXNE) )
	{
		t = USART1->DR; // the character from the USART1 data register is saved in t
	}

	if( t == '\n' )
	{
		cnt = 0;
	}


	if( t == 'u' || t == 'U')
	{
		throttle = throttle + 10;
		sprintf(c, "Throttle = %d\n", throttle);
		USART_puts(USART1, c);
		cnt = 0;
	}

	else if( t == 'q' || t == 'Q')
	{
		d_pitch = 15;
		cnt = 0;
	}


	else if( t == 'f' || t == 'F')
	{
		throttle = throttle + 50;
		sprintf(c, "Throttle = %d\n", throttle);
		USART_puts(USART1, c);
		cnt = 0;
	}

	else if( t == 'l' || t == 'L')
	{
		throttle = throttle - 30;
		sprintf(c, "Throttle = %d\n", throttle);
		USART_puts(USART1, c);
		cnt = 0;
	}

	else if( t == 'b' || t == 'B')
	{
		Delta_FLBR = Delta_FLBR + 1;
		sprintf(c, "Delta_FLBR = %d\n", Delta_FLBR);
		USART_puts(USART1, c);
		cnt = 0;
	}

	else if( t == 'z' || t == 'Z')
	{
		Delta_FRBL = Delta_FRBL + 1;
		sprintf(c, "Delta_FRBL = %d\n", Delta_FRBL);
		USART_puts(USART1, c);
		cnt = 0;
	}

	else if( t == 'p' || t == 'P')
	{
		kp_pitch = kp_pitch + 0.1;
		kp_roll = kp_roll + 0.1;
		USART_puts(USART1, "KP_Pitch Increased by 0.1");
		USART_puts(USART1, "KP_Roll Increased by 0.1");
		cnt = 0;
	}


	else if( t == 'r' || t == 'R')
	{
		kd_pitch = kd_pitch - 0.5;
		kd_roll = kd_roll - 0.5;
		USART_puts(USART1, "KD_Pitch Decreased by 0.5");
		USART_puts(USART1, "KD_Roll Decreased by 0.5");
		cnt = 0;
	}

	else if( t == 'i' || t == 'I')
	{
		ki_pitch = ki_pitch + 0.001;
		ki_roll = ki_roll + 0.0001;
		USART_puts(USART1, "KI_Pitch Increased by 0.0001");
		USART_puts(USART1, "KI_Roll Increased by 0.0001");
		cnt = 0;
	}

	else if( t == 'd' || t == 'D')
	{
		kd_pitch = kd_pitch + 1;
		kd_roll = kd_roll + 1;
		USART_puts(USART1, "KD_Pitch Increased by 1");
		USART_puts(USART1, "KD_Roll Increased by 1");
		cnt = 0;
	}

	else		//Emergency Stop
	{
		throttle = 1010;
		USART_puts(USART1, "Stopping..");
		Delay(20);
		myMotorFL_write(1010);	//640
		myMotorFR_write(1010);
		myMotorBL_write(1010);
		myMotorBR_write(1010);
		Delay(20);
	}


	/* For Communication with the Ground Station */
	/*
	while(cnt != 8 && del < 50)
	{
	// check if the USART1 receive interrupt flag was set
		if( USART_GetITStatus(USART1, USART_IT_RXNE) )
		{
			t = USART1->DR; // the character from the USART1 data register is saved in t

			if( cnt < 9) //MAX_STRLEN )
			{
				received_string[cnt] = t;
				cnt++;
			}

			else cnt = 0;

		}
	}
	*/

	if (cnt > 0)
	{
		//CommandControl(received_string);
	}



	cnt = 0;
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);
}


void Calibrate_AllParams(int16_t *Acc_X, int16_t *Acc_Y, int16_t *Acc_Z, int16_t *Gyro_X, int16_t *Gyro_Y, int16_t *Gyro_Z, int16_t *Mag_X, int16_t *Mag_Y, int16_t *Mag_Z, int16_t *Baro_Val, int16_t *Cal_Acc_X, int16_t *Cal_Acc_Y, int16_t *Cal_Acc_Z, int16_t *Cal_Gyro_X, int16_t *Cal_Gyro_Y, int16_t *Cal_Gyro_Z, int16_t *Cal_Mag_X, int16_t *Cal_Mag_Y, int16_t *Cal_Mag_Z, int16_t *Cal_Baro_Val)
{
	*Cal_Acc_X = *Acc_X;
	*Cal_Acc_Y = *Acc_Y;
	*Cal_Acc_Z = *Acc_Z;

	*Cal_Gyro_X = *Gyro_X;
	*Cal_Gyro_Y = *Gyro_Y;
	*Cal_Gyro_Z = *Gyro_Z;

	*Cal_Mag_X = *Mag_X;
	*Cal_Mag_Y = *Mag_Y;
	*Cal_Mag_Z = *Mag_Z;

	*Cal_Baro_Val = *Baro_Val;
}

void ComplementaryFilter(int16_t accData_X, int16_t accData_Y, int16_t accData_Z, int16_t gyData_X, int16_t gyData_Y, int16_t gyData_Z, int16_t magData_X, int16_t magData_Y, int16_t magData_Z, float *roll, float *pitch, float *yaw, float Cal_roll, float Cal_pitch, float Cal_yaw)
{
    float pitchAcc, rollAcc;

    *roll += ((float)gyData_X /(GYROSCOPE_SENSITIVITY)) * (dt);
    *pitch -= ((float)gyData_Y /(GYROSCOPE_SENSITIVITY)) * (dt);

    rollAcc = atan2f((float)accData_Y/16384, (float)accData_Z/16384) * 180 / M_PI;
    pitchAcc = atan2f((float)accData_X/16384, (float)accData_Z/16384) * 180 / M_PI;

    *pitch = pitchAcc*0.04 + *pitch*0.96;
    *roll = rollAcc*0.04 + *roll*0.96;

    float magXcomp = magData_X*cos(*pitch - Cal_pitch) + (magData_Z)*sin(*pitch - Cal_pitch);
    float magYcomp = magData_X*sin(*roll - Cal_roll)*sin(*pitch - Cal_pitch) + (magData_Y)*cos(*roll - Cal_roll) - (magData_Z)*sin(*roll - Cal_roll)*cos(*pitch - Cal_pitch);

    *yaw = 180*atan2(magYcomp,magXcomp)/M_PI;
}


void ComplexComplementaryFilter(int16_t accData_X, int16_t accData_Y, int16_t accData_Z, int16_t gyData_X, int16_t gyData_Y, int16_t gyData_Z, int16_t magData_X, int16_t magData_Y, int16_t magData_Z, float *roll, float *pitch, float *yaw, float Cal_roll, float Cal_pitch, float Cal_yaw)
{
    float pitchAcc, rollAcc;

    *roll += ((float)gyData_X /(GYROSCOPE_SENSITIVITY)) * (dt);
    *pitch -= ((float)gyData_Y /(GYROSCOPE_SENSITIVITY)) * (dt);

    rollAcc = atan2f((float)accData_Y/16384, (float)accData_Z/16384) * 180 / M_PI;
    pitchAcc = atan2f((float)accData_X/16384, (float)accData_Z/16384) * 180 / M_PI;

    *pitch = pitchAcc*0.08 + *pitch*0.92;//(*pitch); // * 0.98 + pitchAcc * 0.02);
    *roll = rollAcc*0.08 + *roll*0.92;//(*roll); // * 0.98 + rollAcc * 0.02);

    if(*pitch > prevPitch + 0.001 || *pitch < prevPitch - 0.001)
    {
    	*pitch = prevPitch*0.9 + pitchAcc*0.1;
    }

    if(*roll > prevRoll + 0.001 || *roll < prevRoll - 0.001)
    {
    	*roll = prevRoll*0.9 + rollAcc*0.1;
    }

    float magXcomp = magData_X*cos(*pitch - Cal_pitch) + (magData_Z)*sin(*pitch - Cal_pitch);
    float magYcomp = magData_X*sin(*roll - Cal_roll)*sin(*pitch - Cal_pitch) + (magData_Y)*cos(*roll - Cal_roll) - (magData_Z)*sin(*roll - Cal_roll)*cos(*pitch - Cal_pitch);

    *yaw = 180*atan2(magYcomp,magXcomp)/M_PI;

    p_prevRoll = prevRoll;
    p_prevPitch = prevPitch;

    prevRoll = *roll;
    prevPitch = *pitch;
}

void PID_Control(float roll, float pitch, float *roll_pid, float *pitch_pid, float d_roll, float d_pitch)
{
	float error_roll = d_roll - roll;
	float error_pitch = d_pitch - pitch;

	error_sum_roll += ki_roll*error_roll;
	diff_error_roll = (error_roll - error_last_roll);

	error_sum_pitch += ki_pitch*error_pitch;
	diff_error_pitch = (error_pitch - error_last_pitch);

	*roll_pid = kp_roll*(error_roll) + error_sum_roll + kd_roll*diff_error_roll;
	*pitch_pid = kp_pitch*(error_pitch) + error_sum_pitch + kd_pitch*diff_error_pitch;
}

void PID_Yaw(float yaw, float *yaw_pid, float d_yaw)
{
	float error_yaw = d_yaw - yaw;

	error_sum_yaw += ki_yaw*error_yaw;
	diff_error_yaw = (error_yaw - error_last_yaw);

	*yaw_pid = kp_yaw*(error_yaw) + error_sum_yaw + kd_yaw*diff_error_yaw;
}

void PID_ComplexControl(float roll, float pitch, float *roll_pid, float *pitch_pid, float d_roll, float d_pitch)
{
	float error_roll = d_roll - roll;
	float error_pitch = d_pitch - pitch;

	error_sum_roll += ki_roll*error_roll*abs_float(2/(1+(exp(-1*error_roll*0.2))));
	diff_error_roll = (error_roll - error_last_roll);

	error_sum_pitch += ki_pitch*error_pitch*abs_float(2/(1+(exp(-1*error_pitch*0.2))));
	diff_error_pitch = (error_pitch - error_last_pitch);

	*roll_pid = kp_roll*(error_roll)*abs_float(2/(1+(exp(-1*error_roll*0.2))) - 1) + error_sum_roll + kd_roll*diff_error_roll;
	*pitch_pid = kp_pitch*(error_pitch)*abs_float(2/(1+(exp(-1*error_pitch*0.2))) - 1) + error_sum_pitch + kd_pitch*diff_error_pitch;

	/*------------------- Alternative Functions -----------------*/
	//*abs_float((2/(1+(exp(-1*error_pitch*1.3))) - 1))
	//0.3989422804*exp(-1*((0.1*error*error)/10))
	//kd_pitch*diff_error_pitch*0.3989422804*exp(-1*((2*error_pitch*error_pitch)))
	// + 3.7*exp(-1*((error_pitch*error_pitch))/0.8)*kd_pitch_2*diff_error_pitch

	error_last_roll = error_roll;
	error_last_pitch = error_pitch;
}
