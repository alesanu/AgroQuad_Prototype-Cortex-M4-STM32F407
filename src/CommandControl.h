/*
 * CommandControl.h
 *
 *  Created on: Jun 28, 2016
 *      Author: Siddhant Gangapurwala
 */

#ifndef COMMANDCONTROL_H_
#define COMMANDCONTROL_H_

/*Used for Communication with the Ground Station.*/

char Acc_Tag[] = "18462";
char Gyro_Tag[] = "65418";
char Mag_Tag[] = "42179";
char Baro_Tag[] = "53276";
char GPS_Tag[] = "21684";

char X_Axis_Address[] = "26574";
char Y_Axis_Address[] = "39715";
char Z_Axis_Address[] = "56427";

char GPS_Long_Int_Address[] = "10345";
char GPS_Lat_Int_Address[] = "42784";
char GPS_Long_Dec_Address[] = "10384";
char GPS_Lat_Dec_Address[] = "42791";

char Baro_Address[] = "34675";



void CommandControl(volatile char *command)
{

	int count_string = 0;
	int countR = 0, countP = 0;
	//char requestAccess[] = "REQ38127";
	char grantAccess[] = "GRN9430D\n";
	//char requestParamsAccess[] = "PAR76243";
	char grantParamsAccess[] = "GRN40699\n";  //Passing all Params

	char S_RD[12];
	char c[15];

	while(*command)
	{
		S_RD[count_string] = *command;
		count_string++;
		*command++;
	}

	if(S_RD[0] == 'R' && S_RD[1] == 'E' && S_RD[2] == 'Q' && S_RD[3] == '3' && S_RD[4] == '8' && S_RD[5] == '1' && S_RD[6] == '2' && S_RD[7] == '7')
	{
		USART_puts(USART1, grantAccess);
	}

	else if(S_RD[0] == 'P' && S_RD[1] == 'A' && S_RD[2] == 'R' && S_RD[3] == '7' && S_RD[4] == '6' && S_RD[5] == '2' && S_RD[6] == '4' && S_RD[7] == '3')
	{
		USART_puts(USART1, grantParamsAccess);
	}

	else if (S_RD[0] == 'P' && S_RD[1] == 'A' && S_RD[2] == 'R' && S_RD[3] == 'A' && S_RD[4] == 'C' && S_RD[5] == 'C' && S_RD[6] == 'X' && S_RD[7] == '1')
	{
		sprintf(c, "%d\n", Acc_X);
		USART_puts(USART1, c);
	}

	else if (S_RD[0] == 'P' && S_RD[1] == 'A' && S_RD[2] == 'R' && S_RD[3] == 'A' && S_RD[4] == 'C' && S_RD[5] == 'C' && S_RD[6] == 'Y' && S_RD[7] == '1')
	{
		sprintf(c, "%d\n", Acc_Y);
		USART_puts(USART1, c);
	}
	else if (S_RD[0] == 'P' && S_RD[1] == 'A' && S_RD[2] == 'R' && S_RD[3] == 'A' && S_RD[4] == 'C' && S_RD[5] == 'C' && S_RD[6] == 'Z' && S_RD[7] == '1')
	{
		sprintf(c, "%d\n", Acc_Z);
		USART_puts(USART1, c);
	}

	else if (S_RD[0] == 'P' && S_RD[1] == 'A' && S_RD[2] == 'R' && S_RD[3] == 'G' && S_RD[4] == 'Y' && S_RD[5] == 'R' && S_RD[6] == 'O' && S_RD[7] == 'X')
	{
		sprintf(c, "%d\n", Gyro_X);
		USART_puts(USART1, c);
	}

	else if (S_RD[0] == 'P' && S_RD[1] == 'A' && S_RD[2] == 'R' && S_RD[3] == 'G' && S_RD[4] == 'Y' && S_RD[5] == 'R' && S_RD[6] == 'O' && S_RD[7] == 'Y')
	{
		sprintf(c, "%d\n", Gyro_Y);
		USART_puts(USART1, c);
	}

	else if (S_RD[0] == 'P' && S_RD[1] == 'A' && S_RD[2] == 'R' && S_RD[3] == 'G' && S_RD[4] == 'Y' && S_RD[5] == 'R' && S_RD[6] == 'O' && S_RD[7] == 'Z')
	{
		sprintf(c, "%d\n", Gyro_X);
		USART_puts(USART1, c);
	}

	else if (S_RD[0] == 'P' && S_RD[1] == 'A' && S_RD[2] == 'R' && S_RD[3] == 'M' && S_RD[4] == 'A' && S_RD[5] == 'G' && S_RD[6] == 'X' && S_RD[7] == '1')
	{
		sprintf(c, "%d\n", Mag_X);
		USART_puts(USART1, c);
	}

	else if (S_RD[0] == 'P' && S_RD[1] == 'A' && S_RD[2] == 'R' && S_RD[3] == 'M' && S_RD[4] == 'A' && S_RD[5] == 'G' && S_RD[6] == 'Y' && S_RD[7] == '1')
	{
		sprintf(c, "%d\n", Mag_Y);
		USART_puts(USART1, c);
	}

	else if (S_RD[0] == 'P' && S_RD[1] == 'A' && S_RD[2] == 'R' && S_RD[3] == 'M' && S_RD[4] == 'A' && S_RD[5] == 'G' && S_RD[6] == 'Z' && S_RD[7] == '1')
	{
		sprintf(c, "%d\n", Mag_Z);
		USART_puts(USART1, c);
	}


	else if (S_RD[0] == 'P' && S_RD[1] == 'A' && S_RD[2] == 'R' && S_RD[3] == 'B' && S_RD[4] == 'A' && S_RD[5] == 'R' && S_RD[6] == 'O' && S_RD[7] == '1')
	{
		sprintf(c, "%d\n", Baro_Val);
		USART_puts(USART1, c);
	}


	else if (S_RD[0] == 'P' && S_RD[1] == 'A' && S_RD[2] == 'R' && S_RD[3] == 'G' && S_RD[4] == 'P' && S_RD[5] == 'S' && S_RD[6] == 'L' && S_RD[7] == 'I')
	{
		sprintf(c, "%d\n", GPS_Lat_Int);
		USART_puts(USART1, c);
	}


	else if (S_RD[0] == 'P' && S_RD[1] == 'A' && S_RD[2] == 'R' && S_RD[3] == 'G' && S_RD[4] == 'P' && S_RD[5] == 'S' && S_RD[6] == 'L' && S_RD[7] == 'D')
	{
		sprintf(c, "%d\n", GPS_Lat_Dec);
		USART_puts(USART1, c);
	}


	else if (S_RD[0] == 'P' && S_RD[1] == 'A' && S_RD[2] == 'R' && S_RD[3] == 'G' && S_RD[4] == 'P' && S_RD[5] == 'S' && S_RD[6] == 'O' && S_RD[7] == 'I')
	{
		sprintf(c, "%d\n", GPS_Long_Int);
		USART_puts(USART1, c);
	}


	else if (S_RD[0] == 'P' && S_RD[1] == 'A' && S_RD[2] == 'R' && S_RD[3] == 'G' && S_RD[4] == 'P' && S_RD[5] == 'S' && S_RD[6] == 'O' && S_RD[7] == 'D')
	{
		sprintf(c, "%d\n", GPS_Long_Dec);
		USART_puts(USART1, c);

	}


	else if (S_RD[0] == 'P' && S_RD[1] == 'A' && S_RD[2] == 'R' && S_RD[3] == 'R' && S_RD[4] == 'O' && S_RD[5] == 'L' && S_RD[6] == 'L' && S_RD[7] == '1')
	{

		ConvertFloattoInt(roll, &Lroll, &countR);
		sprintf(c, "%d\n", Lroll);
		USART_puts(USART1, c);
		//sprintf(c, "%d\n", count);
		//USART_puts(USART1, c);
	}

	else if (S_RD[0] == 'P' && S_RD[1] == 'A' && S_RD[2] == 'R' && S_RD[3] == 'R' && S_RD[4] == 'O' && S_RD[5] == 'L' && S_RD[6] == 'L' && S_RD[7] == 'C')
	{
		sprintf(c, "%d\n", countR);
		USART_puts(USART1, c);
	}


	else if (S_RD[0] == 'P' && S_RD[1] == 'A' && S_RD[2] == 'R' && S_RD[3] == 'P' && S_RD[4] == 'I' && S_RD[5] == 'T' && S_RD[6] == 'C' && S_RD[7] == 'H')
	{

		ConvertFloattoInt(pitch, &Lpitch, &countP);
		sprintf(c, "%d\n", Lpitch);
		USART_puts(USART1, c);
		//sprintf(c, "%d\n", count);
		//USART_puts(USART1, c);
	}

	else if (S_RD[0] == 'P' && S_RD[1] == 'A' && S_RD[2] == 'R' && S_RD[3] == 'P' && S_RD[4] == 'I' && S_RD[5] == 'T' && S_RD[6] == 'C' && S_RD[7] == 'C')
	{
		sprintf(c, "%d\n", countP);
		USART_puts(USART1, c);
	}

	else if (S_RD[0] == 'C' && S_RD[1] == 'A' && S_RD[2] == 'L' && S_RD[3] == 'P' && S_RD[4] == 'A' && S_RD[5] == 'R' && S_RD[6] == 'A' && S_RD[7] == 'M')
	{
		Calibrate_AllParams(&Acc_X, &Acc_Y, &Acc_Z, &Gyro_X, &Gyro_Y, &Gyro_Z, &Mag_X, &Mag_Y, &Mag_Z, &Baro_Val, &Cal_Acc_X, &Cal_Acc_Y, &Cal_Acc_Z, &Cal_Gyro_X, &Cal_Gyro_Y, &Cal_Gyro_Z, &Cal_Mag_X, &Cal_Mag_Y, &Cal_Mag_Z, &Cal_Baro_Val);
		GPIO_ResetBits(GPIOD, GPIO_Pin_13);
	}

	else if (S_RD[0] == 'M' && S_RD[1] == 'O' && S_RD[2] == 'T' && S_RD[3] == 'S' && S_RD[4] == 'I' && S_RD[5] == 'N' && S_RD[6] == 'I' && S_RD[7] == 'T')
	{
		//Init_Motors();
		GPIO_SetBits(GPIOD, GPIO_Pin_13);
		flagD = 1;
	}


	else if (S_RD[0] == 'C' && S_RD[1] == 'A' && S_RD[2] == 'L' && S_RD[3] == 'U' && S_RD[4] == 'N' && S_RD[5] == 'D' && S_RD[6] == 'O' && S_RD[7] == 'C')
	{
		Cal_Acc_X = 0;
		Cal_Acc_Y = 0;
		Cal_Acc_Z = 0;
		Cal_Gyro_X = 0;
		Cal_Gyro_Y = 0;
		Cal_Gyro_Z = 0;
		Cal_Mag_X = 0;
		Cal_Mag_Y = 0;
		Cal_Mag_Z = 0;
		Cal_Baro_Val = 0;
		GPIO_SetBits(GPIOD, GPIO_Pin_13);
	}

	else
	{
		//alternateCommandControlHandler(S_RD);
	}


}

void alternateCommandControlHandler(volatile char *altcommand)
{
	//Just in case.
}


#endif /* COMMANDCONTROL_H_ */
