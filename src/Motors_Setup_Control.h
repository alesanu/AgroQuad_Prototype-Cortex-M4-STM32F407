/*
 * Motors_Setup_Control.h
 *
 *  Created on: Jun 28, 2016
 *      Author: Siddhant
 */

#ifndef MOTORS_SETUP_CONTROL_H_
#define MOTORS_SETUP_CONTROL_H_

#include "AQDelay.h"
#include "USART_COmmunication.h"

void myMotorFL_write(uint32_t deg)
{
	TIM_Cmd(TIM2, DISABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);

	//0 deg -> 1000 us
	//180 deg -> 2000 us

	GPIO_ResetBits(GPIOA, GPIO_Pin_0);
	Delay_us(deg);
	GPIO_SetBits(GPIOA, GPIO_Pin_0);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

void myMotorFR_write(uint32_t deg)
{
	TIM_Cmd(TIM2, DISABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);

	GPIO_ResetBits(GPIOA, GPIO_Pin_1);
	Delay_us(deg);
	GPIO_SetBits(GPIOA, GPIO_Pin_1);

	TIM_Cmd(TIM2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

}

void myMotorBL_write(uint32_t deg)
{
	TIM_Cmd(TIM2, DISABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);

	GPIO_ResetBits(GPIOA, GPIO_Pin_2);
	Delay_us(deg);
	GPIO_SetBits(GPIOA, GPIO_Pin_2);

	TIM_Cmd(TIM2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

}

void myMotorBR_write(uint32_t deg)
{
	TIM_Cmd(TIM2, DISABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);

	GPIO_ResetBits(GPIOA, GPIO_Pin_3);
	Delay_us(deg);
	GPIO_SetBits(GPIOA, GPIO_Pin_3);

	TIM_Cmd(TIM2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void Init_Motors()
{
	USART_puts(USART1, "Enabling ESCs \n");
	int i = 0;

	USART_puts(USART1, "Setting High Speed\n");
	for(i = 0; i<300; i++)
	{
		myMotorFL_write(2000);	//2400
		myMotorFR_write(2000);
		myMotorBL_write(2000);
		myMotorBR_write(2000);
		Delay_us(4000);
	}

	USART_puts(USART1, "Setting Low Speed\n");
	for(i = 0; i<300; i++)
	{
		myMotorFL_write(1000);		//1000
		myMotorFR_write(1000);
		myMotorBL_write(1000);
		myMotorBR_write(1000);
		Delay_us(8000);
	}
	//Delay(4000);

	USART_puts(USART1, "Calibrated\n");
	for(i = 0; i<300; i++)
	{
		myMotorFL_write(1010);	//1010
		myMotorFR_write(1010);
		myMotorBL_write(1010);
		myMotorBR_write(1010);

		Delay_us(7800);
	}

	USART_puts(USART1, "Starting\n");
	Delay(2000);
}


void Motors_write(uint32_t deg_FL, uint32_t deg_FR, uint32_t deg_BL, uint32_t deg_BR)
{

	GPIO_ResetBits(GPIOA, GPIO_Pin_0);
	GPIO_ResetBits(GPIOA, GPIO_Pin_1);
	GPIO_ResetBits(GPIOA, GPIO_Pin_2);
	GPIO_ResetBits(GPIOA, GPIO_Pin_3);

	if(deg_FL <= deg_FR && deg_FL <= deg_BL && deg_FL <= deg_BR)
	{

		Delay_us(deg_FL);
		GPIO_SetBits(GPIOA, GPIO_Pin_0);

		if(deg_FR <= deg_BL && deg_FR <= deg_BR)
		{
			Delay_us(deg_FR - deg_FL);
			GPIO_SetBits(GPIOA, GPIO_Pin_1);

			if(deg_BL <= deg_BR)
			{
				Delay_us(deg_BL - deg_FR);
				GPIO_SetBits(GPIOA, GPIO_Pin_2);
				Delay_us(deg_BR - deg_BL);
				GPIO_SetBits(GPIOA, GPIO_Pin_3);
			}

			else if(deg_BR < deg_BL)
			{
				Delay_us(deg_BR - deg_FR);
				GPIO_SetBits(GPIOA, GPIO_Pin_3);
				Delay_us(deg_BL - deg_BR);
				GPIO_SetBits(GPIOA, GPIO_Pin_2);
			}
		}

		else if(deg_BL < deg_FR && deg_BL <= deg_BR)
		{
			Delay_us(deg_BL - deg_FL);
			GPIO_SetBits(GPIOA, GPIO_Pin_2);

			if(deg_FR <= deg_BR)
			{
				Delay_us(deg_FR - deg_BL);
				GPIO_SetBits(GPIOA, GPIO_Pin_1);
				Delay_us(deg_BR - deg_FR);
				GPIO_SetBits(GPIOA, GPIO_Pin_3);
			}

			else if(deg_BR < deg_FR)
			{
				Delay_us(deg_BR - deg_BL);
				GPIO_SetBits(GPIOA, GPIO_Pin_3);
				Delay_us(deg_FR - deg_BR);
				GPIO_SetBits(GPIOA, GPIO_Pin_1);
			}
		}

		else if(deg_BR < deg_FR && deg_BR < deg_BL)
		{
			Delay_us(deg_BR - deg_FL);
			GPIO_SetBits(GPIOA, GPIO_Pin_3);

			if(deg_FR <= deg_BL)
			{
				Delay_us(deg_FR - deg_BR);
				GPIO_SetBits(GPIOA, GPIO_Pin_1);
				Delay_us(deg_BL - deg_FR);
				GPIO_SetBits(GPIOA, GPIO_Pin_2);
			}

			else if(deg_BL < deg_FR)
			{
				Delay_us(deg_BL - deg_BR);
				GPIO_SetBits(GPIOA, GPIO_Pin_2);
				Delay_us(deg_FR - deg_BL);
				GPIO_SetBits(GPIOA, GPIO_Pin_1);
			}
		}

	}

	else if(deg_FR < deg_FL && deg_FR <= deg_BL && deg_FR <= deg_BR)
		{
			Delay_us(deg_FR);
			GPIO_SetBits(GPIOA, GPIO_Pin_1);

			if(deg_FL <= deg_BL && deg_FL <= deg_BR)
			{
				Delay_us(deg_FL - deg_FR);
				GPIO_SetBits(GPIOA, GPIO_Pin_0);

				if(deg_BL <= deg_BR)
				{
					Delay_us(deg_BL - deg_FL);
					GPIO_SetBits(GPIOA, GPIO_Pin_2);
					Delay_us(deg_BR - deg_FL);
					GPIO_SetBits(GPIOA, GPIO_Pin_3);
				}

				else if(deg_BR < deg_BL)
				{
					Delay_us(deg_BR - deg_FL);
					GPIO_SetBits(GPIOA, GPIO_Pin_3);
					Delay_us(deg_BL - deg_BR);
					GPIO_SetBits(GPIOA, GPIO_Pin_2);
				}
			}

			else if(deg_BL < deg_FL && deg_BL <= deg_BR)
			{
				Delay_us(deg_BL - deg_FR);
				GPIO_SetBits(GPIOA, GPIO_Pin_2);

				if(deg_FL <= deg_BR)
				{
					Delay_us(deg_FL - deg_BL);
					GPIO_SetBits(GPIOA, GPIO_Pin_0);
					Delay_us(deg_BR - deg_FL);
					GPIO_SetBits(GPIOA, GPIO_Pin_3);
				}

				else if(deg_BR < deg_FL)
				{
					Delay_us(deg_BR - deg_BL);
					GPIO_SetBits(GPIOA, GPIO_Pin_3);
					Delay_us(deg_FL - deg_BR);
					GPIO_SetBits(GPIOA, GPIO_Pin_0);
				}
			}

			else if(deg_BR < deg_FL && deg_BR < deg_BL)
			{
				Delay_us(deg_BR - deg_FR);
				GPIO_SetBits(GPIOA, GPIO_Pin_3);

				if(deg_FL <= deg_BL)
				{
					Delay_us(deg_FL - deg_BR);
					GPIO_SetBits(GPIOA, GPIO_Pin_0);
					Delay_us(deg_BL - deg_FL);
					GPIO_SetBits(GPIOA, GPIO_Pin_2);
				}

				else if(deg_BL < deg_FL)
				{
					Delay_us(deg_BL - deg_BR);
					GPIO_SetBits(GPIOA, GPIO_Pin_2);
					Delay_us(deg_FL - deg_BL);
					GPIO_SetBits(GPIOA, GPIO_Pin_0);
				}
			}

		}

	else if(deg_BL < deg_FL && deg_BL < deg_FR && deg_BL <= deg_BR)
		{
			Delay_us(deg_BL);
			GPIO_SetBits(GPIOA, GPIO_Pin_2);

			if(deg_FL <= deg_FR && deg_FL <= deg_BR)
			{
				Delay_us(deg_FL - deg_BL);
				GPIO_SetBits(GPIOA, GPIO_Pin_0);

				if(deg_FR <= deg_BR)
				{
					Delay_us(deg_FR - deg_FL);
					GPIO_SetBits(GPIOA, GPIO_Pin_1);
					Delay_us(deg_BR - deg_FR);
					GPIO_SetBits(GPIOA, GPIO_Pin_3);
				}

				else
				{
					Delay_us(deg_BR - deg_FL);
					GPIO_SetBits(GPIOA, GPIO_Pin_3);
					Delay_us(deg_FR - deg_BR);
					GPIO_SetBits(GPIOA, GPIO_Pin_1);
				}
			}

			else if(deg_FR < deg_FL && deg_FR <= deg_BR)
			{
				Delay_us(deg_FR - deg_BL);
				GPIO_SetBits(GPIOA, GPIO_Pin_1);

				if(deg_FL <= deg_BR)
				{
					Delay_us(deg_FL - deg_FR);
					GPIO_SetBits(GPIOA, GPIO_Pin_0);
					Delay_us(deg_BR - deg_FL);
					GPIO_SetBits(GPIOA, GPIO_Pin_3);
				}

				else if(deg_BR < deg_FL)
				{
					Delay_us(deg_BR - deg_FR);
					GPIO_SetBits(GPIOA, GPIO_Pin_3);
					Delay_us(deg_FL - deg_BR);
					GPIO_SetBits(GPIOA, GPIO_Pin_0);
				}
			}

			else if(deg_BR < deg_FL && deg_BR < deg_FR)
			{
				Delay_us(deg_BR - deg_BL);
				GPIO_SetBits(GPIOA, GPIO_Pin_3);

				if(deg_FL <= deg_FR)
				{
					Delay_us(deg_FL - deg_BR);
					GPIO_SetBits(GPIOA, GPIO_Pin_0);
					Delay_us(deg_FR - deg_FL);
					GPIO_SetBits(GPIOA, GPIO_Pin_1);
				}

				else if(deg_FR < deg_FL)
				{
					Delay_us(deg_FR - deg_BR);
					GPIO_SetBits(GPIOA, GPIO_Pin_1);
					Delay_us(deg_FL - deg_FR);
					GPIO_SetBits(GPIOA, GPIO_Pin_0);
				}
			}

		}


	else if(deg_BR < deg_FL && deg_BR < deg_FR && deg_BR < deg_BL)
		{
			Delay_us(deg_BR);
			GPIO_SetBits(GPIOA, GPIO_Pin_3);

			if(deg_FL <= deg_FR && deg_FL <= deg_BL)
			{
				Delay_us(deg_FL - deg_BR);
				GPIO_SetBits(GPIOA, GPIO_Pin_0);

				if(deg_FR <= deg_BL)
				{
					Delay_us(deg_FR - deg_FL);
					GPIO_SetBits(GPIOA, GPIO_Pin_1);
					Delay_us(deg_BL - deg_FR);
					GPIO_SetBits(GPIOA, GPIO_Pin_2);
				}

				else if(deg_BL < deg_FR)
				{
					Delay_us(deg_BL - deg_FL);
					GPIO_SetBits(GPIOA, GPIO_Pin_2);
					Delay_us(deg_FR - deg_BL);
					GPIO_SetBits(GPIOA, GPIO_Pin_1);
				}
			}

			else if(deg_FR < deg_FL && deg_FR <= deg_BL)
			{
				Delay_us(deg_FR - deg_BR);
				GPIO_SetBits(GPIOA, GPIO_Pin_1);

				if(deg_FL <= deg_BL)
				{
					Delay_us(deg_FL - deg_FR);
					GPIO_SetBits(GPIOA, GPIO_Pin_0);
					Delay_us(deg_BL - deg_FL);
					GPIO_SetBits(GPIOA, GPIO_Pin_2);
				}

				else if(deg_BL < deg_FL)
				{
					Delay_us(deg_BL - deg_FR);
					GPIO_SetBits(GPIOA, GPIO_Pin_2);
					Delay_us(deg_FL - deg_BL);
					GPIO_SetBits(GPIOA, GPIO_Pin_0);
				}
			}

			else if(deg_BL < deg_FL && deg_BL < deg_FR)
			{
				Delay_us(deg_BL - deg_BR);
				GPIO_SetBits(GPIOA, GPIO_Pin_2);

				if(deg_FL <= deg_FR)
				{
					Delay_us(deg_FL - deg_BL);
					GPIO_SetBits(GPIOA, GPIO_Pin_0);
					Delay_us(deg_FR - deg_FL);
					GPIO_SetBits(GPIOA, GPIO_Pin_1);
				}

				else if(deg_FR < deg_FL)
				{
					Delay_us(deg_FR - deg_BL);
					GPIO_SetBits(GPIOA, GPIO_Pin_1);
					Delay_us(deg_FL - deg_FR);
					GPIO_SetBits(GPIOA, GPIO_Pin_0);
				}
			}

		}
}

#endif /* MOTORS_SETUP_CONTROL_H_ */
