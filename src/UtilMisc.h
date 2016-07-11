/*
 * UtilMisc.h
 *
 *  Created on: Jun 28, 2016
 *      Author: Siddhant Gangapurwala
 */

#ifndef UTILMISC_H_
#define UTILMISC_H_

void ConvertFloattoInt(float num, int *Lnum, int *count)
{
	*count = 0;
	int tnum = 0;

	while(num != tnum)
	{
		num = num*10;
		tnum = (int)num;
		count = count + 1;
	}

	*Lnum = num;
}


int16_t abs_int16(int16_t x)
{
	int16_t no = -1;
	if(x < 0)
		return x*no;
	else
		return x;
}

float abs_float(float x)
{
	if(x < 0)
	{
		x = x*(-1);
	}

	return x;
}


#endif /* UTILMISC_H_ */
