/*
 * AQDelay.h
 *
 *  Created on: Jun 28, 2016
 *      Author: Siddhant
 */

#ifndef AQDELAY_H_
#define AQDELAY_H_

extern __IO uint32_t TimingDelay;

void Delay_us(__IO uint32_t time)
{
		TimingDelay = time;
		while(TimingDelay != 0);
}

void Delay(__IO uint32_t time)
{
	Delay_us(time*1000);
}


#endif /* AQDELAY_H_ */
