/*
 * Copyright (c) 2020, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "sht4x.h"
#include <stdio.h>  // printf
#include "myiic.h"

// added by chelsea
//#include "humidity.h"


extern float tem_org;
extern float hum_org;
//extern uint8_t i2c_index;
void IIC_Init(void);
/**
 * TO USE CONSOLE OUTPUT (PRINTF) AND WAIT (SLEEP) PLEASE ADAPT THEM TO YOUR
 * PLATFORM
 */

extern u16 Test[50];
void delay_ms(u16 nms);
uint8_t SHT4x_Initial(void)
{
	uint8_t count = 0;
	uint8_t ret = 0;
//	i2c_index = 1;
	sensirion_i2c_init();	
	while ((sht4x_probe() != STATUS_OK) && (count++ < 3)) 
	{
		//printf("SHT sensor probing failed\n");
		//sensirion_sleep_usec(10000); /* sleep 1s */		
		delay_ms(1000);
	}	
//	i2c_index = 0;
	if(count <= 3)	
	{
		ret = 2;	
	}
	return ret;
	
}


int8_t Refresh_SHT4x(void)
{
	int32_t temperature, humidity;
	/* Measure temperature and relative humidity and store into variables
	 * temperature, humidity (each output multiplied by 1000).
	 */
	int8_t ret;

	static uint8_t error_cnt = 0;
//	i2c_index = 1;
	ret = sht4x_measure_blocking_read(&temperature, &humidity);
	if (ret == STATUS_OK) {
		tem_org = temperature / 100;
		hum_org = humidity / 100;	
	} else {
		 // printf("error reading measurement\n");
		error_cnt++;
	}		
	if(error_cnt>5)
	{//Test[19]++;
		error_cnt = 0;
		tem_org = 0;
		hum_org = 0;
	}
//	i2c_index = 0;
	return ret;
}

