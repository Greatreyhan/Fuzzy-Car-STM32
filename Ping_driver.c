/*
 * Ping_driver.c
 *
 *  Created on: Jan 24, 2022
 *      Author: Maulana Reyhan Savero
 */

#include "Ping_driver.h"
#include "DWT_Delay.h"

double ping_read(Ping_t ping){
	
		HAL_GPIO_WritePin(ping.PING_PORT, ping.PING_PIN, GPIO_PIN_RESET);
		// Set As Output-----------------------------
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = ping.PING_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(ping.PING_PORT, &GPIO_InitStruct);
		//-------------------------------------------
		
		HAL_GPIO_WritePin(ping.PING_PORT, ping.PING_PIN, GPIO_PIN_SET);
		DWT_Delay_us(5);
		
		HAL_GPIO_WritePin(ping.PING_PORT, ping.PING_PIN, GPIO_PIN_RESET);
		DWT_Delay_us(750);

		
		// Set As Input----------------------------------
		GPIO_InitStruct.Pin = ping.PING_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(ping.PING_PORT, &GPIO_InitStruct);
		//-----------------------------------------------

		GPIO_PinState state;
		for(int i =0; i < 18500; i++){ //18500
			if(HAL_GPIO_ReadPin(ping.PING_PORT, ping.PING_PIN)){
				if(ping.tickA == 0) ping.tickA = i;
			}
			else if( !(HAL_GPIO_ReadPin(ping.PING_PORT, ping.PING_PIN)) && i > 115){
				ping.tickB = i;
				break;
			}
			DWT_Delay_us(1);
		}
		ping.duration = (ping.tickB-ping.tickA);
		ping.distance = (ping.duration)/58;
		
		HAL_Delay(10);
		
		return ping.distance;
}