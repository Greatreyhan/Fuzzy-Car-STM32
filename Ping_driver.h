/*
 * Servo_driver.h
 *
 *  Created on: Jan 23, 2022
 *      Author: Maulana Reyhan Savero
 */
#ifndef PING_DRIVER_H_
#define PING_DRIVER_H_

#include "main.h"

typedef struct{
	  GPIO_TypeDef * PING_PORT;
    uint16_t       PING_PIN;
		double 				 tickA;
		double				 tickB;
		double				 duration;
		double				 distance;
}Ping_t;

double ping_read(Ping_t ping);

#endif