/*
 * L298N_driver.h
 *
 *  Created on: Jan 29, 2022
 *      Author: Maulana Reyhan Savero
 */
#ifndef L298N_DRIVER_H_
#define L298N_DRIVER_H_

#include "main.h"

#include <stdbool.h>

#define IN1 GPIO_PIN_2
#define IN2 GPIO_PIN_3
#define IN3 GPIO_PIN_4
#define IN4 GPIO_PIN_5
#define ENA GPIO_PIN_0
#define ENB GPIO_PIN_1

#define IN5 GPIO_PIN_6
#define IN6 GPIO_PIN_7
#define IN7 GPIO_PIN_0
#define IN8 GPIO_PIN_1
#define ENC GPIO_PIN_10
#define END GPIO_PIN_11

#define INPORTA GPIOA
#define INPORTB GPIOB

#define STARTING_SPEED 50

typedef struct{
	uint8_t A;
	uint8_t B;
	uint8_t C;
	uint8_t D;
	uint8_t E;
	uint8_t F;
	uint8_t G;
	uint8_t H;
}obstacle_t;

typedef struct{
	TIM_HandleTypeDef* tim;
  TIM_TypeDef* tim_numA;
	TIM_TypeDef* tim_numB;
  uint8_t channelA;
  uint8_t channelB;
	uint16_t speedA;
	uint16_t speedB;
}car_t;
void Run_Stop(void);
void Run_Kiri(void);
void Run_Kanan(void);
void Run_Maju(void);
void Rotate_Kanan(car_t car, double rotate_speed);
void Rotate_Kiri(car_t car, double rotate_speed);
void Run_Mundur(void);
void Run_Kiri_Speed(car_t car, double speed_turn, double speed_forward);
void Run_Kanan_Speed(car_t car, double speed_turn, double speed_forward);
void Run_Maju_Speed(car_t car, double speed_forward);
void Run_Mundur_Speed(car_t car, double speed_backward);
void Run_Maju_Speed_Manual(car_t car, double speed_left, double speed_right);
void Speed_Servo_A(car_t car);
void Speed_Servo_B(car_t car);
bool handler_receiver(car_t car, uint8_t msg);
bool handler_lidar(car_t car, obstacle_t obs, uint8_t msg);
#endif