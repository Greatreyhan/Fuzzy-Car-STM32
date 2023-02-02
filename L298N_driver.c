/*
 * L298N_driver.c
 *
 *  Created on: Jan 29, 2022
 *      Author: Maulana Reyhan Savero
 */

#include "L298N_driver.h"


void Run_Stop(void){
	HAL_GPIO_WritePin(INPORTA, IN1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INPORTA, IN2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INPORTA, IN3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INPORTA, IN4, GPIO_PIN_RESET);
}

void Run_Kiri(void){
	HAL_GPIO_WritePin(INPORTA, IN1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(INPORTA, IN2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INPORTA, IN3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INPORTA, IN4, GPIO_PIN_RESET);
}

void Run_Kanan(void){
	HAL_GPIO_WritePin(INPORTA, IN1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INPORTA, IN2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INPORTA, IN3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(INPORTA, IN4, GPIO_PIN_RESET);
	
}

void Run_Maju(void){
	HAL_GPIO_WritePin(INPORTA, IN1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(INPORTA, IN2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INPORTA, IN3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(INPORTA, IN4, GPIO_PIN_RESET);
}

void Rotate_Kanan(car_t car, double rotate_speed){
	HAL_GPIO_WritePin(INPORTA, IN1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INPORTA, IN2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(INPORTA, IN3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(INPORTA, IN4, GPIO_PIN_RESET);
	car.speedB = rotate_speed;
	car.speedA = rotate_speed;
	Speed_Servo_B(car);
	Speed_Servo_A(car);
}

void Rotate_Kiri(car_t car, double rotate_speed){
	HAL_GPIO_WritePin(INPORTA, IN1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(INPORTA, IN2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INPORTA, IN3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INPORTA, IN4, GPIO_PIN_SET);
	car.speedB = rotate_speed;
	car.speedA = rotate_speed;
	Speed_Servo_B(car);
	Speed_Servo_A(car);
}

void Run_Mundur(void){
	HAL_GPIO_WritePin(INPORTA, IN1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INPORTA, IN2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(INPORTA, IN3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INPORTA, IN4, GPIO_PIN_SET);
}

void Run_Kiri_Speed(car_t car, double speed_turn, double speed_forward){
	HAL_GPIO_WritePin(INPORTA, IN1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(INPORTA, IN2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INPORTA, IN3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INPORTA, IN4, GPIO_PIN_RESET);
	car.speedB = speed_forward;
	car.speedA = speed_turn;
	Speed_Servo_B(car);
	Speed_Servo_A(car);
}

void Run_Kanan_Speed(car_t car, double speed_turn, double speed_forward){
	HAL_GPIO_WritePin(INPORTA, IN1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INPORTA, IN2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INPORTA, IN3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(INPORTA, IN4, GPIO_PIN_RESET);
	car.speedB = speed_turn;
	car.speedA = speed_forward;
	Speed_Servo_B(car);
	Speed_Servo_A(car);
}

void Run_Maju_Speed(car_t car, double speed_forward){
	HAL_GPIO_WritePin(INPORTA, IN1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(INPORTA, IN2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INPORTA, IN3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(INPORTA, IN4, GPIO_PIN_RESET);
	car.speedB = speed_forward;
	car.speedA = speed_forward;
	Speed_Servo_B(car);
	Speed_Servo_A(car);
}

void Run_Maju_Speed_Manual(car_t car, double speed_left, double speed_right){
	HAL_GPIO_WritePin(INPORTA, IN1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(INPORTA, IN2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INPORTA, IN3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(INPORTA, IN4, GPIO_PIN_RESET);
	car.speedB = speed_left;
	car.speedA = speed_right;
	Speed_Servo_B(car);
	Speed_Servo_A(car);
}

void Run_Mundur_Speed(car_t car, double speed_backward){
	HAL_GPIO_WritePin(INPORTA, IN1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INPORTA, IN2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(INPORTA, IN3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INPORTA, IN4, GPIO_PIN_SET);
	car.speedB = speed_backward;
	car.speedA = speed_backward;
	Speed_Servo_B(car);
	Speed_Servo_A(car);
}

void Stopper_run(car_t car){
	car.tim = NULL;
	car.tim_numA = NULL;
	car.tim_numB = NULL;
	HAL_GPIO_WritePin(INPORTA, IN1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INPORTA, IN2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INPORTA, IN3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INPORTA, IN4, GPIO_PIN_RESET);
	car.speedB = 0;
	car.speedA = 0;
	Speed_Servo_B(car);
	Speed_Servo_A(car);
}

void Speed_Servo_A(car_t car){
	if(car.channelA == 0x01){
		car.tim_numA->CCR1 = (car.speedA*500/100);
		HAL_TIM_PWM_Start(car.tim, TIM_CHANNEL_1);
	}
	else if(car.channelA == 0x02){
		car.tim_numA->CCR2 = (car.speedA*500/100);
		HAL_TIM_PWM_Start(car.tim, TIM_CHANNEL_2);
	}
	else if(car.channelA == 0x03){
		car.tim_numA->CCR3 = (car.speedA*500/100);
		HAL_TIM_PWM_Start(car.tim, TIM_CHANNEL_3);
	}
	else if(car.channelA == 0x04){
		car.tim_numA->CCR3 = (car.speedA*500/100);
		HAL_TIM_PWM_Start(car.tim, TIM_CHANNEL_3);	
	}
	
}

void Speed_Servo_B(car_t car){
	if(car.channelB == 0x01){
		car.tim_numB->CCR1 = (car.speedB*500/100);
		HAL_TIM_PWM_Start(car.tim, TIM_CHANNEL_1);
	}
	else if(car.channelB == 0x02){
		car.tim_numB->CCR2 = (car.speedB*500/100);
		HAL_TIM_PWM_Start(car.tim, TIM_CHANNEL_2);
	}
	else if(car.channelB == 0x03){
		car.tim_numB->CCR3 = (car.speedB*500/100);
		HAL_TIM_PWM_Start(car.tim, TIM_CHANNEL_3);
	}
	else if(car.channelB == 0x04){
		car.tim_numB->CCR3 = (car.speedB*500/100);
		HAL_TIM_PWM_Start(car.tim, TIM_CHANNEL_3);
	}
}

bool handler_receiver(car_t car, uint8_t msg){
			if(msg == 'G'){
				Run_Stop();
			}
			else if(msg == 'B'){
				Run_Maju_Speed(car, 50);
			}
			else if(msg == 'D'){
				Run_Mundur_Speed(car,50);
			}
			else if(msg == 'A'){
				Run_Kiri_Speed(car, 70, 30);
			}
			else if(msg == 'C'){
				Run_Kanan_Speed(car, 30, 70);
			}
			else if(msg == 'G'){
				Rotate_Kiri(car, 40);
			}
			else if(msg == 'I'){
				Rotate_Kanan(car, 40);
			}
			else if(msg == 'H'){
				car.speedA+= 5;
				car.speedB+= 5;
				Speed_Servo_A(car);
				Speed_Servo_B(car);
			}
			else if(msg == 'J'){
				car.speedA-= 5;
				car.speedB-= 5;
				Speed_Servo_A(car);
				Speed_Servo_B(car);
			}
			else return false;
		return true;
}

bool handler_lidar(car_t car, obstacle_t obs, uint8_t msg){
			obs.A = (msg & 0x01);
			obs.B = ((msg >> 0x01) & 0x01);
			obs.C = ((msg >> 0x02) & 0x01);
			obs.D = ((msg >> 0x03) & 0x01);
			obs.E = ((msg >> 0x04) & 0x01);
			obs.F = ((msg >> 0x05) & 0x01);
			obs.G = ((msg >> 0x06) & 0x01);
			obs.H = ((msg >> 0x07) & 0x01);
			
			if(obs.A || obs.H){
				if(obs.B || obs.C){
					car.speedA = 100;
					car.speedB = 100;
					Speed_Servo_A(car);
					Speed_Servo_B(car);
					Run_Kanan();
				}
				else if(obs.G || obs.H){
					car.speedA = 100;
					car.speedB = 100;
					Speed_Servo_A(car);
					Speed_Servo_B(car);
					Run_Kiri();
				}
			}
			else{
				car.speedA = 50;
				car.speedB = 50;
				Speed_Servo_A(car);
				Speed_Servo_B(car);
				Run_Maju();
			}
}