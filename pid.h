#ifndef __PID_H_
#define __PID_H_

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "main.h"

/* ------------------------ Public ------------------------- */
#define _PID_8BIT_PWM_MAX       UINT8_MAX
#define _PID_SAMPLE_TIME_MS_DEF 1

/* PID Mode */
typedef enum
{	
	PID_MANUAL_MODE    = 0,
	PID_AUTOMATIC_MODE = 1
	
}PID_Mode_TypeDef;

/* PID P On x */
typedef enum
{
	PID_PROPOTIONAL_MEASUREMENT = 0, /* Proportional on Measurement */
	PID_PROPOTIONAL_ERROR = 1
	
}PID_Propotional_TypeDef;

/* PID Control direction */
typedef enum
{
	
	PID_CONTROL_DIRECTION_FORWARD  = 0,
	PID_CONTROL_DIRECTION_REVERSE = 1
	
}PID_Control_Direction_TypeDef;

/* PID Structure */
typedef struct
{
	
	PID_Propotional_TypeDef  Proportional_on_error;
	PID_Mode_TypeDef automatic_input;

	PID_Propotional_TypeDef  POn;
	PID_Control_Direction_TypeDef   ControllerDirection;

	uint32_t        LastTime;
	uint32_t        SampleTime;

	double          DispKp;
	double          DispKi;
	double          DispKd;

	double          Kp;
	double          Ki;
	double          Kd;

	double          *input;
	double          *output;
	double          *setpoint;

	double          OutputSum;
	double          LastInput;

	double          OutMin;
	double          OutMax;
	
}PID_TypeDef;

void PID_Init(PID_TypeDef *uPID);

void PID(PID_TypeDef *uPID, double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd, PID_Propotional_TypeDef POn, PID_Control_Direction_TypeDef ControllerDirection);
void PID2(PID_TypeDef *uPID, double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd, PID_Control_Direction_TypeDef ControllerDirection);

uint8_t PID_Compute(PID_TypeDef *uPID);

void PID_SetMode(PID_TypeDef *uPID, PID_Mode_TypeDef Mode);
PID_Mode_TypeDef PID_GetMode(PID_TypeDef *uPID);

void PID_SetOutputLimits(PID_TypeDef *uPID, double Min, double Max);

void PID_SetTunings(PID_TypeDef *uPID, double Kp, double Ki, double Kd);
void PID_SetTunings2(PID_TypeDef *uPID, double Kp, double Ki, double Kd, PID_Propotional_TypeDef POn);

void PID_SetControllerDirection(PID_TypeDef *uPID, PID_Control_Direction_TypeDef Direction);
PID_Control_Direction_TypeDef PID_GetDirection(PID_TypeDef *uPID);

void PID_SetSampleTime(PID_TypeDef *uPID, int32_t NewSampleTime);

double PID_GetKp(PID_TypeDef *uPID);
double PID_GetKi(PID_TypeDef *uPID);
double PID_GetKd(PID_TypeDef *uPID);

#endif
