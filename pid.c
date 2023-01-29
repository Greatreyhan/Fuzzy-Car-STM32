#include "pid.h"

void PID_Init(PID_TypeDef *hPID)
{
	hPID->OutputSum = *hPID->output;
	hPID->LastInput = *hPID->input;
	
	if (hPID->OutputSum > hPID->OutMax)
	{
		hPID->OutputSum = hPID->OutMax;
	}
	else if (hPID->OutputSum < hPID->OutMin)
	{
		hPID->OutputSum = hPID->OutMin;
	}
	else { }
	
}

void PID(PID_TypeDef *hPID, double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd, PID_Propotional_TypeDef POn, PID_Control_Direction_TypeDef ControllerDirection)
{
	hPID->output   = Output;
	hPID->input    = Input;
	hPID->setpoint = Setpoint;
	hPID->automatic_input     = (PID_Mode_TypeDef) 0;
	
	PID_SetOutputLimits(hPID, 0, _PID_8BIT_PWM_MAX);
	
	hPID->SampleTime = _PID_SAMPLE_TIME_MS_DEF; /* default Controller Sample Time is 0.1 seconds */
	
	PID_SetControllerDirection(hPID, ControllerDirection);
	PID_SetTunings2(hPID, Kp, Ki, Kd, POn);
	
	hPID->LastTime = HAL_GetTick() - hPID->SampleTime;
	
}

void PID2(PID_TypeDef *hPID, double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd, PID_Control_Direction_TypeDef ControllerDirection)
{
	PID(hPID, Input, Output, Setpoint, Kp, Ki, Kd, PID_PROPOTIONAL_ERROR, ControllerDirection);
}

/* ~~~~~~~~~~~~~~~~~ Computing ~~~~~~~~~~~~~~~~~ */
uint8_t PID_Compute(PID_TypeDef *hPID)
{
	
	uint32_t now;
	uint32_t timeChange;
	
	double input;
	double error;
	double dInput;
	double output;
	
	/* ~~~~~~~~~~ Check PID mode ~~~~~~~~~~ */
	if (!hPID->automatic_input)
	{
		return 0;
	}
	
	/* ~~~~~~~~~~ Calculate time ~~~~~~~~~~ */
	now        = HAL_GetTick();
	timeChange = (now - hPID->LastTime);
	
	if (timeChange >= hPID->SampleTime)
	{
		/* ..... Compute all the working error variables ..... */
		input   = *hPID->input;
		error   = *hPID->setpoint - input;
		dInput  = (input - hPID->LastInput);
		
		hPID->OutputSum     += (hPID->Ki * error);
		
		/* ..... Add Proportional on Measurement, if P_ON_M is specified ..... */
		if (!hPID->Proportional_on_error)
		{
			hPID->OutputSum -= hPID->Kp * dInput;
		}
		
		if (hPID->OutputSum > hPID->OutMax)
		{
			hPID->OutputSum = hPID->OutMax;
		}
		else if (hPID->OutputSum < hPID->OutMin)
		{
			hPID->OutputSum = hPID->OutMin;
		}
		else { }
		
		/* ..... Add Proportional on Error, if P_ON_E is specified ..... */
		if (hPID->Proportional_on_error)
		{
			output = hPID->Kp * error;
		}
		else
		{
			output = 0;
		}
		
		/* ..... Compute Rest of PID Output ..... */
		output += hPID->OutputSum - hPID->Kd * dInput;
		
		if (output > hPID->OutMax)
		{
			output = hPID->OutMax;
		}
		else if (output < hPID->OutMin)
		{
			output = hPID->OutMin;
		}
		else { }
		
		*hPID->output = output;
		
		/* ..... Remember some variables for next time ..... */
		hPID->LastInput = input;
		hPID->LastTime = now;
		
		return 1;
		
	}
	else
	{
		return 0;
	}
	
}

/* ~~~~~~~~~~~~~~~~~ PID Mode ~~~~~~~~~~~~~~~~~~ */
void            PID_SetMode(PID_TypeDef *hPID, PID_Mode_TypeDef Mode)
{
	
	uint8_t newAuto = (Mode == PID_AUTOMATIC_MODE);
	
	/* ~~~~~~~~~~ Initialize the PID ~~~~~~~~~~ */
	if (newAuto && !hPID->automatic_input)
	{
		PID_Init(hPID);
	}
	
	hPID->automatic_input = (PID_Mode_TypeDef)newAuto;
	
}
PID_Mode_TypeDef PID_GetMode(PID_TypeDef *hPID)
{
	return hPID->automatic_input ? PID_AUTOMATIC_MODE : PID_MANUAL_MODE;
}

/* ~~~~~~~~~~~~~~~~ PID Limits ~~~~~~~~~~~~~~~~~ */
void PID_SetOutputLimits(PID_TypeDef *hPID, double Min, double Max)
{
	/* ~~~~~~~~~~ Check value ~~~~~~~~~~ */
	if (Min >= Max)
	{
		return;
	}
	
	hPID->OutMin = Min;
	hPID->OutMax = Max;
	
	/* ~~~~~~~~~~ Check PID Mode ~~~~~~~~~~ */
	if (hPID->automatic_input)
	{
		
		/* ..... Check out value ..... */
		if (*hPID->output > hPID->OutMax)
		{
			*hPID->output = hPID->OutMax;
		}
		else if (*hPID->output < hPID->OutMin)
		{
			*hPID->output = hPID->OutMin;
		}
		else { }
		
		/* ..... Check out value ..... */
		if (hPID->OutputSum > hPID->OutMax)
		{
			hPID->OutputSum = hPID->OutMax;
		}
		else if (hPID->OutputSum < hPID->OutMin)
		{
			hPID->OutputSum = hPID->OutMin;
		}
		else { }
		
	}
	
}

/* ~~~~~~~~~~~~~~~~ PID Tunings ~~~~~~~~~~~~~~~~ */
void PID_SetTunings(PID_TypeDef *hPID, double Kp, double Ki, double Kd)
{
	PID_SetTunings2(hPID, Kp, Ki, Kd, hPID->POn);
}
void PID_SetTunings2(PID_TypeDef *hPID, double Kp, double Ki, double Kd, PID_Propotional_TypeDef POn)
{
	
	double SampleTimeInSec;
	
	/* ~~~~~~~~~~ Check value ~~~~~~~~~~ */
	if (Kp < 0 || Ki < 0 || Kd < 0)
	{
		return;
	}
	
	/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
	hPID->POn    = POn;
	hPID->Proportional_on_error   = (PID_Propotional_TypeDef)(POn == PID_PROPOTIONAL_ERROR);

	hPID->DispKp = Kp;
	hPID->DispKi = Ki;
	hPID->DispKd = Kd;
	
	/* ~~~~~~~~~ Calculate time ~~~~~~~~ */
	SampleTimeInSec = ((double)hPID->SampleTime) / 1000;
	
	hPID->Kp = Kp;
	hPID->Ki = Ki * SampleTimeInSec;
	hPID->Kd = Kd / SampleTimeInSec;
	
	/* ~~~~~~~~ Check direction ~~~~~~~~ */
	if (hPID->ControllerDirection == PID_CONTROL_DIRECTION_REVERSE)
	{
		
		hPID->Kp = (0 - hPID->Kp);
		hPID->Ki = (0 - hPID->Ki);
		hPID->Kd = (0 - hPID->Kd);
		
	}
	
}

/* ~~~~~~~~~~~~~~~ PID Direction ~~~~~~~~~~~~~~~ */
void          PID_SetControllerDirection(PID_TypeDef *hPID, PID_Control_Direction_TypeDef Direction)
{
	/* ~~~~~~~~~~ Check parameters ~~~~~~~~~~ */
	if ((hPID->automatic_input) && (Direction !=hPID->ControllerDirection))
	{
		
		hPID->Kp = (0 - hPID->Kp);
		hPID->Ki = (0 - hPID->Ki);
		hPID->Kd = (0 - hPID->Kd);
		
	}
	
	hPID->ControllerDirection = Direction;
	
}
PID_Control_Direction_TypeDef PID_GetDirection(PID_TypeDef *hPID)
{
	return hPID->ControllerDirection;
}

/* ~~~~~~~~~~~~~~~ PID Sampling ~~~~~~~~~~~~~~~~ */
void PID_SetSampleTime(PID_TypeDef *hPID, int32_t NewSampleTime)
{
	
	double ratio;
	
	/* ~~~~~~~~~~ Check value ~~~~~~~~~~ */
	if (NewSampleTime > 0)
	{
		
		ratio = (double)NewSampleTime / (double)hPID->SampleTime;
		
		hPID->Ki *= ratio;
		hPID->Kd /= ratio;
		hPID->SampleTime = (uint32_t)NewSampleTime;
		
	}
	
}

/* ~~~~~~~~~~~~~ Get Tunings Param ~~~~~~~~~~~~~ */
double PID_GetKp(PID_TypeDef *hPID)
{
	return hPID->DispKp;
}
double PID_GetKi(PID_TypeDef *hPID)
{
	return hPID->DispKi;
}
double PID_GetKd(PID_TypeDef *hPID)
{
	return hPID->DispKd;
}
