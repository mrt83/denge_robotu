#include "pid.h"

void pid_reset( PID * device );
void pid_control( PID * device , LimitingType type);
void pid_load_value(PID * device, float setValue);

float _fabs(float value);

float _fabs(float value)
{
    if(value < 0)
    {
        return (-1.0*value);
    }
    else
    {
        return (value);
    }
}

void pid_reset(PID * device)
{
	device->previousError = 0.0;
	device->error = 0.0;
	device->integral = device->INTEGRAL_INITIAL_VALUE;
	device->setValue = 0.0;
	device->feedbackValue = 0.0;
	device->output = 0.0;
	device->derivative = 0.0;
}

void pid_control(PID * device, LimitingType type)
{
	static float tempOutput;

	device->error = device->setValue - device->feedbackValue;

	if(device->error > device->PID_ERROR_LIMIT)
	{
		device->error = device->PID_ERROR_LIMIT;
	}
	else if(device->error < (-1.0 * (device->PID_ERROR_LIMIT)))
	{
		device->error = (-1.0 * device->PID_ERROR_LIMIT);
	}

	if(_fabs(device->error) > device->EPSILON)
	{
		device->integral += (device->error * device->DT * device->KI);

		if(device->integral > device->INTEGRAL_MAX_VAL)	device->integral = device->INTEGRAL_MAX_VAL;
		if(device->integral < device->INTEGRAL_MIN_VAL)	device->integral = device->INTEGRAL_MIN_VAL;
	}

	tempOutput = (device->KP * device->error) + (device->integral); // + (device->KD * device->derivative);

	device->previousError = device->error;


	//device->derivative = ((device->error - device->previousError) / device->DT);
	//Limiting output
	if(type == negativeLimiting)
	{
		if(tempOutput > 0.0)
		{
			device->output = 0.0;
		}
		else if(tempOutput < (-1 * device->MAX_OUTPUT))
		{
			device->output = (-1 * device->MAX_OUTPUT);
		}
		else
		{
			device->output = tempOutput;
		}
	}
	else if(type == positiveLimiting)
	{
		if(tempOutput > device->MAX_OUTPUT)
		{
			device->output = device->MAX_OUTPUT;
		}
		else if(tempOutput < 0.0)
		{
			device->output = 0.0;
		}
		else
		{
			device->output = tempOutput;
		}
	}
	else if(type == fullLimiting)
	{
		if(tempOutput > device->MAX_OUTPUT)
		{
			device->output = device->MAX_OUTPUT;
		}
		else if(tempOutput < (-1 * device->MAX_OUTPUT))
		{
			device->output = (-1 * device->MAX_OUTPUT);
		}
		else
		{
			device->output = tempOutput;
		}
	}
	else if(type == nonLimiting)
	{
		device->output = tempOutput;
	}



}

void pid_load_value(PID * device, float setValue)
{
	device->previousError = 0.0;
    device->error = 0.0;
    device->integral = setValue / device->KI;
    device->setValue = 0.0;
    device->feedbackValue = 0.0;
    device->output = device->KI * device->integral;
    device->derivative = 0.0;
}
