#include "filter.h"

float medianFilterAppend(float currentValue, float newValue, float coefficient, float maxDifference);


float medianFilterAppend(float currentValue, float newValue, float coefficient, float maxDifference)//0-1024
{
	static float newValueLimitted;

	if((newValue - currentValue) > 0)
	{
		if((newValue - currentValue) > maxDifference)
		{
			newValueLimitted = currentValue + maxDifference;
		}
		else
		{
			newValueLimitted = newValue;
		}
	}
	else
	{
		if((currentValue - newValue) > maxDifference)
		{
			newValueLimitted = currentValue - maxDifference;
		}
		else
		{
			newValueLimitted = newValue;
		}
	}

	return (currentValue + ((newValueLimitted - currentValue) * coefficient));
}
