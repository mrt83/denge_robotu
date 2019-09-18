#include "interpolation.h"

float interpolation(float inputVal, float x1, float x2, float y1, float y2);
float limitedInterpolation(float inputVal, float x1, float x2, float y1, float y2);

float interpolation(float inputVal, float x1, float x2, float y1, float y2)
{
    static float deltaX = 0.0;
    static float deltaY = 0.0;

    static float m = 0.0;
    static float c = 0.0;

    static float outputVal = 0.0;

    //y = m.x + c

    deltaX = (x2 - x1);
    deltaY = (y2 - y1);

    m = (deltaY/deltaX);

    c = (y1 - (m*x1));

    outputVal = ((m*inputVal) + c);

	return outputVal;
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
float limitedInterpolation(float inputVal, float x1, float x2, float y1, float y2)
{
	float limitedOutputVal = 0;

	limitedOutputVal = interpolation( inputVal, x1, x2, y1, y2 );

	if(y1 > y2)
	{
		if(limitedOutputVal > y1)
		{
			return y1;
		}
		else if(limitedOutputVal < y2)
		{
			return y2;
		}
		else
		{
			return limitedOutputVal;
		}
	}
	else
	{
		if(limitedOutputVal > y2)
		{
			return y2;
		}
		else if(limitedOutputVal < y1)
		{
			return y1;
		}
		else
		{
			return limitedOutputVal;
		}
	}
}
