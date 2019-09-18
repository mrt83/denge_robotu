//#ifndef PIDCONTROL_H_
//#define PIDCONTROL_H_

typedef struct
{
	//Default
	float EPSILON;
	float DT;
	float KP;
	float KI;
	float KD;
	float PID_ERROR_LIMIT;
	float INTEGRAL_INITIAL_VALUE;
	float INTEGRAL_MAX_VAL;
	float INTEGRAL_MIN_VAL;
	float MAX_OUTPUT;
	//Input
	float feedbackValue;
	float setValue;
	//Internal
	float previousError;
	float integral;
	float error;
	float derivative;
	//Output
	float output;
}PID;

typedef enum
{
	nonLimiting = 0,
	negativeLimiting,
	positiveLimiting,
	fullLimiting
}LimitingType;

extern void pid_reset( PID * device );
extern void pid_control( PID * device , LimitingType type);
extern void pid_load_value(PID * device, float setValue);

//#endif /* PIDCONTROL_H_ */
