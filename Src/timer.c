#include "timer.h"
#include "main.h"



//------------------------------------------------------------------------------
//	CONSTANT & DEFINITIONS
//------------------------------------------------------------------------------
#define TRUE  1
#define FALSE 0
//------------------------------------------------------------------------------
//	FUNCTION PROTOTYPES
//------------------------------------------------------------------------------
void timer1Init(void);
void timerTask(void);
void startTimeScanFlags(void);
void clearTimeScanFlags(void);
//------------------------------------------------------------------------------
//	VARIABLES
//------------------------------------------------------------------------------
unsigned char scanStart1Msec = FALSE;
unsigned char scanStart2Msec = FALSE;
unsigned char scanStart10Msec = FALSE;
unsigned char scanStart50Msec = FALSE;
unsigned char scanStart100Msec = FALSE;
unsigned char scanStart150Msec = FALSE;
unsigned char scanStart250Msec = FALSE;
unsigned char scanStart500Msec = FALSE;
unsigned char scanStart1Sec = FALSE;
unsigned char scanStart10Sec = FALSE;
unsigned char scanStart1Min = FALSE;

unsigned char scan1Msec = FALSE;
unsigned char scan2Msec = FALSE;
unsigned char scan10Msec = FALSE;
unsigned char scan50Msec = FALSE;
unsigned char scan100Msec = FALSE;
unsigned char scan150Msec = FALSE;
unsigned char scan250Msec = FALSE;
unsigned char scan500Msec = FALSE;
unsigned char scan1Sec = FALSE;
unsigned char scan10Sec	= FALSE;
unsigned char scan1Min = FALSE;

unsigned short counter2Msec = 0;
unsigned short counter10Msec = 0;
unsigned short counter50Msec = 0;
unsigned short counter100Msec = 0;
unsigned short counter150Msec = 0;
unsigned short counter250Msec = 0;
unsigned short counter500Msec = 0;
unsigned short counter1Sec = 0;
unsigned short counter10Sec = 0;
unsigned short counter1Min = 0;
//------------------------------------------------------------------------------
//	FUNCTIONS
//------------------------------------------------------------------------------
void timer1Init(void)
{

}
//------------------------------------------------------------------------------
void timerTask(void)
{



	scanStart1Msec = TRUE;

	counter2Msec++;
		if(counter2Msec > 1)
		{
			counter2Msec = 0;
			scanStart2Msec = TRUE;
		}

	counter10Msec++;
	if(counter10Msec > 9)
	{
		counter10Msec = 0;
		scanStart10Msec = TRUE;
	}

	counter50Msec++;
	if(counter50Msec > 49)
	{
		counter50Msec = 0;
		scanStart50Msec = TRUE;
	}

	counter100Msec++;
	if(counter100Msec > 99)
	{
		counter100Msec = 0;
		scanStart100Msec = TRUE;
	}

	counter150Msec++;
	if (counter150Msec > 149)
	{
		counter150Msec = 0;
		scanStart150Msec = TRUE;
	}

	counter250Msec++;
	if(counter250Msec > 249)
	{
		counter250Msec = 0;
		scanStart250Msec = TRUE;
	}

	counter500Msec++;
	if(counter500Msec > 499)
	{
		counter500Msec = 0;
		scanStart500Msec = TRUE;
	}

	counter1Sec++;
	if(counter1Sec > 999)
	{
		counter1Sec = 0;
		scanStart1Sec = TRUE;

		counter10Sec++;
		if(counter10Sec > 9)
		{
			counter10Sec = 0;
			scanStart10Sec = TRUE;
		}

		counter1Min++;
		if(counter1Min > 59)
		{
			counter1Min = 0;
			scanStart1Min = TRUE;
		}
	}
}
//------------------------------------------------------------------------------
void startTimeScanFlags(void)
{

	if(scanStart1Msec)
	{
		scanStart1Msec = FALSE;
		scan1Msec = TRUE;
	}

	if(scanStart2Msec)
		{
			scanStart2Msec = FALSE;
			scan2Msec = TRUE;
		}

	if(scanStart10Msec)
	{
		scanStart10Msec = FALSE;
		scan10Msec = TRUE;
	}
	if(scanStart50Msec)
	{
		scanStart50Msec = FALSE;
		scan50Msec = TRUE;
	}
	if(scanStart100Msec)
	{
		scanStart100Msec = FALSE;
		scan100Msec = TRUE;
	}
	if(scanStart150Msec)
	{
		scanStart150Msec = FALSE;
		scan150Msec = TRUE;
	}
	if(scanStart250Msec)
	{
		scanStart250Msec = FALSE;
		scan250Msec = TRUE;
	}
	if(scanStart500Msec)
	{
		scanStart500Msec = FALSE;
		scan500Msec = TRUE;
	}
	if(scanStart1Sec)
	{
		scanStart1Sec = FALSE;
		scan1Sec = TRUE;
	}
	if(scanStart10Sec)
	{
		scanStart10Sec = FALSE;
		scan10Sec = TRUE;
	}
	if(scanStart1Min)
	{
		scanStart1Min = FALSE;
		scan1Min = TRUE;
	}
}
//------------------------------------------------------------------------------
void clearTimeScanFlags(void)
{
	scan1Msec = FALSE;
	scan2Msec = FALSE;
	scan10Msec = FALSE;
	scan50Msec = FALSE;
	scan100Msec = FALSE;
	scan150Msec = FALSE;
	scan250Msec = FALSE;
	scan500Msec = FALSE;
	scan1Sec = FALSE;
	scan10Sec = FALSE;
	scan1Min = FALSE;
}
