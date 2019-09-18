#ifndef TIMER_H_
#define TIMER_H_
//	CONSTANT & DEFINITIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	FUNCTION PROTOTYPES
//------------------------------------------------------------------------------
extern void timerTask(void);
extern void startTimeScanFlags(void);
extern void clearTimeScanFlags(void);
//------------------------------------------------------------------------------
//	VARIABLES
//------------------------------------------------------------------------------
extern unsigned char scan1Msec;
extern unsigned char scan2Msec;
extern unsigned char scan10Msec;
extern unsigned char scan50Msec;
extern unsigned char scan100Msec;
extern unsigned char scan150Msec;
extern unsigned char scan250Msec;
extern unsigned char scan500Msec;
extern unsigned char scan1Sec;
extern unsigned char scan10Sec;
extern unsigned char scan1Min;

#endif
