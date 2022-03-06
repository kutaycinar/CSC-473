#ifndef DEFS_H
#define DEFS_H 1

#ifdef WIN32
#include <windows.h>
#endif

#include <cstdio>
#include <cstdlib>


const int STRLEN = 100;
typedef char STR[STRLEN];

enum GlobalStates
{ 
	STATE_PLAYBACK, 
	STATE_ANIMATE,
	STATE_SIMULATE,
	STATE_IDLE
};

struct GLMouseButtonInfo
{
	int button;
	int state;
};

void PrintMessage(char *format, ...) ; // must be implemented by the console handling module
const int A_ERROR = 0 ;
const int A_OK = 1 ;


#endif