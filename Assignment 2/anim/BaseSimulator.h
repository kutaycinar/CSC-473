#ifndef MY_SIMULATOR_H
#define MY_SIMULATOR_H

#include "BaseObject.h"
#include <string>
#include <assert.h>

/* 

base simulator class. This class is the starting point for every simulator.

NOTE: This class MUST NEVER be instantiated directly! you must derive your custom simulators
	from this one and OVERLOAD the two methods step and init
	
*/

class BaseSimulator : public BaseObject 
{
public:

	BaseSimulator( const std::string& name ):
		BaseObject( name )
	{
	}

	virtual int step(double time);
	virtual int init(double time);

} ;

#endif