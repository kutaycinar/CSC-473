#ifndef MY_SYSTEM_H
#define MY_SYSTEM_H

#include "BaseObject.h"

#include <string>

#include <assert.h>

/* 

base system class. This class is the starting point for every system.

NOTE: This class MUST NEVER be instantiated directly! you must derive your custom systems
	from this one and OVERLOAD the two methods getState and setState
	
*/

class BaseSystem : public BaseObject 
{
public:

	BaseSystem( const std::string& name ):
		BaseObject( name )
	{
	}

	// TODO: these must be overloaded by any derived class

	virtual void getState( double *p );
	virtual void setState( double  *p );

} ;

#endif