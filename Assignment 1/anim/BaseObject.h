#ifndef MY_OBJECT_H
#define MY_OBJECT_H

#include <GLModel/GLModel.h>
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"


#include <string>

// Top level class. All objects must inherit from this one

class BaseObject 
{

public:

	// constructor. All objects must have a name

	BaseObject( const std::string& name );
	virtual ~BaseObject();

	// obtain the name of the object

	void getName( std::string& name ) const;
	std::string retrieveName() const;

	// override this method if you want the object to be drawable

	virtual void display( GLenum mode = GL_RENDER );

	// override this method to enable shell interaction

	virtual int command( int argc, myCONST_SPEC char **argv );

	// override this method to specify what happens when the object is reset

	virtual void reset( double time );

protected:

	std::string m_name;

		// this mutator is not publicly available because changing the name 
	// of an object directly messes up the consistency of ResourceManager

	void _setName( std::string& name );

};

#endif