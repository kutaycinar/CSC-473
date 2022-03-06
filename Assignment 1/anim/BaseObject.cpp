#include "BaseObject.h"

BaseObject::BaseObject( const std::string& name ):
	m_name( name )
{	
}	// BaseObject::BaseObject

BaseObject::~BaseObject()
{
}	// BaseObject::~BaseObject

void BaseObject::getName( std::string& name ) const
{ 

	name = m_name;

}	// BaseObject::getName

std::string BaseObject::retrieveName() const
{

	return m_name;

}	// BaseObject::retrieveName

void BaseObject::display( GLenum mode )
{ 

	// TODO: you must overload this function if you want 
	// derived objects to draw themselves

	return;

}	// BaseObject::display

int BaseObject::command( int argc, myCONST_SPEC char **argv )
{

	// TODO: you must overload this function if you want your function
	// to interact with the shell

	return TCL_OK;

}	// BaseObject::command

void BaseObject::reset( double time ) 
{ 

	// TODO: overload this function to specify how an object should be reset

	return;

}

void BaseObject::_setName( std::string& name )
{

	m_name = name;

}	// BaseObject::setName
