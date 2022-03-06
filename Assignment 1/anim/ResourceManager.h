#ifndef RESOURCE_MANAGER_H
#define RESOURCE_MANAGER_H

#include "BaseObject.h"
#include "BaseSystem.h"
#include "BaseSimulator.h"

#include <shared/defs.h>

#include <vector>
#include <map>
#include <set>
#include <string>
#include <iostream>

typedef std::vector< BaseObject* > ListOfObjects;
typedef std::vector< BaseSystem* > ListOfSystems;
typedef std::vector< BaseSimulator* > ListOfSimulators;

typedef std::set< BaseObject* > DisplayList;

typedef std::map< std::string, BaseObject* > DatabaseOfObjects;

const double SIMULATION_TIME_STEP = 0.01 ;

class ResourceManager
{
public:
	ResourceManager(void);
	~ResourceManager(void);

	// use this methods to register new objects with the resource manager

	bool addObject( BaseObject* newObject, bool visible = true );
	bool addSystem( BaseSystem* newSystem, bool visible = false );
	bool addSimulator( BaseSimulator* newSimulator, bool visible = false );

	// this method simply calls the reset method for all the objects that were
	// registered. You must implement the reset method for your object for this
	// to have any effect

	void resetAll();

	// accessors

	unsigned int getNumberOfSimulators() const;
	unsigned int getNumberOfSystems() const;

	BaseObject* getObject( const std::string& name ) const;
	BaseSystem* getSystem( const std::string& name ) const;
	BaseSimulator* getSimulator( const std::string& name ) const;

	BaseObject* getObjectFromIndex( unsigned int index ) const;
	BaseSystem* getSystemFromIndex( unsigned int index ) const;
	BaseSimulator* getSimulatorFromIndex( unsigned int index ) const;
	
	GLMouseButtonInfo getMouseButtonInfo() const;
	void setMouseButtonInfo( int button, int state );

	// mutators

	void setSimulationTime( double time );
	double getSimulationTime() const;

	void setActualTime( double time );
	double getActualTime() const;

	// core methods

	void advanceSimulationTime( double timeStep = SIMULATION_TIME_STEP );
	void advanceActualTime( double timeStep = SIMULATION_TIME_STEP );

	void initializeAllSimulators();
	void stepAllSimulators();

	void display( GLenum mode = GL_RENDER );

	// remove all registered systems from the resource manager. All objects are
	// also deallocated from memory, so after you call this function all the 
	// pointers to your objects become invalid.

	void clearAll();

private:

	ListOfObjects m_objectList;
	ListOfSystems m_systemList;
	ListOfSimulators m_simulatorList;

	double m_simulationTime;
	double m_actualTime;

	DisplayList m_displaylist;

	DatabaseOfObjects m_objectDatabase;

// graphics window params
	unsigned int m_windowWidth;
	unsigned int m_windowHeight;
	unsigned int m_windowButton;

// the state of the resource manager
	GlobalStates m_state;

	GLMouseButtonInfo m_mouseButtonInfo;

	bool _checkDuplicateName( const std::string& name ) const;
	BaseObject* _getObjectFromDatabase( const std::string& name ) const;
	void _cleanUp();

};

#endif