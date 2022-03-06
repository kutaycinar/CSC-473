#include "ResourceManager.h"

ResourceManager::ResourceManager(void)
{
}

ResourceManager::~ResourceManager(void)
{

	_cleanUp();

}	// ResourceManager::~ResourceManager

bool ResourceManager::addObject( BaseObject* newObject, 
	bool visible )
{

	// why did you pass me a null?

	if( newObject == NULL )
		return false;

	// if the pointer is not an object then bail

	if( dynamic_cast< BaseObject* >( newObject ) == NULL )
		return false;

	// if the name is already in use then bail

	if( _checkDuplicateName( newObject->retrieveName() ) == true )
		return false;	

	m_objectList.push_back( newObject );

	m_objectDatabase[ newObject->retrieveName() ] = newObject;

	if( visible )
		m_displaylist.insert( newObject );

	return true;

}	// ResourceManager::addObject

bool ResourceManager::addSystem( BaseSystem* newSystem, 
	bool visible )
{

	// why did you pass me a null?

	if( newSystem == NULL )
		return false;

	// if the pointer is not a system then bail

	if( dynamic_cast< BaseSystem* >( newSystem ) == NULL )
		return false;

	// if the name is already in use then bail

	if( _checkDuplicateName( newSystem->retrieveName() ) == true )
		return false;	

	m_systemList.push_back( newSystem );

	m_objectDatabase[ newSystem->retrieveName() ] = newSystem;

	if( visible )
		m_displaylist.insert( newSystem );

	return true;

}	// ResourceManager::addSystem

bool ResourceManager::addSimulator( BaseSimulator* newSimulator, 
	bool visible )
{

	// why did you pass me a null?

	if( newSimulator == NULL )
		return false;

	// if the pointer is not a simulator then bail

	if( dynamic_cast< BaseSimulator* >( newSimulator ) == NULL )
		return false;

	// if the name is already in use then bail

	if( _checkDuplicateName( newSimulator->retrieveName() ) == true )
		return false;	

	m_simulatorList.push_back( newSimulator );

	m_objectDatabase[ newSimulator->retrieveName() ] = newSimulator;

	if( visible )
		m_displaylist.insert( newSimulator );

	return true;

}	// ResourceManager::addSimulator

void ResourceManager::resetAll()
{

	ListOfObjects::iterator objectIter;

	for( objectIter = m_objectList.begin(); objectIter != m_objectList.end(); objectIter++ )
	{

		( *objectIter )->reset( m_actualTime );

	}

	ListOfSystems::iterator systemIter;

	for( systemIter = m_systemList.begin(); systemIter != m_systemList.end(); systemIter++ )
	{

		( *systemIter )->reset( m_actualTime );

	}

	ListOfSimulators::iterator simulatorIter;

	for( simulatorIter = m_simulatorList.begin(); simulatorIter != m_simulatorList.end(); simulatorIter++ )
	{

		( *simulatorIter )->reset( m_actualTime );

	}

}	// ResourceManager::resetAll

unsigned int ResourceManager::getNumberOfSystems() const
{

	return static_cast< unsigned int >( m_systemList.size() );

}	// ResourceManager::getNumberOfSystems

unsigned int ResourceManager::getNumberOfSimulators() const
{

	return static_cast< unsigned int >( m_simulatorList.size() );

}	// ResourceManager::getNumberOfSimulators

BaseObject* ResourceManager::getObject( const std::string& name ) const
{

	BaseObject* tempObject;

	// retrieve the object with the given name from the database

	tempObject = _getObjectFromDatabase( name );

	return tempObject;

}	// ResourceManager::getObject

BaseObject* ResourceManager::getObjectFromIndex( unsigned int index ) const
{

	if( index >= m_objectList.size() )
		return NULL;

	return m_objectList.at( index );

}	// ResourceManager::getObjectFromIndex

BaseSystem* ResourceManager::getSystem( const std::string& name ) const
{

	BaseObject* tempObject;

	// retrieve the object with the given name from the database

	tempObject = _getObjectFromDatabase( name );

	// make sure the object was found in the database

	if( tempObject == NULL )
		return NULL;

	// the following ensures that the objects is of the right type. 
	// dynamic_cast returns NULL in case of failure

	BaseSystem* tempSystem = dynamic_cast< BaseSystem* >( tempObject );

	return tempSystem;

}	// ResourceManager::getSystem

BaseSystem* ResourceManager::getSystemFromIndex( unsigned int index ) const
{

	if( index >= m_systemList.size() )
		return NULL;

	return m_systemList.at( index );

}	// ResourceManager::getSystemFromIndex

BaseSimulator* ResourceManager::getSimulator( const std::string& name ) const
{

	BaseObject* tempObject;

	// retrieve the object with the given name from the database

	tempObject = _getObjectFromDatabase( name );

	// make sure the object was found in the database

	if( tempObject == NULL )
		return NULL;

	// the following ensures that the objects is of the right type. 
	// dynamic_cast returns NULL in case of failure

	BaseSimulator* tempSimulator = dynamic_cast< BaseSimulator* >( tempObject );

	return tempSimulator;

}	// ResourceManager::getSimulator

BaseSimulator* ResourceManager::getSimulatorFromIndex( unsigned int index ) const
{

	if( index >= m_simulatorList.size() )
		return NULL;

	return m_simulatorList.at( index );

}	// ResourceManager::getSimulatorFromIndex

GLMouseButtonInfo ResourceManager::getMouseButtonInfo() const
{

	return m_mouseButtonInfo;

}	// ResourceManager::getMouseButtonState

void ResourceManager::setMouseButtonInfo( int button, int state )
{

	m_mouseButtonInfo.button = button;
	m_mouseButtonInfo.state = state;

}	// ResourceManager::setMouseButtonState

void ResourceManager::setSimulationTime( double time )
{

	m_simulationTime = time;

}	// ResourceManager::setSimulationTime

double ResourceManager::getSimulationTime() const
{

	return m_simulationTime;

}	// ResourceManager::getSimulationTime

void ResourceManager::setActualTime( double time )
{

	m_actualTime = time;

}	// ResourceManager::setActualTime

double ResourceManager::getActualTime() const
{

	return m_actualTime;

}	// ResourceManager::getActualTime

void ResourceManager::advanceSimulationTime( double timeStep )
{

	m_simulationTime += timeStep;

}	// ResourceManager::advanceSimulationTime

void ResourceManager::advanceActualTime( double timeStep )
{

	m_actualTime += timeStep;

}	// ResourceManager::advanceActualTime


void ResourceManager::initializeAllSimulators()
{

	// initialize all simulators to the current simulationTime

	ListOfSimulators::iterator iter;

	for( iter = m_simulatorList.begin(); iter != m_simulatorList.end(); iter++ )
	{

		(*iter)->init( m_simulationTime );

	}

}	// ResourceManager::initializeAllSimulators


void ResourceManager::stepAllSimulators()
{

	// invoke the step function for all simulators

	ListOfSimulators::iterator iter;

	for( iter = m_simulatorList.begin(); iter != m_simulatorList.end(); iter++ )
	{

		(*iter)->step( m_simulationTime );

	}


}	// ResourceManager::stepAllSimulators

void ResourceManager::display( GLenum mode )
{

	// iterate over the display list and call the display function of each item

	DisplayList::const_iterator iter;

	for( iter = m_displaylist.begin(); iter != m_displaylist.end(); iter++ )
	{

		(*iter)->display( mode );

	}

}	// ResourceManager::display

void ResourceManager::clearAll()
{

	_cleanUp();

}	// ResourceManager::clearAll

bool ResourceManager::_checkDuplicateName( const std::string& name ) const
{

	// check if the name was already stored in the database of objects

	DatabaseOfObjects::const_iterator itemFound;

	itemFound = m_objectDatabase.find( name );

	if( itemFound == m_objectDatabase.end() )
		return false;

	return true;

}	// ResourceManager::_checkDuplicateName

BaseObject* ResourceManager::_getObjectFromDatabase( const std::string& name ) const
{

	DatabaseOfObjects::const_iterator itemFound;

	itemFound = m_objectDatabase.find( name );

	// if the object is not in the database then return NULL
	if( itemFound == m_objectDatabase.end() )
		return NULL;

	// dereference the iterator and return the actual object ( send item of the std::map )
	return (*itemFound).second;

}	// ResourceManager::_getObjectFromDatabase

void ResourceManager::_cleanUp()
{

	ListOfObjects::iterator objectIter;

	for( objectIter = m_objectList.begin(); objectIter != m_objectList.end(); objectIter++ )
	{

		delete ( *objectIter );

	}

	m_objectList.clear();

	ListOfSystems::iterator systemIter;

	for( systemIter = m_systemList.begin(); systemIter != m_systemList.end(); systemIter++ )
	{

		delete ( *systemIter );

	}

	m_systemList.clear();

	ListOfSimulators::iterator simulatorIter;

	for( simulatorIter = m_simulatorList.begin(); simulatorIter != m_simulatorList.end(); simulatorIter++ )
	{

		delete ( *simulatorIter );

	}

	m_simulatorList.clear();

	m_objectDatabase.clear();

	m_displaylist.clear();

}	// ResourceManager::_cleanUp