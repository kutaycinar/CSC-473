#ifndef GLOBAL_RESOURCE_MANAGER_H
#define	GLOBAL_RESOURCE_MANAGER_H

/*

	This class wraps ResourceManager in a globally accessible object. This 
	implementation follows the singleton design pattern

*/

#include <anim/ResourceManager.h>

class GlobalResourceManager
{
public:
	~GlobalResourceManager(void);

	static ResourceManager* use();

private:

	GlobalResourceManager(void);

	static ResourceManager* m_resourceManagerImplementation;

};

#endif