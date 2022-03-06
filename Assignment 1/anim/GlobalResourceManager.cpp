#include "GlobalResourceManager.h"

ResourceManager* GlobalResourceManager::m_resourceManagerImplementation = NULL;

GlobalResourceManager::GlobalResourceManager(void)
{
}	// GlobalResourceManager::GlobalResourceManager

GlobalResourceManager::~GlobalResourceManager(void)
{
}	// GlobalResourceManager::~GlobalResourceManager

ResourceManager* GlobalResourceManager::use()
{

	if( m_resourceManagerImplementation == NULL)
	{
		m_resourceManagerImplementation = new ResourceManager();
	}

	return m_resourceManagerImplementation;

}	// ResourceManager* GlobalResourceManager::use
