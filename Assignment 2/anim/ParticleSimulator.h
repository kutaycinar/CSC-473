#pragma once

#include "BaseSystem.h"
#include "BaseSimulator.h"

#include "ParticleSystem.h"
#include "GlobalResourceManager.h"

class ParticleSimulator : public BaseSimulator
{
protected:

	double delta_t;
	double last_t;

	int maxSpringCount;
	int integration;

	ParticleSystem* partSys;
	BaseSystem* m_object;

public:

	ParticleSimulator(const std::string& name, BaseSystem* target);

	int step(double time);
	int init(double time);

	int command(int argc, myCONST_SPEC char** argv);
};