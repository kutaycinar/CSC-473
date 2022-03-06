#pragma once

#include "BaseSimulator.h"


#include <GLModel/GLModel.h>
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"

#include "BaseSimulator.h"
#include "BaseSystem.h"

#include "HermiteSystem.h"

#include <string>

#define SCALE_FACTOR_METER 10000

class TankPathSimulator : public BaseSimulator
{
	public:
		TankPathSimulator(const std::string& name, BaseSystem* target);
		int step(double time);
		int init(double time);
		int command(int argc, myCONST_SPEC char** argv) { return TCL_OK; }

	protected:
		double t;
		double delta_t;
		double last_t;
		double tank_t;
		double velocity;
		double acceleration;
		BaseSystem* m_object;
};