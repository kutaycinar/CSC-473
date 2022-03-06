#include "MissileSimulator.h"

MissileSimulator::MissileSimulator(const std::string& name, BaseSystem* target) : BaseSimulator(name), m_object(target)
{
	t = 0.0;
	delta_t = 0.0;
	last_t = 0.0;

	velocity = 0.0;
	acceleration = 0.0002;
}

void MissileSimulator::setTankPath(BaseSystem* tankpathsystem)
{
	tank = tankpathsystem;
}

int MissileSimulator::init(double time)
{
	return 0;
};

int MissileSimulator::step(double time)
{
	double tank_position[10];
	tank->getState(tank_position);

	double missile_position[10];
	m_object->getState(missile_position);

	tank_position[9] = missile_position[9];
	m_object->setState(tank_position);

	if (missile_position[9] == true)
	{
		tank->setState(tank_position);
	}

	return 0;
}
