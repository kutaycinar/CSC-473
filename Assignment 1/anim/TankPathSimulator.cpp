#include "TankPathSimulator.h"

TankPathSimulator::TankPathSimulator(const std::string& name, BaseSystem* target) : BaseSimulator(name), m_object(target)
{
	t = 0.0;
	delta_t = 0.0;
	last_t = 0.0;

	tank_t = 0;

	velocity = 0.0;
	acceleration = 0.00001;
}

int TankPathSimulator::init(double time)
{
	// Reset velocity when Tank Path System resets tank to start position
	double pos[10];
	m_object->getState(pos);
	tank_t = pos[3];
	if (tank_t < 0.01)
	{
		velocity = 0;
	}
	animTcl::OutputMessage("[TankPathSimulator]: SPEED = %.1f KM/H", velocity * SCALE_FACTOR_METER);
	return 0;
};

int TankPathSimulator::step(double time)
{
	t = time;
	delta_t = time - last_t;
	last_t = t;

	double pos[10];
	m_object->getState(pos);

	double stop_simulation = pos[9];
	double tank_t = pos[3];
	double tank_moving = pos[7];

	// If tank is not stopped
	if (!stop_simulation){

		// If tank is not moving, velocity is 0
		if (tank_moving == 0)
			velocity = 0;

		// Print speed/velocity once per second
		if (std::fmod(time, 1) < 0.01)
		{
			animTcl::OutputMessage("[TankPathSimulator]: SPEED = %.1f KM/H", velocity * SCALE_FACTOR_METER);
		}
	}

	// Ease-In motion for t < 0.10
	if (tank_t < 0.10)
	{
		velocity = velocity + acceleration;
	}
	// Ease-Out motion for t > 0.90
	else if (tank_t > 0.90)
	{
		velocity = abs(velocity - acceleration);
	}

	// Move tank only along the curve and stop at the end
	if (tank_t < 1)
	{
		pos[0] = velocity;
		m_object->setState(pos);
	}

	return 0;

}
