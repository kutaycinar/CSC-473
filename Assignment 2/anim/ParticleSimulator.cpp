#include "ParticleSimulator.h"

ParticleSimulator::ParticleSimulator(const std::string& name, BaseSystem* target) :
	BaseSimulator(name),
	m_object(target)
{
	delta_t = 0;
	last_t= 0;
}

int ParticleSimulator::step(double time)
{
	delta_t = time - last_t;
	last_t = time;

	Vector load;
	load[0] = delta_t;
	load[1] = integration;

	m_object->setState(load);

	return 0;
}

int ParticleSimulator::init(double time)
{
	last_t = 0; // For resetting simulation

	return 0;
}

int ParticleSimulator::command(int argc, myCONST_SPEC char** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("simulator %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "link") == 0)
	{
		if (argc == 3)
		{
			partSys = dynamic_cast<ParticleSystem*>(GlobalResourceManager::use()->getSystem(argv[1]));

			if (!partSys)
			{
				animTcl::OutputMessage("[ParticleSimulator] Cannot link particle system");
				return TCL_ERROR;
			}

			partSys->setMaximumSpring(std::stoi(argv[2]));

			animTcl::OutputMessage("[ParticleSimulator] Initialized %d springs in ParticleSystem", std::stoi(argv[2]));

		}
		else
		{
			animTcl::OutputMessage("[ParticleSimulator] Usage: simulator <sim_name> link <sys name> <Number of Springs>");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "spring") == 0)
	{
		if (argc == 6)
		{
			int p1 = std::stoi(argv[1]);
			int p2 = std::stoi(argv[2]);
			double ks = std::stod(argv[3]);
			double kd = std::stod(argv[4]);
			double restlength = std::stod(argv[5]);

			partSys->addSpring(p1, p2, ks, kd, restlength);

			animTcl::OutputMessage("[ParticleSimulator] Added spring p1=%d p2=%d ks=%.1f kd=%.1f restlength=%.1f", p1, p2, ks, kd, restlength);

		}
		else
		{
			animTcl::OutputMessage("[ParticleSimulator] Usage: simulator <sim_name> spring <index1> <index2> <ks> <kd> <restlength>");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "fix") == 0)
	{
		if (argc == 2)
		{
			partSys->setNailed(std::stoi(argv[1]));
			animTcl::OutputMessage("[ParticleSimulator] Fixed particle %d", std::stoi(argv[1]));
		}
		else
		{
			animTcl::OutputMessage("[ParticleSimulator] Usage: simulator <sim_name> spring <index1> <index2> <ks> <kd> <restlength>");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "integration") == 0)
	{
		if (argc == 3)
		{
			if (strcmp(argv[1], "euler") == 0)
			{
				integration = Euler;
			}
			if (strcmp(argv[1], "symplectic") == 0)
			{
				integration = Symplectic;
			}
			if (strcmp(argv[1], "verlet") == 0)
			{
				integration = Verlet;
			}

			// Sets time step for simulation in ResourceManager class
			GlobalResourceManager::use()->setSimulationStep(std::stod(argv[2]));
		}
		else
		{
			animTcl::OutputMessage("[ParticleSimulator] Usage: simulator <sim_name> integration <euler|symplectic|verlet> <time step>");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "ground") == 0)
	{
		if (argc == 3)
		{
			partSys->setGroundForces(std::stod(argv[1]), std::stod(argv[2]));
			animTcl::OutputMessage("[ParticleSimulator] Set ground ks=%.1f kd=%.1f", std::stod(argv[1]), std::stod(argv[2]));
		}
		else
		{
			animTcl::OutputMessage("[ParticleSimulator] Usage: simulator <sim_name> ground <ks> <kd>");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "gravity") == 0)
	{
		if (argc == 2)
		{
			partSys->setGravity(std::stod(argv[1]));

			animTcl::OutputMessage("[ParticleSimulator] Set gravity=%.1f", std::stod(argv[1]));
		}
		else
		{
			animTcl::OutputMessage("[ParticleSimulator] Usage: simulator <sim_name> gravity <g>");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "drag") == 0)
	{
		if (argc == 2)
		{
			partSys->setGlobalDrag(std::stod(argv[1]));
			animTcl::OutputMessage("[ParticleSimulator] Set kdrag=%.1f", std::stod(argv[1]));
		}
		else
		{
			animTcl::OutputMessage("[ParticleSimulator] Usage: simulator <sim_name> drag <kdrag>");
			return TCL_ERROR;
		}
	}

	glutPostRedisplay();
	return TCL_OK;
}