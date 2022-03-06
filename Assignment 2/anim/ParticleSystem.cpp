#include "ParticleSystem.h"


ParticleSystem::ParticleSystem(const std::string& name) : BaseSystem(name)
{
	kdrag = 0;
	gravity = -9.8;
	maxSpringCount = 0;

	setVector(groundPoint, 0, 0, 0);
	setVector(groundNormal, 0, 1, 0);

	ground_ks = 0;
	ground_kd = 0;
}

void ParticleSystem::getState(double* p)
{
}

void ParticleSystem::setState(double* p)
{
	double delta_t = p[0];
	int integration = p[1];

	calculateSpringForces();

	switch (integration)
	{
		case Euler:
			forwardEuler(delta_t);
			break;
		case Symplectic:
			symplectic(delta_t);
			break;
		case Verlet:
			verlet(delta_t);
			break;
	}

}

void ParticleSystem::calculateSpringForces()
{
	for (int i = 0; i < Springs.size(); i++)
	{
		// Spring Force
		Vector length;
		VecSubtract(length, Particles[Springs[i].p1].position, Particles[Springs[i].p2].position);
		double inner = Springs[i].restlength - VecLength(length);
		VecScale(length, 1 / VecLength(length));
		VecScale(length, inner * Springs[i].ks);
		VecCopy(Springs[i].f_sp, length);

		// Damper Force
		Vector v_result;
		VecSubtract(v_result, Particles[Springs[i].p1].velocity, Particles[Springs[i].p2].velocity);
		VecSubtract(length, Particles[Springs[i].p1].position, Particles[Springs[i].p2].position);
		VecScale(length, 1 / VecLength(length));
		VecScale(length, VecDotProd(v_result, length) * -Springs[i].kd);
		VecCopy(Springs[i].f_d, length);
	}
}

void ParticleSystem::forwardEuler(double delta_t)
{
	Vector f_net, f_external, f_spring;

	for (int i = 0; i < Particles.size(); i++)
	{
		// FIXED / NAILED CHECK
		if (Particles[i].nailed)
			continue;

		// GROUND FORCE
		calculateGroundForce(f_external, Particles[i]);

		// SPRING FORCE
		calculateSpringForce(f_spring, i);

		// FORCES
		calculateNetForce(f_net, f_external, f_spring, i);

		// POSITIONS (with old velocities)
		Particles[i].position[0] = Particles[i].position[0] + delta_t * Particles[i].velocity[0];
		Particles[i].position[1] = Particles[i].position[1] + delta_t * Particles[i].velocity[1];
		Particles[i].position[2] = Particles[i].position[2] + delta_t * Particles[i].velocity[2];

		// VELOCITIES
		Particles[i].velocity[0] = Particles[i].velocity[0] + delta_t * f_net[0] / Particles[i].mass;
		Particles[i].velocity[1] = Particles[i].velocity[1] + delta_t * f_net[1] / Particles[i].mass;
		Particles[i].velocity[2] = Particles[i].velocity[2] + delta_t * f_net[2] / Particles[i].mass;
	}

}

void ParticleSystem::symplectic(double delta_t)
{
	Vector f_net, f_external, f_spring;

	for (int i = 0; i < Particles.size(); i++)
	{
		// FIXED / NAILED CHECK
		if (Particles[i].nailed)
			continue;

		// GROUND FORCE
		calculateGroundForce(f_external, Particles[i]);

		// SPRING FORCE
		calculateSpringForce(f_spring, i);

		// FORCES
		calculateNetForce(f_net, f_external, f_spring, i);

		// VELOCITIES
		Particles[i].velocity[0] = Particles[i].velocity[0] + delta_t * f_net[0] / Particles[i].mass;
		Particles[i].velocity[1] = Particles[i].velocity[1] + delta_t * f_net[1] / Particles[i].mass;
		Particles[i].velocity[2] = Particles[i].velocity[2] + delta_t * f_net[2] / Particles[i].mass;

		// POSITIONS (with new velocities)
		Particles[i].position[0] = Particles[i].position[0] + delta_t * Particles[i].velocity[0];
		Particles[i].position[1] = Particles[i].position[1] + delta_t * Particles[i].velocity[1];
		Particles[i].position[2] = Particles[i].position[2] + delta_t * Particles[i].velocity[2];
	}
}


void ParticleSystem::verlet(double delta_t)
{
	Vector f_net, f_external, f_spring;

	for (int i = 0; i < Particles.size(); i++)
	{
		// FIXED / NAILED CHECK
		if (Particles[i].nailed)
			continue;

		// GROUND FORCE
		calculateGroundForce(f_external, Particles[i]);

		// SPRING FORCE
		calculateSpringForce(f_spring, i);

		// FORCES
		calculateNetForce(f_net, f_external, f_spring, i);

		// Verlet requires the position at t-h. As per slides, simply use Euler for first step
		if (Particles[i].verlet_firststep)
		{
			Particles[i].verlet_firststep = false;

			// Set t-h (as original position) 
			VecCopy(Particles[i].position_prev, Particles[i].position);

			// Then update new position using Euler integration
			Particles[i].velocity[0] = Particles[i].velocity[0] + delta_t * f_net[0] / Particles[i].mass;
			Particles[i].velocity[1] = Particles[i].velocity[1] + delta_t * f_net[1] / Particles[i].mass;
			Particles[i].velocity[2] = Particles[i].velocity[2] + delta_t * f_net[2] / Particles[i].mass;

			Particles[i].position[0] = Particles[i].position[0] + delta_t * Particles[i].velocity[0];
			Particles[i].position[1] = Particles[i].position[1] + delta_t * Particles[i].velocity[1];
			Particles[i].position[2] = Particles[i].position[2] + delta_t * Particles[i].velocity[2];

			continue;
		}

		// keep track of current position to update previous position at the end
		Vector currentPosition;
		VecCopy(currentPosition, Particles[i].position);

		// POSITIONS (using position prev)
		Particles[i].position[0] = 2 * Particles[i].position[0] - Particles[i].position_prev[0] + (f_net[0] / Particles[i].mass) * delta_t * delta_t;
		Particles[i].position[1] = 2 * Particles[i].position[1] - Particles[i].position_prev[1] + (f_net[1] / Particles[i].mass) * delta_t * delta_t;
		Particles[i].position[2] = 2 * Particles[i].position[2] - Particles[i].position_prev[2] + (f_net[2] / Particles[i].mass) * delta_t * delta_t;

		// VELOCITIES
		Particles[i].velocity[0] = (Particles[i].position[0] - Particles[i].position_prev[0]) / (2 * delta_t);
		Particles[i].velocity[1] = (Particles[i].position[1] - Particles[i].position_prev[1]) / (2 * delta_t);
		Particles[i].velocity[2] = (Particles[i].position[2] - Particles[i].position_prev[2]) / (2 * delta_t);


		VecCopy(Particles[i].position_prev, currentPosition);
	}
}

void ParticleSystem::calculateGroundForce(Vector f, Particle p)
{
	zeroVector(f);
	Vector groundCheck;
	VecSubtract(groundCheck, p.position, groundPoint);
	if (VecDotProd(groundCheck, groundNormal) < 0)
	{
		f[1] = -ground_ks * VecDotProd(groundCheck, groundNormal) * groundNormal[1]
			- ground_kd * VecDotProd(p.velocity, groundNormal) * groundNormal[1];
	}
}

void ParticleSystem::calculateSpringForce(Vector f_spring, int p_index)
{
	zeroVector(f_spring);
	for (int j = 0; j < Springs.size(); j++)
	{
		if (Springs[j].p1 == p_index)
		{
			VecAdd(f_spring, f_spring, Springs[j].f_sp);
			VecAdd(f_spring, f_spring, Springs[j].f_d);
		}
		if (Springs[j].p2 == p_index)
		{
			VecSubtract(f_spring, f_spring, Springs[j].f_sp);
			VecSubtract(f_spring, f_spring, Springs[j].f_d);
		}
	}
}

void ParticleSystem::calculateNetForce(Vector f_net, Vector f_external, Vector f_spring, int p_index)
{
	f_net[0] = -kdrag * Particles[p_index].velocity[0] + f_external[0] + f_spring[0];
	f_net[1] = -kdrag * Particles[p_index].velocity[1] + Particles[p_index].mass * gravity + f_external[1] + f_spring[1];
	f_net[2] = -kdrag * Particles[p_index].velocity[2] + f_external[2] + f_spring[2];
}


void ParticleSystem::reset(double time)
{
	for (int i = 0; i < Particles.size(); i++)
	{
		VecCopy(Particles[i].position, Particles[i].position_initial);
		VecCopy(Particles[i].velocity, Particles[i].velocity_initial);
		Particles[i].verlet_firststep = true; // to re-run verlet first step
	}
}

int ParticleSystem::command(int argc, myCONST_SPEC char** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "dim") == 0)
	{
		if (argc == 2)
		{
			clearSystem();

			for (int i = 0; i < std::stoi(argv[1]); i++)
			{
				Particle newParticle;
				zeroVector(newParticle.position);
				zeroVector(newParticle.velocity);
				newParticle.mass = 0;
				newParticle.nailed = false;
				Particles.push_back(newParticle);
			}
			animTcl::OutputMessage("[ParticleSystem] Created %i particles at (0, 0, 0)", std::stoi(argv[1]));
		}
		else
		{
			animTcl::OutputMessage("[ParticleSystem] Usage: system <sys_name> dim <Number of Particles>");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "particle") == 0)
	{
		if (argc == 9)
		{
			int index = std::stoi(argv[1]);

			if (index > Particles.size() - 1)
			{
				animTcl::OutputMessage("[ParticleSystem] Index out of range");
				return TCL_ERROR;
			}

			Particles.at(index).mass = std::stoi(argv[2]);
			Particles.at(index).position[0] = std::stoi(argv[3]);
			Particles.at(index).position[1] = std::stoi(argv[4]);
			Particles.at(index).position[2] = std::stoi(argv[5]);
			Particles.at(index).velocity[0] = std::stoi(argv[6]);
			Particles.at(index).velocity[1] = std::stoi(argv[7]);
			Particles.at(index).velocity[2] = std::stoi(argv[8]);

			// For Resetting Simulation
			VecCopy(Particles.at(index).position_initial, Particles.at(index).position);
			VecCopy(Particles.at(index).velocity_initial, Particles.at(index).velocity);

			animTcl::OutputMessage("[ParticleSystem] Set particle %d with m=%.1f x=(%.1f %.1f %.1f) v=(%.1f %.1f %.1f)",
				index, Particles.at(index).mass,
				Particles.at(index).position[0], Particles.at(index).position[1], Particles.at(index).position[2],
				Particles.at(index).velocity[0], Particles.at(index).velocity[1], Particles.at(index).velocity[2]
			);
		}
		else
		{
			animTcl::OutputMessage("[ParticleSystem] Usage: system <sys_name> particle <index> <mass> <x y z vx vy vz>");
			return TCL_ERROR;

		}
	}
	else if (strcmp(argv[0], "all_velocities") == 0)
	{
		if (argc == 4)
		{
			double vx, vy, vz;

			vx = std::stoi(argv[1]);
			vy = std::stoi(argv[2]);
			vz = std::stoi(argv[3]);

			for (Particle p : Particles)
				setVector(p.velocity, vx, vy, vz);

			animTcl::OutputMessage("[ParticleSystem] Set all particles v=(%.1f %.1f %.1f)", vx, vy, vz);

		}
		else
		{
			animTcl::OutputMessage("[ParticleSystem] Usage: system <sys_name> all_velocities <vx vy vz> ");
			return TCL_ERROR;

		}
	}

	glutPostRedisplay();
	return TCL_OK;

}

void ParticleSystem::clearSystem()
{
	Particles.clear();
	Springs.clear();
}

void ParticleSystem::setMaximumSpring(int count)
{
	maxSpringCount = count;
}

void ParticleSystem::setGravity(double g)
{
	gravity = g;
}

void ParticleSystem::setGlobalDrag(double kd)
{
	kdrag = kd;
}

void ParticleSystem::setGroundForces(double ks, double kd)
{
	ground_ks = ks;
	ground_kd = kd;
}

void ParticleSystem::setNailed(int index)
{
	Particles[index].nailed = true;
}

void ParticleSystem::addSpring(int p1, int p2, double ks, double kd, double restlength)
{
	Spring newSpring;
	newSpring.p1 = p1;
	newSpring.p2 = p2;
	newSpring.ks = ks;
	newSpring.kd = kd;
	newSpring.restlength = restlength;

	Springs.push_back(newSpring);
}

void ParticleSystem::display(GLenum mode)
{
	// Draw Particles
	glPointSize(5);
	glBegin(GL_POINTS);
	for (Particle p : Particles)
		glVertex3dv(p.position);
	glEnd();

	// Draw Springs
	for (Spring s : Springs)
	{
		glBegin(GL_LINES);
		glVertex3dv(Particles[s.p1].position);
		glVertex3dv(Particles[s.p2].position);
		glEnd();
	}

	// Draw Ground
	glBegin(GL_QUADS);
	glVertex3d(-1, 0, 1);
	glVertex3d(1, 0, 1);
	glVertex3d(1, 0, -1);
	glVertex3d(-1, 0, -1);
	glEnd();
}