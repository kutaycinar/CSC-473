#pragma once

#include "BaseSystem.h"

#include <vector>
#include <math.h>

enum Integration { Euler, Symplectic, Verlet };

struct Particle
{
	Vector position;
	Vector velocity;
	double mass;
	boolean nailed;

	boolean verlet_firststep;

	Vector position_prev; // for Verlet integration
	Vector position_initial; // for resetting simulation
	Vector velocity_initial; // for resetting simulation

};

struct Spring
{
	int p1;
	int p2;
	double ks;
	double kd;
	double restlength;

	Vector f_sp;
	Vector f_d;
};

class ParticleSystem : public BaseSystem
{
protected:

	std::vector<Particle> Particles;
	std::vector<Spring> Springs;

	double kdrag;
	double gravity;

	Vector groundPoint;
	Vector groundNormal;

	double ground_ks;
	double ground_kd;

	int maxSpringCount;

public:

	ParticleSystem(const std::string& name);

	void setMaximumSpring(int count);
	void setGlobalDrag(double kd);
	void setGravity(double g);
	void setGroundForces(double ks, double kd);
	void setNailed(int index);
	void addSpring(int p1, int p2, double ks, double kd, double restlength);

	void calculateSpringForces();
	void calculateGroundForce(Vector f, Particle p);
	void calculateSpringForce(Vector f_spring, int p_index);
	void calculateNetForce(Vector f_net, Vector f_external, Vector f_spring, int p_index);

	void forwardEuler(double delta_t);
	void symplectic(double delta_t);
	void verlet(double delta_t);

	virtual void getState(double* p);
	virtual void setState(double* p);

	void clearSystem();
	void reset(double time);
	void display(GLenum mode = GL_RENDER);

	int command(int argc, myCONST_SPEC char** argv);
};