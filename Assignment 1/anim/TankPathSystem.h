#pragma once

#include "BaseSystem.h"

#include "HermiteSystem.h"

#include <math.h>

class TankPathSystem : public BaseSystem
{
	public:
		TankPathSystem(const std::string& name);
		virtual void getState(double* p);
		virtual void setState(double* p);
		void reset(double time);
		void loadModel();
		void calculateMovement(double velocity);
		void calculateRotation();
		void getTankPosition(double* p);
		void setRotation(Quaternion rotation);
		int command(int argc, myCONST_SPEC char** argv);
		void display(GLenum mode);

	protected:
		HermiteSystem* Hermite;
		boolean stop_simulation;
		Vector tank_position;
		Vector tangent;
		Vector orientation;
		double path_position;
		double path_position_last;
		GLMmodel m_model;
		GLfloat rotation_matrix[16] = { 0 };
};

