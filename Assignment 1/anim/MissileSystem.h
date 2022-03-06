#pragma once


#include "BaseSystem.h"

#include "TankPathSystem.h"
#include "HermiteSystem.h"

class MissileSystem : public BaseSystem
{
	public:
		MissileSystem(const std::string& name);

		virtual void getState(double* p);
		virtual void setState(double* p);
		void loadModel();
		void setRotation(Quaternion rotation);
		void setMissleState(double x, double y, double z, double v1, double v2, double v3, double e);
		void calculateRotation();
		void setPathDisplay(double x, double y, double z, double sx, double sy, double sz);
		void reset(double time);
		void display(GLenum mode);
		int command(int argc, myCONST_SPEC char** argv);

	protected:
		HermiteSystem* Hermite;
		HermiteSystem* PathMissile;
		GLMmodel m_model;
		GLfloat rotation_matrix[16] = { 0 };
		Vector length;
		Vector tangent;
		Quaternion rotation;
		Vector missile_position;
		Vector tank_position;
		boolean stop_simulation;
		double speed;
};
