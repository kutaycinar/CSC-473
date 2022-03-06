#pragma once

#include "BaseSystem.h"
#include <Eigen/Dense>
#include <cmath>

#define SCALE 2.75

enum Color { Green, Red };

struct Skeleton
{
	// 3 translational at the root
	Vector torso_T;
	// 3 rotational at the shoulder : (first around x, then y, then z)
	Vector shoulder_T;
	Vector shoulder_R; // x,y,z
	// 2 rotational at the elbow(first around x then around y)
	Vector elbow_T;
	Vector elbow_R; // x, y
	// 2 rotational at the wrist(y, z) : first around y then around z * /
	Vector wrist_T;
	Vector wrist_R; // y,z
	// For resting position of other arm (could be expanded to support IK if needed)
	Vector shoulder_L;
	Vector elbow_L;
	// P hand
	Vector phand;
	Vector currentP;
	Vector targetP;

	boolean updateRestingPosition;

	Skeleton() {
		zeroVector(torso_T);
		setVector(shoulder_T, 0.55 * SCALE, 1.1 * SCALE, 0);
		zeroVector(shoulder_R);
		setVector(elbow_T, 2 * SCALE, 0, 0);
		zeroVector(elbow_R);
		setVector(wrist_T, 2 * SCALE, 0, 0);
		zeroVector(wrist_R);

		zeroVector(currentP);
		zeroVector(targetP);

		setVector(phand, 1.8, 0, 0); // 1 added in matrix calculation

		zeroVector(shoulder_L);
		zeroVector(elbow_L);

		// starting distance from blackboard for Bob
		torso_T[0] = -2;
		torso_T[1] = -3;
		torso_T[2] = 7.5; 
	}
};

class Bob : public BaseSystem
{
protected:
	Skeleton* skel = new Skeleton();

public:
	Bob(const std::string& name);

	void getState(double* p);
	void setState(double* p);

	void reset(double time);
	void display(GLenum mode = GL_RENDER);

	int command(int argc, myCONST_SPEC char** argv);

	void drawEllipse(double x, double y, Color color);
	void glRotate3D(double x, double y, double z);
	
	Eigen::Matrix4d getTranslation(Vector position);
	Eigen::Matrix4d getRotationX(double theta);
	Eigen::Matrix4d getRotationY(double theta);
	Eigen::Matrix4d getRotationZ(double theta);
	Eigen::Matrix4d getRotationXDerivative(double theta);
	Eigen::Matrix4d getRotationYDerivative(double theta);
	Eigen::Matrix4d getRotationZDerivative(double theta);

	void calculateCurrentP();
	void IKSolver();
};