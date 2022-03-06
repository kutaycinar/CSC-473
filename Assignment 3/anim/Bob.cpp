#include "Bob.h"

Bob::Bob(const std::string& name) : BaseSystem(name)
{
	skel = new Skeleton();
	calculateCurrentP();
}

void Bob::getState(double* p)
{
	VecCopy(p, skel->currentP);
	glutPostRedisplay();
}

void Bob::setState(double* p)
{
	Skeleton *ikSkeleton = (Skeleton*)p;
	if (ikSkeleton->updateRestingPosition == true)
	{
		skel = ikSkeleton;
		calculateCurrentP();
		return;
	}

	VecCopy(skel->targetP, p);
	IKSolver();
}

void Bob::reset(double time)
{
}

int Bob::command(int argc, myCONST_SPEC char** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "position") == 0)
	{
		if (argc == 4)
		{
			skel->torso_T[0] = atof(argv[1]);
			skel->torso_T[1] = atof(argv[2]);
			skel->torso_T[2] = atof(argv[3]);

			animTcl::OutputMessage("[bob] Repositioned bob");
		}
		else
		{
			animTcl::OutputMessage("[bob] Usage: system bob position <x y z>");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "tr") == 0)
	{
		if (argc == 4)
		{
			skel->targetP[0] += atof(argv[1]);
			skel->targetP[1] += atof(argv[2]);
			skel->targetP[2] += atof(argv[3]);

			animTcl::OutputMessage("[bob] Repositioned target");
		}
	}
	else if (strcmp(argv[0], "ta") == 0)
	{
		if (argc == 4)
		{
			skel->targetP[0] = atof(argv[1]);
			skel->targetP[1] = atof(argv[2]);
			skel->targetP[2] = atof(argv[3]);

			animTcl::OutputMessage("[bob] Repositioned target");
		}
	}
	else if (strcmp(argv[0], "jacobian") == 0)
	{
		IKSolver();
	}
	glutPostRedisplay();
	return TCL_OK;

}

void Bob::drawEllipse(double x, double y, Color color = Green)
{
	switch (color)
	{
		case Green:
			set_colour(0, 1, 0);
			break;
		case Red:
			set_colour(1, 0, 0);
			break;
	}
	glBegin(GL_LINE_LOOP);
	for (int i = 0; i < 360; i++)
	{
		glVertex2f(cos(i * PI / 180) * x / 2, sin(i * PI / 180) * y / 2);
	}
	glEnd();
}

Eigen::Matrix4d Bob::getTranslation(Vector position)
{
	Eigen::Matrix4d matrix;
	matrix << 1, 0, 0, position[0],
		0, 1, 0, position[1],
		0, 0, 1, position[2],
		0, 0, 0, 1;
	return matrix;
}

Eigen::Matrix4d Bob::getRotationX(double theta)
{
	Eigen::Matrix4d matrix;
	matrix << 1, 0, 0, 0,
		0, cos(theta * PI / 180), -sin(theta * PI / 180), 0,
		0, sin(theta * PI / 180), cos(theta * PI / 180), 0,
		0, 0, 0, 1;
	return matrix;	
}

Eigen::Matrix4d Bob::getRotationY(double theta)
{
	Eigen::Matrix4d matrix;
	matrix << cos(theta * PI / 180), 0, sin(theta * PI / 180), 0,
		0, 1, 0, 0,
		-sin(theta * PI / 180), 0, cos(theta * PI / 180), 0,
		0, 0, 0, 1;
	return matrix;
}

Eigen::Matrix4d Bob::getRotationZ(double theta)
{
	Eigen::Matrix4d matrix;
	matrix << cos(theta * PI / 180), -sin(theta * PI / 180), 0, 0,
		sin(theta * PI / 180), cos(theta * PI / 180), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	return matrix;
}

Eigen::Matrix4d Bob::getRotationXDerivative(double theta)
{
	Eigen::Matrix4d matrix;
	matrix << 1, 0, 0, 0,
		0, -sin(theta * PI / 180), -cos(theta * PI / 180), 0,
		0, cos(theta * PI / 180), -sin(theta * PI / 180), 0,
		0, 0, 0, 1;
	return matrix;
}

Eigen::Matrix4d Bob::getRotationYDerivative(double theta)
{
	Eigen::Matrix4d matrix;
	matrix << -sin(theta * PI / 180), 0, cos(theta * PI / 180), 0,
		0, 1, 0, 0,
		-cos(theta * PI / 180), 0, -sin(theta * PI / 180), 0,
		0, 0, 0, 1;
	return matrix;
}

Eigen::Matrix4d Bob::getRotationZDerivative(double theta)
{
	Eigen::Matrix4d matrix;
	matrix << -sin(theta * PI / 180), -cos(theta * PI / 180), 0, 0,
		cos(theta * PI / 180), -sin(theta * PI / 180), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	return matrix;
}

void Bob::calculateCurrentP()
{
	Eigen::Vector4d phand= { skel->phand[0], skel->phand[1], skel->phand[2], 1 };
	Eigen::Vector4d currentP4 =
		getTranslation(skel->torso_T) * getTranslation(skel->shoulder_T) *
			getRotationX(skel->shoulder_R[0]) *
			getRotationY(skel->shoulder_R[1]) *
			getRotationZ(skel->shoulder_R[2]) *
		getTranslation(skel->elbow_T) *
			getRotationX(skel->elbow_R[0]) *
			getRotationY(skel->elbow_R[1]) *
		getTranslation(skel->wrist_T) *
			getRotationY(skel->wrist_R[1]) *
			getRotationZ(skel->wrist_R[2])*
		phand
	;
	Eigen::Vector3d currentP;
	currentP[0] = currentP4[0];
	currentP[1] = currentP4[1];
	currentP[2] = currentP4[2];

	// set current P to bob
	setVector(skel->currentP, currentP[0], currentP[1], currentP[2]);
}

void Bob::IKSolver()
{
	Eigen::Matrix<double, 3, 7> jacobian;
	Eigen::Matrix<double, 7, 3> jacobianTranspose;
	Eigen::Matrix<double, 3, 3> jjT;

	calculateCurrentP();

	Eigen::Vector3d currentP = { skel->currentP[0], skel->currentP[1], skel->currentP[2] };
	Eigen::Vector3d targetP = { skel->targetP[0], skel->targetP[1], skel->targetP[2] };
	Eigen::Vector4d phand = { skel->phand[0], skel->phand[1], skel->phand[2], 1 };

	Eigen::Vector3d error = targetP - currentP;

	// target out of reach, cannot solve
	if (error.norm() > 14)
		return;

	double threshold = 0.05;
	while (true)
	{
		currentP = { skel->currentP[0], skel->currentP[1], skel->currentP[2] };
		targetP = { skel->targetP[0], skel->targetP[1], skel->targetP[2] };

		error = targetP - currentP;

		if (error.norm() < threshold)
			return;

		// JACBOIAN
		for (int i = 0; i < 7; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				Eigen::Vector4d result =
					getTranslation(skel->torso_T) * getTranslation(skel->shoulder_T) *
						(i == 2 ? getRotationZDerivative(skel->shoulder_R[2]) : getRotationZ(skel->shoulder_R[2])) *
						(i == 1 ? getRotationYDerivative(skel->shoulder_R[1]) : getRotationY(skel->shoulder_R[1])) *
						(i == 0 ? getRotationXDerivative(skel->shoulder_R[0]) : getRotationX(skel->shoulder_R[0])) *
					getTranslation(skel->elbow_T) *
						(i == 4 ? getRotationYDerivative(skel->elbow_R[1]) : getRotationY(skel->elbow_R[1])) *
						(i == 3 ? getRotationXDerivative(skel->elbow_R[0]) : getRotationX(skel->elbow_R[0])) *
					getTranslation(skel->wrist_T) *
						(i == 6 ? getRotationZDerivative(skel->wrist_R[2]) : getRotationZ(skel->wrist_R[2])) *
						(i == 5 ? getRotationYDerivative(skel->wrist_R[1]) : getRotationY(skel->wrist_R[1])) *
					phand;

				jacobian(0, i) = result[0];
				jacobian(1, i) = result[1];
				jacobian(2, i) = result[2];
			}
		}

		// JTRANPOSE
		jacobianTranspose = jacobian.transpose();

		// JJT
		jjT = jacobian * jacobianTranspose;

		// IK SOLVING VIA PSUEDOINVERSE WITH Singular-value decomposition
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(jjT, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::Vector<double, 3> beta = svd.solve(error);
		Eigen::Vector<double, 7> end_result = jacobianTranspose * beta;

		// UPDATE ANGLES
		skel->shoulder_R[0] += end_result[0] * 180 / PI;
		skel->shoulder_R[1] += end_result[1] * 180 / PI;
		skel->shoulder_R[2] += end_result[2] * 180 / PI;
		skel->elbow_R[0] += end_result[3] * 180 / PI;
		skel->elbow_R[1] += end_result[4] * 180 / PI;
		skel->wrist_R[1] += end_result[5] * 180 / PI;
		skel->wrist_R[2] += end_result[6] * 180 / PI;

		// CLAMP ROTATIONS TO 0-360 DEGREES (avoids numbers getting too large)
		skel->shoulder_R[0] = fmod((skel->shoulder_R[0]), 360);
		skel->shoulder_R[1] = fmod((skel->shoulder_R[1]), 360);
		skel->shoulder_R[2] = fmod((skel->shoulder_R[2]), 360);
		skel->elbow_R[0] = fmod((skel->elbow_R[0]), 360);
		skel->elbow_R[1] = fmod((skel->elbow_R[1]), 360);
		skel->wrist_R[1] = fmod((skel->wrist_R[1]), 360);
		skel->wrist_R[2] = fmod((skel->wrist_R[2]), 360);

		// UPDATE CURRENT P
		calculateCurrentP();
	}

}

void Bob::glRotate3D(double x, double y, double z)
{
	glRotated(x, 1, 0, 0);
	glRotated(y, 0, 1, 0);
	glRotated(z, 0, 0, 1);
}

void Bob::display(GLenum mode)
{
	// Show currentP (blue) and targetP (purple)
	GLfloat ambient[] = { 5, 5, 5, 1.0 };
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);
	glPointSize(1);
	glBegin(GL_POINTS);
	set_colour(0, 0, 1);
	glVertex3dv(skel->currentP);
	set_colour(1, 0, 1);
	glVertex3dv(skel->targetP);
	glEnd();

	// Bob Skeleton

	// Body
	glPushMatrix();
		glTranslated(skel->torso_T[0], skel->torso_T[1], skel->torso_T[2]);
		glRotate3D(0, 0, 0);
		drawEllipse(0.01 * SCALE, 0.01 * SCALE, Red); // object origin
		drawEllipse(1.5* SCALE, 2.8* SCALE);

		// Head
		glPushMatrix();
			glTranslated(0, 2* SCALE, 0);
			drawEllipse(1* SCALE, 1* SCALE);
		glPopMatrix();
		
		// Right Shoulder
		glPushMatrix();
			// Set offset
			glTranslated(skel->shoulder_T[0], skel->shoulder_T[1], skel->shoulder_T[2]);
			glRotate3D(skel->shoulder_R[0], skel->shoulder_R[1], skel->shoulder_R[2]);
			drawEllipse(0.01 * SCALE, 0.01 * SCALE, Red); // joint origin
			glPushMatrix();
				glTranslated(1* SCALE, 0, 0);
				drawEllipse(2 * SCALE, 0.3 * SCALE);
			glPopMatrix();

			// Right Elbow
			glPushMatrix();
				// Set offset
				glTranslated(skel->elbow_T[0], skel->elbow_T[1], skel->elbow_T[2]);
				glRotate3D(skel->elbow_R[0], skel->elbow_R[1], 0);
				drawEllipse(0.01 * SCALE, 0.01 * SCALE, Red); // joint origin
				glPushMatrix();
					glTranslated(1 * SCALE, 0, 0);
					drawEllipse(2 * SCALE, 0.3 * SCALE);
				glPopMatrix();

				// Right Hand
				glPushMatrix();
					// Set offset
					glTranslated(skel->wrist_T[0], skel->wrist_T[1], skel->wrist_T[2]);
					glRotate3D(0, skel->wrist_R[1], skel->wrist_R[2]);
					drawEllipse(0.01 * SCALE, 0.01 * SCALE, Red); // joint origin
					glPushMatrix();
						glTranslated(0.9, 0, 0);
						drawEllipse(1.8, 1.2);
					glPopMatrix();
				glPopMatrix();
			glPopMatrix();
		glPopMatrix();

		// Left Shoulder
		glPushMatrix();
			glTranslated(-0.55 * SCALE, 1.1 * SCALE, 0);
			glRotate3D(skel->shoulder_L[0], skel->shoulder_L[1], skel->shoulder_L[2]);
			drawEllipse(0.01 * SCALE, 0.01 * SCALE, Red); // joint origin
			glPushMatrix();
				glTranslated(-1* SCALE, 0, 0);
				drawEllipse(2 * SCALE, 0.3 * SCALE);
			glPopMatrix();

			// Left Arm
			glPushMatrix();
				glTranslated(-2 * SCALE, 0, 0);
				glRotate3D(skel->elbow_L[0], skel->elbow_L[1], 0);
				drawEllipse(0.01 * SCALE, 0.01 * SCALE, Red); // joint origin
				glPushMatrix();
					glTranslated(-1 * SCALE, 0, 0);
					drawEllipse(2 * SCALE, 0.3 * SCALE);
				glPopMatrix();

				// Left Hand
				glPushMatrix();
					glTranslated(-2 * SCALE, 0, 0);
					drawEllipse(0.01 * SCALE, 0.01 * SCALE, Red); // joint origin
					glPushMatrix();
						glTranslated(-0.3 * SCALE, 0, 0);
						drawEllipse(0.6 * SCALE, 0.4 * SCALE);
					glPopMatrix();
				glPopMatrix();
			glPopMatrix();
		glPopMatrix();

		
		// Left Upper Leg
		glPushMatrix();
			glTranslated(-0.3 * SCALE, -1.4 * SCALE, 0);
			drawEllipse(0.01 * SCALE, 0.01 * SCALE, Red); // joint origin
			glPushMatrix();
				glTranslated(0, -1 * SCALE, 0);
				drawEllipse(0.3 * SCALE, 2 * SCALE);
			glPopMatrix();

			// Left Lower Leg
			glPushMatrix();
				glTranslated(0, -2 * SCALE, 0);
				drawEllipse(0.01 * SCALE, 0.01 * SCALE, Red); // joint origin
				glPushMatrix();
					glTranslated(0, -1 * SCALE, 0);
					drawEllipse(0.3 * SCALE, 2 * SCALE);
				glPopMatrix();

				// Left Foot
				glPushMatrix();
					glTranslated(0, -2 * SCALE, 0);
					drawEllipse(0.01 * SCALE, 0.01 * SCALE, Red); // joint origin
					glPushMatrix();
						glTranslated(0, -0.01 * SCALE, 0);
						drawEllipse(0.4 * SCALE, 0.2 * SCALE);
					glPopMatrix();
				glPopMatrix();
			glPopMatrix();
		glPopMatrix();

		
		// Right Upper Leg
		glPushMatrix();
			glTranslated(0.3 * SCALE, -1.4 * SCALE, 0);
			drawEllipse(0.01 * SCALE, 0.01 * SCALE, Red); // joint origin
			glPushMatrix();
				glTranslated(0, -1 * SCALE, 0);
				drawEllipse(0.3 * SCALE, 2 * SCALE);
			glPopMatrix();

			// Right Lower Leg
			glPushMatrix();
				glTranslated(0, -2 * SCALE, 0);
				drawEllipse(0.01 * SCALE, 0.01 * SCALE, Red); // joint origin
				glPushMatrix();
					glTranslated(0, -1 * SCALE, 0);
					drawEllipse(0.3 * SCALE, 2 * SCALE);
				glPopMatrix();

				// Right Foot
				glPushMatrix();
					glTranslated(0, -2 * SCALE, 0);
					drawEllipse(0.01 * SCALE, 0.01 * SCALE, Red); // joint origin
					glPushMatrix();
						glTranslated(0, -0.01 * SCALE, 0);
						drawEllipse(0.4 * SCALE, 0.2 * SCALE);
					glPopMatrix();
				glPopMatrix();
			glPopMatrix();
		glPopMatrix();
	glPopMatrix();
}
