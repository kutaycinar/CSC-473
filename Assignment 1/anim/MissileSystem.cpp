#include "MissileSystem.h"

MissileSystem::MissileSystem(const std::string& name) : BaseSystem(name)
{
	animTcl::OutputMessage("[MissileSystem]: Speed initially set at 100m/s");

	PathMissile = new HermiteSystem("missile_path");
	PathMissile->disablePrint();

	stop_simulation = false;

	// Load missile model
	loadModel();

	// Set missile position and orientation at an arbitrary point 
	setVector(missile_position, -5, 5, 10);
	setRotation(Quaternion(0, 0, 0, 1));
	speed = 0.01;
}


void MissileSystem::loadModel()
{
	m_model.ReadOBJ("data/missile.obj");
	glmFacetNormals(&m_model);
	glmVertexNormals(&m_model, 90);
}

void MissileSystem::setRotation(Quaternion rotation)
{
	float quaternion_matrix[4][4];
	rotation.toMatrix(quaternion_matrix);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			rotation_matrix[4 * i + j] = quaternion_matrix[i][j];
		}
	}
	glutPostRedisplay();
}


void MissileSystem::reset(double time)
{
	// Reset variables
	stop_simulation = false;
	setVector(missile_position, -5, 5, 10);
	setRotation(Quaternion(0, 0, 0, 1));
	speed = 0.01;
	PathMissile->clear();
	setVector(length, 10, 10, 10);
}


void MissileSystem::getState(double* p)
{
	// If missile reaches tank, stop simulation flag set to true
	if (VecLength(length) < 1)
	{
		stop_simulation = true;
		p[9] = true;
	}
}

void MissileSystem::setPathDisplay(double x, double y, double z, double sx, double sy, double sz)
{
	// Set missile actual path
	PathMissile->clear();
	PathMissile->addControlPoint(missile_position[0], missile_position[1], missile_position[2]);
	PathMissile->addControlPoint(x, y, z);
	PathMissile->setTangent(1, sx* VecLength(length)*0.9, sy* VecLength(length) * 0.9, sz* VecLength(length) * 0.9);
	//PathMissile->computeCatmullTangents();
}
void MissileSystem::setState(double* p)
{
	if (stop_simulation)
	{
		return;
	}

	// Move tank physics based velocity addition
	setVector(tank_position, p[0], p[1], p[2]);
	VecSubtract(length, tank_position, missile_position);
	setPathDisplay(p[0], p[1], p[2], p[4], p[5], p[6]);
	LineSegment path_current = PathMissile->getArcLength(0.001);
	Vector target_direction;
	VecSubtract(target_direction, path_current.end, missile_position);
	VecNormalize(target_direction);
	VecCopy(tangent, target_direction);
	VecScale(target_direction, speed);
	VecAdd(missile_position, missile_position, target_direction);
	calculateRotation();
}

void MissileSystem::calculateRotation()
{
	Vector u, v, w;

	// w = P’(s) tangent along the derivative
	VecCopy(w, tangent);

	// u = P’(s) x P’’(s) perpendicular to tangent and curvature
	setVector(u, 0, 0, 1);

	// v = w x u perpendicular to the other two
	VecCrossProd(v, tangent, u);

	// Normalize!!
	VecNormalize(u);
	VecNormalize(v);
	VecNormalize(w);

	// Set Rotation Matrix with values above
	rotation_matrix[0] = w[0];
	rotation_matrix[1] = w[1];
	rotation_matrix[2] = w[2];
	// 0
	rotation_matrix[4] = u[0];
	rotation_matrix[5] = u[1];
	rotation_matrix[6] = u[2];
	// 0
	rotation_matrix[8] = v[0];
	rotation_matrix[9] = v[1];
	rotation_matrix[10] = v[2];
	// 0
	rotation_matrix[15] = 1;

}

void MissileSystem::setMissleState(double x, double y, double z, double v1, double v2, double v3, double e)
{
	setVector(missile_position, x, y, z);
	setRotation(Quaternion(v1, v2, v3, e));
}

int MissileSystem::command(int argc, myCONST_SPEC char** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("[MissileSystem]: Wrong number of params");
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "state") == 0)
	{
		if (argc == 8)
		{
			setMissleState(std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3]), std::stod(argv[4]), std::stod(argv[5]), std::stod(argv[6]), std::stod(argv[7]));
		}
	}
	else if (strcmp(argv[0], "speed") == 0)
	{
		if (argc == 2)
		{
			speed = std::stod(argv[1]) / 10000;
		}
	}
	glutPostRedisplay();
}


void MissileSystem::display(GLenum mode)
{
	PathMissile->display();

	glEnable(GL_LIGHTING);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glTranslated(missile_position[0], missile_position[1], missile_position[2]);
	glScalef(2, 2, 2);
	glMultMatrixf(rotation_matrix);
	glmDraw(&m_model, GLM_SMOOTH | GLM_MATERIAL);

	glPopMatrix();
	glPopAttrib();

}
