#include "TankPathSystem.h"

TankPathSystem::TankPathSystem(const std::string& name) : BaseSystem(name)
{
	Hermite = new HermiteSystem("hermite_tankpath");
	Hermite->disablePrint();

	// Initiliaze speed variables
	path_position = 0;
	path_position_last = 0;
	stop_simulation = false;

	// Load Tank model
	animTcl::OutputMessage("[TankPathSystem]: Loading Tank Model (takes a couple seconds)");
	loadModel();

	// Reset tank position and rotation
	zeroVector(tank_position);
	calculateRotation();
	setRotation(Quaternion(0.707, 0, 0, 0.707));
}

void TankPathSystem::reset(double time)
{
	// Reset variables
	path_position = 0;
	path_position_last = 0;
	stop_simulation = false;

	// Reset tank to first position
	VecCopy(tank_position, Hermite->getControlPoint(0).point);
	VecCopy(tangent, Hermite->getControlPoint(0).tangent);
	if (Hermite->getSize() > 0)
	{
		calculateRotation();
	}
}

void TankPathSystem::getTankPosition(double* p)
{
	VecCopy(p, tank_position);
}

void TankPathSystem::loadModel()
{
	m_model.ReadOBJ("data/tank.obj");
	glmFacetNormals(&m_model);
	glmVertexNormals(&m_model, 90);
}

void TankPathSystem::calculateMovement(double velocity)
{
	LineSegment path_current = Hermite->getLookUpTableArc(path_position);
	VecSubtract(orientation, path_current.end, tank_position);
	VecNormalize(orientation);
	VecCopy(tangent, orientation);
	VecScale(orientation, velocity);
	VecAdd(tank_position, tank_position, orientation);
}

void TankPathSystem::getState(double* p)
{
	LineSegment line = Hermite->getLookUpTableArc(path_position);

	p[0] = tank_position[0];
	p[1] = tank_position[1];
	p[2] = tank_position[2];
	p[3] = line.t;
	p[4] = tangent[0];
	p[5] = tangent[1];
	p[6] = tangent[2];
	p[7] = Hermite->getSize();
	p[9] = stop_simulation;
}

void TankPathSystem::setState(double* p)
{
	path_position = path_position_last + p[0];

	if (p[9] == true)
	{
		stop_simulation = true;
		return;
	}
	if (stop_simulation)
	{
		return;
	}

	calculateMovement(p[0]);

	if (Hermite->getSize() > 0)
	{
		calculateRotation();
	}

	path_position_last += VecLength(orientation);
}

int TankPathSystem::command(int argc, myCONST_SPEC char** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("[TankPathSystem]: Wrong number of params");
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "load") == 0)
	{
		if (argc == 2)
		{
			Hermite->loadFile(argv[1]);
			Hermite->generateLookUpTable();

			// Get first line and set tank position to its start
			LineSegment line = Hermite->getArcLength(0, true);
			VecCopy(tank_position, line.start);
			VecCopy(tangent, Hermite->getControlPoint(0).tangent);
			calculateRotation();
		}
		else
		{
			animTcl::OutputMessage("[TankPathSystem]: Usage 'system %s load <file name>'", m_name.c_str());
			return TCL_ERROR;
		}
	}

	glutPostRedisplay();
	return TCL_OK;
}

void TankPathSystem::setRotation(Quaternion rotation)
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
}

void TankPathSystem::calculateRotation()
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


void TankPathSystem::display(GLenum mode)
{
	Hermite->drawPoints();
	Hermite->drawLines();


	glEnable(GL_LIGHTING);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glTranslated(tank_position[0], tank_position[1], tank_position[2]);
	glScalef(2, 2, 2);
	glMultMatrixf(rotation_matrix);

	glmDraw(&m_model, GLM_SMOOTH | GLM_MATERIAL);

	glPopMatrix();
	glPopAttrib();
}