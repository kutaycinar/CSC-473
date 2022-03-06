#include "Picker.h"

#ifdef WIN32
#include <windows.h>
#endif

#include "shared/opengl.h"

void pickFromXYPlane(Vector result, int x, int y)
{
	double modelView[16];
	double projection[16];
	int viewport[4];

	double x1, y1, z1, x2, y2, z2;

	glGetDoublev(GL_MODELVIEW_MATRIX, modelView);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);

	y = viewport[3] - y;
	gluUnProject(x, y, 0, modelView, projection, viewport, &x1, &y1, &z1);
	gluUnProject(x, y, 1, modelView, projection, viewport, &x2, &y2, &z2);

	double t = z1 / (z1 - z2);

	result[0] = (1 - t) * x1 + t * x2;
	result[1] = (1 - t) * y1 + t * y2;
	result[2] = 0;

	double z = (1 - t) * z1 + t * z2;
}