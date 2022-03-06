#include "Hermite.h"
#include <util/vectorObj.h>

// static variables
double	Hermite::s_splineSampleStepArcLength = DEFAULT_SAMPLE_STEP_ARC_LENGTH;
double	Hermite::s_splineSampleStepDisplay = DEFAULT_SAMPLE_STEP_DISPLAY;
bool	Hermite::s_tangentVisibility = false;
bool	Hermite::s_controlPointVisibility = true;
bool	Hermite::s_selectedPointVisibility = false;

Hermite::Hermite(const std::string& name) :
	BaseSystem(name),
	_numberOfControlPoints(0),
	_selectedTValue(0.0),
	_startingArcLength(0.0)
{
}	// Hermite::Hermite

Hermite::~Hermite(void)
{
}	// Hermite::~Hermite

void Hermite::_getIntermediatePoint(VectorObj& p, double t)
{
	int base_index;
	double base_t;

	assert(_numberOfControlPoints > 0);

	if (_numberOfControlPoints == 1)
	{
		p = _listOfControlPoints[0].p;
		return;
	}

	_getSectionIndex(t, base_index, base_t);

	ControlPoint c0 = _listOfControlPoints[base_index];
	ControlPoint c1 = _listOfControlPoints[base_index + 1];

	_getIntermediatePoint(p, c0, c1, base_t);

}	// Hermite::_getIntermediatePoint

void Hermite::_getIntermediatePoint(VectorObj& p, const ControlPoint& c0, const ControlPoint& c1, double t)
{
	// compute weights
	double w0 = 2 * t * t * t - 3 * t * t + 1;
	double w1 = t * t * t - 2 * t * t + t;
	double w2 = t * t * t - t * t;
	double w3 = -2 * t * t * t + 3 * t * t;

	// compute the point as weighted sum
	p = w0 * c0.p + w1 * c0.t + w2 * c1.t + w3 * c1.p;

}	// Hermite::_getIntermediatePoint (overload)

VectorObj Hermite::getIntermediatePoint(double t)
{
	VectorObj p;
	_getIntermediatePoint(p, t);

	return p;

}	// Hermite::getIntermediatePoint

void Hermite::_getIntermediateTangent(VectorObj& tng, double t)
{
	int base_index;
	double base_t;

	assert(_numberOfControlPoints > 0);

	if (_numberOfControlPoints == 1)
	{
		tng = _listOfControlPoints[0].t;
		return;
	}

	_getSectionIndex(t, base_index, base_t);

	ControlPoint c0 = _listOfControlPoints[base_index];
	ControlPoint c1 = _listOfControlPoints[base_index + 1];

	_getIntermediateTangent(tng, c0, c1, base_t);

}	// Hermite::_getIntermediateTangent

void Hermite::_getIntermediateTangent(VectorObj& tng, const ControlPoint& c0, const ControlPoint& c1, double t)
{
	// compute weights
	double w0 = 6 * t * t - 6 * t;
	double w1 = 3 * t * t - 4 * t + 1;
	double w2 = 3 * t * t - 2 * t;
	double w3 = -6 * t * t + 6 * t;

	// compute the point as weighted sum
	tng = w0 * c0.p + w1 * c0.t + w2 * c1.t + w3 * c1.p;

	tng.normalize();

}	// Hermite::_getIntermediateTangent ( overload)

void Hermite::_getIntermediateCurvature(VectorObj& tng, double t)
{
	int base_index;
	double base_t;

	assert(_numberOfControlPoints > 0);

	if (_numberOfControlPoints == 1)
	{
		tng = _listOfControlPoints[0].t;
		return;
	}

	_getSectionIndex(t, base_index, base_t);

	ControlPoint c0 = _listOfControlPoints[base_index];
	ControlPoint c1 = _listOfControlPoints[base_index + 1];

	_getIntermediateCurvature(tng, c0, c1, base_t);

}	// Hermite::_getIntermediateCurvature

void Hermite::_getIntermediateCurvature(VectorObj& tng, const ControlPoint& c0, const ControlPoint& c1, double t)
{
	// compute weights
	double w0 = 12 * t - 6;
	double w1 = 6 * t - 4;
	double w2 = 6 * t - 2;
	double w3 = -12 * t + 6;

	// compute the point as weighted sum
	tng = w0 * c0.p + w1 * c0.t + w2 * c1.t + w3 * c1.p;

	tng.normalize();

}	// Hermite::_getIntermediateCurvature (overload)


VectorObj Hermite::getIntermediateTangent(double t)
{
	VectorObj tng;
	_getIntermediateTangent(tng, t);

	return tng;

}	// Hermite::getIntermediateTangent

void Hermite::addControlPoint(const ControlPoint& c)
{

	_listOfControlPoints.push_back(c);
	_numberOfControlPoints++;

	//	animTcl::OutputMessage("Added Control Point <%f, %f, %f, %f, %f %f>\n", c.p.x(), c.p.y(), c.p.z(), c.t.x(), c.t.y(), c.t.z());

	int last = _NCP - 1;

	if (_numberOfControlPoints == 2)	// with two points I have a line
		_storeApproxDistance(_CPLIST[0], _CPLIST[1]);
	else if (_numberOfControlPoints == 3)	// with three points I start having nonlinear behavior, so I need to recompute arclength from beginning
	{
		_tableArcLength.clear();
		_storeApproxDistance(_CPLIST[0], _CPLIST[1]);
		_storeApproxDistance(_CPLIST[1], _CPLIST[2]);
	}
	else if (_numberOfControlPoints > 3)
	{
		_storeApproxDistance(_CPLIST[_NCP - 2], _CPLIST[_NCP - 1]);
	}

}	// Hermite::addControlPoint

void Hermite::addControlPoint(double x, double y, double z, double tx, double ty, double tz)
{
	ControlPoint c;

	c.p.assign(x, y, z);
	c.t.assign(tx, ty, tz);

	Hermite::addControlPoint(c);

}	// Hermite::addControlPoint

void Hermite::addControlPoint(double x, double y, double z)
{
	// create a new control point with the given coordinates and a tangent determined using Catmull-Rom algorithm

	ControlPoint c;
	double tx, ty, tz;

	c.p.assign(x, y, z);

	if (_numberOfControlPoints == 0)	// first point
		c.t.assign(0, 0, 0);
	else if (_numberOfControlPoints == 1)	// second point
	{
		tx = x - _listOfControlPoints[0].p.x();
		ty = y - _listOfControlPoints[0].p.y();
		tz = z - _listOfControlPoints[0].p.z();

		_listOfControlPoints[0].t.assign(tx, ty, tz);
		c.t.assign(tx, ty, tz);
	}
	else	// third point
	{
		c.t.assign(0.5, 0.5, 0.0);
	}

	Hermite::addControlPoint(c);

	if (_numberOfControlPoints >= 3)
	{
		for (int i = _NCP - 1; i >= _NCP - 3; i--)
		{
			_computeTangent(i);
		}

	}

}	// Hermite::addControlPoint (overload)

void Hermite::_getSectionIndex(double t, int& section_index, double& section_t)
{
	assert(t >= 0.0 && t <= 1.0);
	assert(_numberOfControlPoints > 1);

	section_index = (int)floor(t * (_numberOfControlPoints - 1));
	section_t = t * (double)(_numberOfControlPoints - 1) - (double)section_index;

}	// Hermite::_getSectionIndex

void Hermite::display(GLenum mode)
{
	// if there are no control points there is nothing to do
	if (_numberOfControlPoints == 0)
		return;

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glColor3f(1.0, 1.0, 1.0);
	glDisable(GL_LIGHTING);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	VectorObj p;

	if (_numberOfControlPoints == 1)	// if there is only one cotrol point just draw the point
	{
		p = _listOfControlPoints[0].p;

		glBegin(GL_POINT);
		glVertex3f((GLfloat)p.x(), (GLfloat)p.y(), (GLfloat)p.z());
		glEnd();
	}
	else	// there is a spline to draw
	{
		glBegin(GL_LINE_STRIP);

		p = _CPLIST[0].p;
		glVertex3f((GLfloat)p.x(), (GLfloat)p.y(), (GLfloat)p.z());

		for (double t = 0.0; t <= 1.0; t += s_splineSampleStepDisplay)
		{
			_getIntermediatePoint(p, t);
			glVertex3f((GLfloat)p.x(), (GLfloat)p.y(), (GLfloat)p.z());
		}

		p = _CPLIST[_numberOfControlPoints - 1].p;
		glVertex3f((GLfloat)p.x(), (GLfloat)p.y(), (GLfloat)p.z());

		glEnd();
	}

	if (s_controlPointVisibility)
		_drawControlPoints();

	if (s_tangentVisibility)
	{
		glColor3f(1.0, 0.0, 0.0);
		_drawControlTangents();
	}

	if (s_selectedPointVisibility)
		_drawSelectedPoint();

	glPopMatrix();
	glPopAttrib();

}	// Hermite::display

void Hermite::_drawControlPoint(const ControlPoint& c)
{
	int size = 0.1;

	// TODO: use display list instead
	glPushMatrix();
	glTranslatef(c.p.x(), c.p.y(), c.p.z());
	glutSolidSphere(0.1, 6, 6);
	glPopMatrix();

}	// Hermite::_drawControlPoint

void Hermite::_drawControlTangent(const ControlPoint& c)
{
	int size = 0.1;

	VectorObj end = c.p + c.t;

	// TODO: use display list instead
	glPushMatrix();
	glBegin(GL_LINES);
	glVertex3d(c.p.x(), c.p.y(), c.p.z());
	glVertex3d(end.x(), end.y(), end.z());
	glEnd();

	glTranslatef(end.x(), end.y(), end.z());
	glutSolidSphere(0.1, 6, 6);
	glPopMatrix();

}	// Hermite::_drawControlTangent

void Hermite::_drawControlPoints()
{
	ControlPoint c;

	for (int i = 0; i < _numberOfControlPoints; i++)
	{
		c = _listOfControlPoints[i];
		_drawControlPoint(c);
	}

}	// Hermite::_drawControlPoints

void Hermite::_drawControlTangents()
{
	ControlPoint c;

	for (int i = 0; i < _numberOfControlPoints; i++)
	{
		c = _listOfControlPoints[i];
		_drawControlTangent(c);
	}

}	// Hermite::_drawControlTangents

void Hermite::_drawSelectedPoint()
{
	ControlPoint c;

	double t = getTFromArcLengthNormalized(_selectedTValue);

	_getIntermediatePoint(c.p, t);
	_getIntermediateTangent(c.t, t);

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glColor3f(0.0, 1.0, 0.0);

	_drawControlPoint(c);
	_drawControlTangent(c);

}	// Hermite::_drawSelectedPoint

void Hermite::_catmullStart()
{
	_CPLIST[0].t = 2.0 * (_CPLIST[1].p - _CPLIST[0].p) -
		(_CPLIST[2].p - _CPLIST[0].p) / 2.0;

}	// Hermite::_catmullStart

void Hermite::_catmullMiddle(int index)
{
	_CPLIST[index].t = (_CPLIST[index + 1].p - _CPLIST[index - 1].p) / 2.0;

}	// Hermite::_catmullMiddle

void Hermite::_catmullEnd()
{
	int last = _numberOfControlPoints - 1;
	_CPLIST[last].t = 2.0 * (_CPLIST[last].p - _CPLIST[last - 1].p) -
		(_CPLIST[last].p - _CPLIST[last - 2].p) / 2.0;

}	// Hermite::_catmullEnd

void Hermite::_computeTangent(int index)
{
	if (index == 0)
		_catmullStart();
	else if (index >= _numberOfControlPoints - 1)
	{
		_catmullEnd();
	}
	else
		_catmullMiddle(index);

}	// Hermite::_computeTangent

void Hermite::_storeApproxDistance(ControlPoint c0, ControlPoint c1)
{
	VectorObj p0, p1;
	VectorObj pStart, pEnd;

	p0 = c0.p;
	p1 = c1.p;

	pStart = p0;

	// sample the spline segment between control points c0 and c1. Conpute the length using piecewise linear approximation
	for (double t = s_splineSampleStepArcLength; t <= 1.0; t += s_splineSampleStepArcLength)
	{
		_getIntermediatePoint(pEnd, c0, c1, t);
		_addEntryArcLength((pEnd - pStart).length()); // compute Euclidean distance between points
		pStart = pEnd;
	}

}	// Hermite::_storeApproxDistance

void Hermite::_addEntryArcLength(double arcLength)
{
	EntryArcLength e;

	if (_tableArcLength.size() == 0)
	{
		e.t = 0.0;
		e.arcLength = _startingArcLength;
		_tableArcLength.push_back(e);
	}

	e.t = _tableArcLength.size() * s_splineSampleStepArcLength;
	e.arcLength = _tableArcLength.back().arcLength + arcLength;

	_tableArcLength.push_back(e);

}	// Hermite::_addEntryArcLength

double Hermite::getTFromArcLengthNormalized(double arcLength)
{
	return  _findArcLengthLinearSearchNormalized(arcLength);
	//	return	_findArcLengthBisectionNormalized(arcLength);

}	// Hermite::getTFromArcLengthNormalized

double Hermite::getTFromArcLength(double arcLength)
{
	return  _findArcLengthLinearSearch(arcLength);

}	// Hermite::getTFromArcLength

double Hermite::getArcLengthFromT(double t)
{
	assert(_tableArcLength.size() >= 2);

	double normalizedSampling = _getTableArcLengthNormalizedT(1) - _getTableArcLengthNormalizedT(0);
	int index = (int)(t / normalizedSampling);
	double normalizedArcLength = _getTableArcLengthNormalizedArcLength(index);

	normalizedArcLength = (t - _getTableArcLengthNormalizedT(index)) / (_getTableArcLengthNormalizedT(index + 1) - _getTableArcLengthNormalizedT(index));

	// TODO:: linearly interpolate the arc length between closest pair to get a better estimate for t

	return normalizedArcLength;

}	// Hermite::getArcLengthFromT

double Hermite::getRawArcLengthFromT(double t)
{
	assert(_tableArcLength.size() >= 2);

	// since the values the values of t are normalized the distance between the entries depends on the length of the table
	// and must be computed every time
	double normalizedSampling = _getTableArcLengthNormalizedT(1) - _getTableArcLengthNormalizedT(0);

	int index = (int)(t / normalizedSampling);
	double arcLength = this->_getTableArcLengthRawArcLength(index);

	// interpolate the value
	arcLength = arcLength + ((t - _getTableArcLengthNormalizedT(index)) / normalizedSampling) * (_getTableArcLengthRawArcLength(index + 1) - _getTableArcLengthRawArcLength(index));

	// TODO:: linearly interpolate the arc length between closest pair to get a better estimate for t

	return arcLength;

}	// Hermite::getRawArcLengthFromT

double Hermite::_findArcLengthLinearSearchNormalized(double arcLength)
{
	double previousError = -arcLength;
	double nextError;
	double output;
	double parm;
	int index;

	if (arcLength == 0.0)
		return 0.0;
	else if (arcLength == 1.0)
		return 1.0;

	for (index = 1; index < _tableArcLength.size(); index++)
	{
		nextError = this->_getTableArcLengthNormalizedArcLength(index) - arcLength;

		if (previousError * nextError < 0.0)
			break;

		previousError = nextError;
	}

	parm = this->_getTableArcLengthNormalizedArcLength(index - 1);
	parm = arcLength - parm;

	parm = parm / (_getTableArcLengthNormalizedArcLength(index) - _getTableArcLengthNormalizedArcLength(index - 1));


	double low = _getTableArcLengthNormalizedT(index - 1);
	double high = _getTableArcLengthNormalizedT(index);

	output = parm * high + (1.0 - parm) * low;

	return output;

}	// Hermite::_findArcLengthLinearSearchNormalized

double Hermite::_findArcLengthLinearSearch(double arcLength)
{
	double previousError = -arcLength;
	double nextError;
	double output;
	double parm;
	int index;

	if (arcLength <= _startingArcLength)
		return 0.0;

	//		else if(arcLength == 1.0)
	//			return 1.0;

	if (arcLength > _tableArcLength.back().arcLength)
		return 1.0;

	for (index = 1; index < _tableArcLength.size(); index++)
	{
		nextError = this->_getTableArcLengthRawArcLength(index) - arcLength;

		if (previousError * nextError < 0.0)
			break;

		previousError = nextError;
	}

	parm = this->_getTableArcLengthRawArcLength(index - 1);
	parm = arcLength - parm;

	parm = parm / (_getTableArcLengthRawArcLength(index) - _getTableArcLengthRawArcLength(index - 1));


	double low = this->_getTableArcLengthNormalizedT(index - 1);
	double high = _getTableArcLengthNormalizedT(index);

	output = parm * high + (1.0 - parm) * low;

	animTcl::OutputMessage("t value %f", output);

	return output;

}	// Hermite::_findArcLengthLinearSearch

double Hermite::_findArcLengthBisectionNormalized(double arcLength)
{

	// FIXME: this code is not peforming bisection on the normalized arclentgth
	// as expected by the name of this method. Modify the code to make it 
	// consistent

	// PRECONDITION: t is in [0,1]

	double arcLengthHigh = _tableArcLength.back().arcLength;


	if (arcLength <= 0.0)
		return 0.0;

	if (arcLength >= arcLengthHigh)
		return 1.0;

	double low = 0.0;
	double high = _tableArcLength.back().t;
	double middle = (low + high) / 2.0;

	//	should be zero
	double f_low = _tableArcLength.front().arcLength - arcLength;

	// should be one (if arcLength is normalized)
	double f_high = _tableArcLength.back().arcLength - arcLength;

	double f_middle = getArcLengthFromT(middle) - arcLength;
	double f_middleOld;
	double error = f_middle;

	while (abs((double)error) > TOLERANCE_BISECTION)
	{
		if (f_low * f_middle < 0.0)
		{
			f_high = f_middle;
			high = middle;
		}
		else
		{
			f_low = f_middle;
			low = middle;
		}

		f_middleOld = f_middle;
		middle = (low + high) / 2.0;
		f_middle = getArcLengthFromT(middle) - arcLength;
		error = abs(f_middleOld - f_middle);

	}

	return middle;

}	// Hermite::_findArcLengthBisectionNormalized

void Hermite::printTableArcLength() const
{
	animTcl::OutputMessage("t\t\tarcLength\n");

	double normT, normArcLength;

	for (int i = 0; i < _tableArcLength.size(); i++)
	{

		normT = _getTableArcLengthNormalizedT(i);
		normArcLength = this->_getTableArcLengthRawArcLength(i);

		animTcl::OutputMessage("%f\t%f\n", normT, normArcLength);
	}

}	// Hermite::printTableArcLength

void Hermite::setPoint(int index, double x, double y, double z, double tx, double ty, double tz)
{
	if (index > _NCP)
		return;

	_CPLIST[index].p.assign(x, y, z);
	_CPLIST[index].t.assign(tx, ty, tz);

}	// Hermite::setPoint

void Hermite::setPoint(int index, double x, double y, double z)
{
	if (index > _NCP)
		return;

	_CPLIST[index].p.assign(x, y, z);

	// FIXME: do not recompute all tangents for better performance	

	_recomputeAllTangents();

}	// Hermite::setPoint (overload)

void Hermite::setTangent(int index, double tx, double ty, double tz)
{
	if (index > _NCP)
		return;

	_CPLIST[index].t.assign(tx, ty, tz);

}	// Hermite::setTangent

void Hermite::_recomputeAllTangents()
{
	for (int i = 0; i < _NCP; i++)
	{
		this->_computeTangent(i);
	}

}	// Hermite::_recomputeAllTangents

void Hermite::reset()
{
	_tableArcLength.clear();

	_listOfControlPoints.clear();

	_numberOfControlPoints = 0;

	_selectedTValue = 0.0;

}	// Hermite::reset

/*
	Available Commands

		COMMAND					(SHORTCUT)	ARGS					DESCRIPTION
	o	tangentVisibility		(tanv)		[on/off]				set the visibility of tangents
	o	controlPointVisibility	(cpv)		[on/off]				set the visibility of control points
	o	selectedPointVisibility	(spv)		[on/off]				set the visibility of the selected point
	o	getArcLength			(getal)								print the lookup table for the arc length
	o	getArcLengthFromT		(getalt)	[t]						print the arc length corresponding to the given parameter
	o	getTFromArcLength		(gettal)	[arcLength]				print the parameter t corresponding to the given arc length
	o	set tangent							[index tx ty tz]		set the tangent of the control point referenced by index
	o	set point							[index x y z tx ty tz]	set the the control point referenced by index
	o	set selectedValue		(set sv)	[arcLength]				set the parameter of the selected point
	o	add point							[x y z tx ty tz]		add a new control point
	o	reset					(clear)								clear the hermite curve
	o	export					(save)		[filename]				saves the spline to file
	o	load3D								[filename]				load a 3D spline from file
	o	load2D								[filename]				load a 2D spline from file
	o	startArcLength			(sal)		[arcLength				sets the initial arcLength
*/

int Hermite::command(int argc, myCONST_SPEC char** argv)
{
	int returnCode = -1;

	if (argc < 1) return returnCode;
	if (ISCOMMAND(argv[0], "tangentVisibility") || ISCOMMAND(argv[0], "tanv"))
	{
		returnCode = _cmd_setTangentVisibility(argc, argv);
	}

	if (ISCOMMAND(argv[0], "controlPointVisibility") || ISCOMMAND(argv[0], "cpv"))
	{
		returnCode = _cmd_setControlPointVisibility(argc, argv);
	}

	if (ISCOMMAND(argv[0], "selectedPointVisibility") || ISCOMMAND(argv[0], "spv"))
	{
		returnCode = _cmd_setSelectedPointVisibility(argc, argv);
	}

	if (ISCOMMAND(argv[0], "getArcLength") || ISCOMMAND(argv[0], "getal"))
	{
		returnCode = _cmd_getArcLengthTable(argc, argv);
	}

	if (ISCOMMAND(argv[0], "getArcLengthFromT") || ISCOMMAND(argv[0], "getalt"))
	{
		returnCode = _cmd_getArcLengthFromT(argc, argv);
	}

	if (ISCOMMAND(argv[0], "getTFromArcLength") || ISCOMMAND(argv[0], "gettal"))
	{
		returnCode = _cmd_getTFromArcLength(argc, argv);
	}

	if (ISCOMMAND(argv[0], "reset") || ISCOMMAND(argv[0], "clear"))
		returnCode = _cmd_reset(argc, argv);

	if (ISCOMMAND(argv[0], "export") || ISCOMMAND(argv[0], "save"))
		returnCode = _cmd_export(argc, argv);

	if (ISCOMMAND(argv[0], "load") || ISCOMMAND(argv[0], "load3D"))
		returnCode = _cmd_load3D(argc, argv);

	if (ISCOMMAND(argv[0], "load2D"))
		returnCode = _cmd_load2D(argc, argv);

	if (ISCOMMAND(argv[0], "startArcLength") || ISCOMMAND(argv[0], "sal"))
		returnCode = _cmd_setStartingArclength(argc, argv);

	if (ISCOMMAND(argv[0], "set"))
	{
		CHECK_NUM_ARGS(argc, 1);

		if (ISCOMMAND(argv[1], "tangent"))
			returnCode = _cmd_setTangent(argc - 1, &argv[1]);

		if (ISCOMMAND(argv[1], "point"))
			returnCode = _cmd_setPoint(argc - 1, &argv[1]);

		if (ISCOMMAND(argv[1], "selectedPoint") || ISCOMMAND(argv[1], "selectedValue") || ISCOMMAND(argv[1], "sv"))
			returnCode = _cmd_setSelectedValue(argc - 1, &argv[1]);
	}

	if (ISCOMMAND(argv[0], "add"))
	{
		CHECK_NUM_ARGS(argc, 1);

		if (ISCOMMAND(argv[1], "point"))
			returnCode = _cmd_addPoint(argc - 1, &argv[1]);

	}

	if (returnCode == TCL_ERROR)
	{
		animTcl::OutputMessage("Hermite:: unknown command %s\n", argv[0]);
		return TCL_ERROR;
	}
	else if (returnCode == TCL_OK)
		return TCL_OK;

	// if the command was not recognized we reach this point
	animTcl::OutputMessage("Hermite:: unknown command %s\n", argv[0]);
	return TCL_ERROR;

}	// Hermite::command


int	Hermite::_cmd_setPoint(int argc, myCONST_SPEC char** argv)
{
	CHECK_NUM_ARGS(argc, 4);

	if (argc == 5)
		this->setPoint(atoi(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]));
	else if (argc >= 8)
	{
		this->setPoint(atoi(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]),
			atof(argv[5]), atof(argv[6]), atof(argv[7]));
	}

	return TCL_OK;

}	// Hermite::_cmd_setPoint

int	Hermite::_cmd_setTangent(int argc, myCONST_SPEC char** argv)
{
	CHECK_NUM_ARGS(argc, 4);

	setTangent(atoi(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]));

	return TCL_OK;

}	// Hermite::_cmd_setTangent

int	Hermite::_cmd_setSelectedValue(int argc, myCONST_SPEC char** argv)
{
	CHECK_NUM_ARGS(argc, 1);

	this->setSelectedValue(atof(argv[1]));
	return TCL_OK;

}	// Hermite::_cmd_setSelectedValue

int	Hermite::_cmd_setTangentVisibility(int argc, myCONST_SPEC char** argv)
{
	CHECK_NUM_ARGS(argc, 1);

	if (ISCOMMAND(argv[1], "on"))
		s_tangentVisibility = true;
	else if (ISCOMMAND(argv[1], "off"))
		s_tangentVisibility = false;
	else
		return TCL_ERROR;

	return TCL_OK;

}	// Hermite::_cmd_setTangentVisibility

int	Hermite::_cmd_setControlPointVisibility(int argc, myCONST_SPEC char** argv)
{
	CHECK_NUM_ARGS(argc, 1);

	if (ISCOMMAND(argv[1], "on"))
		s_controlPointVisibility = true;
	else if (ISCOMMAND(argv[1], "off"))
		s_controlPointVisibility = false;
	else
		return TCL_ERROR;

	return TCL_OK;

}	// Hermite::_cmd_setControlPointVisibility

int	Hermite::_cmd_setSelectedPointVisibility(int argc, myCONST_SPEC char** argv)
{
	CHECK_NUM_ARGS(argc, 1);

	if (ISCOMMAND(argv[1], "on"))
		s_selectedPointVisibility = true;
	else if (ISCOMMAND(argv[1], "off"))
		s_selectedPointVisibility = false;
	else
		return TCL_ERROR;

	return TCL_OK;

}	// Hermite::_cmd_setSelectedPointVisibility


int	Hermite::_cmd_addPoint(int argc, myCONST_SPEC char** argv)
{
	CHECK_NUM_ARGS(argc, 6);

	addControlPoint(atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]), atof(argv[5]), atof(argv[6]));

	return TCL_OK;

}	// Hermite::_cmd_addPoint

int	Hermite::_cmd_getArcLengthTable(int argc, myCONST_SPEC char** argv)
{
	printTableArcLength();
	return TCL_OK;

}	// Hermite::_cmd_getArcLengthTable

int	Hermite::_cmd_getArcLengthFromT(int argc, myCONST_SPEC char** argv)
{
	CHECK_NUM_ARGS(argc, 1);
	animTcl::OutputMessage("%f", getRawArcLengthFromT(atof(argv[1])));
	return TCL_OK;

}	// Hermite::_cmd_getArcLengthFromT

int Hermite::_cmd_getTFromArcLength(int argc, myCONST_SPEC char** argv)
{
	CHECK_NUM_ARGS(argc, 1);
	animTcl::OutputMessage("%f", getTFromArcLength(atof(argv[1])));
	return TCL_OK;

}	// Hermite::_cmd_getTFromArcLength

int	Hermite::_cmd_load3D(int argc, myCONST_SPEC char** argv)
{
	CHECK_NUM_ARGS(argc, 1);

	this->loadFromFile(argv[1]);

	return TCL_OK;

}	// Hermite::_cmd_load3D

int	Hermite::_cmd_load2D(int argc, myCONST_SPEC char** argv)
{
	CHECK_NUM_ARGS(argc, 1);

	this->loadFromFile2D(argv[1]);

	return TCL_OK;

}	// Hermite::_cmd_load2D

int	Hermite::_cmd_export(int argc, myCONST_SPEC char** argv)
{
	CHECK_NUM_ARGS(argc, 1);

	this->saveToFile(argv[1]);

	return TCL_OK;

}	// Hermite::_cmd_export

int	Hermite::_cmd_setStartingArclength(int argc, myCONST_SPEC char** argv)
{
	CHECK_NUM_ARGS(argc, 1);

	this->setStartingArclength(atof(argv[1]));

	return TCL_OK;

}	// Hermite::_cmd_setStartingArclength


int Hermite::_cmd_reset(int argc, myCONST_SPEC char** argv)
{
	reset();

	return TCL_OK;

}	// Hermite::_cmd_reset

void Hermite::saveToFile(const char* filename)
{
	ofstream outFile;
	ControlPoint cp;

	outFile.open(filename, ios::out);

	if (!outFile.is_open())
	{
		animTcl::OutputMessage("Cannot open %s for writing", filename);
		return;
	}

	// print header
	outFile << this->retrieveName() << " " << _NCP << endl;


	for (int i = 0; i < _NCP; i++)
	{
		cp = _CPLIST[i];

		// print point
		_saveControlPoint(cp, outFile);
	}

}	// Hermite::saveToFile

void Hermite::loadFromFile(const char* filename)
{
	// PRECONDITION: the input file must be formatted correclty. no error checking is performed on the file format

	ifstream inFile;
	ControlPoint cp;
	HermiteFileHeader header;
	string tempToken;

	inFile.open(filename, ios::in);

	if (!inFile.is_open())
	{
		animTcl::OutputMessage("Cannot open %s for reading", filename);
		return;
	}

	this->reset();

	inFile >> header.splineName;
	inFile >> header.numberOfControlPoints;

	this->reset();

	for (int i = 0; i < header.numberOfControlPoints; i++)
	{
		cp = _loadControlPoint(inFile);
		this->addControlPoint(cp);
	}

}	// Hermite::loadFromFile

void Hermite::loadFromFile2D(const char* filename)
{
	// PRECONDITION: the input file must be formatted correclty. no error checking is performed on the file format

	ifstream inFile;
	ControlPoint cp;
	HermiteFileHeader header;
	string tempToken;

	inFile.open(filename, ios::in);

	if (!inFile.is_open())
	{
		animTcl::OutputMessage("Cannot open %s for reading", filename);
		return;
	}

	this->reset();

	inFile >> header.splineName;
	inFile >> header.numberOfControlPoints;

	this->reset();

	double x, y;

	for (int i = 0; i < header.numberOfControlPoints; i++)
	{
		_loadControlPoint2D(inFile, x, y);
		this->addControlPoint(x, y, 0.0);
	}

}	// Hermite::loadFromFile2D


void Hermite::_saveControlPoint(const ControlPoint& cp, ostream& os)
{
	os << cp.p.x() << TOKEN_DELIMITER
		<< cp.p.y() << TOKEN_DELIMITER
		<< cp.p.z() << TOKEN_DELIMITER
		<< cp.t.x() << TOKEN_DELIMITER
		<< cp.t.y() << TOKEN_DELIMITER
		<< cp.t.z() << endl;

}	// Hermite::_saveControlPoint

ControlPoint Hermite::_loadControlPoint(istream& is)
{
	double x, y, z;
	double tx, ty, tz;
	ControlPoint cp;

	is >> x;
	is >> y;
	is >> z;
	is >> tx;
	is >> ty;
	is >> tz;

	cp.p.assign(x, y, z);
	cp.t.assign(tx, ty, tz);

	return cp;

}	// Hermite::_loadControlPoint

void Hermite::_loadControlPoint2D(istream& is, double& x, double& y)
{

	is >> x;
	is >> y;

}	// Hermite::_loadControlPoint2D


double Hermite::getLength() const
{
	if (_tableArcLength.size() == 0)
		return 0.0;

	return _tableArcLength.back().arcLength - _tableArcLength.front().arcLength;

}	// Hermite::getLength

void Hermite::setStartingArclength(double arcLength)
{
	if (_tableArcLength.size() != 0)
	{
		for (int i = 0; i < _tableArcLength.size(); i++)
		{
			_tableArcLength[i].arcLength += arcLength;
		}
	}

	_startingArcLength = arcLength;

}	// Hermite::setStartingArclength



double Hermite::getStartLength() const
{
	return _startingArcLength;

}	// Hermite::getStartLength