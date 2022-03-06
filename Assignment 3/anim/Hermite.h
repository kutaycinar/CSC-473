#pragma once
#include <shared/defs.h>	// FIXME: this should be in animTcl.h
#include "anim.h"
#include <util/VectorObj.h>
#include <assert.h>
#include <GL/glut.h>
#include <vector>
#include <algorithm>
#include <fstream>
#include <string>

using std::string;
using std::vector;
using std::min;
using std::istream;
using std::ostream;
using std::ofstream;
using std::ifstream;
using std::ios;
using std::endl;

// constants
#define DEFAULT_SAMPLE_STEP_ARC_LENGTH	0.1
#define DEFAULT_SAMPLE_STEP_DISPLAY		0.001
#define TOLERANCE_BISECTION				0.01

#define TOKEN_DELIMITER " "

// aliases
#define _CPLIST	_listOfControlPoints
#define _NCP	_numberOfControlPoints

// macros
#define ISCOMMAND(in, cmd)		strcmp(in, cmd) == 0
#define CHECK_NUM_ARGS(argc, n)	if(argc < n + 1) return TCL_ERROR;

struct ControlPoint
{
	VectorObj p;	// point
	VectorObj t;	// tangent
};

struct HermiteFileHeader
{
	string splineName;
	int numberOfControlPoints;
};

struct EntryArcLength
{
	double t;
	double arcLength;
};

typedef vector<EntryArcLength> TableArcLength;

class Hermite :
	public BaseSystem
{
public:
	Hermite(const std::string& name);
	~Hermite(void);

	VectorObj getIntermediatePoint(double t);
	VectorObj getIntermediateTangent(double t);

	void display(GLenum mode = GL_RENDER);

	// TODO: return the center of the spline's bounding box / convex hull
	void GetState(double* p) {}
	void SetState(double* p) {}

	inline void	addControlPoint(const ControlPoint& c);

	void		addControlPoint(double x, double y, double z,
		double tx, double ty, double tz);

	void		addControlPoint(
		double x, double y, double z);	// uses Catmull-Rom for tangents

	void		printTableArcLength() const;

	double		getTFromArcLengthNormalized(double arcLength);
	double		getTFromArcLength(double arcLength);
	double		getArcLengthFromT(double t);
	double		getRawArcLengthFromT(double t);
	int			size() const { return _NCP; }
	double		getLength() const;
	double		getStartLength() const;
	void		setTangentVisibility(bool status) { s_tangentVisibility = status; }
	void		setControlPointVisibility(bool status) { s_controlPointVisibility = status; }
	void		setSelectedPointVisibility(bool status) { s_selectedPointVisibility = status; }
	void		setStartingArclength(double arcLength);

	void		setPoint(int index, double x, double y,
		double z, double tx, double ty, double tz);

	void		setPoint(int index, double x, double y, double z);	// uses Catmull-Rom for tangents
	void		setTangent(int index, double tx, double ty, double tz);
	void		reset();

	void		setSelectedValue(double arcLength) { _selectedTValue = arcLength; }

	int         command(int  argc, myCONST_SPEC char** argv);

	void		saveToFile(const char* filename);
	void		loadFromFile(const char* filename);
	void		loadFromFile2D(const char* filename);

	// TODO: these functions should be parameterized with respect to arcLength

	void		getPoint(VectorObj& p, double t) { _getIntermediatePoint(p, t); }
	void		getTangent(VectorObj& tng, double t) { _getIntermediateTangent(tng, t); }
	void		getCurvature(VectorObj& crv, double t) { _getIntermediateCurvature(crv, t); }

private:
	// member functions
	inline void		_getSectionIndex(double t,

		int& section_index, double& section_t);

	void			_getIntermediatePoint(VectorObj& p, double t);

	void			_getIntermediatePoint(VectorObj& p,
		const ControlPoint& c0, const ControlPoint& c1, double t);

	void			_getIntermediateTangent(VectorObj& tng, double t);

	void			_getIntermediateTangent(VectorObj& tng,
		const ControlPoint& c0, const ControlPoint& c1, double t);

	void			_getIntermediateCurvature(VectorObj& tng, double t);

	void			_getIntermediateCurvature(VectorObj& tng,
		const ControlPoint& c0, const ControlPoint& c1, double t);

	double			_getTableArcLengthNormalizedT(int index) const { return _tableArcLength[index].t / _tableArcLength.back().t; }
	double			_getTableArcLengthNormalizedArcLength(int index) const { return _tableArcLength[index].arcLength / _tableArcLength.back().arcLength; }
	double			_getTableArcLengthRawArcLength(int index) const { return _tableArcLength[index].arcLength; }

	double			_findArcLengthLinearSearchNormalized(double arcLength);
	double			_findArcLengthBisectionNormalized(double arcLength);

	double			_findArcLengthLinearSearch(double arcLength);

	void			_saveControlPoint(const ControlPoint& cp, ostream& os);
	ControlPoint	_loadControlPoint(istream& is);
	void			_loadControlPoint2D(istream& is,
		double& x, double& y);


	void			_catmullStart();
	void			_catmullMiddle(int index);
	void			_catmullEnd();
	void			_computeTangent(int index);
	void			_recomputeAllTangents();
	void			_drawControlPoint(const ControlPoint& c);
	void			_drawControlTangent(const ControlPoint& c);
	void			_drawControlPoints();
	void			_drawControlTangents();
	void			_drawSelectedPoint();
	void			_addEntryArcLength(double arcLength);
	void			_storeApproxDistance(ControlPoint c0, ControlPoint c1);

	// commands
	int				_cmd_setPoint(int argc, myCONST_SPEC char** argv);
	int				_cmd_setTangent(int argc, myCONST_SPEC char** argv);
	int				_cmd_setTangentVisibility(int argc, myCONST_SPEC char** argv);
	int				_cmd_setControlPointVisibility(int argc, myCONST_SPEC char** argv);
	int				_cmd_setSelectedPointVisibility(int argc, myCONST_SPEC char** argv);
	int				_cmd_setSelectedValue(int argc, myCONST_SPEC char** argv);
	int				_cmd_setStartingArclength(int argc, myCONST_SPEC char** argv);
	int				_cmd_addPoint(int argc, myCONST_SPEC char** argv);
	int				_cmd_getArcLengthTable(int argc, myCONST_SPEC char** argv);
	int				_cmd_getArcLengthFromT(int argc, myCONST_SPEC char** argv);
	int				_cmd_getTFromArcLength(int argc, myCONST_SPEC char** argv);
	int				_cmd_load3D(int argc, myCONST_SPEC char** argv);
	int				_cmd_load2D(int argc, myCONST_SPEC char** argv);
	int				_cmd_export(int argc, myCONST_SPEC char** argv);
	int				_cmd_reset(int argc, myCONST_SPEC char** argv);

	// data members
	TableArcLength	_tableArcLength;
	vector< ControlPoint >	_listOfControlPoints;

	// FIXME: this member is redundant and must be deprecated
	int				_numberOfControlPoints;

	double			_selectedTValue;
	double			_startingArcLength;

	static double	s_splineSampleStepArcLength;
	static double	s_splineSampleStepDisplay;
	static bool		s_tangentVisibility;
	static bool		s_controlPointVisibility;
	static bool		s_selectedPointVisibility;

};

