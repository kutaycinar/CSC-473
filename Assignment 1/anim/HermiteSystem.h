#pragma once

#include "BaseSystem.h"

#include <vector>
#include <list>
#include <math.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <map>

#define MAX_POINTS 40
#define LINE_SAMPLES 20
#define INCREMENT 0.001

struct ControlPoint
{
	Vector point;
	Vector tangent;
};

struct LineSegment
{
	Vector start;
	Vector end;
	double t, arc;
};

class HermiteSystem : public BaseSystem
{
	public:
		HermiteSystem(const std::string& name);

		void display(GLenum mode = GL_RENDER);
		void drawPoints();
		void drawLines();
		void reset(double time);
		void clear();

		int command(int argc, myCONST_SPEC char** argv);

		int getSize();
		void setTangent(int index, double sx, double sy, double sz);
		void setPoint(int index, double x, double y, double z);
		void loadFile(std::string file_name);
		void computeCatmullTangents();
		void generateLookUpTable();
		void disablePrint();
		void getCurvePosition(Vector result, ControlPoint p1, ControlPoint p2, double t);
		void addControlPoint(double x, double y, double z, double sx = 0, double sy = 0, double sz = 0);

		ControlPoint getControlPoint(int n);

		LineSegment getLookUpTable(double t);
		LineSegment getLookUpTableArc(double arc);
		LineSegment getArcLength(double t, boolean generateTable = false);

	protected:
		std::vector<ControlPoint> controlPoints;
		std::map<double, LineSegment> lookUpTable;
		boolean print = true;
};

