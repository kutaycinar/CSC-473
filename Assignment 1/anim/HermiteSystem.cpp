#include "HermiteSystem.h"

HermiteSystem::HermiteSystem(const std::string& name) :	BaseSystem(name)
{
	if(strcmp(m_name.c_str(), "hermite") == 0)
		animTcl::OutputMessage("[HermiteSystem]: Uses %d maximum points and %d line samples", MAX_POINTS, LINE_SAMPLES);
}


void HermiteSystem::reset(double time)
{
	if (print)
		animTcl::OutputMessage("[HermiteSystem]: reset");
	controlPoints.clear();
}

void HermiteSystem::setTangent(int index, double sx, double sy, double sz)
{
	controlPoints.at(index).tangent[0] = sx;
	controlPoints.at(index).tangent[1] = sy;
	controlPoints.at(index).tangent[2] = sz;
}

void HermiteSystem::setPoint(int index, double x, double y, double z)
{
	controlPoints.at(index).point[0] = x;
	controlPoints.at(index).point[1] = y;
	controlPoints.at(index).point[2] = z;
}

int HermiteSystem::command(int argc, myCONST_SPEC char** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("[HermiteSystem]: Wrong number of params");
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "cr") == 0)
	{
		computeCatmullTangents();
	}
	else if (strcmp(argv[0], "set") == 0)
	{
		if (strcmp(argv[1], "tangent") == 0)
		{
			if (argc == 6)
			{
				setTangent(std::stoi(argv[2]), std::stod(argv[3]), std::stod(argv[4]), std::stod(argv[5]));
			}
			else
			{
				animTcl::OutputMessage("[HermiteSystem]: Usage 'system %s set tangent <index> <sx sy sz>'", m_name.c_str());
				return TCL_ERROR;
			}
		}
		else if (strcmp(argv[1], "point") == 0)
		{
			if (argc == 6)
			{
				setPoint(std::stoi(argv[2]), std::stod(argv[3]), std::stod(argv[4]), std::stod(argv[5]));
			}
			else
			{
				animTcl::OutputMessage("[HermiteSystem]: Usage 'system %s set point <index> <x y z>'", m_name.c_str());
				return TCL_ERROR;
			}
		}
	}
	else if (strcmp(argv[0], "add") == 0)
	{
		if (strcmp(argv[1], "point") == 0)
		{
			if (argc == 8)
			{
				addControlPoint(std::stoi(argv[2]), std::stod(argv[3]), std::stod(argv[4]), std::stod(argv[5]), std::stod(argv[6]), std::stod(argv[7]));
			}
			else
			{
				animTcl::OutputMessage("[HermiteSystem]: Usage 'system %s add point <x y z sx sy sz>'", m_name.c_str());
				return TCL_ERROR;
			}
		}
	}
	else if (strcmp(argv[0], "getArcLength") == 0)
	{
		if (argc == 2)
		{
			getArcLength(std::stod(argv[1]));
		}
		else
		{
			animTcl::OutputMessage("[HermiteSystem]: Usage 'system %s getArcLength <t>'", m_name.c_str());
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "generate") == 0)
	{
		generateLookUpTable();
		for (const auto& x : lookUpTable) {
			animTcl::OutputMessage("[HermiteSystem]: index=%.3f | t=%.3f start(%.3f,%.3f,%.3f) end=(%.3f,%.3f,%.3f) arc=%.3f",
				x.first, x.second.t, x.second.start[0], x.second.start[1], x.second.start[2],
				x.second.end[0], x.second.end[1], x.second.end[2], x.second.arc			
			);
		}

	}
	else if (strcmp(argv[0], "getLUT") == 0)
	{
		if (argc == 2)
		{
			LineSegment line = getLookUpTable(std::stod(argv[1]));
			animTcl::OutputMessage("[HermiteSystem]: input=%.3f | t=%.3f start(%.3f,%.3f,%.3f) end=(%.3f,%.3f,%.3f) arc=%.3f",
				std::stod(argv[1]), line.t, line.start[0], line.start[1], line.start[2],
				line.end[0], line.end[1], line.end[2], line.arc
			);

		}
		else
		{
			animTcl::OutputMessage("[HermiteSystem]: Usage 'system %s getArcLength <t>'", m_name.c_str());
			return TCL_ERROR;
		}

	}
	else if (strcmp(argv[0], "getLUTarc") == 0)
	{
		if (argc == 2)
		{
			LineSegment line = getLookUpTableArc(std::stod(argv[1]));
			animTcl::OutputMessage("[HermiteSystem]: input=%.3f | t=%.3f start(%.3f,%.3f,%.3f) end=(%.3f,%.3f,%.3f) arc=%.3f",
				std::stod(argv[1]), line.t, line.start[0], line.start[1], line.start[2],
				line.end[0], line.end[1], line.end[2], line.arc
			);

		}
		else
		{
			animTcl::OutputMessage("[HermiteSystem]: Usage 'system %s getArcLength <t>'", m_name.c_str());
			return TCL_ERROR;
		}

	}
	else if (strcmp(argv[0], "load") == 0)
	{
		if (argc == 2)
		{
			loadFile(argv[1]);
		}
		else
		{
			animTcl::OutputMessage("[HermiteSystem]: Usage 'system %s load <file name>'", m_name.c_str());
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "export") == 0)
	{
		if (argc == 2)
		{
			std::fstream export_file;

			export_file.open(argv[1], std::ios::out);
			export_file << std::setprecision(12);

			if (export_file.is_open())
			{
				// First line with system name and number of control points
				export_file << m_name.c_str() << " " << controlPoints.size() << "\n";

				// Export control points positions and tangents
				for (ControlPoint cp : controlPoints)
				{
					export_file << cp.point[0] << " " << cp.point[1] << " " << cp.point[2] << " ";
					export_file << cp.tangent[0] << " " << cp.tangent[1] << " " << cp.tangent[2] << "\n";
				}

				export_file.close();
			}
			else
			{
				animTcl::OutputMessage("[HermiteSystem]: unable to open provided filename'");
				return TCL_ERROR;
			}
		}
		else
		{
			animTcl::OutputMessage("[HermiteSystem]: Usage 'system %s export <file name>'", m_name.c_str());
			return TCL_ERROR;

		}
	}

	glutPostRedisplay();
	return TCL_OK;

}

void HermiteSystem::disablePrint()
{
	print = false;
}

void HermiteSystem:: loadFile(std::string file_name)
{
	std::fstream input_file;
	std::string line;

	input_file.open(file_name, std::ios::in);
	input_file << std::setprecision(12);

	controlPoints.clear(); // clear existing spline

	std::string x, y, z, sx, sy, sz;

	if (input_file.is_open())
	{
		getline(input_file, line); // skip first line

		while (getline(input_file, line))
		{
			std::istringstream word(line);

			// Read positions and tangets
			word >> x, word >> y, word >> z;
			word >> sx, word >> sy, word >> sz;

			addControlPoint(std::stod(x), std::stod(y), std::stod(z), std::stod(sx), std::stod(sy), std::stod(sz));
		}
	}
	else
	{
		animTcl::OutputMessage("[HermiteSystem]: unable to open provided filename'");
	}
}

void HermiteSystem::addControlPoint(double x, double y, double z, double sx, double sy, double sz)
{
	ControlPoint addMe;

	setVector(addMe.point, x, y, z);
	setVector(addMe.tangent, sx, sy, sz);

	if (controlPoints.size() < MAX_POINTS)
	{
		controlPoints.push_back(addMe);

		if (print)
			animTcl::OutputMessage("(%d/%d) Point Added", controlPoints.size(), MAX_POINTS);
	}
	else {
		if (print)
			animTcl::OutputMessage("(%d/%d) Maximum number of points reached", MAX_POINTS, MAX_POINTS);
	}

}

void HermiteSystem::clear()
{
	controlPoints.clear();
}

void HermiteSystem::getCurvePosition(Vector result, ControlPoint p1, ControlPoint p2, double t)
{
	double mh[16] = {
		 2, -3,  0,  1,
		-2,  3,  0,  0,
		 1, -2,  1,  0,
		 1, -1,  0,  0
	};

	double gh[12] = {
		p1.point[0], p2.point[0], p1.tangent[0], p2.tangent[0],
		p1.point[1], p2.point[1], p1.tangent[1], p2.tangent[1],
		p1.point[2], p2.point[2], p1.tangent[2], p2.tangent[2],
	};

	glm::mat4 Mh = glm::make_mat4(mh);
	glm::mat3x4 Gh = glm::make_mat3x4(gh);
	glm::vec4 T = glm::vec4(pow(t, 3), pow(t, 2), t, 1);
	
	glm::vec3 calc = T * Mh * Gh;

	setVector(result, calc[0], calc[1], calc[2]);
}

void HermiteSystem::computeCatmullTangents()
{
	int size = controlPoints.size();

	if (size > 2)
	{
		// Calculate second-order accurate boundary conditions
		for (int i = 0; i < size; i++)
		{
			// For S_0
			if (i == 0)
			{
				// 2(y1 - y0) 
				Vector calc_1;
				VecSubtract(calc_1, controlPoints.at(1).point, controlPoints.at(0).point);
				VecScale(calc_1, 2);

				// (y2-y0)/2
				Vector calc_2;
				VecSubtract(calc_2, controlPoints.at(2).point, controlPoints.at(0).point);
				VecScale(calc_2, 0.5);

				// 2(y1 - y0) - (y2-y0)/2
				Vector result;
				VecSubtract(result, calc_1, calc_2);
				VecCopy(controlPoints.at(0).tangent, result);
				continue;
			}
			else if (i == size - 1)
			{
				// 2(y1 - y0) 
				Vector calc_1;
				VecSubtract(calc_1, controlPoints.at(i-1).point, controlPoints.at(i).point);
				VecScale(calc_1, -2);

				// (y2-y0)/2
				Vector calc_2;
				VecSubtract(calc_2, controlPoints.at(i-2).point, controlPoints.at(i).point);
				VecScale(calc_2, -0.5);

				// 2(y1 - y0) - (y2-y0)/2
				Vector result;
				VecSubtract(result, calc_1, calc_2);
				VecCopy(controlPoints.at(i).tangent, result);
				continue;
			}
			// For S_i where i=1,..,n-2
			else
			{
				//  (y+1 - yi-1)/2.0
				Vector result;
				VecSubtract(result, controlPoints.at(i + 1).point, controlPoints.at(i - 1).point);
				VecScale(result, 0.5);

				// Store result at ControlPoint index i
				VecCopy(controlPoints.at(i).tangent, result);
			}

		}
		if (print)
			animTcl::OutputMessage("[HermiteSystem]: Catmull-Rom initialized");
	}
	else
	{
		animTcl::OutputMessage("[HermiteSystem]: Not enough points for Catmull-Rom, need at least 3]");
	}

}

int HermiteSystem::getSize()
{
	return controlPoints.size();
}

ControlPoint HermiteSystem::getControlPoint(int n)
{
	if (controlPoints.size() > 0)
	{
		return controlPoints.at(n);
	}

	ControlPoint x;
	zeroVector(x.point);
	zeroVector(x.tangent);

	return x;
	
}

void HermiteSystem::generateLookUpTable()
{
	for (float t = 0; t < 1; t += INCREMENT)
	{
		getArcLength(t, true);
	}
	getArcLength(1, true);
}

LineSegment HermiteSystem::getArcLength(double t, boolean generateTable)
{
	int size = controlPoints.size();

	// calculate linear length
	double linear_length = 0;
	Vector temp;
	for (int i = 0; i < size - 1; i++)
	{
		VecSubtract(temp, controlPoints.at(i + 1).point, controlPoints.at(i).point);
		linear_length += VecLength(temp);
	}

	// find target t algorithm from slides
	double target_length = linear_length * t;
	double target_t = 0;
	double line_sum = 0;
	int last_point = 0;
	for (int i = 0; i < size - 1; i++)
	{
		VecSubtract(temp, controlPoints.at(i + 1).point, controlPoints.at(i).point);
		line_sum = VecLength(temp);
		if (line_sum >= target_length)
		{
			target_t = target_length / line_sum;
			last_point = i + 1;
			break;
		}
		else {
			target_length -= line_sum;
			line_sum = 0;
		}
	}

	// calculate arc length based on target
	Vector point1, point2;
	double hermite_sum = 0;
	for (int i = 0; i < size - 1; i++)
	{
		if (i == last_point - 1)
		{
			getCurvePosition(point2, controlPoints.at(i), controlPoints.at(i + 1), 0);

			for (double j = 0; j <= target_t; j += 0.001)
			{
				VecCopy(point1, point2);
				getCurvePosition(point2, controlPoints.at(i), controlPoints.at(i + 1), j);
				VecSubtract(temp, point2, point1);
				hermite_sum += VecLength(temp);
			}
			break;

		}
		getCurvePosition(point2, controlPoints.at(i), controlPoints.at(i + 1), 0);

		for (double j = 1; j <= LINE_SAMPLES; j++)
		{
			VecCopy(point1, point2);
			getCurvePosition(point2, controlPoints.at(i), controlPoints.at(i + 1), j / LINE_SAMPLES);
			VecSubtract(temp, point2, point1);
			hermite_sum += VecLength(temp);
		}
	}

	LineSegment newLine;
	VecCopy(newLine.start, point1);
	VecCopy(newLine.end, point2);
	newLine.arc = hermite_sum;
	newLine.t = t;

	// populate to lookuptable based on t or print value
	if (generateTable)
	{
		lookUpTable[t] = newLine;
	}
	else {
		if(print)
			animTcl::OutputMessage("Arc length %.3f", hermite_sum);
	}

	return newLine;

}

LineSegment HermiteSystem::getLookUpTable(double t)
{
	LineSegment line;
	zeroVector(line.start);
	zeroVector(line.end);
	line.arc = 0;
	line.t = 0;
	for (const auto& x : lookUpTable) {
		if (abs(x.first - t) < INCREMENT)
		{
			line = x.second;
			break;
		}
		line = x.second;
	}
	return line;
}

LineSegment HermiteSystem::getLookUpTableArc(double arc)
{
	LineSegment line;
	zeroVector(line.start);
	zeroVector(line.end);
	line.arc = 0;
	line.t = 0;
	for (const auto& x : lookUpTable) {
		if (x.second.arc > arc)
		{
			line = x.second;
			break;
		}
		line = x.second;
	}
	return line;
}


void HermiteSystem::drawPoints()
{
	glPointSize(5); // more visible points

	glBegin(GL_POINTS);

	// Iterate control points and plot positions
	for (ControlPoint x : controlPoints)
	{
		glVertex3d(x.point[0], x.point[1], x.point[2]);
	}

	glEnd();
}

void HermiteSystem::drawLines()
{
	Vector point1, point2;
	int size = controlPoints.size();

	glBegin(GL_LINES);

	// Iterate control points
	for (int i = 0; i < size - 1; i++)
	{
		getCurvePosition(point2, controlPoints.at(i), controlPoints.at(i + 1), 0);

		// Draw line (2 points) with resolution using defined sample count
		for (double j = 1; j <= LINE_SAMPLES; j++)
		{
			VecCopy(point1, point2);
			getCurvePosition(point2, controlPoints.at(i), controlPoints.at(i + 1), j / LINE_SAMPLES);
			glVertex3d(point1[0], point1[1], point1[2]);
			glVertex3d(point2[0], point2[1], point2[2]);
		}
	}

	glEnd();
}

void HermiteSystem::display(GLenum mode)
{
	drawPoints();
	drawLines();
}