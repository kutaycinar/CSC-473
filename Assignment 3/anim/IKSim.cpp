#include "IKSim.h"

IKSim::IKSim(const std::string& name, BaseSystem* target) :
	BaseSimulator(name),
	m_object(target)
{
	fileLoaded = false;
	hermite_t = 0;
	delta_t = 0;
	prev_t = 0;
	ikSkeleton = {};
	initial = 0.01;
	increment = 0.01;
}

void IKSim::setHermite(Hermite* target)
{
	hermite = target;
}

int IKSim::init(double time)
{
	return 0;

}

int IKSim::step(double time)
{
	delta_t = time - prev_t;

	// reposition target to start when at end of spline
	if (hermite_t > 0.990)
	{
		hermite_t = 0;
		transition = true;
		initial = 0.1;
		increment = 0.002;
	}

	if (fileLoaded)
	{
		Vector currentP, targetP, error;

		// get bob's current hand position
		m_object->getState(currentP);

		// get target position based on hermite t value
		VectorObj target = hermite->getIntermediatePoint(hermite_t);
		setVector(targetP, target[0], target[1], target[2]);

		// calculate error and set new target position if bob is close
		VecSubtract(error, targetP, currentP);

		if (VecLength(error) < 0.15)
		{
			hermite_t += 0.00015;
		}

		if (hermite_t < 0.00005)
		{
			hermite_t = 0;
			if (transition)
			{
				VecCopy(pTargetP, currentP);
				VecCopy(velocity, error);
				transition = false;
			}
			Vector velocity_med;
			VecCopy(velocity_med, velocity);
			VecScale(velocity_med, initial);
			Vector pTarget;
			VecAdd(pTarget, pTargetP, velocity_med);
			m_object->setState(pTarget);
			if (initial < 1) 
				initial += increment;
		}
		else {
			m_object->setState(targetP);

		}
	}

	prev_t = time;
	return 0;

}

int IKSim::command(int argc, myCONST_SPEC char** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("simulator %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "read") == 0)
	{
		if (argc == 2)
		{
			hermite->loadFromFile2D(argv[1]);
			animTcl::OutputMessage("[iksim] Read spline from file");
			fileLoaded = true;

			// Reset LERP variables
			hermite_t = 0;
			transition = true;
			initial = 0.1;
			increment = 0.01;

			// Send natural hand resting position to Bob
			ikSkeleton.updateRestingPosition = true;
			setVector(ikSkeleton.shoulder_R, -107.57, -325.06, -317.46);
			setVector(ikSkeleton.elbow_R, 101.07, -267.92, 0);
			setVector(ikSkeleton.wrist_R, 0, 72.87, 85.12);
			setVector(ikSkeleton.shoulder_L, 90, 70, 0);
			setVector(ikSkeleton.elbow_L, 0, 20, 0);
			m_object->setState((double*)&ikSkeleton);
			ikSkeleton.updateRestingPosition = false;
		}
		else
		{
			animTcl::OutputMessage("[iksim] Usage: simulator iksim read spline.txt");
			return TCL_ERROR;
		}
	}

	glutPostRedisplay();
	return TCL_OK;
}