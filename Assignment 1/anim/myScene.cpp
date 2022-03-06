////////////////////////////////////////////////////
// // Template code for  CS 174C
////////////////////////////////////////////////////

#ifdef WIN32
#include <windows.h>
#endif


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <shared/defs.h>

#include "shared/opengl.h"

#include <string.h>
#include <util/util.h>
#include <GLModel/GLModel.h>
#include "anim.h"
#include "animTcl.h"
#include "myScene.h"
#include "SampleParticle.h"
#include "SampleGravitySimulator.h"
//#include <util/jama/tnt_stopwatch.h>
//#include <util/jama/jama_lu.h>

#include "HermiteSystem.h"
#include "TankPathSystem.h"
#include "TankPathSimulator.h"
#include "MissileSystem.h"
#include "MissileSimulator.h"

// register a sample variable with the shell.
// Available types are:
// - TCL_LINK_INT 
// - TCL_LINK_FLOAT

int g_testVariable = 10;

SETVAR myScriptVariables[] = {
	"testVariable", TCL_LINK_INT, (char*)&g_testVariable,
	"",0,(char*)NULL
};


//---------------------------------------------------------------------------------
//			Hooks that are called at appropriate places within anim.cpp
//---------------------------------------------------------------------------------

// start or end interaction
void myMouse(int button, int state, int x, int y)
{

	// let the global resource manager know about the new state of the mouse 
	// button
	GlobalResourceManager::use()->setMouseButtonInfo(button, state);

	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
	{
		//animTcl::OutputMessage(
		//	"My mouse received a mouse button press event\n");

		////////////////////////
		// Part 1 Mouse Actions
		////////////////////////

		HermiteSystem* part1 = dynamic_cast<HermiteSystem*>(GlobalResourceManager::use()->getSystem("hermite"));

		if (part1)
		{
			part1->addControlPoint(12.0 * x / glutGet(GLUT_WINDOW_WIDTH) - 6, (-12.0) * y / glutGet(GLUT_WINDOW_HEIGHT) + 6, 0);
		}

	}
	if (button == GLUT_LEFT_BUTTON && state == GLUT_UP)
	{
		//animTcl::OutputMessage(
		//	"My mouse received a mouse button release event\n");

	}
}	// myMouse

// interaction (mouse motion)
void myMotion(int x, int y)
{

	GLMouseButtonInfo updatedMouseButtonInfo =
		GlobalResourceManager::use()->getMouseButtonInfo();

	if (updatedMouseButtonInfo.button == GLUT_LEFT_BUTTON)
	{
		//animTcl::OutputMessage(
		//	"My mouse motion callback received a mousemotion event\n");
	}

}	// myMotion

void MakeScene(void)
{

	/*

	This is where you instantiate all objects, systems, and simulators and
	register them with the global resource manager

	*/

	/* SAMPLE SCENE */

	bool success;

	// register a system
	SampleParticle* sphere1 = new SampleParticle("sphere1");

	success = GlobalResourceManager::use()->addSystem(sphere1, true);

	// make sure it was registered successfully
	assert(success);

	// register a simulator
	SampleGravitySimulator* gravitySimulator =
		new SampleGravitySimulator("gravity", sphere1);

	success = GlobalResourceManager::use()->addSimulator(gravitySimulator);

	// make sure it was registered successfully
	assert(success);

	/* END SAMPLE SCENE */

	// the following code shows you how to retrieve a system that was registered 
	// with the resource manager. 

	BaseSystem* sampleSystemRetrieval;

	// retrieve the system
	sampleSystemRetrieval =
		GlobalResourceManager::use()->getSystem("sphere1");

	// make sure you got it
	assert(sampleSystemRetrieval);

	BaseSimulator* sampleSimulatorRetrieval;

	// retrieve the simulator
	sampleSimulatorRetrieval =
		GlobalResourceManager::use()->getSimulator("gravity");

	// make sure you got it
	assert(sampleSimulatorRetrieval);

	

}	// MakeScene

// OpenGL initialization
void myOpenGLInit(void)
{
	animTcl::OutputMessage("Initialization routine was called.");

}	// myOpenGLInit

void myIdleCB(void)
{

	return;

}	// myIdleCB

void myKey(unsigned char key, int x, int y)
{
	// animTcl::OutputMessage("My key callback received a key press event\n");
	return;

}	// myKey

static int testGlobalCommand(ClientData clientData, Tcl_Interp* interp, int argc, myCONST_SPEC char** argv)
{
	animTcl::OutputMessage("This is a test command!");
	animTcl::OutputResult("100");
	return TCL_OK;

}	// testGlobalCommand


//////////////////////////////////////
// 
// Part 1: Hermite System Registration
// 
//////////////////////////////////////

static int part1Command(ClientData clientData, Tcl_Interp* interp, int argc, myCONST_SPEC char** argv)
{
	animTcl::OutputMessage("Execute Part 1!");

	// Clear scene resources and rerender
	GlobalResourceManager::use()->clearAll();
	glutPostRedisplay();

	////////////////////////
	// Creating Part 1 Scene
	////////////////////////

	bool success;

	// Create, register and assert Hermite System
	HermiteSystem* hermite = new HermiteSystem("hermite");
	success = GlobalResourceManager::use()->addSystem(hermite, true);
	assert(success);

	return TCL_OK;
}


static int part2Command(ClientData clientData, Tcl_Interp* interp, int argc, myCONST_SPEC char** argv)
{
	animTcl::OutputMessage("Execute Part 2!");

	// Clear scene resources and rerender
	GlobalResourceManager::use()->clearAll();
	glutPostRedisplay();

	////////////////////////
	// Creating Part 2 Scene
	////////////////////////

	bool success;

	// Create, register and assert Tank Path System
	TankPathSystem* tankpath = new TankPathSystem("tankpath");
	success = GlobalResourceManager::use()->addSystem(tankpath, true);
	assert(success);


	// Assign Tank Path Simulator
	TankPathSimulator* tank_sim = new TankPathSimulator("tank_sim", tankpath);
	success = GlobalResourceManager::use()->addSimulator(tank_sim);
	assert(success);

	// Create, register and assert Missle System
	MissileSystem* missile = new MissileSystem("missile");
	success = GlobalResourceManager::use()->addSystem(missile, true);
	assert(success);

	// Assign Missile Simulator
	MissileSimulator* missile_sim = new MissileSimulator("missile_sim", missile);
	success = GlobalResourceManager::use()->addSimulator(missile_sim);
	assert(success);

	missile_sim->setTankPath(tankpath);

	return TCL_OK;

}

void mySetScriptCommands(Tcl_Interp* interp)
{

	// here you can register additional generic (they do not belong to any object) 
	// commands with the shell

	Tcl_CreateCommand(interp, "test", testGlobalCommand, (ClientData)NULL,
		(Tcl_CmdDeleteProc*)NULL);

	Tcl_CreateCommand(interp, "part1", part1Command, (ClientData)NULL,
		(Tcl_CmdDeleteProc*)NULL);

	Tcl_CreateCommand(interp, "part2", part2Command, (ClientData)NULL,
		(Tcl_CmdDeleteProc*)NULL);

}	// mySetScriptCommands