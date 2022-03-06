#ifndef ANIM_H
#define ANIM_H

#include <util/util.h>
#include "animTcl.h"
#include "BaseObject.h"
#include "BaseSimulator.h"
#include "BaseSystem.h"
#include <GLModel/GLModel.h>

#include "GlobalResourceManager.h"

// ---- Hoooks to anim driver and GLUT

void MakeScene(void) ; // builds the objects in the scene
void SimulationStep(void) ; // Called every time the simulators need to take a step forward
void InitSimulation(void) ; // called every time the simulators start

// set a display event function
void PostRedisplay(void) ;

void Reset(void) ;

#endif