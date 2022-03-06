#ifndef MYSCENE_H
#define MYSCENE_H

#include "anim.h"

#include "GlobalResourceManager.h"

// create your scene (you need to modify this!)
void MakeScene(void);

// initialize OpenGL 
void myOpenGLInit(); 

// the main display callback
void DrawScene(GLenum mode=GL_RENDER) ;

// called every time a key is pressed
void myKey(unsigned char key, int x, int y) ; 

// called every time a mouse button is press/released
void myMouse(int button, int state, int x, int y); 

// called every time the mouse moves
void myMotion(int x, int y);

// called repeteadly every time the CPU is idle (not necessarily called at
// regular time intervals)
void myIdleCB(void) ;

// Commands you specify here will be recognized by the interpreter
void mySetScriptCommands(Tcl_Interp *interp, int argc, char **argv) ;

// Variables you specify here will be recognised by the tcl interpreter
extern SETVAR myScriptVariables[] ;

#endif	// MYSCENE_H