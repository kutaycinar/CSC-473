////////////////////////////////////////////////////
// anim.cpp
// Template code for drawing an articulated figure.
// CS 174C Winter 2010
// Original Author: Petros Faloutsos
// Modified by: Gabriele Nataneli
////////////////////////////////////////////////////

#ifdef WIN32
#include <windows.h>
#endif


#include <stdio.h>
#include <cstdlib>
#include <cmath>

#include "shared/opengl.h"

#include "shared/defs.h"

#include <util/util.h>
#include "anim.h"
#include "animTcl.h"
#include "myScene.h"

FrameSaver g_frameSaver ;
Timer g_globalTimer ;

BallData *g_arcball = NULL ;

int g_width = 500 ;
int g_height = 500 ;
float g_zoom = 0.25 ;
int g_prevY = 0 ;

int g_state = STATE_IDLE ;
int g_frameDumping = 0 ;

void resetArcball() ;
void save_image();
void instructions();

#define X 0
#define Y 1
#define Z 2

// The eye point and look-at point.
//
double g_eye[3] = {0, 0.0, 50.0};
double g_ref[3] = {0.0, 0.0, 0.0};

void DrawScene(GLenum mode) 
{

	// invoke the global display function

	GlobalResourceManager::use()->display( mode );

}

void InitSimulation(void)
{

	GlobalResourceManager::use()->initializeAllSimulators();

}

void SimulationStep(void)
{

	GlobalResourceManager::use()->advanceSimulationTime();

	GlobalResourceManager::use()->stepAllSimulators();

	animTcl::OutputMessage("This is the simulation time %lf", 
		GlobalResourceManager::use()->getSimulationTime() );
}



// resets time to 0.
void resetTime(void)
{
	g_globalTimer.GetElapsedTime() ;
	// reset the timer to point the current time	
	g_globalTimer.Reset() ;

	GlobalResourceManager::use()->setActualTime( 0.0 );
	GlobalResourceManager::use()->setSimulationTime( 0.0 );

}

void Reset(void)
{
	resetTime() ;

	GlobalResourceManager::use()->resetAll();

}

// start or end interaction
void MouseCB(int button, int state, int x, int y)
{

	GlobalResourceManager::use()->setMouseButtonInfo( button, state );


	if( button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN )
	{
		HVect arcball_coords;
		arcball_coords.x = 2.0*(float)x/(float)g_width-1.0;
		arcball_coords.y = -2.0*(float)y/(float)g_height+1.0;
		Ball_Mouse(g_arcball, arcball_coords) ;
		Ball_Update(g_arcball);
		Ball_BeginDrag(g_arcball);

	}

	if( button == GLUT_RIGHT_BUTTON && state == GLUT_UP )
	{
		Ball_EndDrag(g_arcball);

		//		Button = -1 ;

		GlobalResourceManager::use()->setMouseButtonInfo( -1, state );

	}

	if( button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN )
	{
		g_prevY = y ;
	}

	myMouse(button, state, x,y) ;

	// Tell the system to redraw the window
	glutPostRedisplay() ;
}

// interaction (mouse motion)
void MotionCB(int x, int y)
{

	GLMouseButtonInfo mouseButtonInfo;

	mouseButtonInfo =  GlobalResourceManager::use()->getMouseButtonInfo();

	if( mouseButtonInfo.button == GLUT_RIGHT_BUTTON )
	{
		HVect arcball_coords;
		arcball_coords.x = 2.0*(float)x/(float)g_width - 1.0 ;
		arcball_coords.y = -2.0*(float)y/(float)g_height + 1.0 ;
		Ball_Mouse(g_arcball,arcball_coords);
		Ball_Update(g_arcball);
		glutPostRedisplay() ;
	}
	else if( mouseButtonInfo.button == GLUT_MIDDLE_BUTTON )
	{
		if( y - g_prevY > 0 )
			g_zoom  = g_zoom * 1.03 ;
		else 
			g_zoom  = g_zoom * 0.97 ;

		g_prevY = y ;

		glutPostRedisplay() ;
	}

	myMotion(x,y) ;

}

void resetArcball()
{
	Ball_Init(g_arcball);
	Ball_Place(g_arcball,qOne,0.75);
}

//////////////////////////////////////////////////////
//    PROC: myKey()
//    DOES: this function gets called for any keypresses
// 
//////////////////////////////////////////////////////

void KeyboardCB(unsigned char key, int x, int y)
{
	switch (key) {
		case 'q':
		case 27:
			exit(0); 
		case 'f': // take a snapshot in ppm format
			g_frameSaver.DumpPPM(g_width,g_height) ;
			break;
		case 't':
			resetTime() ;
			break ;
		case 'r':
			resetArcball() ;
			Reset() ;
			g_zoom = 1 ;
			glutPostRedisplay() ;
			break ;
		case 'a': // toggle animation state
			if( g_state == STATE_ANIMATE )
			{
				g_state = STATE_IDLE ;
			}
			else
			{
				resetTime() ;
				g_state = STATE_ANIMATE ;
			}
			break ;
		case 's': // togle simulation state
			if( g_state == STATE_SIMULATE )
			{
				g_state = STATE_IDLE ;
			}
			else
			{
				resetTime() ;
				InitSimulation() ;
				g_state = STATE_SIMULATE ;
			}
			break ;
		case 'p': // togle playback
			if( g_state == STATE_PLAYBACK )
			{
				g_state = STATE_IDLE ;
			}
			else
			{
				g_state = STATE_PLAYBACK ;
			}
			break ;
		case '0':
			//reset your object
			break ;
		case 'm': // toggle frame dumping
			if( g_frameDumping == 1 )
			{
				animTcl::OutputMessage("Frame Dumping disabled.\n") ;
				g_frameDumping = 0 ;
			}
			else
			{
				animTcl::OutputMessage("Frame Dumping enabled.\n") ;
				g_frameDumping = 1  ;
			}
			g_frameSaver.Toggle(g_width);
			break ;
		case 'h':
		case '?':
			instructions();
			break;
	}

	myKey(key, x, y) ;

	glutPostRedisplay() ;

}


/*********************************************************
PROC: Init()
DOES: performs most of the OpenGL intialization
-- change these with care, if you must.
**********************************************************/

void Init(void)
{
	GLfloat ambient[] = { 0.2, 0.2, 0.2, 1.0 };
	GLfloat diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat specular[] = { 1.0, 1.0, 1.0, 1.0 };
	//GLfloat position[] = { -3.0, 3.0, 3.0, 0.0 };
	GLfloat position[] = { 0.0, 0.0, 1.0, 0 };

	GLfloat lmodel_ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
	GLfloat local_view[] = { 0.0 };

	/**** set lighting parameters ****/
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, position);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
	glLightModelfv(GL_LIGHT_MODEL_LOCAL_VIEWER, local_view);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE) ;

	/*    glFrontFace (GL_CW); */
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_AUTO_NORMAL);
	glEnable(GL_NORMALIZE);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	glPixelStorei(GL_PACK_ALIGNMENT,1) ;
	glPixelStorei(GL_UNPACK_ALIGNMENT,1) ;
	glShadeModel(GL_SMOOTH) ;

	g_arcball = new BallData ;
	Ball_Init(g_arcball);
	Ball_Place(g_arcball,qOne,0.75);

	glViewport(0, 0, g_width, g_height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	//glFrustum(-5,5,-5,5,4,100);
	glOrtho(-6,6,-6,6,-500,500) ;

	myOpenGLInit() ;

}



/*********************************************************
**********************************************************
**********************************************************

PROC: display()
DOES: this gets called by the event handler to draw
the scene
**********************************************************
**********************************************************
**********************************************************/

void mainLoop(void)
{
	glutMainLoop() ;
}

void display(void)
{


	static double oldTime = GlobalResourceManager::use()->getActualTime();
	oldTime = g_globalTimer.GetElapsedTime() ;


	glClearColor(0.2f,0.2f,1.0f,0.5f);   /* set the background colour */
	/* OK, now clear the screen with the background colour */
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	glMatrixMode(GL_MODELVIEW) ;
	glLoadIdentity();

	g_eye[Z] = 8 ;
	gluLookAt (g_eye[X], g_eye[Y], g_eye[Z],
		g_ref[X], g_ref[Y], g_ref[Z], 0.0,1.0,0.0);

	HMatrix arcball_rot;
	Ball_Value(g_arcball,arcball_rot);
	glMultMatrixf((float *)arcball_rot);

	glScalef(g_zoom, g_zoom, g_zoom) ;
	DrawScene() ;

	glMatrixMode(GL_PROJECTION) ;
	glPushMatrix() ;
	glLoadIdentity();

	glMatrixMode(GL_MODELVIEW) ;
	glPushMatrix() ;
	glLoadIdentity();

	// display frame rate
	glPushAttrib(GL_LIGHTING|GL_TEXTURE_2D) ;
	glDisable(GL_LIGHTING) ;
	glDisable(GL_TEXTURE_2D) ;
	glColor3f(1,1,1) ;
	//	int fps  =  (int) ((float) 1.0) / (TIME - oldTime) ;
	int fps  =  (int) ((float) 1.0) / (g_globalTimer.GetElapsedTime() - oldTime) ;
	oldTime = GlobalResourceManager::use()->getActualTime() ;
	char s[80] ;
	sprintf(s,"FPS: %5d  TIME %6.3lf SIMTIME %6.3lf%c", 
		fps,GlobalResourceManager::use()->getActualTime(),
		GlobalResourceManager::use()->getSimulationTime(),'\0') ;

	glRasterPos2f(-0.98,-0.98) ;
	GLlabel(s, 15) ;

	glPopAttrib() ;

	glMatrixMode(GL_PROJECTION) ;
	glPopMatrix() ;
	glMatrixMode(GL_MODELVIEW) ;
	glPopMatrix() ;



	glutSwapBuffers();
	if( g_frameDumping == 1) g_frameSaver.DumpPPM(g_width,g_height) ;

}

/**********************************************
PROC: myReshape()
DOES: handles the window being resized 

-- don't change
**********************************************************/

void ReshapeCB(int w, int h)
{
	g_width = w;
	g_height = h;

	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	/*** this defines the field of view of the camera   ***/
	/*** Making the first 4 parameters larger will give ***/
	/*** a larger field of view, therefore making the   ***/
	/*** objects in the scene appear smaller            ***/
	// glFrustum(-1,1,-1,1,7,100);
	glOrtho(-6,6,-6,6,-500,500) ;
	//glOrtho(-6,6,-6,6,-500,500) ;
	//gluPerspective(20,(float) w/(float) h,1,100) ;
	//glFrustum(-1,1,-1,1,4,100);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	/*** this sets the virtual camera          ***/
	/*** gluLookAt( x,y,z,   x,y,z   x,y,z );  ***/
	/***            camera  look-at camera-up  ***/
	/***            pos'n    point   vector    ***/

	gluLookAt(g_eye[X],g_eye[Y],g_eye[Z],
		g_ref[X],g_ref[Y],g_ref[Z], 0,1,0);
	HMatrix arcball_rot;
	Ball_Value(g_arcball,arcball_rot);
	glMultMatrixf((float *)arcball_rot);
}
void instructions() {
	animTcl::OutputMessage("Keys:\n");
	animTcl::OutputMessage("  s - save the image\n");
	animTcl::OutputMessage("  r - restore the original view.\n") ;
	animTcl::OutputMessage("  0 - reset all systems.\n") ;
	animTcl::OutputMessage("  a - toggle the animation.\n") ;
	animTcl::OutputMessage("  s - toggle the simulation.\n") ;
	animTcl::OutputMessage("  t - reset all timers.\n") ;
	animTcl::OutputMessage("  m - toggle frame dumping.\n") ;
	animTcl::OutputMessage("  q - quit.\n");
	animTcl::OutputMessage("  h - print these instructions.\n");
}

void idleCB(void)
{
	animTcl::checkEvents() ;
	if( g_state == STATE_ANIMATE )
	{
		if( g_frameDumping == 0 )
			GlobalResourceManager::use()->setActualTime( g_globalTimer.GetElapsedTime() );
		else 
		{

			GlobalResourceManager::use()->advanceActualTime( 0.33 );

		}
		glutPostRedisplay() ; 
	}
	else if( g_state == STATE_SIMULATE )
	{

		GlobalResourceManager::use()->setActualTime( g_globalTimer.GetElapsedTime() );

		SimulationStep() ;
		glutPostRedisplay() ; 
	}
	myIdleCB() ;
}


/*********************************************************
PROC: mainInit()
DOES: calls initialization, then hands over control
to the event handler, which calls 
display() whenever the screen needs to be redrawn
**********************************************************/

int mainInit(int argc, char** argv) {

	glutInit(&argc, argv);

	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowPosition (0, 0);
	glutInitWindowSize(g_width,g_height);
	glutCreateWindow(argv[0]);

	Init();
	resetTime() ;
	glutIdleFunc(idleCB) ;
	glutReshapeFunc (ReshapeCB);
	glutKeyboardFunc( KeyboardCB );
	glutMouseFunc(MouseCB) ;
	glutMotionFunc(MotionCB) ;
	instructions();

	MakeScene() ;
	glutDisplayFunc(display);
	animTcl::InitTclTk(argc,argv) ;
	glutMainLoop();


	return 0;         // never reached
}



#ifndef	WIN32

int main(int argc, char **argv)
{
	mainInit(argc,argv);
	return 0;
}
#else

#include <windows.h>

// _ANSI_ARGS_ defined in tcl.h
static void		setargv	_ANSI_ARGS_((int *argcPtr, char	***argvPtr));

int APIENTRY
WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance,
		LPSTR lpszCmdLine, int nCmdShow)
{
	char **argv, *p;
	int	argc;
	char buffer[MAX_PATH];

	setargv(&argc, &argv);

	/*
	* Replace argv[0] with full pathname of executable, and forward
	* slashes substituted for backslashes.
	*/

	GetModuleFileName(NULL, buffer, sizeof(buffer));
	argv[0] = buffer;
	for	(p = buffer; *p	!= '\0'; p++) {
		if (*p == '\\')	{
			*p = '/';
		}
	}

	mainInit(argc,argv);
	return 1;
}
/*
*-------------------------------------------------------------------------
*
* setargv --
*
*	Parse the Windows command line string into argc/argv.  Done here
*	because	we don't trust the builtin argument parser in crt0.
*	Windows	applications are responsible for breaking their	command
*	line into arguments.
*
*	2N backslashes + quote -> N backslashes	+ begin	quoted string
*	2N + 1 backslashes + quote -> literal
*	N backslashes +	non-quote -> literal
*	quote +	quote in a quoted string -> single quote
*	quote +	quote not in quoted string -> empty string
*	quote -> begin quoted string
*
* Results:
*	Fills argcPtr with the number of arguments and argvPtr with the
*	array of arguments.
*
* Side	effects:
*	Memory allocated.
*
*--------------------------------------------------------------------------
*/

static void
setargv(int *argcPtr, char ***argvPtr)
{
	char *cmdLine, *p, *arg, *argSpace;
	char **argv;
	int	argc, size, inquote, copy, slashes;

	cmdLine = GetCommandLine();

	/*
	* Precompute an overly pessimistic	guess at the number of arguments
	* in the command line by counting non-space spans.
	*/

	size = 2;
	for	(p = cmdLine; *p != '\0'; p++) {
		if (isspace(*p)) {
			size++;
			while (isspace(*p))	{
				p++;
			}
			if (*p == '\0') {
				break;
			}
		}
	}
	argSpace = (char *)	ckalloc((unsigned) (size * sizeof(char *)
		+ strlen(cmdLine) +	1));
	argv = (char **) argSpace;
	argSpace +=	size * sizeof(char *);
	size--;

	p =	cmdLine;
	for	(argc =	0; argc	< size;	argc++)	{
		argv[argc] = arg = argSpace;
		while (isspace(*p)) {
			p++;
		}
		if (*p == '\0')	{
			break;
		}

		inquote	= 0;
		slashes	= 0;
		while (1) {
			copy = 1;
			while (*p == '\\') {
				slashes++;
				p++;
			}
			if (*p == '"') {
				if ((slashes & 1) == 0)	{
					copy = 0;
					if ((inquote) && (p[1] == '"')) {
						p++;
						copy = 1;
					} else {
						inquote	= !inquote;
					}
				}
				slashes	>>= 1;
			}

			while (slashes) {
				*arg = '\\';
				arg++;
				slashes--;
			}

			if ((*p == '\0') ||	(!inquote && isspace(*p))) {
				break;
			}
			if (copy !=	0) {
				*arg = *p;
				arg++;
			}
			p++;
		}
		*arg = '\0';
		argSpace = arg + 1;
	}
	argv[argc] = NULL;

	*argcPtr = argc;
	*argvPtr = argv;
}

void PostRedisplay(void)
{
	glutPostRedisplay() ;
}

#endif

