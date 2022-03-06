
#ifndef GLutilities_h
#define GLutilities_h

#include <shared/opengl.h>

void GLdrawAxes(double length);
void GLdrawCircle(double radius, int numSides);
void GLdrawCylinder(double radLengthRatio, double point1[3], double point2[3]);
void GLdrawVector(double point[3], double vector[3]);
void GLdrawVector(double point[3], double direction[3], double mag);
void GLmakeRasterFont(void);
void GLlabel(char *s, int size = 12);
void GLprintString(char *s);
void GLdrawSphere(double radius, double centre[3]);

void drawCylinder(void) ;

void drawCone(void) ;

void drawCube(void) ;

void drawSphere(void) ;

void set_colour(float r, float g, float b) ;

extern GLubyte rasters[][13];

#endif

