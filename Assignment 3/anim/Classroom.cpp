#include "Classroom.h"


Classroom::Classroom(const std::string& name) : BaseSystem(name) {

}

void Classroom::display(GLenum mode)
{
    GLfloat ambient[] = { 0.2, 0.2, 0.2, 1 };
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

    // Blackboard
    glBegin(GL_QUADS);
    set_colour(0, 0, 0);
    glVertex3d(9, 6, -0.05);
    glVertex3d(9, -6, -0.05);
    glVertex3d(-9, -6, -0.05);
    glVertex3d(-9, 6, -0.05);
    glEnd();

    // Classroom Wall
    glDisable(GL_LIGHTING);
    glBegin(GL_QUADS);
    set_colour(1, 0.2, 0.2);
    glVertex3d(18, 9, -0.06);
    glVertex3d(18, -18.3, -0.06);
    glVertex3d(-18, -18.3, -0.06);
    glVertex3d(-18, 9, -0.06);
    glEnd();

    // Classroom Ground
    glBegin(GL_QUADS);
    glVertex3d(-18, -18.3, 0);
    glVertex3d(18, -18.3, 0);
    glVertex3d(18, -18.3, 36);
    glVertex3d(-18, -18.3, 36);
    glEnd();
    glEnable(GL_LIGHTING);

}