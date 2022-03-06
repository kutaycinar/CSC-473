#pragma once

#include "BaseSystem.h"

class Classroom : public BaseSystem
{
public:
	Classroom(const std::string& name);
	void display(GLenum mode = GL_RENDER);
protected:

};
