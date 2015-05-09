#pragma once

#include <glut.h>

class Force
{
public:
	virtual ~Force(){}
	virtual void draw() = 0;
	virtual void apply() = 0;
};