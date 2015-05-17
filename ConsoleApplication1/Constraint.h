#pragma once

#include <vector>
#include "include/gfx/vec2.h"
using namespace std;

class Constraint
{
public:
	virtual ~Constraint(){}

	virtual void draw() = 0;
	virtual float getC() = 0;
	virtual float getCDot() = 0;
	virtual vector<Vec2f> getJ() = 0;
	virtual vector<Vec2f> getJdot() = 0;
	virtual vector<Particle> getParticles() = 0;
};