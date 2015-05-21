#pragma once

#include "include/gfx/vec2.h"

class Particle
{
public:

	Particle(const Vec2f & ConstructPos, float mass, int ID, int colour);
	virtual ~Particle(void);

	void reset();
	void draw();
	int getParticleID();

	Vec2f m_ConstructPos;
	Vec2f m_Position;
	Vec2f m_Velocity;
	Vec2f m_Force;
	Vec2f m_ForceConstraint;
	float m_mass;
	int m_Number;
	int m_ID;
	int m_colour;
};
