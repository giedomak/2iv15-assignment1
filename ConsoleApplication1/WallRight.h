#include "include/gfx/vec2.h"
#include "Particle.h"
#include "Force.h"
#include "Constraint.h"

class WallRight : public Force
{

public:

	WallRight(vector<Particle*> pVector, float xPos, float dt);

	void draw();
	void apply();

	vector<Particle*> m_pVector;
	float m_xPos;
	float m_dt;
};