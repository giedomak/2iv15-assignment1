#include "include/gfx/vec2.h"
#include "Particle.h"
#include "Force.h"
#include "Constraint.h"

class DragForce : public Force
{

public:

	DragForce(Particle *p1, float drag);

	void draw();
	void apply();

	Particle * const m_p1;
	float m_drag;
};
