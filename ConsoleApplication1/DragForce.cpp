#include "DragForce.h"

DragForce::DragForce(Particle * p1, float drag) :
m_p1(p1), m_drag(drag){}

void DragForce::draw()
{
	//no drawing
}

void DragForce::apply()
{
	m_p1->m_Velocity *= m_drag;
}