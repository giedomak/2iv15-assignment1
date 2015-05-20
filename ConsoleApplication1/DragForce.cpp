#include "DragForce.h"
#include <glut.h>

DragForce::DragForce(Particle * p1, float drag) :
m_p1(p1), m_drag(drag){}

void DragForce::draw()
{
	//no drawing
}

void DragForce::apply()
{
	//F = m * a
	//a = F/m
	m_p1->m_Velocity *= m_drag;
	//m_p1->m_Velocity += g*0.1;
}