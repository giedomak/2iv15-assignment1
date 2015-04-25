#include "SpringForce.h"
#include <glut.h>

//Spring system force calculations between 2 particles
//ks = strength
//kd = damping
//dist = resting distance
SpringForce::SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd) :
  m_p1(p1), m_p2(p2), m_dist(dist), m_ks(ks), m_kd(kd) {}

void SpringForce::draw()
{
  glBegin( GL_LINES );
  glColor3f(1, 0, 0);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(1, 0, 0);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();
}

void SpringForce::apply()
{
	Vec2f posdif = (m_p1->m_Position - m_p2->m_Position);
	Vec2f speeddif = (m_p1->m_Velocity - m_p2->m_Velocity);
	float posLength = (sqrt(posdif[0] * posdif[0] + posdif[1] * posdif[1]));
	float dotProduct = (speeddif[0] * posdif[0] + speeddif[1] * posdif[1]);

	Vec2f force_p1 = (posdif/posLength)*((m_ks * (posLength - m_dist)) + (m_kd * ( dotProduct / posLength)));
	Vec2f force_p2 = -force_p1;

	m_p1->m_Velocity -= force_p1;
	m_p2->m_Velocity -= force_p2;
}