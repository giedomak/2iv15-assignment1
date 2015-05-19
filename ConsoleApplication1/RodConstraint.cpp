#include "RodConstraint.h"
#include <glut.h>
#include <iostream>
#include "linearSolver.h"
using namespace std;

RodConstraint::RodConstraint(Particle *p1, Particle * p2, double dist) :
  m_p1(p1), m_p2(p2), m_dist(dist) {}

void RodConstraint::draw()
{
  glBegin( GL_LINES );
  glColor3f(0, 1, 0);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0, 1, 0);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();

}

// return the C => C(x1, y1, x2, y2) = (x1 - x2)^2 + (y1 - y2)^2 - r^2
float RodConstraint::getC(){
	Vec2f posdif = m_p1->m_Position - m_p2->m_Position;
	float C = (posdif[0]*posdif[0] + posdif[1]*posdif[1] - m_dist*m_dist);
	return C;
}

//from the slides C = 1/2(x.x-1) =  (1/2)x.(1/2)x-(1/2)
//from the slides CDot = (1/2)xDot.(1/2)x + (1/2)x.(1/2)xDot
//our C = (x - xc)^2 + (y - yc)^2 - r^2
//return the cDot: (dx/dy)C(x, y) = ((x - xc)^2 + (y - yc)^2 - r^2)dx
//CDot = 2*(x-xc)*vx + 2*(y-yc)*vy - 0
//CDot = 2*((x-xc)*vx + (y-yc)*vy)
float RodConstraint::getCDot(){
	Vec2f posdif = (m_p1->m_Position - m_p2->m_Position);
	Vec2f veldif = (m_p1->m_Velocity - m_p2->m_Velocity);
	float CDot = vecDotNew(2 * posdif, 2 * veldif);
	return CDot;
}

vector<Vec2f> RodConstraint::getJacobian(){
	vector<Vec2f> J;
	J.push_back(2.0 * (m_p1->m_Position - m_p2->m_Position));
	J.push_back(2.0 * (m_p2->m_Position - m_p1->m_Position));
	return J;
}

vector<Vec2f> RodConstraint::getJacobianDot(){
	vector<Vec2f> JDot;
	JDot.push_back(2.0 * (m_p1->m_Velocity - m_p2->m_Velocity));
	JDot.push_back(2.0 * (m_p2->m_Velocity - m_p1->m_Velocity));
	return JDot;
}

vector<Particle*> RodConstraint::getParticles(){
	vector<Particle*> result;
	result.push_back(m_p1);
	result.push_back(m_p2);
	return result;
}