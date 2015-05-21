#include "PointConstraint.h"
#include <glut.h>
#include <iostream>
#include <math.h>
#include "linearSolver.h"
using namespace std;

#define PI 3.1415926535897932384626433832795


PointConstraint::PointConstraint(Particle *p, Vec2f point) :
m_Point(point), m_p(p) {}

void PointConstraint::draw()
{
	const double h = 0.03;
	glColor3f(0.f, 1.f, 1.f);
	glBegin(GL_QUADS);
	glVertex2f(m_Point[0] - h / 2.0, m_Point[1] - h / 2.0);
	glVertex2f(m_Point[0] + h / 2.0, m_Point[1] - h / 2.0);
	glVertex2f(m_Point[0] + h / 2.0, m_Point[1] + h / 2.0);
	glVertex2f(m_Point[0] - h / 2.0, m_Point[1] + h / 2.0);
	glEnd();
}

// return the c: C(x,y) = (x-xc)^2-(y-yc)^2
float PointConstraint::getC()
{
	Vec2f posdif = (m_p->m_Position - m_Point);
	float C = (posdif[0] * posdif[0] + posdif[1] * posdif[1]);
	return C;
}

float PointConstraint::getCDot(){
	Vec2f posdif = (m_p->m_Position - m_Point);
	Vec2f veldif = (m_p->m_Velocity);
	float CDot = vecDotNew(2 * posdif, 2 * veldif);
	return CDot;
}

vector<Vec2f> PointConstraint::getJacobian(){
	//a vector, only 1 now, might implement multiple particles
	vector<Vec2f> J;
	Vec2f posdif = (m_p->m_Position - m_Point);
	J.push_back(Vec2f(2 * posdif[0], 2 * posdif[1]));
	return J;
}

//return JDot, if there are more use same order as particle
vector<Vec2f> PointConstraint::getJacobianDot(){
	vector<Vec2f> JDot;
	JDot.push_back(Vec2f(0, 0));
	return JDot;
}

vector<Particle*> PointConstraint::getParticles(){
	//only 1 here, but might implement ability for more
	vector<Particle*> result;
	result.push_back(m_p);
	return result;
}