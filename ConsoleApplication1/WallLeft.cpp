#include "WallLeft.h"
#include <glut.h>

WallLeft::WallLeft(vector<Particle*> pVector, float xPos, float dt) :
m_pVector(pVector), m_xPos(xPos), m_dt(dt){}

void WallLeft::draw()
{
	glBegin(GL_LINES);
	glColor3f(0, 1, 1);
	glVertex2f(m_xPos, -1);
	glColor3f(1, 1, 0);
	glVertex2f(m_xPos, 1);
	glEnd();
}

void WallLeft::apply()
{
	//wall is only implemented for placement to the left of the particles
	//we also assume the Wall is the last of the forces array.
	//F = m * a
	//a = F/m
	float ks = 50.0;
	for (int i = 0; i < m_pVector.size(); i++)
	{
		//check for next x position, if it's inside the wall. stop the force
		if ((m_pVector[i]->m_Position[0] + m_pVector[i]->m_Velocity[0] * m_dt) <= m_xPos)
		{
			float posdif = ((m_pVector[i]->m_Position[0] + m_pVector[i]->m_Velocity[0] * m_dt) - m_xPos);
			float veldif = (m_pVector[i]->m_Velocity[0]);
			m_pVector[i]->m_Force[0] = -ks*(posdif);
		}
	}
	//m_p1->m_Velocity += g*0.1;
}