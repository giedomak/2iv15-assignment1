#include "AngularForce.h"
#include <glut.h>
#include <iostream>
#include "linearSolver.h"
using namespace std;

//Spring system force calculations between 2 particles
//ks = strength
//kd = damping
//dist = resting distance
AngularForce::AngularForce(Particle *p1, Particle * p2, Particle * p3, double angle, double ks, double kd) :
m_p1(p1), m_p2(p2), m_p3(p3), m_angle(angle), m_ks(ks), m_kd(kd) {}


void AngularForce::draw()
{
	//m_p1 is connected with m_p2 and m_p3 also with m_p2.
	//so m_p2 will be the joint where the angle is calculated
	Vec2f posdif12 = (m_p1->m_Position - m_p2->m_Position);
	Vec2f posdif32 = (m_p3->m_Position - m_p2->m_Position);
	// length of both vectors
	float length12 = sqrt(posdif12[0] * posdif12[0] + posdif12[1] * posdif12[1]);
	float length32 = sqrt(posdif32[0] * posdif32[0] + posdif32[1] * posdif32[1]);

	//determine the ratio of the x and y
	posdif12 = posdif12 / length12;
	posdif32 = posdif32 / length32;
	
	//we want to position the line 1/3 of the way of both lines.
	float drawpos12 = length12 / 3;
	float drawpos32 = length12 / 3;

	//determine the point on the line between 1 and 2, and pick it 1/3 of the way from point 2.
	Vec2f on12 = Vec2f(m_p2->m_Position[0] + posdif12[0] * drawpos12, m_p2->m_Position[1] + posdif12[1] * drawpos12);
	Vec2f on32 = Vec2f(m_p2->m_Position[0] + posdif32[0] * drawpos32, m_p2->m_Position[1] + posdif32[1] * drawpos32);

	glBegin(GL_LINES);
	glColor3f(0.0, 1.0, 1.0);
	glVertex2f(on12[0], on12[1]);
	glVertex2f(on32[0], on32[1]);
	glEnd();

}

void AngularForce::apply()
{
	Vec2f posdif12 = (m_p1->m_Position - m_p2->m_Position);
	Vec2f posdif32 = (m_p3->m_Position - m_p2->m_Position);
	// length of both vectors
	float length12 = sqrt(posdif12[0] * posdif12[0] + posdif12[1] * posdif12[1]);
	float length32 = sqrt(posdif32[0] * posdif32[0] + posdif32[1] * posdif32[1]);
	float length12Sqr = posdif12[0] * posdif12[0] + posdif12[1] * posdif12[1];
	float length32Sqr = posdif32[0] * posdif32[0] + posdif32[1] * posdif32[1];

	// dot product  
	float dot = (posdif12[0] * posdif32[0] + posdif12[1] * posdif32[1]);

	// square of cosine of the needed angle    
	float cosSqr = dot * dot / length12Sqr / length32Sqr;

	// this is a known trigonometric equality:
	// cos(alpha * 2) = [ cos(alpha) ]^2 * 2 - 1
	float cos2 = 2 * cosSqr - 1;

	// Here's the only invocation of the heavy function.
	// It's a good idea to check explicitly if cos2 is within [-1 .. 1] range

	const float pi = 3.141592f;

	float alpha2 =
		(cos2 <= -1) ? pi :
		(cos2 >= 1) ? 0 :
		acosf(cos2);

	float rslt = alpha2 / 2;

	float rs = rslt * 180. / pi;


	// Now revolve the ambiguities.
	// 1. If dot product of two vectors is negative - the angle is definitely
	// above 90 degrees. Still we have no information regarding the sign of the angle.

	// NOTE: This ambiguity is the consequence of our method: calculating the cosine
	// of the double angle. This allows us to get rid of calling sqrt.

	if (dot < 0)
		rs = 180 - rs;

	// 2. Determine the sign. For this we'll use the Determinant of two vectors.

	//rs = (int)floor(rs + 0.5);
	float det = (posdif12[0] * posdif32[1] - posdif12[1] * posdif32[0]);
	if (det < 0)
	{
		rs = -rs;
	}

	//the angle is from -180 to +180, there are 2 points where it wants to go
	float restAngle = 0;
	if (rs < 0)
	{
		restAngle = rs + m_angle;
	}
	else
	{
		restAngle = rs - m_angle;
	}


	Vec2f veldif = ((m_p1->m_Velocity+m_p3->m_Velocity) - m_p2->m_Velocity);
	//Vec2f veldif32 = (m_p3->m_Velocity - m_p2->m_Velocity);
	float dotProduct = vecDotNew(veldif, (posdif12+posdif32));

	cout << "rest angle: " << restAngle << endl;
	//m_p1->m_Force -= (m_ks*restAngle);
	//m_p1->m_Force += Vec2f((posdif12 / length12)*(m_ks*restAngle));
	//m_p3->m_Force[0] -= ((m_ks*restAngle) / length32*posdif32[1]);
	//m_p3->m_Force[1] += ((m_ks*restAngle) / length32*posdif32[0]);
	//m_p1->m_Force -= Vec2f(((m_ks*restAngle) / (length12))*posdif12[1], 0);
	//m_p1->m_Force += Vec2f(((m_ks*restAngle) / (length12 * 2))*posdif12[1], -((m_ks*restAngle) / (length12 * 2))* posdif12[0]);
	//m_p2->m_Force -= Vec2f(posdif32[1] * (m_ks*restAngle) / (length32 * 2), -posdif32[0] * (m_ks*restAngle) / (length32 * 2));

	m_p1->m_Force[0] -= ((m_ks*restAngle) / length12*posdif12[1]);
	m_p1->m_Force[1] += ((m_ks*restAngle) / length12*posdif12[0]);
	m_p3->m_Force[0] += ((m_ks*restAngle) / length32*posdif32[1]);
	m_p3->m_Force[1] -= ((m_ks*restAngle) / length32*posdif32[0]);
}