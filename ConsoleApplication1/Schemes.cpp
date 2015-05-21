#include "Particle.h"
#include <vector>
#include <algorithm>
#include "Gravity.h"
#include <iostream>
#include <list>
#include "linearSolver.h"
using namespace std;

void CalculateForces(std::vector<Force*> forces, std::vector<Constraint*> constraints, std::vector<Particle*> pVector);

void Euler(std::vector<Force*> forces, std::vector<Constraint*> constraints, std::vector<Particle*> pVector, float dt)
{
	//euler
	CalculateForces(forces, constraints, pVector);
	for (int i = 0; i < pVector.size(); i++)
	{
		pVector[i]->m_Velocity += ((pVector[i]->m_Force * dt) / pVector[i]->m_mass);
		pVector[i]->m_Position += pVector[i]->m_Velocity * dt;
		//pVector[i]->m_Velocity = DAMP*pVector[i]->m_Velocity + Vec2f(RAND, RAND) * 0.005;
	}
}

void MidPoint(std::vector<Force*> forces, std::vector<Constraint*> constraints, std::vector<Particle*> pVector, float dt)
{
	//midpoint
	std::vector<Vec2f> posTemp;

	//save original position
	for (int i = 0; i < pVector.size(); i++)
	{
		posTemp.push_back(pVector[i]->m_Position);
	}
	//apply forces for midpoint calculation
	CalculateForces(forces, constraints, pVector);
	//calculate midpoint position
	for (int i = 0; i < pVector.size(); i++)
	{
		pVector[i]->m_Velocity += ((pVector[i]->m_Force * dt) / pVector[i]->m_mass);
		pVector[i]->m_Position += pVector[i]->m_Velocity * (dt / 2);
	}
	CalculateForces(forces, constraints, pVector);
	//use midpoint forces on the original position
	for (int i = 0; i < pVector.size(); i++)
	{
		pVector[i]->m_Velocity += ((pVector[i]->m_Force * dt) / pVector[i]->m_mass);
		pVector[i]->m_Position = posTemp[i] + pVector[i]->m_Velocity*dt;
	}
}

void RungeKutta(std::vector<Force*> forces, std::vector<Constraint*> constraints, std::vector<Particle*> pVector, float dt)
{
	//runge-kutta
	std::vector<Vec2f> posTemp;
	std::vector<Vec2f> k1;
	std::vector<Vec2f> k2;
	std::vector<Vec2f> k3;
	std::vector<Vec2f> k4;

	//save original position and initialize the 4 k's
	for (int i = 0; i < pVector.size(); i++)
	{
		posTemp.push_back(pVector[i]->m_Position);
		k1.push_back(Vec2f(0.0, 0.0));
		k2.push_back(Vec2f(0.0, 0.0));
		k3.push_back(Vec2f(0.0, 0.0));
		k4.push_back(Vec2f(0.0, 0.0));
	}
	//calculate forces
	CalculateForces(forces, constraints, pVector);
	//calculate midpoint position and save k1
	for (int i = 0; i < pVector.size(); i++)
	{
		pVector[i]->m_Velocity += ((pVector[i]->m_Force * dt) / pVector[i]->m_mass);
		k1[i] = pVector[i]->m_Velocity;
		pVector[i]->m_Position += pVector[i]->m_Velocity * (dt / 2);
	}
	CalculateForces(forces, constraints, pVector);
	//use midpoint forces to calculate k3 and save k2
	for (int i = 0; i < pVector.size(); i++)
	{
		pVector[i]->m_Velocity += ((pVector[i]->m_Force * dt) / pVector[i]->m_mass);
		k2[i] = pVector[i]->m_Velocity;
		pVector[i]->m_Position = posTemp[i] + pVector[i]->m_Velocity * (dt / 2);
	}
	CalculateForces(forces, constraints, pVector);
	//save k3 and set position for k4
	for (int i = 0; i < pVector.size(); i++)
	{
		pVector[i]->m_Velocity += ((pVector[i]->m_Force * dt) / pVector[i]->m_mass);
		k3[i] = pVector[i]->m_Velocity;
		pVector[i]->m_Position = posTemp[i] + pVector[i]->m_Velocity * dt;
	}
	CalculateForces(forces, constraints, pVector);
	//save k4 and use it to calculate the actual position
	for (int i = 0; i < pVector.size(); i++)
	{
		pVector[i]->m_Velocity += ((pVector[i]->m_Force * dt) / pVector[i]->m_mass);
		k4[i] = pVector[i]->m_Velocity;
		pVector[i]->m_Position = posTemp[i] + (dt / 6)*(k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
	}
}

std::vector<Vec2f> previousPos;
void Verlet(std::vector<Force*> forces, std::vector<Constraint*> constraints, std::vector<Particle*> pVector, float dt)
{
	//verlet
	CalculateForces(forces, constraints, pVector);
	if (previousPos.empty())
	{
		for (int i = 0; i < pVector.size(); i++)
		{
			pVector[i]->m_Velocity += ((pVector[i]->m_Force * dt) / pVector[i]->m_mass);
			pVector[i]->m_Position += pVector[i]->m_Velocity * dt + 0.5 * pVector[i]->m_Force / pVector[i]->m_mass * dt * dt;
			//pVector[i]->m_Velocity = DAMP*pVector[i]->m_Velocity + Vec2f(RAND, RAND) * 0.005;
		}
	}
	else
	{
		for (int i = 0; i < pVector.size(); i++)
		{
			pVector[i]->m_Position = 2 * pVector[i]->m_Position - previousPos[i] + pVector[i]->m_Force/pVector[i]->m_mass * dt * dt;
		}
	}

	CalculateForces(forces, constraints, pVector);
	previousPos.clear();
	//save previous position
	for (int i = 0; i < pVector.size(); i++)
	{
		previousPos.push_back(pVector[i]->m_Position);
	}
	
}