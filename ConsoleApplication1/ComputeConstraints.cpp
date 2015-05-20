#include "Particle.h"
#include <vector>
#include <algorithm>
#include "Gravity.h"
#include <iostream>
#include <list>
#include "linearSolver.h"
using namespace std;


void ComputeConstraint(std::vector<Particle*> pVector, std::vector<Constraint*> constraints)
{
	if (constraints.empty())
	{
		return;
	}
	//strength and damping factor
	float ks = 60.0f;
	float kd = 5.0f;

	//initialize the M and W matrix, M is a matrix of size 2n*2n, where n is the number of particles.
	//on the identity of M is the mass of the particles, so it will look like [m_p1, m_p1, m_p2, m_p2, ..., m_pn, m_pn]
	//it is size 2n*2n because it's 2D, in 3D it would be 3n*en
	//the matrix W is the same as M except it has inverse mass of particles.
	//we also create the vectors q, qDot and Q
	//q is the particle positions, qDot the velocity position and Q is the current forces on the particles.
	vector<vector<float>> M = vector<vector<float>>(pVector.size() * 2, vector<float>(pVector.size() * 2, 0));
	vector<vector<float>> W = vector<vector<float>>(pVector.size() * 2, vector<float>(pVector.size() * 2, 0));
	vector<float>q = vector<float>(pVector.size() * 2);
	vector<float>qDot = vector<float>(pVector.size() * 2);
	vector<float>Q = vector<float>(pVector.size() * 2);
	for (int i = 0; i < pVector.size(); i++)
	{
		M[i * 2 + 0][i * 2 + 0] = pVector[i]->m_mass;
		M[i * 2 + 1][i * 2 + 1] = pVector[i]->m_mass;
		W[i * 2 + 0][i * 2 + 0] = 1 / pVector[i]->m_mass;
		W[i * 2 + 1][i * 2 + 1] = 1 / pVector[i]->m_mass;
		q[i * 2 + 0] = pVector[i]->m_Position[0];
		q[i * 2 + 1] = pVector[i]->m_Position[1];
		qDot[i * 2 + 0] = pVector[i]->m_Velocity[0];
		qDot[i * 2 + 1] = pVector[i]->m_Velocity[1];
		Q[i * 2 + 0] = pVector[i]->m_Force[0];
		Q[i * 2 + 1] = pVector[i]->m_Force[1];
	}

	//create the C and CDot vectors also the Jacobian Matrices and JacobianDot matrices.
	//they are calculate in their corresponding constraint classes and put into the vector here.
	//they are also calculated in their corresponding classes. they are placed int he jacobian matrix for the correct particle.
	//the JT is the transpose of J.

	//Loop over the constraints, letting each evaluate its own portion of C, CDot, J and JDot.Each
	//constraint points to one or more matrix blocks that receive its contributions to the global
	//matrices. source*
	vector<float>C = vector<float>(constraints.size());
	vector<float>CDot = vector<float>(constraints.size());
	vector<vector<float>> J = vector<vector<float>>(constraints.size(), vector<float>(pVector.size() * 2));
	vector<vector<float>> JDot = vector<vector<float>>(constraints.size(), vector<float>(pVector.size() * 2));
	vector<vector<float>> JT = vector<vector<float>>((pVector.size() * 2), vector<float>(constraints.size()));
	for (int i = 0; i < constraints.size(); i++)
	{
		C[i] = constraints[i]->getC();
		CDot[i] = constraints[i]->getCDot();
		vector<Vec2f> jac = constraints[i]->getJacobian();
		vector<Vec2f> jacDot = constraints[i]->getJacobianDot();
		vector<Particle*> particles = constraints[i]->getParticles(); 
		for (int j = 0; j < particles.size(); j++)
		{
			for (int k = 0; k < 2; k++)
			{
				J[i][particles[j]->getParticleID() * 2 + k] = jac[j][k];
				JT[particles[j]->getParticleID() * 2 + k][i] = jac[j][k];
				JDot[i][particles[j]->getParticleID() * 2 + k] = jacDot[j][k];
			}
		}
	}

	//We multiply J and W, for this we use the vector multiplier in LinearSolver.cpp
	//We do the same for the resulting JW and JT
	//Form the right - hand - side vector of equation 11.
	//(equation 11: JWJTLamba = -JDotqDot - JWQ -ksC - kdCDot)*
	vector<vector<float>>JW = vector<vector<float>>(constraints.size(), vector<float>(pVector.size() * 2));
	vector<vector<float>>JWJT = vector<vector<float>>(constraints.size(), vector<float>(pVector.size() * 2));
	JW = VectorMultiplication(J, W);
	JWJT = VectorMultiplication(JW, JT);

	//We know that J*W*JT*Lambda = -JDot*qDot-J*W*Q - ks*C - kd*CDot and we want to know Lambda.
	//We calculate -JDot*qdot and make it negative. Than we calculate JWQ
	vector<float>  JDotqDot = vector<float>(qDot.size());
	vector<float> JWQ = vector<float>(Q.size());
	JDotqDot = VectorMultiplication(JDot, qDot);
	JDotqDot = VectorScalarMultiplication(JDotqDot, -1);
	JWQ = VectorMultiplication(JW, Q);

	//We than calculate ks*C and kd*CDot
	vector<float> CStrength = vector<float>(constraints.size());
	vector<float> CDotDamping = vector<float>(constraints.size());
	CStrength = VectorScalarMultiplication(C, ks);
	CDotDamping = VectorScalarMultiplication(CDot, kd);

	//finnaly we subtract everything from JDot*qDot, which was negative so we get the correct result for JWJTLambda
	vector<float> JWJTLambda = vector<float>(constraints.size());
	JWJTLambda = VectorSubtraction(JDotqDot, JWQ);
	JWJTLambda = VectorSubtraction(JWJTLambda, CStrength);
	JWJTLambda = VectorSubtraction(JWJTLambda, CDotDamping);

	//to find Lambda, we use the ConjGrad provide, for this JWJTLambda needs to be a double[]
	double* JWJTLambdaDouble = new double[JWJTLambda.size()];
	for (int i = 0; i < JWJTLambda.size(); i++)
	{
		JWJTLambdaDouble[i] = JWJTLambda[i];
	}

	//We use JWJT as the implicit matrix, so we can find the value for Lambda.
	implicitMatrix *ImJWJT = new implicitMatrix(&JWJT);
	double* lambda = new double[constraints.size()];

	//stepsize is 100, which is standard
	//Invoke the conjugate gradient solver to obtain the Lagrange multiplier vector, lambda.*
	int stepSize = 100;
	ConjGrad(constraints.size(), ImJWJT, lambda, JWJTLambdaDouble, 1e-30f, &stepSize);

	//turn the resulting Lambda back into a float vector.
	vector<float>lambdaFloat = vector<float>(constraints.size());
	for (unsigned i = 0; i < constraints.size(); i++)
	{
		lambdaFloat[i] = lambda[i];
	}

	//We can now calculate the resulting force QHat, we do this by multiplying Lambda with JT
	//Multiply lambda by JT to obtain the global constraint force vector, and scatter this force to the
	//particles. source*
	vector<float>Qhat = vector<float>(constraints.size());
	Qhat = VectorMultiplication(JT, lambdaFloat);

	//we apply the forces to the force accumelator.
	for (int i = 0; i < pVector.size(); i++)
	{
		for (int j = 0; j < 2; j++)
		{
			pVector[i]->m_Force[j] += Qhat[2 * i + j];
		}
	}
	//source* -> notesf.pdf (summary)
}
