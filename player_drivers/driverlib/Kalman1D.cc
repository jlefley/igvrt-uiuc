/*
 *  Kalman.cc
 *  Kalman filter
 *
 */

#include "Kalman1D.h"

Kalman1D::Kalman1D()
{
	x = 0;
	z = 0;
	u = 0;

	A = 1;
	P = 0;
	B = 1;
	Q = 0;
	R = 0;
	H = 1;
	
}

double Kalman1D::getx() const
{
	return x;
}

void Kalman1D::setx(double new_x)
{
	x = new_x;
}

void Kalman1D::setz(double new_z)
{
	z = new_z;
}

void Kalman1D::setu(double new_u)
{
	u = new_u;
}

void Kalman1D::setA(double new_A)
{
	A = new_A;
}

void Kalman1D::setP(double new_P)
{
	P = new_P;
}

void Kalman1D::setB(double new_B)
{
	B = new_B;
}

void Kalman1D::setQ(double new_Q)
{
	Q = new_Q;
}

void Kalman1D::setR(double new_R)
{
	R = new_R;
}

void Kalman1D::setH(double new_H)
{
	H = new_H;
}


void Kalman1D::update(double new_u, double new_z)
{
	setu(new_u);
	setz(new_z);

	predict();
	correct();
}

void Kalman1D::predict()
{
	x = (A*x) + (B*u);

	P = (A*P*A) + Q;
}

void Kalman1D::correct()
{
	double K = (P*H) / ((H*P*H) + R);

	x = x + (K*(z - (H*x)));

	P = P - (K*H*P);
}



