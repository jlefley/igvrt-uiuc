/*
 *  Kalman1D.h
 *  Kalman1D filter
 *
 */

#ifndef KALMAN1D_H
#define KALMAN1D_H

//c++ libs

using namespace std;

class Kalman1D
{
public:

	Kalman1D();
	
	double getx() const;

	void setx(double new_x);
	void setz(double new_z);
	void setu(double new_u);
	void setA(double new_A);
	void setP(double new_P);
	void setB(double new_B);
	void setQ(double new_Q);
	void setR(double new_R);
	void setH(double new_H);

	void update(double new_u, double new_z);

protected:

	double x; // state vector
	double z; // observation vector
	double u; // control vector

	double A; // state transition matrix
	double P; // covariance matrix of state estimate
	double B; // control matrix
	double Q; // process covariance matrix
	double R; // measurement covariance matrix
	double H; // observation matrix

	void predict();

	void correct();

};

#endif



