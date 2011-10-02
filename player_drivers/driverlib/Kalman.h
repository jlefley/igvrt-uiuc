/*
 *  Kalman.h
 *  Kalman filter
 *
 */

#ifndef KALMAN_H
#define KALMAN_H

//c++ libs
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>

using namespace std;
using namespace boost::numeric::ublas;

class Kalman
{
public:

	Kalman(unsigned int new_n);
	
	const matrix<double> & getx() const;

	void setx(const matrix<double> & new_x);
	void setz(const matrix<double> & new_z);
	void setu(const matrix<double> & new_u);
	void setA(const matrix<double> & new_A);
	void setP(const matrix<double> & new_P);
	void setB(const matrix<double> & new_B);
	void setQ(const matrix<double> & new_Q);
	void setR(const matrix<double> & new_R);
	void setH(const matrix<double> & new_H);

	void update(const matrix<double> & new_u, const matrix<double> & new_z);

protected:
	unsigned int n;   // dimension of problem

	matrix<double> x; // state vector
	matrix<double> z; // observation vector
	matrix<double> u; // control vector

	matrix<double> A; // state transition matrix
	matrix<double> P; // covariance matrix of state estimate
	matrix<double> B; // control matrix
	matrix<double> Q; // process covariance matrix
	matrix<double> R; // measurement covariance matrix
	matrix<double> H; // observation matrix

	bool matrixInverse(const matrix<double> & input, matrix<double> & inverse);

	void predict();

	void correct();

};

#endif



