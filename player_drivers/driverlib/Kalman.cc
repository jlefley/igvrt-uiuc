/*
 *  Kalman.cc
 *  Kalman filter
 *
 */

#include "Kalman.h"

Kalman::Kalman(unsigned int new_n)
	: x(new_n,1), z(new_n,1), u(new_n,1), A(new_n,new_n), P(new_n,new_n), B(new_n,new_n), Q(new_n,new_n), R(new_n,new_n), H(new_n,new_n)
{
	n = new_n;

	identity_matrix<double> I(n,n);
	zero_matrix<double> Z(n,n);
	zero_matrix<double> Zvec(n,1);
	
	x = Zvec;
	z = Zvec;
	u = Zvec;

	A = I;
	P = Z;
	B = I;
	Q = Z;
	R = Z;
	H = I;
	
}

const matrix<double> & Kalman::getx() const
{
	return x;
}

void Kalman::setx(const matrix<double> & new_x)
{
	x = new_x;
}

void Kalman::setz(const matrix<double> & new_z)
{
	z = new_z;
}

void Kalman::setu(const matrix<double> & new_u)
{
	u = new_u;
}

void Kalman::setA(const matrix<double> & new_A)
{
	A = new_A;
}

void Kalman::setP(const matrix<double> & new_P)
{
	P = new_P;
}

void Kalman::setB(const matrix<double> & new_B)
{
	B = new_B;
}

void Kalman::setQ(const matrix<double> & new_Q)
{
	Q = new_Q;
}

void Kalman::setR(const matrix<double> & new_R)
{
	R = new_R;
}

void Kalman::setH(const matrix<double> & new_H)
{
	H = new_H;
}


void Kalman::update(const matrix<double> & new_u, const matrix<double> & new_z)
{
	setu(new_u);
	setz(new_z);

	predict();
	correct();
}

bool Kalman::matrixInverse(const matrix<double> & input, matrix<double> & inverse)
{
	inverse = inverse*0;

 	typedef permutation_matrix<std::size_t> pmatrix;
 	// create a working copy of the input
 	matrix<double> Atemp(input);
 	// create a permutation matrix for the LU-factorization
 	pmatrix pm(Atemp.size1());

 	// perform LU-factorization
 	int res = lu_factorize(Atemp,pm);
        if( res != 0 ) return false;

 	// create identity matrix of "inverse"
 	inverse.assign(identity_matrix<double>(Atemp.size1()));

 	// backsubstitute to get the inverse
 	lu_substitute(Atemp, pm, inverse);
	
 	return true;
}

void Kalman::predict()
{
	matrix<double> temp1(n,1);
	matrix<double> temp2(n,1);
	
	// predict x
	// x = A*s + B*u;
	axpy_prod(A,x,temp1,true);
	axpy_prod(B,u,temp2,true);
	x = temp1 + temp2;

	// predict P
	// P = A*P*A' + Q;
	axpy_prod(A,P,temp1,true);
	axpy_prod(temp1,trans(A),temp2,true);
	P = temp2 + Q;
}

void Kalman::correct()
{
	matrix<double> K(n,n);
	matrix<double> temp1(n,1);
	matrix<double> temp2(n,1);

	// calc K (Kalman gain factor)
	// K = P*H'*inv(H*P*H' + R);
	axpy_prod(H,P,temp1,true);
	axpy_prod(temp1,trans(H),temp2,true);
	temp1 = temp2 + R;

	matrixInverse(temp1,temp2);

	axpy_prod(P,trans(H),temp1,true);
	axpy_prod(temp1,temp2,K,true);

	// correct x
	// x = x + K*(z - H*x);
	axpy_prod(H,x,temp1,true);
	temp1 = z - temp1;
	axpy_prod(K,temp1,temp2,true);
	x = x + temp2;

	// correct P
	// P = P - K*H*P;
	axpy_prod(K,H,temp1,true);
	axpy_prod(temp1,P,temp2,true);
	P = P - temp2;
}



