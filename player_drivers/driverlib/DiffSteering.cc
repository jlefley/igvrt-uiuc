/*
 *  DiffSteering.cc
 *  Differential Steering
 *
 */

#include "DiffSteering.h"

DiffSteering::DiffSteering(double init_x, double init_y, double init_a, double init_vr, double init_vl, double init_proc_var, double init_meas_var, double init_axle_len)
	: kf_xy(4), kf_a(4)
{
	nf_x = init_x;
	nf_y = init_y;
	nf_a = init_a;

	proc_var = init_proc_var;
	meas_var = init_meas_var;
	axle_len = init_axle_len;

	matrix<double> x(4,1);

	x(0,0) = init_x;
	x(1,0) = init_y;
	x(2,0) = init_vr;
	x(3,0) = init_vl;

	kf_xy.setx(x);

	x(0,0) = init_a;
	x(1,0) = 0;
	x(2,0) = init_vr;
	x(3,0) = init_vl;

	kf_a.setx(x);

	identity_matrix<double> I(4,4);
	matrix<double> H(4,4);
	
	H = I;
	H(0,0) = H(1,1) = 0;

	kf_xy.setQ(I*proc_var);
	kf_xy.setR(I*meas_var);
	kf_xy.setH(H);

	kf_a.setQ(I*proc_var);
	kf_a.setR(I*meas_var);
	kf_a.setH(H);
}

void DiffSteering::update(double dt, double vr_target, double vl_target, double vr_meas, double vl_meas)
{
	update_kalman(dt,vr_target,vl_target,vr_meas,vl_meas);
	update_unfiltered(dt,vr_target,vl_target,vr_meas,vl_meas);
}

void DiffSteering::update_kalman(double dt, double vr_target, double vl_target, double vr_meas, double vl_meas)
{
	matrix<double> z(4,1);
	matrix<double> u(4,1);
	matrix<double> x_xy;
	matrix<double> x_a;
	identity_matrix<double> I(4,4);
	matrix<double> A(4,4);
	double a_norm = 0;

	// update a
	x_a = kf_a.getx();

	z(0,0) = 0;
	z(1,0) = 0;
	z(2,0) = vr_meas;
	z(3,0) = vl_meas;

	u(0,0) = 0;
	u(1,0) = 0;
	u(2,0) = vr_target - x_a(2,0);
	u(3,0) = vl_target - x_a(3,0);

	A = I;
	A(0,2) = dt/axle_len;
	A(0,3) = (-1)*A(0,2);
	A(1,2) = 1/axle_len;
	A(1,3) = (-1)*A(1,2);
	A(1,1) = 0;
	kf_a.setA(A);

	kf_a.update(u,z);

	// update xy
	x_a = kf_a.getx();
	a_norm = norm_ang(x_a(0,0));
	x_xy = kf_xy.getx();

	z(0,0) = 0;
	z(1,0) = 0;
	z(2,0) = vr_meas;
	z(3,0) = vl_meas;

	u(0,0) = 0;
	u(1,0) = 0;
	u(2,0) = vr_target - x_xy(2,0);
	u(3,0) = vl_target - x_xy(3,0);

	A = I;
	A(0,2) = A(0,3) = 0.5*dt*cos(a_norm);
	A(1,2) = A(1,3) = 0.5*dt*sin(a_norm);
	kf_xy.setA(A);

	kf_xy.update(u,z);
}

void DiffSteering::update_unfiltered(double dt, double vr_target, double vl_target, double vr_meas, double vl_meas)
{
	double a_norm = 0;
	double D = 0;

	nf_a += (vr_meas - vl_meas)*(dt/axle_len);

	a_norm = norm_ang(nf_a);
	D = (vr_meas + vl_meas)*(dt/2);

	nf_x += D*cos(a_norm);
	nf_y += D*sin(a_norm);
}

double DiffSteering::norm_ang(double ang) const
{
	/**************************
	if(ang > (2*M_PI))
	{
		while(ang > (2*M_PI))
		{
			ang -= (2*M_PI);
		}
	}
	else if(ang < ((-2)*pi))
	{
		while(ang  < ((-2)*M_PI))
		{
			ang += (2*M_PI);
		}
	}
	**********************************/

	/**********************************
	ang = ang - ((2*M_PI)*floor(ang / (2*M_PI)));
	***********************************/

	return ang;
}

double DiffSteering::x() const
{
	matrix<double> x_xy = kf_xy.getx();
	return x_xy(0,0);
}

double DiffSteering::y() const
{
	matrix<double> x_xy = kf_xy.getx();
	return x_xy(1,0);
}

double DiffSteering::a() const
{
	matrix<double> x_a = kf_a.getx();
	return x_a(0,0);
}

double DiffSteering::vr() const
{
	matrix<double> x_xy = kf_xy.getx();
	return x_xy(2,0);
}

double DiffSteering::vl() const
{
	matrix<double> x_xy = kf_xy.getx();
	return x_xy(3,0);
}

double DiffSteering::x_nofilt() const
{
	return nf_x;
}

double DiffSteering::y_nofilt() const
{
	return nf_y;
}

double DiffSteering::a_nofilt() const
{
	return nf_a;
}


