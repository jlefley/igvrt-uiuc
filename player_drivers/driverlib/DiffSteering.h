/*
 *  DiffSteering.h
 *  Differential Steering
 *
 */

#ifndef DIFFSTEERING_H
#define DIFFSTEERING_H

#define _USE_MATH_DEFINES
#include <math.h>
#include "Kalman.h"

class DiffSteering
{
public:

	DiffSteering(double init_x, double init_y, double init_a, double init_vr, double init_vl, double init_proc_var, double init_meas_var, double init_axle_len);

	double x() const;
	double y() const;
	double a() const;
	double vr() const;
	double vl() const;
	double x_nofilt() const;
	double y_nofilt() const;
	double a_nofilt() const;

	void update(double dt, double vr_target, double vl_target, double vr_meas, double vl_meas);

protected:
	
	double nf_x;	// unfiltered x position
	double nf_y;	// unfiltered y position
	double nf_a;	// unfiltered angle

	Kalman kf_xy;	// kalman filter for xy position
	Kalman kf_a;	// kalman filter for angle

	double proc_var;
	double meas_var;
	double axle_len;

	void update_kalman(double dt, double vr_target, double vl_target, double vr_meas, double vl_meas);
	void update_unfiltered(double dt, double vr_target, double vl_target, double vr_meas, double vl_meas);
	double norm_ang(double ang) const;

};

#endif


