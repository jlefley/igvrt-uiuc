/*
 *  Roomba.h
 *  roomba
 *
 */

#ifndef WORLDVIEW_H
#define WORLDVIEW_H


//c++ libs
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

//c libs
#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>



using namespace std;

class WorldView
{
public:
	WorldView(CvMat * homographic_mat);
	
	void load(const ImageView & iv);
	const CvMat& Obstacles();
	const CvMat& Lines();
	
	const CvPoint& RobotPos();
	
private:
	const CvMat* H;
	struct Point_Obj_Prime
	{
	  int x_p;
	  int y_p;
	  int z_p;
	};

	Point_Obj_Prime transformPointHomographic(int x, int y);
	/* Ian */ CvPoint transformPointStereo(CvPoint point);
	
	/* fill in missing line segments */
	/* Koushik */ void extrapolateLines();
};

