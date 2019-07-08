/***************************************************************************

file : driver_cruise.cpp
description : user module for CyberFollow

***************************************************************************/

/*
WARNING !

DO NOT MODIFY CODES BELOW!
*/

#ifdef _WIN32
#include <windows.h>
#endif

#include "driver_lead.h"
#include<iostream>
using  std::cout;
double constrain(double lowerBoundary, double upperBoundary, double input);
using  std::endl;
static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm, float DistanceFromStart, int laps);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);
static int num = 0;
static int num2 = 0;
double getError(float mid[][2]);
double distance(float a, float b);
int sgn(float x);//
const int topGear = 6;
// Module Entry Point
extern "C" int driver_lead(tModInfo *modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_lead";	// name of the module (short).
	modInfo[0].desc = "leader module for CyberFollow";	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId = 0;
	modInfo[0].index = 0;
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void *pt)
{
	tLeaderItf *itf = (tLeaderItf *)pt;
	itf->userDriverGetParam = userDriverGetParam;
	itf->userDriverSetParam = userDriverSetParam;
	return 0;
}

/*
WARNING!

DO NOT MODIFY CODES ABOVE!
*/

/*
define your variables here.
following are just examples
*/
static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm, _DistanceFromStart;
static int _gearbox, _laps;
double expectedSpeed;
double curSpeedErr;
void updateGear(int *cmdGear);													//
																				// Function constrain:															//
																				//		Given input, outputs value with boundaries.								//
double constrain(double lowerBoundary, double upperBoundary, double input);

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm, float DistanceFromStart, int laps) {
	/* write your own code here */

	for (int i = 0; i< 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
	_DistanceFromStart = DistanceFromStart;
	_laps = laps;

	printf("speed %f DFS %f lap %d 10m far target(%f, %f)\n", _speed, _DistanceFromStart, _laps, _midline[10][0], _midline[10][1]);

}

static float ki, k; //定义变量

static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {
	/* write your own code here */
	/*******************************路径设计********************************/
	switch (_laps)
	{
	case 1: ki += 0.01 * PI; num++; break;           //第一圈正常行驶
	case 2: ki += 0.01 * PI; num2++; break; //第二圈曲线行驶
	case 3: ki = 0;//第三圈的时候进行一定的保护
	default:break;
	}
	k = sin(ki) / 2.0;

	/*******************************车辆控制********************************/
	curSpeedErr = expectedSpeed - _speed;
	if (num < 350)
	{
		*cmdSteer = (_yaw - 8 * atan2(_midline[30][0] + 0.7*k * _width, _midline[30][1])) / 3.14;//设定舵机方向
		expectedSpeed = 300;
		curSpeedErr = expectedSpeed - _speed;
		if (_speed < expectedSpeed)
		{
			*cmdAcc = 1;
			*cmdBrake = 0;

		}
		else
		{
			*cmdBrake = 0;
			*cmdAcc = 0;
		}
	}
	else if (num<1100)
	{
		*cmdSteer = (_yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / 3.14;//设定舵机方向
		expectedSpeed = 100;
		curSpeedErr = expectedSpeed - _speed;
		if (curSpeedErr > 0)
		{

			if (abs(*cmdSteer)<0.7&abs(*cmdSteer)>0.15)
			{

				*cmdAcc = constrain(0.0, 1.0, 0.005 * curSpeedErr);
				*cmdBrake = 0;
			}
			else if (abs(*cmdSteer)>0.7)
			{
				*cmdAcc = 0.004;
				*cmdBrake = 0;
			}
			else
			{
				*cmdAcc = 1;
				*cmdBrake = 0;
			}

		}
		else if (curSpeedErr < 0)
		{
			*cmdBrake = constrain(0.0, 1.0, -0.05 * curSpeedErr*0.56);
			*cmdAcc = 0;
		}
	}
	else if (num < 2500)
	{
		expectedSpeed = 200;
		if (num / 200 % 4 == 0)
		{
			*cmdAcc = 0;
			*cmdBrake = 1;
			*cmdSteer = (2 * _yaw - 8 * atan2(_midline[30][0] - 0.3*_width, _midline[30][1])) / 3.14;
		}
		else if (num / 200 % 4 == 1)
		{
			*cmdAcc = 0.5;
			*cmdBrake = 0;
			*cmdSteer = (2 * _yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / 3.14;
		}
		else if (num / 200 % 4 == 2)
		{
			*cmdSteer = (2 * _yaw - 8 * atan2(_midline[30][0] + 0.4*_width, _midline[30][1])) / 3.14;
			*cmdAcc = 0.5;
			*cmdBrake = 0;
		}
		else
		{
			*cmdAcc = 0.5;
			*cmdBrake = 0;
			*cmdSteer = (2 * _yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / 3.14;
		}
	}
	else if (num < 2600)
	{
		*cmdSteer = 1;
		*cmdAcc = 0.5;
		*cmdBrake = 0.5;
	}
	else {
		*cmdSteer = constrain(-1, 1, 2 * _yaw - atan(65 * getError(_midline) / _speed));
		
		expectedSpeed = 100;
		curSpeedErr = expectedSpeed - _speed;
		if (curSpeedErr > 0)
		{

			if (abs(*cmdSteer) < 0.7&abs(*cmdSteer) > 0.15)
			{

				*cmdAcc = constrain(0.0, 1.0, 0.005 * curSpeedErr);
				*cmdBrake = 0;
			}
			else if (abs(*cmdSteer) > 0.7)
			{
				*cmdAcc = 0.004;
				*cmdBrake = 0;
			}
			else
			{
				*cmdAcc = 1;
				*cmdBrake = 0;
			}
		}
		else if (curSpeedErr < 0)
		{
			*cmdBrake = constrain(0.0, 1.0, -0.05 * curSpeedErr*0.56);
			*cmdAcc = 0;
		}
		}
	


	updateGear(cmdGear);
	//printf("lap %d \n", _laps);
	//printf("jishu %d \n", num);

}
void updateGear(int *cmdGear)
{
	if (_gearbox == 1)
	{
		if (_speed >= 60.2 && topGear >1)
		{
			*cmdGear = 2;
		}
		else
		{
			*cmdGear = 1;
		}
	}
	else if (_gearbox == 2)
	{
		if (_speed <= 42.6)
		{
			*cmdGear = 1;
		}
		else if (_speed >= 106.6 && topGear >2)
		{
			*cmdGear = 3;
		}
		else
		{
			*cmdGear = 2;
		}
	}
	else if (_gearbox == 3)
	{
		if (_speed <= 90.1)
		{
			*cmdGear = 2;
		}
		else if (_speed >= 146.1 && topGear >3)
		{
			*cmdGear = 4;
		}
		else
		{
			*cmdGear = 3;
		}
	}
	else if (_gearbox == 4)
	{
		if (_speed <= 130.1)
		{
			*cmdGear = 3;
		}
		else if (_speed >= 187.9 && topGear >4)
		{
			*cmdGear = 5;
		}
		else
		{
			*cmdGear = 4;
		}
	}
	else if (_gearbox == 5)
	{
		if (_speed <= 171.7)
		{
			*cmdGear = 4;
		}
		else if (_speed >= 234.3 && topGear >5)
		{
			*cmdGear = 6;
		}
		else
		{
			*cmdGear = 5;
		}
	}
	else if (_gearbox == 6)
	{
		if (_speed <= 217.8)
		{
			*cmdGear = 5;
		}
		else
		{
			*cmdGear = 6;
		}
	}
	else
	{
		*cmdGear = 1;
	}
}

double constrain(double lowerBoundary, double upperBoundary, double input)
{
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
	else
		return input;
}


double getError(float mid[][2])
{
	double minerror = 10000;
	for (int i = 0; i < 200; i++)
	{
		if (distance(mid[i][0], mid[i][1]) < minerror)
			minerror = distance(mid[i][0], mid[i][1])*sgn(mid[i][0]);
	}
	return minerror;
}

double distance(float a, float b)
{
	return sqrt(a*a + b * b);
}

int sgn(float x)
{
	if (x > 0)
		return 1;
	else
		return -1;
}
