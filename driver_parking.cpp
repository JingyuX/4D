
/*
WARNING !

DO NOT MODIFY CODES BELOW!
*/

#include <iostream>


#ifdef _WIN32
#include <windows.h>
#endif

#include <math.h>
#include "driver_parking.h"
using std::cout;
using std::endl;
void getpointlinedist();
void getAngle();
float dist(float x1, float x2, float y1, float y2);
float getR(float x1, float x2, float x3, float y1, float y2, float y3);
static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_parking(tModInfo *modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_parking";	// name of the module (short).
	modInfo[0].desc = "user module for CyberParking";	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId = 0;
	modInfo[0].index = 0;
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void *pt)
{
	tUserItf *itf = (tUserItf *)pt;
	itf->userDriverGetParam = userDriverGetParam;
	itf->userDriverSetParam = userDriverSetParam;
	printf("OK!\n");
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
static float k; //����
static float _yaw, _yawrate, _speed, _acc, _width, _rpm, _lotX, _lotY, _lotAngle, _carX, _carY, _caryaw;
static int _gearbox, w = 0;
static bool _bFrontIn;
static float targX, targY, targdist, angle, pointlinedist;

static int count = 0;
static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm) {
	/* write your own code here */

	_lotX = lotX;
	_lotY = lotY;      //��λ���ĵ��������
	_lotAngle = lotAngle;//��λ���Գ���
	_bFrontIn = bFrontIn;
	_carX = carX;
	_carY = carY;     //������������
	_caryaw = caryaw; //�������Գ���
	float point[21][2];
	int i, j = 0;
	for (int i = 0; i< 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	k = getR(_midline[0][0], _midline[10][0], _midline[20][0], _midline[0][1], _midline[10][1], _midline[20][1]);
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
	if (k >-0.005)
	{
		targX = lotX + 5.5*cos(_lotAngle + 0.25);
		targY = lotY + 5.5*sin(_lotAngle + 0.25);
	}
	else
	{
		targX = lotX + 7.9*cos(_lotAngle + 0.25);
		targY = lotY + 7.9*sin(_lotAngle + 0.25);

	}
	if (cos(_lotAngle) >= 0) { w = -1; }
	else { w = 1; } //�жϳ�λ����



	targdist = sqrt((_carX - targX) * (_carX - targX) + (_carY - targY) * (_carY - targY));//����Ŀ��ͣ��׼����

	pointlinedist = w * (tan(_lotAngle)*(_carX - _lotX - 2.5*cos(_lotAngle)) +
		(_lotY - _carY + 2.5*sin(_lotAngle))) / (sqrt(tan(_lotAngle)*tan(_lotAngle) + 1));
	//�㵽ֱ�߾��빫ʽ

	if (fabs(_lotAngle - _caryaw) <= PI) { angle = _lotAngle - _caryaw; }
	else {
		if (_lotAngle - _caryaw >= 0)
			angle = _lotAngle - _caryaw - 2 * PI;   //����ת��angle�Ƕȣ��복λ�����غ�
		else                                        //ͨ��2PI������ʹת����С
			angle = _lotAngle - _caryaw + 2 * PI;
	}
}
static float dpointlinelist[5], avepointlinelist, dangle[5], aveangle;
static int back = 0;
static float speed;
static void userDriverSetParam(bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {
	/* write your own code here */
	getpointlinedist();
	getAngle();
	avepointlinelist = (dpointlinelist[0] + dpointlinelist[1] + dpointlinelist[2] + dpointlinelist[3] + dpointlinelist[4]) / 5;
	aveangle = (dangle[0] + dangle[1] + dangle[2] + dangle[3] + dangle[4]) / 5;
	if (!*bFinished) {
		if((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 0.01){                    //ͣס   
			cout << "5ͣ" << endl;
			*cmdSteer = -33.6*angle / PI- 2.16*(pointlinedist);
			*cmdBrake = 1.0; *cmdGear = -1; *cmdAcc = 0.0;
			if (fabs(_speed) < 0.2)  *bFinished = true;
		}
		else if (back) {                                                                                       //��ʼ����    
			cout << "4����" << endl;
			cout << k << endl;
			cout << (_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) << endl;
			if (k<-0.005)  //��λ1��2  ����λ��ֱ·�ϣ�
			{
				*cmdSteer = -20.5*angle / PI - 26.88*aveangle / PI - 1.404*(pointlinedist)-2.472*avepointlinelist;
			}
			else  //��λ3��4��5       ����λ������ϣ�
			{
				*cmdSteer = -21.09*angle / PI - 26.88*aveangle / PI- 1.404*(pointlinedist)-2.472*avepointlinelist;
			}

			if (fabs(_speed) >4.2*sqrt((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY)) + 5.15)
			{
				*cmdBrake = 0.2; *cmdGear = -1; *cmdAcc = 0;
			}//�����ٶȿ���
			else { *cmdBrake = 0.0; *cmdGear = -1; *cmdAcc = 0.55; }
		}
		else if (targdist<15) {                                                                                //��תƯ��ͣס
			if (targdist > 6) {
				cout << "2Ư��" << endl;
				*cmdSteer = -0.5*fabs(atan2(targX - _carX, targY - _carY));
				if (_speed<targdist) { *cmdAcc = 0.3; *cmdBrake = 0; }
				else { *cmdAcc = 0; *cmdBrake = 0.2; }
			}
			if (targdist < 6) {
				cout << "3Ư��ɲ��" << endl;
				*cmdSteer = (_caryaw - _lotAngle - 0.6) / PI;
				*cmdAcc = 0;
				*cmdBrake = 0.1*_speed + 0.05*targdist;

				if (_speed<2.05) { back = 1; }
			}
		}
		else if ((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 3500) {                //��һ����Χʱ�������ӵ�������·���
			cout << "1����" << endl;
			*cmdSteer = (_yaw - 3.98*atan2(_midline[20][0] - _width / 3, _midline[20][1])) / PI;
			*cmdGear = 1; *cmdAcc = 0.2; *cmdBrake = 0;
		}
		else {                                                                                                 //����·�ΰ�Ѳ�߷�ʽ��ʻ
			cout << "0����Ѳ��" << endl;
			*cmdAcc = 1;//���Ÿ�100%
			*cmdBrake = 0;//��ɲ��
			*cmdSteer = (_yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / PI;//�趨�������
			*cmdGear = 1;//��λʼ�չ�1
		}
		//printf("���� %f \n", k[0]);
	}
	if (*bFinished)
	{
		if ((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 0.05)
		{
			cout << "6ͣ����ɣ���ʼʻ��" << endl;
			*cmdSteer = 0; *cmdAcc = 1; *cmdBrake = 0; *cmdGear = 1;
		}
		else
		{
			cout << "7ʻ��" << endl;
			*cmdSteer = (_yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / PI;
			*cmdAcc = 1 - 0.006*fabs(*cmdSteer*_speed);//��ֹ��
			*cmdBrake = 0;
			*cmdGear = 1;
		}
	}
	if (*cmdAcc>1) { *cmdAcc = 1; }
	if (*cmdAcc<0) { *cmdAcc = 0; }
	if (*cmdBrake>1) { *cmdBrake = 1; }
	if (*cmdBrake<0) { *cmdBrake = 0; }
	if (*cmdSteer>1) { *cmdSteer = 1; }
	if (*cmdSteer<-1) { *cmdSteer = -1; }
	//printf("Steer:%.2f\tspeed:%.2f\tlotAngle:%.2f\tcaryaw:%.2f\tback:%d\n",*cmdSteer,_speed,_lotAngle,_caryaw,back);
	//printf("����%.2fɲ��%.2f\n��λ����X%.2fY%.2f��������X%.2fY%.2f\n",*cmdAcc,*cmdBrake,_lotX,_lotY,_carX,_carY);
	//printf("ͣ������X%.2fY%.2fͣ������%.2fpointlinedist%.2fw%d\n",_carX-3*cos(_lotAngle),_carY-3*sin(_lotAngle),targdist,pointlinedist,w);
	//if(count<100) cout << _lotAngle << endl;
	//if (fabs(_caryaw) < 0.0001) cout << '0' << endl;
	//else cout << _caryaw << endl;
	//cout << pointlinedist << endl;
	count++;
}
void getpointlinedist() {
	int i;
	for (i = 0; i<4; i++) {
		dpointlinelist[i] = dpointlinelist[i + 1];
	}
	dpointlinelist[4] = pointlinedist;
}
void getAngle() {
	int i;
	for (i = 0; i<4; i++) {
		dangle[i] = dangle[i + 1];
	}
	dangle[4] = angle;
}

float dist(float x1, float x2, float y1, float y2)                   //�������
{
	return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}
float getR(float x1, float x2, float x3, float y1, float y2, float y3)  //��������
{
	float s, a, b, c;
	s = -((x2 - x1)*(y3 - y1) - (x3 - x1)*(y2 - y1)) / 2;
	a = dist(x1, x2, y1, y2);
	b = dist(x1, x3, y1, y3);
	c = dist(x3, x2, y3, y2);
	return 4 * s / (a*b*c);
}