#pragma once
#include<cmath>
#include<iostream>
#include<fstream>
using namespace std;
const static string dataFILENAME = "data.scbapht";//��������"dat.txt";
static string POINTFILE = "bundleadjustment_SCBA_Point_Result.scbapts";//���Ƶ㼰�������Ϣ�ļ�
static string CAMERAFILE = "bundleadjustment_SCBA_Camera_Result.scbacmr";//��������ļ�
static string PHOTOFILE = "bundleadjustment_SCBA_Photo_Result.scbapht";//����Ӱ���ⷽλԪ��
class mpoint {
public:
	int id;
	double X;
	double Y;
	double Z;
	double lx;
	double ly;
	double rx;
	double ry;

	friend ostream& operator<<(ostream&, mpoint&);
	friend istream& operator>>(istream&, mpoint&);
};
class mcamera {
public:
	double x0, y0;
	double f;
	double ccd;
	double k1, k2, p1, p2, alpha, beta;
	friend istream& operator>>(istream&, mcamera&);
};
class mphoto {
public:
	double Xs, Ys, Zs, fai, omega, kafa;
	friend istream& operator>>(istream&, mphoto&);
	void SolveMtrR(double *p) {
		p[0] = cos(fai) * cos(kafa) - sin(fai) * sin(omega) * sin(kafa);
		p[1] = -1.0 * cos(fai) * sin(kafa) - sin(fai) * sin(omega) * cos(kafa);
		p[2] = -1.0 * sin(fai) * cos(omega);
		p[3] = cos(omega) * sin(kafa);
		p[4] = cos(omega) * cos(kafa);
		p[5] = -1.0 * sin(omega);
		p[6] = sin(fai) * cos(kafa) + cos(fai) * sin(omega) * sin(kafa);
		p[7] = -1.0 * sin(fai) * sin(kafa) + cos(fai) * sin(omega) * cos(kafa);
		p[8] = cos(fai) * cos(omega);
	}
};
void rearward();
void front(mphoto& right, mphoto& left, fstream& file);
void relative(mphoto& left, mphoto& right, fstream& file);
void AO();