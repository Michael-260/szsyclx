#pragma once
#include<cmath>
#include<iostream>
#include<fstream>
using namespace std;
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
ostream& operator<<(ostream& output, mpoint& p) {
	output << "id:" << p.id << endl
		<< "(" << p.X << "," << p.Y << "," << p.Z << ")" << endl
		<< p.lx << "   " << p.ly << "   " << p.rx << "   " << p.ry << endl;
	return output;
}
istream& operator>>(istream& input, mpoint& p) {
	double tem;
	input >> p.id >> p.X >> p.Y >> p.Z
	>> tem>> tem >> tem
	>> p.lx >> p.ly >> tem >> p.rx >> p.ry;
	return input;
}
class mcamera {
public:
	double x0, y0;
	double f;
	double ccd;
	double k1, k2, p1, p2, alpha, beta;
	friend istream& operator>>(istream&, mcamera&);
};
istream& operator>>(istream& input, mcamera& p) {
	input >> p.x0 >> p.y0 >> p.f >> p.ccd >> p.ccd >> p.ccd >> 
		p.k1 >> p.k2 >> p.p1 >> p.p2 >> p.alpha >> p.beta;
	return input;
}
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
istream& operator>>(istream& input, mphoto& p) {
	double tem;
	input >> p.Xs >> p.Ys >> p.Zs >> p.fai >> p.omega >> p.kafa >> tem >> tem;
	return input;
}
void front(mphoto& right, mphoto& left, fstream& file);
void relative(mphoto& left, mphoto& right, fstream& file);
void AO();