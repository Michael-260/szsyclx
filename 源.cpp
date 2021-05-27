#include "head1.h"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
using namespace std;
using namespace Eigen;
const static string dataFILENAME = "data.scbapht";//计算数据"dat.txt";
int main() {
	fstream file; double tem;
	//读取左右影像精确外方位元素
	mphoto right, left;
	file.open(dataFILENAME, ios::in);
	file >> tem;
	file >> left >> right;
	file.close();
	//front(right, left, file);
	//relative(left, right, file);
	AO();
	return 0;
}
/*********************前方交会*********************/
void front(mphoto &right,mphoto &left,fstream &file) {
	    mpoint pointdat[100]; double temp; int nnum = 100;
	    file.open("bundleadjustment_SCBA_Point_Result.scbapts", ios::in);
	    file >> temp >> temp;
	    for (int i = 0; i < nnum; i++)file >> pointdat[i];//读入100个控制点的数据
		file.close(); file.open("front_result.txt", ios::out);
		for (int i = 0; i < nnum; ++i) {
			double x0 = 0.4321, y0 = 0.1174, f = 40.9349, x1 = pointdat[i].lx - x0, y1 = pointdat[i].ly - y0, x2 = pointdat[i].rx - x0, y2 = pointdat[i].ry - y0;
			double Bu = right.Xs - left.Xs, Bv = right.Ys - left.Ys, Bw = right.Zs - left.Zs;
			double Rout1[9], Rout2[9]; left.SolveMtrR(Rout1); right.SolveMtrR(Rout2);
			double leftpho[3], rightpht[3];//u1,v1,w1,u2,v2,w2
			leftpho[0] = Rout1[0] * x1 + Rout1[1] * y1 - Rout1[2] * f;
			leftpho[1] = Rout1[3] * x1 + Rout1[4] * y1 - Rout1[5] * f;
			leftpho[2] = Rout1[6] * x1 + Rout1[7] * y1 - Rout1[8] * f;
			rightpht[0] = Rout2[0] * x2 + Rout2[1] * y2 - Rout2[2] * f;
			rightpht[1] = Rout2[3] * x2 + Rout2[4] * y2 - Rout2[5] * f;
			rightpht[2] = Rout2[6] * x2 + Rout2[7] * y2 - Rout2[8] * f;
			//点投影系数N1,N2
			double N1, N2;
			N1 = (Bu * rightpht[2] - Bw * rightpht[0]) / (leftpho[0] * rightpht[2] - rightpht[0] * leftpho[2]);
			N2 = (Bu * leftpho[2] - Bw * leftpho[0]) / (leftpho[0] * rightpht[2] - rightpht[0] * leftpho[2]);
			//计算物方空间坐标
			double X, Y, Z;
			X = left.Xs + N1 * leftpho[0];
			Y = left.Ys + N1 * leftpho[1];
			Z = left.Zs + N1 * leftpho[2];
			file << X << "   " << Y << "   " << Z << endl;
		}
		file.close();
		cout << "请查看front_result.txt结果文件";
}
/*********************相对定向*********************/
void relative(mphoto& left, mphoto& right, fstream& file) {
	mpoint pointdat[100];
	double tem; int count = 0, nnum = 100;
	file.open("bundleadjustment_SCBA_Point_Result.scbapts", ios::in);
	file >> tem >> tem;
	for (int i = 0; i < nnum; i++)file >> pointdat[i];//读入100个控制点的数据
	file.close();
	/*定义矩阵
	  未知数顺序：μ，v，fai，omega，kafa*/
	MatrixXd A(100, 5), l(100, 1), X(5, 1), x, V, pointmtr(100, 3);
	for (int i = 0; i < 5; ++i)X(i, 0) = 0;
	double error = 3e-4;
	//打开数据输出文件
	fstream ofs; ofs.open("relative_report.txt",ios::out);
	while (1) {
		for (int i = 0; i < nnum; ++i) {
			double x0 = 0.4321, y0 = 0.1174, f = 40.9349, x1 = pointdat[i].lx - x0, y1 = pointdat[i].ly - y0, x2 = pointdat[i].rx - x0, y2 = pointdat[i].ry - y0;
			double Bx = right.Xs - left.Xs, By = Bx * X(0, 0), Bz = Bx * X(1, 0);
			double leftpho[3], rightpht[3],mrtR[9];//X1,Y1,Z1,X2,Y2,Z2
			right.fai = X(2, 0); right.omega = X(3, 0); right.kafa = X(4, 0); right.SolveMtrR(mrtR);
			leftpho[0] = x1;
			leftpho[1] = y1;
			leftpho[2] = -f;
			rightpht[0] = mrtR[0] * x2 + mrtR[1] * y2 - mrtR[2] * f;
			rightpht[1] = mrtR[3] * x2 + mrtR[4] * y2 - mrtR[5] * f;
			rightpht[2] = mrtR[6] * x2 + mrtR[7] * y2 - mrtR[8] * f;
			//点投影系数N1,N2
			double N1, N2;
			N1 = (Bx * rightpht[2] - Bz * rightpht[0]) / (leftpho[0] * rightpht[2] - rightpht[0] * leftpho[2]);
			N2 = (Bx * leftpho[2] - Bz * leftpho[0]) / (leftpho[0] * rightpht[2] - rightpht[0] * leftpho[2]);
			pointmtr(i, 0) = N1 * leftpho[0];
			pointmtr(i, 1) = N1 * leftpho[1];
			pointmtr(i, 2) = N1 * leftpho[2];
			A(i, 0) = Bx; A(i, 1) = -1.0 * Bx * rightpht[1] / rightpht[2]; A(i, 2) = -1.0 * N2 * rightpht[0] * rightpht[1] / rightpht[2];
			A(i, 3) = -1.0 * (rightpht[2] + rightpht[1] * rightpht[1] / rightpht[2]) * N2; A(i, 4) = rightpht[0] * N2;
			l(i, 0) = N1 * leftpho[1] - N2 * rightpht[1] - By;
		}
		x = (A.transpose()* A).inverse() * A.transpose() * l;
		V = A * x - l; double sigema = sqrt((V.transpose() * V)(0,0)/(100.0-5.0));
		X = X + x;
		++count;
		cout << "第" << count << "次迭代：sigema为 " << sigema << endl;
		bool flag = true;
		for (int i = 0; i < 5; i++) {
			if (abs(x(i, 0)) > error)flag = false;
		}
		if (flag) {
			ofs << "共迭代" << count << "次，代入计算的点个数为"<<nnum << endl<<"相对定向元素的值为：\nμ=" << X(0, 0) << endl << "v=" << X(1, 0) << endl
				<< "fai=" << X(2, 0) << endl << "omega=" << X(3, 0) << endl << "kafa=" << X(4, 0) << endl;
			ofs << "A矩阵：" << A << endl << "参数改正数：" << x << endl;
			ofs.close();
			ofs.open("pointdata.txt", ios::out);
			ofs << pointmtr << endl;
			ofs.close();
			cout << "请查看结果文件relative_report.txt\n";
			break;
		}
	}

}
/*********************绝对定向*********************/
void AO() {
	mpoint pointdat[100]; double temp; int nnum = 100;
	double _Xtp, _Ytp, _Ztp, _Xg, _Yg, _Zg, xx = 0.0, yy = 0.0, zz = 0.0;//声明重心化坐标
	fstream file; file.open("bundleadjustment_SCBA_Point_Result.scbapts", ios::in);
	file >> temp >> temp;
	for (int i = 0; i < nnum; ++i)file >> pointdat[i];//读入100个控制点的数据
	file.close();
	file.open("pointdata.txt", ios::in);
	double rpoint[300];
	for (int i = 0; i < 3 * nnum; ++i) {
		file >> rpoint[i];
		if (i % 3 == 0) {  _Xg += rpoint[i]; }
		else if (i % 3 == 1) {  _Yg += rpoint[i]; }
		else {  _Zg += rpoint[i]; }
	}file.close();
	for (int i = 0; i < nnum; ++i) {
		xx += pointdat[i].X;
		yy += pointdat[i].Y;
		zz += pointdat[i].Z;
	}
	//计算地面摄测系的重心坐标
	_Xtp = xx / nnum; _Ytp = yy / nnum; _Ztp = zz / nnum;
	//计算空间辅助坐标系的重心坐标
	_Xg = _Xg / nnum; _Yg = _Yg / nnum; _Zg = _Zg / nnum;
	cout << _Xtp << "  " << _Ytp << "  " << _Ztp << "  " << _Xg << "  " << _Yg << "  " << _Zg << "  \n";
	//重心化计算
	for (int i = 0; i < 3 * nnum; ++i) {
		if (i % 3 == 0) { rpoint[i] = rpoint[i] - _Xg; }
		else if (i % 3 == 1) { rpoint[i] = rpoint[i] - _Yg; }
		else { rpoint[i] = rpoint[i] - _Zg; }
	}
	/* x^2+y^2,  x^2+z^2,  y^2+z^2,  xy,  xz,  yz*/
	double *modulus=new double[6];//计算ATA所需的系数
	for (int i = 0; i < 6; ++i)modulus[i] = 0.0;
	for (int i = 0; i < 3 * nnum; ++i) {
		if (i % 3 == 0) {
			modulus[0] = rpoint[i]*rpoint[i] + rpoint[i+1] * rpoint[i+1]+ modulus[0];
			modulus[1] = rpoint[i] * rpoint[i] + rpoint[i + 2] * rpoint[i + 2] + modulus[1];
			modulus[2] = rpoint[i + 1] * rpoint[i + 1] + rpoint[i + 2] * rpoint[i + 2] + modulus[2];
			modulus[3] = rpoint[i] * rpoint[i + 1];
			modulus[4] = rpoint[i] * rpoint[i + 2];
			modulus[5] = rpoint[i + 1] * rpoint[i + 2];
		}
	}
	MatrixXd ATA(7, 7), ATL(7, 1);
	ATA << nnum, 0, 0, 0, 0, 0, 0,
		0, nnum, 0, 0, 0, 0, 0,
		0, 0, nnum, 0, 0, 0, 0,
		0, 0, 0;
}