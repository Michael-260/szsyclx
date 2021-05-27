#include "head1.h"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
using namespace std;
using namespace Eigen;
//const static string dataFILENAME = "data.scbapht";//计算数据"dat.txt";
//static string POINTFILE = "bundleadjustment_SCBA_Point_Result.scbapts";//控制点及其像点信息文件
//static string CAMERAFILE = "bundleadjustment_SCBA_Camera_Result.scbacmr";//相机参数文件
//static string PHOTOFILE = "bundleadjustment_SCBA_Photo_Result.scbapht";//左右影像外方位元素
int main() {
	rearward();
	fstream file; double tem;
	//读取左右影像精确外方位元素
	mphoto right, left;
	file.open(dataFILENAME, ios::in);
	file >> tem;
	file >> left >> right;
	file.close();
	front(right, left, file);
	relative(left, right, file);
	AO();
	//test();
	return 0;
}
///*********************后方交会*********************/
//void rearward() {
//	fstream file;
//	//*********************读取点的坐标*********************
//	//像点信息：X，Y，Z，lx，ly，rx，ry
//	int pointnum = 0, tem = 0;//pointnum 点的总数
//	file.open(POINTFILE, ios::in);
//	file >> pointnum >> tem;
//	//cout << pointnum << "   " << tem << endl;
//	//定义存储点信息的对象数组
//	mpoint* mpt = new mpoint[pointnum], * mpt_tem = mpt;
//	for (int i = 0; i < pointnum; i++) {
//		file >> mpt_tem[i];
//	}
//	/*for (int i = 0; i < pointnum; i++) {
//		cout << mpt_tem[i];
//	}*/
//	file.close();
//
//	//******************读取相机参数******************
//	//相机参数：x0，y0，f，ccd，k1, k2, p1, p2, alpha, beta
//	mcamera mca;
//	file.open(CAMERAFILE, ios::in);
//	file >> tem; file >> mca;
//	file.close();
//
//	//******************读取外方位元素初值******************
//	//外方位元素值：Xs, Ys, Zs, fai, omega, kafa;
//	file.open(PHOTOFILE, ios::in);
//	mphoto left, right;
//	file >> tem; file >> left >> right;
//	file.close();
//
//	//******************数据处理、平差运算******************
//	int mod_code = 1;//首先计算左右影像代码
//	while (mod_code < 3)
//	{
//		MatrixXd X0(6, 1);
//		if (mod_code == 1) {
//			X0(0, 0) = left.Xs; X0(1, 0) = left.Ys; X0(2, 0) = left.Zs;
//			X0(3, 0) = left.fai; X0(4, 0) = left.omega; X0(5, 0) = left.kafa;
//		}
//		if (mod_code == 2) {
//			X0(0, 0) = right.Xs; X0(1, 0) = right.Ys; X0(2, 0) = right.Zs;
//			X0(3, 0) = right.fai; X0(4, 0) = right.omega; X0(5, 0) = right.kafa;
//		}
//		tem = 1;
//		double sigemad = 0;
//		if (mod_code == 1) { file.open("result_left.txt", ios::out); }
//		if (mod_code == 2) { file.open("result_right.txt", ios::out); }
//
//		while (1)//迭代条件
//		{
//			double deltx, delty, xba, yba, zba;
//			double MtrRoat[9];
//			mphoto mx0;
//			mx0.Xs = X0(0, 0); mx0.Ys = X0(1, 0); mx0.Zs = X0(2, 0);
//			mx0.fai = X0(3, 0); mx0.omega = X0(4, 0); mx0.kafa = X0(5, 0);
//			mx0.SolveMtrR(MtrRoat);
//			mpt_tem = mpt;
//			MatrixXd L(pointnum * 2, 1);
//			MatrixXd L0(pointnum * 2, 1);
//			MatrixXd MtrA(pointnum * 2, 6);//！！！此处使用动态矩阵，或者用运算符new
//			//矩阵赋值
//			for (int i = 0; i < pointnum; i++, mpt_tem++) {
//				double xs = X0(0, 0), ys = X0(1, 0), zs = X0(2, 0);
//				xba = MtrRoat[0] * (mpt_tem->X - xs) + MtrRoat[3] * (mpt_tem->Y - ys) + MtrRoat[6] * (mpt_tem->Z - zs);
//				yba = MtrRoat[1] * (mpt_tem->X - xs) + MtrRoat[4] * (mpt_tem->Y - ys) + MtrRoat[7] * (mpt_tem->Z - zs);
//				zba = MtrRoat[2] * (mpt_tem->X - xs) + MtrRoat[5] * (mpt_tem->Y - ys) + MtrRoat[8] * (mpt_tem->Z - zs);
//				int index1 = i * 2, index2 = i * 2 + 1;
//				double x, y;
//				if (mod_code == 1) x = mpt_tem->lx, y = mpt_tem->ly;
//				if (mod_code == 2) x = mpt_tem->rx, y = mpt_tem->ry;
//				double x0 = mca.x0, y0 = mca.y0, f = mca.f,
//					fai = X0(3, 0), omega = X0(4, 0), kafa = X0(5, 0);
//				double k1 = mca.k1, k2 = mca.k2, p1 = mca.p1, p2 = mca.p2, alpha = mca.alpha, beta = mca.beta, r = sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0));
//				deltx = (x - x0) * (k1 * r * r + k2 * r * r * r * r) + p1 * (r * r + 2 * (x - x0) * (x - x0)) + 2 * p2 * (x - x0) * (y - y0) + alpha * (x - x0) + beta * (y - y0);
//				delty = (y - y0) * (k1 * r * r + k2 * r * r * r * r) + p2 * (r * r + 2 * (y - y0) * (y - y0)) + 2 * p1 * (x - x0) * (y - y0);
//				L0(index1, 0) = x0 + deltx - f * xba / zba;
//				L0(index2, 0) = y0 + delty - f * yba / zba;
//				L(index1, 0) = x - L0(index1, 0);
//				L(index2, 0) = y - L0(index2, 0);
//				MtrA(index1, 0) = (MtrRoat[0] * f + MtrRoat[2] * (x - x0)) / zba;//a11
//				MtrA(index1, 1) = (MtrRoat[3] * f + MtrRoat[5] * (x - x0)) / zba;//a12
//				MtrA(index1, 2) = (MtrRoat[6] * f + MtrRoat[8] * (x - x0)) / zba;//a13
//				MtrA(index2, 0) = (MtrRoat[1] * f + MtrRoat[2] * (y - y0)) / zba;//a21
//				MtrA(index2, 1) = (MtrRoat[4] * f + MtrRoat[5] * (y - y0)) / zba;//a22
//				MtrA(index2, 2) = (MtrRoat[7] * f + MtrRoat[8] * (y - y0)) / zba;//a23
//				MtrA(index1, 3) = (y - y0) * sin(omega) - cos(omega) * ((x - x0) * ((x - x0) * cos(kafa) - (y - y0) * sin(kafa)) / f + f * cos(kafa));//a14
//				MtrA(index1, 4) = -1.0 * f * sin(kafa) - (x - x0) * ((x - x0) * sin(kafa) + (y - y0) * cos(kafa)) / f;//a15
//				MtrA(index1, 5) = y - y0;//a16
//				MtrA(index2, 3) = -1.0 * (x - x0) * sin(omega) - cos(omega) * ((y - y0) * ((x - x0) * cos(kafa) - (y - y0) * sin(kafa)) / f - f * sin(kafa));//a24
//				MtrA(index2, 4) = -1.0 * f * cos(kafa) - (y - y0) * ((x - x0) * sin(kafa) + (y - y0) * cos(kafa)) / f;//a25
//				MtrA(index2, 5) = -1.0 * (x - x0);//a26
//
//			}
//			MatrixXd X = (MtrA.transpose() * MtrA).inverse() * MtrA.transpose() * L;
//			file << "**********第" << tem << "次迭代**********" << endl <<
//				"MtrA:\n" << MtrA << endl << "估计值:\n" << L0 << endl << "L:\n" << L << endl <<
//				"参数:\n" << X0 << endl << "参数改正:\n" << X << endl;
//			X0 = X0 + X; tem++;
//			file << "改正后参数:\n" << X0 << endl;
//			MatrixXd v = MtrA * X - L;
//			MatrixXd sigema = (v.transpose() * v) / (pointnum * 2.0 - 6.0);
//			sigemad = sqrt(sigema(0, 0));
//			file << "sigema:\n" << sigemad << endl;
//			if (mod_code == 1) {
//				cout << "左影像分析：done\n";
//				if (abs(X(0, 0)) < 0.0001 && abs(X(1, 0)) < 0.0001 && abs(X(2, 0)) < 0.0001 && abs(X(3, 0)) < 0.0001 && abs(X(4, 0)) < 0.0001 && abs(X(5, 0)) < 0.0001) { cout << "请查看result_left.txt文件\n"; break; }
//			}
//			if (mod_code == 2) {
//				cout << "右影像分析：done\n";
//				if (abs(X(0, 0)) < 0.0001 && abs(X(1, 0)) < 0.0001 && abs(X(2, 0)) < 0.0001 && abs(X(3, 0)) < 0.0001 && abs(X(4, 0)) && abs(X(5, 0)) < 0.0001) { cout << "请查看result_right.txt文件\n"; break; }
//			}
//		}
//		file.close();
//		mod_code++;
//	}
//}
//
///*********************前方交会*********************/
//void front(mphoto &right,mphoto &left,fstream &file) {
//	    mpoint pointdat[100]; double temp; int nnum = 100;
//	    file.open("bundleadjustment_SCBA_Point_Result.scbapts", ios::in);
//	    file >> temp >> temp;
//	    for (int i = 0; i < nnum; i++)file >> pointdat[i];//读入100个控制点的数据
//		file.close(); file.open("front_result.txt", ios::out);
//		for (int i = 0; i < nnum; ++i) {
//			double x0 = 0.4321, y0 = 0.1174, f = 40.9349, x1 = pointdat[i].lx - x0, y1 = pointdat[i].ly - y0, x2 = pointdat[i].rx - x0, y2 = pointdat[i].ry - y0;
//			double Bu = right.Xs - left.Xs, Bv = right.Ys - left.Ys, Bw = right.Zs - left.Zs;
//			double Rout1[9], Rout2[9]; left.SolveMtrR(Rout1); right.SolveMtrR(Rout2);
//			double leftpho[3], rightpht[3];//u1,v1,w1,u2,v2,w2
//			leftpho[0] = Rout1[0] * x1 + Rout1[1] * y1 - Rout1[2] * f;
//			leftpho[1] = Rout1[3] * x1 + Rout1[4] * y1 - Rout1[5] * f;
//			leftpho[2] = Rout1[6] * x1 + Rout1[7] * y1 - Rout1[8] * f;
//			rightpht[0] = Rout2[0] * x2 + Rout2[1] * y2 - Rout2[2] * f;
//			rightpht[1] = Rout2[3] * x2 + Rout2[4] * y2 - Rout2[5] * f;
//			rightpht[2] = Rout2[6] * x2 + Rout2[7] * y2 - Rout2[8] * f;
//			//点投影系数N1,N2
//			double N1, N2;
//			N1 = (Bu * rightpht[2] - Bw * rightpht[0]) / (leftpho[0] * rightpht[2] - rightpht[0] * leftpho[2]);
//			N2 = (Bu * leftpho[2] - Bw * leftpho[0]) / (leftpho[0] * rightpht[2] - rightpht[0] * leftpho[2]);
//			//计算物方空间坐标
//			double X, Y, Z;
//			X = left.Xs + N1 * leftpho[0];
//			Y = left.Ys + N1 * leftpho[1];
//			Z = left.Zs + N1 * leftpho[2];
//			/*X = N1 * leftpho[0];
//			Y = N1 * leftpho[1];
//			Z = N1 * leftpho[2];*/
//			file << X << "   " << Y << "   " << Z << endl;
//		}
//		file.close();
//		cout << "请查看front_result.txt结果文件";
//}
///*********************相对定向*********************/
//void relative(mphoto& left, mphoto& right, fstream& file) {
//	mpoint pointdat[117];
//	double tem; int count = 0, nnum = 117;
//	file.open("bundleadjustment_SCBA_Point_Result.scbapts", ios::in);
//	file >> tem >> tem;
//	for (int i = 0; i < nnum; i++)file >> pointdat[i];//读入117个控制点的数据
//	file.close();
//	/*定义矩阵
//	  未知数顺序：μ，v，fai，omega，kafa*/
//	MatrixXd A(117, 5), l(117, 1), X(5, 1), x, V, pointmtr(117, 3);
//	for (int i = 0; i < 5; ++i)X(i, 0) = 0;
//	double error = 3e-4;
//	//打开数据输出文件
//	fstream ofs; ofs.open("relative_report.txt",ios::out);
//	while (1) {
//		for (int i = 0; i < nnum; ++i) {
//			double x0 = 0.4321, y0 = 0.1174, f = 40.9349, x1 = pointdat[i].lx - x0, y1 = pointdat[i].ly - y0, x2 = pointdat[i].rx - x0, y2 = pointdat[i].ry - y0;
//			double Bx = right.Xs - left.Xs, By = Bx * X(0, 0), Bz = Bx * X(1, 0);
//			double leftpho[3], rightpht[3],mrtR[9];//X1,Y1,Z1,X2,Y2,Z2
//			right.fai = X(2, 0); right.omega = X(3, 0); right.kafa = X(4, 0); right.SolveMtrR(mrtR);
//			leftpho[0] = x1;
//			leftpho[1] = y1;
//			leftpho[2] = -f;
//			rightpht[0] = mrtR[0] * x2 + mrtR[1] * y2 - mrtR[2] * f;
//			rightpht[1] = mrtR[3] * x2 + mrtR[4] * y2 - mrtR[5] * f;
//			rightpht[2] = mrtR[6] * x2 + mrtR[7] * y2 - mrtR[8] * f;
//			//点投影系数N1,N2
//			double N1, N2;
//			N1 = (Bx * rightpht[2] - Bz * rightpht[0]) / (leftpho[0] * rightpht[2] - rightpht[0] * leftpho[2]);
//			N2 = (Bx * leftpho[2] - Bz * leftpho[0]) / (leftpho[0] * rightpht[2] - rightpht[0] * leftpho[2]);
//			pointmtr(i, 0) = N1 * leftpho[0];
//			pointmtr(i, 1) = N1 * leftpho[1];
//			pointmtr(i, 2) = N1 * leftpho[2];
//			//矩阵赋值
//			A(i, 0) = Bx; A(i, 1) = -1.0 * Bx * rightpht[1] / rightpht[2]; A(i, 2) = -1.0 * N2 * rightpht[0] * rightpht[1] / rightpht[2];
//			A(i, 3) = -1.0 * (rightpht[2] + rightpht[1] * rightpht[1] / rightpht[2]) * N2; A(i, 4) = rightpht[0] * N2;
//			l(i, 0) = N1 * leftpho[1] - N2 * rightpht[1] - By;
//		}
//		x = (A.transpose()* A).inverse() * A.transpose() * l;
//		V = A * x - l; double sigema = sqrt((V.transpose() * V)(0,0)/(100.0-5.0));
//		X = X + x;
//		++count;
//		cout << "第" << count << "次迭代：sigema为 " << sigema << endl;
//		bool flag = true;
//		for (int i = 0; i < 5; i++) {
//			if (abs(x(i, 0)) > error)flag = false;
//		}
//		if (flag) {
//			ofs << "共迭代" << count << "次，代入计算的点个数为"<<nnum << endl<<"相对定向元素的值为：\nμ=" << X(0, 0) << endl << "v=" << X(1, 0) << endl
//				<< "fai=" << X(2, 0) << endl << "omega=" << X(3, 0) << endl << "kafa=" << X(4, 0) << endl;
//			ofs << "A矩阵：" << A << endl << "参数改正数：" << x << endl;
//			ofs.close();
//			ofs.open("pointdata.txt", ios::out);
//			ofs << pointmtr << endl;
//			ofs.close();
//			cout << "请查看结果文件relative_report.txt\n";
//			break;
//		}
//	}
//
//}
///*********************绝对定向*********************/
//void AO() {
//	mpoint pointdat[117]; double temp; int nnum = 117;
//	double _Xtp=0.0, _Ytp=0.0, _Ztp=0.0 , _Xg=0.0, _Yg=0.0, _Zg=0.0, xx = 0.0, yy = 0.0, zz = 0.0;//初始化重心化坐标
//	fstream file; file.open("bundleadjustment_SCBA_Point_Result.scbapts", ios::in);
//	file >> temp >> temp;
//	for (int i = 0; i < nnum; ++i)file >> pointdat[i];//读入117个控制点的数据
//	file.close();
//	file.open("pointdata.txt", ios::in);
//	double rpoint[351], grpoint[351];//重心化之前/后的像空间辅助坐标
//	for (int i = 0; i < 3 * nnum; ++i) {
//		file >> rpoint[i];
//		if (i % 3 == 0) {  _Xg += rpoint[i]; }
//		else if (i % 3 == 1) {  _Yg += rpoint[i]; }
//		else {  _Zg += rpoint[i]; }
//	}file.close();
//	for (int i = 0; i < nnum; ++i) {
//		xx += pointdat[i].X;
//		yy += pointdat[i].Y;
//		zz += pointdat[i].Z;
//	}
//	//计算地面摄测系的重心坐标
//	_Xtp = xx / nnum; _Ytp = yy / nnum; _Ztp = zz / nnum;
//	//计算空间辅助坐标系的重心坐标
//	_Xg = _Xg / nnum; _Yg = _Yg / nnum; _Zg = _Zg / nnum;
//	cout << _Xtp << "  " << _Ytp << "  " << _Ztp << "  " << _Xg << "  " << _Yg << "  " << _Zg << "  \n";
//
//	//重心化计算
//	for (int i = 0, j = 0; i < 3 * nnum; ++i) {
//		j = i / 3;
//		if (i % 3 == 0) { grpoint[i] = (rpoint[i] - _Xg); rpoint[i] = pointdat[j].X - _Xtp; }
//		else if (i % 3 == 1) { grpoint[i] = (rpoint[i] - _Yg); rpoint[i] = pointdat[j].Y - _Ytp;}
//		else { grpoint[i] = (rpoint[i] - _Zg); rpoint[i] = pointdat[j].Z - _Ztp;}
//	}
//	//grpoint 辅助重心化坐标  rpoint 物空间重心化坐标
//	//初值计算
//	MatrixXd bA(351, 12), bX(12, 1), bL(351, 1), m_Roat(3,3);
//	double lambda[3], lam;
//	for (int i = 0, j = 0; i < nnum * 3; ++i) {
//		if (i % 3 == 0) {j = i / 3;
//			bA(i, 0) = 1; bA(i, 1) = 0; 
//			bA(i, 2) = 0; bA(i, 3) = grpoint[i]; 
//			bA(i, 4) = grpoint[i + 1]; bA(i, 5) = grpoint[i + 2]; 
//			bA(i, 6) = 0; bA(i, 7) = 0; bA(i, 8) = 0; bA(i, 9) = 0; 
//			bA(i, 10) = 0; bA(i, 11) = 0;
//			bL(i, 0) = pointdat[j].X;
//		}
//		else if (i % 3 == 1) { 
//			bL(i, 0) = pointdat[j].Y; 
//			bA(i, 0) = 0; bA(i, 1) = 1; bA(i, 2) = 0; 
//			bA(i , 3) = 0; bA(i , 4) = 0; bA(i , 5) = 0; 
//			bA(i , 6) = grpoint[i-1]; bA(i, 7) = grpoint[i]; 
//			bA(i , 8) = grpoint[i+1]; bA(i , 9) = 0; bA(i , 10) = 0; bA(i, 11) = 0;}
//		else{
//			bL(i, 0) = pointdat[j].Z;
//			bA(i, 0) = 0; bA(i, 1) = 0; bA(i, 2) = 1; 
//			bA(i, 3) = 0; bA(i, 4) = 0; bA(i, 5) = 0; 
//			bA(i, 6) = 0; bA(i, 7) = 0; bA(i, 8) = 0; 
//			bA(i, 9) = grpoint[i-2]; bA(i, 10) = grpoint[i-1]; bA(i, 11) = grpoint[i];
//		}
//	}
//	bX = (bA.transpose() * bA).inverse()* bA.transpose() * bL ;
//	//cout << bX << endl;
//	m_Roat << bX(3, 0), bX(4, 0), bX(5, 0), bX(6, 0), bX(7, 0), bX(8, 0), bX(9, 0), bX(10, 0), bX(11, 0);
//	for (int i = 0; i < 3; ++i)lambda[i] = sqrt(m_Roat(i, 0) * m_Roat(i, 0) + m_Roat(i, 1) * m_Roat(i, 1) + m_Roat(i, 2) * m_Roat(i, 2));
//	lam = (lambda[0] + lambda[1] + lambda[2]) / 3.0;
//	m_Roat = m_Roat / lam;//m_Roat:改化角度矩阵，lam:改化比例系数
//	//比例尺改化
//	for (int i = 0, j = 0; i < 3 * nnum; ++i) {
//		if (i % 3 == 0) {
//			grpoint[i] = grpoint[i] * lam;
//		}
//
//	}
//	/*for (int i = 0; i < 300; ++i) {
//		cout << grpoint[i] <<"   "<<rpoint[i]<< endl;
//	}*/
//	//grpoint 辅助重心化坐标  rpoint 物空间重心化坐标
//	file.open("absolute_report.txt", ios::out);
//	double fai, omega, kafa, m_Roat1[9], error = 3e-4; MatrixXd A(351, 3), L(351, 1), X(3, 1), x(3, 1); X << 0.0, 0.0, 0.0;
//	while (1) {
//		mphoto tempht; tempht.fai = X(0, 0); tempht.omega = X(1, 0); tempht.kafa = X(2, 0); tempht.SolveMtrR(m_Roat1);
//		for (int i = 0, j = 0, k = 0; i < 3 * nnum; ++i) {
//			if (i % 3 == 0) {
//				j = i + 1; k = i + 2;
//				A(i, 0) = -1.0 * grpoint[i + 2]; A(i, 1) = 0; A(i, 2) = -1.0 * grpoint[i + 1];
//				A(j, 0) = 0; A(j, 1) = -1.0 * grpoint[i + 2]; A(j, 2) = grpoint[i];
//				A(k, 0) = grpoint[i]; A(k, 1) = grpoint[i + 1]; A(k, 2) = 0;
//				L(i, 0) = rpoint[i] - (m_Roat1[0] * grpoint[i] + m_Roat1[1] * grpoint[i + 1] + m_Roat1[2] * grpoint[i + 2]);
//				L(j, 0) = rpoint[i + 1] - (m_Roat1[3] * grpoint[i] + m_Roat1[4] * grpoint[i + 1] + m_Roat1[5] * grpoint[i + 2]);
//				L(k, 0) = rpoint[i + 2] - (m_Roat1[6] * grpoint[i] + m_Roat1[7] * grpoint[i + 1] + m_Roat1[8] * grpoint[i + 2]);
//			}
//		}
//		x = (A.transpose() * A).inverse() * A.transpose() * L;
//		X = X + x;
//		fai = X(0, 0); omega = X(1, 0); kafa = X(2, 0);
//		bool flag = true;
//		for (int i = 0; i < 3; i++) {
//			if (abs(x(i, 0)) > error)flag = false;
//		}
//		if (flag) {
//			file << "绝对定向参数（ψ，ω，k，λ，deltX，deltY，deltZ）：\n" << X(0, 0) << "  " << X(1, 0) << X(2, 0) << "  "
//				<<lam<<"  "<< bX(0, 0) << "  " << bX(1, 0) << "  " << bX(2, 0);
//			cout <<"请查看absolute_report.txt结果文件\n" << endl;
//			file.close();
//			break;
//		}
//	}
//}
