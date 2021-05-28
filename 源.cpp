#include "head1.h"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
using namespace std;
using namespace Eigen;
int main() {
	cout << "开始后方交会计算\n";rearward();
	fstream file; double tem;
	//读取左右影像精确外方位元素
	mphoto right, left;
	file.open(dataFILENAME, ios::in);
	file >> tem;
	file >> left >> right;
	file.close();
	cout << "开始前方交会计算\n";front(right, left, file);
	//cout << "开始相对定向计算\n";relative(left, right, file);
	//cout << "开始绝对定向计算\n";AO();
	return 0;
}