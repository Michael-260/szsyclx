#include "head1.h"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
using namespace std;
using namespace Eigen;
int main() {
	cout << "��ʼ�󷽽������\n";rearward();
	fstream file; double tem;
	//��ȡ����Ӱ��ȷ�ⷽλԪ��
	mphoto right, left;
	file.open(dataFILENAME, ios::in);
	file >> tem;
	file >> left >> right;
	file.close();
	cout << "��ʼǰ���������\n";front(right, left, file);
	//cout << "��ʼ��Զ������\n";relative(left, right, file);
	//cout << "��ʼ���Զ������\n";AO();
	return 0;
}