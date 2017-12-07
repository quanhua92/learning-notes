#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

void main() {
	MatrixXd m(2, 2);
	m(0, 0) = 3;
	m(1, 0) = 2.5; // row, col
	m(0, 1) = -1;
	m(1, 1) = m(1, 0) + m(0, 1);
	std::cout << "Example 01: " << std::endl;
	std::cout << m << std::endl;

	{
		std::cout << "Example 02: Size set at run time" << std::endl;
		MatrixXd m = MatrixXd::Random(3, 3);
		m = (m + MatrixXd::Constant(3, 3, 1.2)) * 50;
		cout << "m = " << endl << m << endl;
		VectorXd v(3);
		v << 1, 2, 3;
		cout << "m * v = " << endl << m * v << endl;
	}
	{
		std::cout << "Example 02: Size set at compile time" << std::endl;
		Matrix3d m = Matrix3d::Random();
		m = (m + Matrix3d::Constant(1.2)) * 50;
		cout << "m = " << endl << m << endl;
		Vector3d v(1, 2, 3);
		cout << "m * v = " << endl << m * v << endl;
	}
	system("pause");
}
