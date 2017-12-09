#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

void main() {

	{
		Matrix2f m;
		m << 1, 2, 3, 4;
		cout << m << endl;

		RowVectorXd vec1(3);
		vec1 << 1, 2, 3;
		cout << "vec1 : " << vec1 << endl;

		RowVectorXd vec2(4);
		vec2 << 1, 4, 9, 16;
		cout << "vec2 : " << vec2 << endl;

		RowVectorXd joined(7);
		joined << vec1, vec2;
		cout << "joined = " << joined << endl;

		MatrixXf matA(2, 2); matA << 1, 2, 3, 4;
		MatrixXf matB(4, 4);
		matB << matA, matA / 10, matA / 10, matA;
		cout << "matA: \n" << matA << endl;
		cout << "matB: \n" << matB << endl;
	}
	{
		Matrix3f m;
		m.row(0) << 1, 2, 3;
		m.block(1, 0, 2, 2) << 4, 5, 7, 8;
		m.col(2).tail(2) << 6, 9;
		cout << m << endl;
	}
	{
		cout << "A fixed-size array:\n";
		Array33f a1 = Array33f::Zero();
		cout << a1 << endl;

		cout << "A one-dimensional dynamic-size array: \n";
		ArrayXf a2 = ArrayXf::Zero(3);
		cout << a2 << endl;

		cout << "A two-dimensional dynamic-size array:\n";
		ArrayXXf a3 = ArrayXXf::Zero(3, 4);
		cout << a3 << endl;
	}
	{
		float M_PI = 3.14;
		ArrayXXf table(10, 4);
		table.col(0) = ArrayXf::LinSpaced(10, 0, 90);
		table.col(1) = M_PI / 180 * table.col(0);
		table.col(2) = table.col(1).sin();
		table.col(3) = table.col(1).cos();
		std::cout << "  Degrees   Radians      Sine    Cosine\n";
		std::cout << table << std::endl;
	}

	system("pause");
}