#include <Eigen\Dense>
#include <iostream>
using namespace Eigen;
using namespace std;

void main() {
	{
		Eigen::MatrixXf m(4, 4);
		m << 1, 2, 3, 4,
			5, 6, 7, 8,
			9, 10, 11, 12,
			13, 14, 15, 16;
		cout << "Block in the middle " << endl;
		cout << m.block<2, 2>(1, 1) << endl;

		for (int i = 1; i < 3; ++i) {
			cout << "Block of size " << i << "x" << i << endl;
			cout << m.block(0, 0, i, i) << endl << endl;
		}
	}
	{
		Array22f m;
		m << 1, 2, 3, 4;
		Array44f a = Array44f::Constant(0.6);
		cout << "a: \n" << a << endl;
		a.block<2, 2>(1, 1) = m;
		cout << "a after a.block<2, 2>(1, 1) = m \n" << a << endl;

		a.block(0, 0, 2, 3) = a.block(2, 1, 2, 3);
		cout << "a after a.block(0, 0, 2, 3) = a.block(2, 1, 2, 3) \n" << a << endl;
	}
	{
		Eigen::MatrixXf m(3, 3);
		m << 1, 2, 3,
			 4, 5, 6,
			 7, 8, 9;
		cout << "m : \n" << m << endl;
		cout << "2nd row: \n" << m.row(1) << endl;
		m.col(2) += 3 * m.col(0);
		cout << "after m.col(2) += 3 * m.col(0): \n" << m << endl;
	}
	{
		Eigen::Matrix4f m;
		m << 1, 2, 3, 4,
			5, 6, 7, 8,
			9, 10, 11, 12,
			13, 14, 15, 16;
		cout << "m.leftCols(2) =" << endl << m.leftCols(2) << endl << endl;
		cout << "m.bottomRows<2>() =" << endl << m.bottomRows<2>() << endl << endl;
		m.topLeftCorner(1, 3) = m.bottomRightCorner(3, 1).transpose();
		cout << "After assignment, m = " << endl << m << endl;
	}

	{
		Eigen::ArrayXf v(6);
		v << 1, 2, 3, 4, 5, 6;
		cout << "v.head(3) = " << endl << v.head(3) << endl;
		cout << "v.tail<3>() = " << endl << v.tail<3>() << endl;
		v.segment(1, 4) *= 2;
		cout << "after v.segment(1, 4) *= 2 \n " << v << endl;
	}

	system("pause");
}