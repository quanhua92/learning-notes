#include <iostream>
#include <Eigen\Dense>

using namespace Eigen;
using namespace std;

void main() {
	{
		ArrayXXf m(2, 2);
		m(0, 0) = 1.0;
		m(0, 1) = 2.0;
		m(1, 0) = 3.0;
		m(1, 1) = m(0, 1) + m(1, 0);

		cout << m << endl;
		m << 1.0, 2.0, 3.0, 4.0;
		cout << m << endl;
	}

	{
		ArrayXXf a(3, 3);
		ArrayXXf b(3, 3);
		a << 1, 2, 3,
			4, 5, 6,
			7, 8, 9;
		b << 1, 2, 3,
			1, 2, 3,
			1, 2, 3;

		// Adding two arrays
		cout << "a + b = " << endl << a + b << endl << endl;
		// Subtracting a scalar from an array
		cout << "a - 2 = " << endl << a - 2 << endl;
	}

	{
		ArrayXXf a(2, 2);
		ArrayXXf b(2, 2);

		a << 1, 2, 3, 4;
		b << 5, 6, 7, 8;

		cout << "a * b = \n" << a * b << endl;
	}

	{
		// COEFFICIENT-WISE OPERATIONS
		cout << "COEFFICIENT-WISE OPERATIONS" << endl;
		ArrayXf a = ArrayXf::Random(5);
		a *= 2;
		cout << "a = " << endl << a << endl;
		cout << "a.abs() = " << endl << a.abs() << endl;
		cout << "a.abs().sqrt() = " << endl << a.abs().sqrt() << endl;
		cout << "a.min(a.abs().sqrt()) = " << endl << a.min(a.abs().sqrt()) << endl;

	}

	{
		MatrixXf m(2, 2);
		MatrixXf n(2, 2);
		MatrixXf result(2, 2);
		m << 1, 2,
			 3, 4;
		n << 5, 6,
			 7, 8;

		result = m * n;
		cout << "-- m : -- \n" << m << endl;
		cout << "-- n : -- \n" << n << endl;
		cout << "-- Matrix m * n : -- \n" << result << endl;
		result = m.array() * n.array();
		cout << "-- Matrix m.array() * n.array() : -- \n" << result << endl;
		result = m.cwiseProduct(n);
		cout << "-- m.cwiseProduct(n) : -- \n" << result << endl;
		result = m.array() + 4;
		cout << "-- Array m + 4: -- \n" << result << endl;
		result = (m.array() + 4).matrix() * m;
		cout << "-- Combo 1: (m.array() + 4).matrix() * m -- \n" << result << endl;
		result = (m.array() * n.array()).matrix() * m;
		cout << "-- Combo 2: (m.array() * n.array()).matrix() * m --\n" << result << endl;
	}

	system("pause");
}