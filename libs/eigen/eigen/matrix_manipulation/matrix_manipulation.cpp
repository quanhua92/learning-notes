#include <iostream>
#include <Eigen\Dense>

using namespace Eigen;
using namespace std;

int main(int argc, char** argv) {

	// Constructor
	MatrixXf a(10, 15);
	Vector3d b(5, 6, 7);

	// Coefficient accessors
	{
		MatrixXd m(2, 2);
		m(0, 0) = 3;
		m(1, 0) = 2.5;
		m(0, 1) = -1;
		m(1, 1) = m(1, 0) + m(0, 1);
	}

	{
		// Comma-initialization
		Matrix3f m;
		m << 1, 2, 3,
			4, 5, 6,
			7, 8, 9;
	}
	{
		// Resizing
		MatrixXd m(2, 5);
		m.resize(4, 3);
		cout << "The matrix m is of size " << m.rows() << "x" << m.cols() << endl; // 4x3
		cout << "It has " << m.size() << " coefficients" << endl; // 12
		VectorXd v(2);
		v.resize(5);
		cout << "The vector v is of size " << v.size() << endl; // 5
		cout << "As a matrix, v is of size " << v.rows() << "x" << v.cols() << endl; //5x1 

	}

	// assignment and resizing
	{
		MatrixXf a(2, 2); // size 2x2
		MatrixXf b(3, 3);
		a = b; 
		cout << a.size() << endl; // 3x3
	}

	{
		// addition and subtraction
		Matrix2d a;
		a << 1, 2, 
			 3, 4;
		MatrixXd b(2, 2);
		b << 2, 3,
			 1, 4;
		cout << "a = \n" << a << endl << "b = \n" << b << endl;
		cout << "a + b = \n" << a + b << endl;
		cout << "a - b = \n" << a - b << endl;
		cout << "Doing a += b." << endl;
		a += b;
		cout << "Now a = \n" << a << endl;
		Vector3d v(1, 2, 3);
		Vector3d w(1, 0, 0);
		cout << "-v + w - v = \b" << -v + w - v << endl;
	}
	{
		// scalar multiplication and division
		Matrix2d a;
		a << 1, 2,
			 3, 4;
		Vector3d v(1, 2, 3);
		cout << "a = \n" << a << endl << "v = \n" << v << endl;
		cout << "a * 2.5 = \n" << a * 2.5 << endl;
		cout << "0.1 * v = \n" << 0.1 * v << endl;
		cout << "Doing v*= 2; " << endl;
		v *= 2;
		cout << "Now v =\n" << v << endl;
	}

	system("pause");
	return 0;
}