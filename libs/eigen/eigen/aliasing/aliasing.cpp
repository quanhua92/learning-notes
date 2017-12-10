#include <iostream>>
#include <Eigen\Dense>

using namespace Eigen;
using namespace std;

void main() {
	{
		MatrixXi mat(3, 3);
		mat << 1, 2, 3, 4, 5, 6, 7, 8, 9;
		cout << "mat = \n" << mat << endl;

		// This assignment shows the aliasing problem
		mat.bottomRightCorner(2, 2) = mat.topLeftCorner(2, 2);
		cout << "After the assignment, mat =\n" << mat << endl;
		cout << "--> The aliasing problem" << endl << endl;
	}
	{
		Matrix2i a; a << 1, 2, 3, 4;
		cout << "Matrix a =\n" << a << endl;
		//a = a.transpose(); // !!! do NOT do this !!!
		//cout << "Another aliasing problem with a = a.tranpose()." << endl;
	}
	cout << "------------" << endl;
	cout << "Resolving aliasing issues" << endl;
	{
		MatrixXi mat(3, 3);
		mat << 1, 2, 3, 4, 5, 6, 7, 8, 9;
		cout << "mat = \n" << mat << endl;
		mat.bottomRightCorner(2, 2) = mat.topLeftCorner(2, 2).eval();
		cout << "After the assignment with .eval(), mat = \n" << mat << endl;
	}
	{
		Matrix2i a; a << 1, 2, 3, 4;
		cout << "Matrix a =\n" << a << endl;
		a.transposeInPlace();
		cout << "After the transposeInPlace(), a = \n" << a << endl;
	}
	cout << "-------------" << endl;
	cout << "Aliasing and component-wise operations" << endl;
	{
		MatrixXf mat(2, 2);
		mat << 1, 2, 4, 7;
		cout << "mat = \n" << mat << endl;
		mat = 2 * mat;
		cout << "after mat = 2 * mat \n" << mat << endl;
		mat = mat - MatrixXf::Identity(2, 2);
		cout << "After the subtraction: - Identity(2,2)\n" << mat << endl;

		ArrayXXf arr = mat;
		arr = arr.square();
		cout << "After squaring \n" << arr << endl;

		mat << 1, 2, 4, 7;
		mat = (2 * mat - MatrixXf::Identity(2, 2)).array().square();
		cout << "Doing everything at once \n" << mat << endl;
	}

	{
		cout << "-------------" << endl;
		cout << "In matrix multiplication, Eigen assumes aliasing by default under the condition that the dest matrix is not resized" << endl;
		MatrixXf matA(2, 2);
		matA << 2, 0, 0, 2;
		matA = matA * matA;
		cout << "matA=\n" << matA << endl;
		cout << "---> Eigen evaluates the product in a temporary matrix" << endl;
	}
	{
		cout << "---> Use .noalias() to indicate there is no aliasing" << endl;
		MatrixXf matA(2, 2), matB(2, 2);
		matA << 2, 0, 0, 2;
		// Simple but not quite as efficient
		matB = matA * matA;
		cout << matB << endl << endl;
		// More complicated but also more efficient
		matB.noalias() = matA * matA;
		cout << matB << endl;
	}
	{
		cout << "aliasing is NOT assumsed if the dest matrix is resized" << endl;
		{
			MatrixXf A(2, 2), B(3, 2);
			B << 2, 0, 0, 3, 1, 1;
			A << 2, 0, 0, -2;
			cout << "B* A\n" << B* A << endl;
			A = (B * A).cwiseAbs(); // aliasing problem
			cout << A << endl;
		}
		{
			MatrixXf A(2, 2), B(3, 2);
			B << 2, 0, 0, 3, 1, 1;
			A << 2, 0, 0, -2;
			A = (B * A).eval().cwiseAbs();
			cout << A << endl;
		}
	}

	system("pause");
}