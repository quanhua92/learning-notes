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

	{
		// transposition and conjugation
		MatrixXcf a = MatrixXcf::Random(2, 2);
		cout << "Here is the matrix a\n" << a << endl;
		cout << "Here is the matrix a^T\n" << a.transpose() << endl;
		cout << "Here is the conjugate of a\n" << a.conjugate() << endl;
		cout << "Here is the matrix a^*\n" << a.adjoint() << endl;
	}
	{
		// transpose and adjoint return a proxy object without doing the actual transpotition
		Matrix2i a; a << 1, 2, 3, 4;
		cout << "a = \n" << a << endl;
		Matrix2i b = a.transpose(); // this is ok
		cout << "b = \n" << b << endl;
		//a = a.transpose(); // !!! do NOT do this. This is the aliasing issue and automatically detected in debug.
		//cout << "a doesn't change after a = a.tranpose(). a = \n" << a << endl;
		// Should use the transposeInPlace()
		a.transposeInPlace();
		cout << "a.transposeInPlace() = \n" << a << endl;
	}
	{
		// Matrix multiplication
		Matrix2d mat;
		mat << 1, 2,
			   3, 4;
		Vector2d u(-1, 1), v(2, 0);
		cout << "Here is mat * mat: \n" << mat * mat << endl;
		cout << "Here is mat * u: \n" << mat * u << endl;
		cout << "Here is u^T * mat: \n" << u.transpose()*mat << endl;
		cout << "Here is u^T * v: \n" << u.transpose() * v << endl;
		cout << "Here is u * v^T: \n" << u * v.transpose() << endl;
		cout << "Let's multiply mat by itself " << endl;
		mat *= mat;
		cout << "Now mat is mat: \n" << mat << endl;
	}
	{
		// Dot product
		Vector3d v(1, 2, 3);
		Vector3d w(0, 1, 2);
		cout << "Dot product: " << v.dot(w) << endl;
		double dp = v.adjoint() * w;
		cout << "Dot product via a matrix product: " << dp << endl;
		cout << "Cross product: \n" << v.cross(w) << endl;
	}

	{
		// basic arithmetic reduction operations
		Eigen::Matrix2d mat;
		mat << 1, 2, 3, 4;
		cout << "mat = " << mat << endl;
		cout << "mat.sum()  = " << mat.sum() << endl;
		cout << "mat.prod() = " << mat.prod() << endl;
		cout << "mat.mean() = " << mat.mean() << endl;
		cout << "mat.minCoeff() = " << mat.minCoeff() << endl;
		cout << "mat.maxCoeff() = " << mat.maxCoeff() << endl;
		cout << "mat.trace() = " << mat.trace() << endl;

		Matrix3f m = Matrix3f::Random();
		std::ptrdiff_t i, j;
		float minOfM = m.minCoeff(&i, &j);
		cout << "m = \n" << m << endl;
		cout << "minOfM = " << minOfM << " at i " << i << " j " << j << endl;

		RowVector4i v = RowVector4i::Random();
		int maxOfV = v.maxCoeff(&i);
		cout << "v = \n" << v << endl;
		cout << "maxOfV = " << maxOfV << " at i = " << i << endl;

	}


	system("pause");
	return 0;
}