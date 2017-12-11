#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

void main(int argc, char** argv) {
	{
		cout << "Matrix3f A, Vector3f b" << endl;
		Matrix3f A;
		Vector3f b;

		A << 1, 2, 3, 4, 5, 6, 7, 8, 10;
		b << 3, 3, 4;
		cout << "Matrix3f A: \n" << A << endl;
		cout << "Vector3f b: \n" << b << endl;

		Vector3f x = A.colPivHouseholderQr().solve(b);
		cout << "Solution:\n" << x << endl;
	}
	{
		cout << "Matrix2f A, b" << endl;
		Matrix2f A, b;
		A << 2, -1, -1, 3;
		b << 1, 2, 3, 1;

		cout << "Matrix2f A: \n" << A << endl;
		cout << "Matrix2f b: \n" << b << endl;
		Matrix2f x = A.ldlt().solve(b);
		cout << "Solution: \n" << x << endl;
	}

	{
		cout << "Checking if a solution really exists" << endl;
		MatrixXd A = MatrixXd::Random(100, 100);
		MatrixXd b = MatrixXd::Random(100, 50);

		MatrixXd x = A.fullPivLu().solve(b);
		double relative_error = (A*x - b).norm() / b.norm(); 
		cout << "Relative error :\n" << relative_error << endl;
	}

	{
		cout << "Computing eigenvalues and eigenvectors" << endl;
		Matrix2f A;
		A << 1, 2, 2, 3;
		cout << "Matrix2f A:\n" << A << endl;

		SelfAdjointEigenSolver<Matrix2f> eigensolver(A);
		if (eigensolver.info() != Success) {
			cout << "EigenSolver failed" << endl;
		}
		else {
			cout << "Eigenvalues of A: \n" << eigensolver.eigenvalues() << endl;
			cout << "Matrix whose columns are eigenvectors of A: \n" << eigensolver.eigenvectors() << endl;
		}
	}
	{
		cout << "Least squares solving" << endl;
		MatrixXf A = MatrixXf::Random(3, 2);
		cout << "MatrixXf A: \n" << A << endl;
		VectorXf b = VectorXf::Random(3);
		cout << "VectorXf b: \n" << b << endl;
		cout << "The least-squares solution is: \n" << A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b) << endl;
	}
	{
		cout << "Separating the computation from the construction" << endl;
		Matrix2f A, b;
		LLT<Matrix2f> llt;
		A << 2, -1, -1, 3;
		b << 1, 2, 3, 1;
		cout << "A:\n" << A << endl;
		cout << "b:\n" << b << endl;
		cout << "Computing LLT decomposition..." << endl;
		llt.compute(A);
		cout << "The solution is: \n" << llt.solve(b) << endl;
		A(1, 1)++;
		cout << "The matrix A is now: \n" << A << endl;
		cout << "Computing LLT decomposition..." << endl;
		llt.compute(A);
		cout << "The solution is: \n" << llt.solve(b) << endl;
	}

	{
		Matrix3f A;
		A << 1, 2, 5,
			2, 1, 4,
			3, 0, 3;
		cout << "Here is the matrix A:\n" << A << endl;
		FullPivLU<Matrix3f> lu_decomp(A);
		cout << "The rank of A is " << lu_decomp.rank() << endl;
		cout << "Here is a matrix whose columns form a basis of the null-space of A:\n"
			<< lu_decomp.kernel() << endl;
		cout << "Here is a matrix whose columns form a basis of the column-space of A:\n"
			<< lu_decomp.image(A) << endl; // yes, have to pass the original A
	}

	{
		cout << "Inplace matrix decompositions" << endl;
		cout << "Useful when dealing with huge matrices and or when the memory is limited" << endl;

		MatrixXd A(2, 2);
		A << 2, -1, 1, 3;
		cout << "A:\n" << A << endl;
		PartialPivLU<Ref<MatrixXd>> lu(A);
		cout << "Matrix A after decomposition: \n" << A << endl;
		cout << "Here is the matrix storing the L and U factors: \n" << lu.matrixLU() << endl;
		cout << "Use lu object to solve the Ax=b problem: " << endl;
		VectorXd b(2); b << 1, 2;
		VectorXd x = lu.solve(b);
		MatrixXd A0(2, 2); A0 << 2, -1, 1, 3; // only for verification of the result.
		cout << "Residual: " << (A0 * x - b).norm() << endl;
	}

	system("pause");
}