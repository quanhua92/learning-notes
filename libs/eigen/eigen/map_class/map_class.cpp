#include <Eigen/Dense>
#include <iostream>

using namespace std;
using namespace Eigen;

typedef Matrix<float, 1, Dynamic> MatrixType;
typedef Map<MatrixType> MapType;
typedef Map<const MatrixType> MapTypeConst; // a read-only map
const int n_dims = 5;

void main() {
	{
		int arr[8];
		for (int i = 0; i < 8; i++) {
			arr[i] = i;
		}

		cout << "Column-major: \n" << Map<Matrix<int, 2, 4>>(arr) << endl;
		cout << "Row-major: \n" << Map<Matrix<int, 2, 4, RowMajor>>(arr) << endl;
		cout << "Row-major using stride: \n" << Map<Matrix<int, 2, 4>, Unaligned, Stride<1, 4>>(arr) << endl;
	}

	{
		MatrixType m1(n_dims), m2(n_dims);
		m1.setRandom();
		m2.setRandom();

		float * p = &m2(0); // get the addres storing the data for m2
		MapType m2map(p, m2.size()); //m2map shares data with m2
		MapTypeConst m2mapconst(p, m2.size()); // a read-only accessor for m2

		cout << "m1: \n" << m1 << endl;
		cout << "m2: \n" << m2 << endl;
		cout << "Squared euclidean distance: \n" << (m1 - m2).squaredNorm() << endl;
		cout << "Squared euclidean distance, using map: \n" << (m1 - m2map).squaredNorm() << endl;
		m2map(3) = 7; // this will change m2, since they share the same array
		cout << "Updated m2: \n" << m2 << endl;
		cout << "m2 coefficient 2, constant accessor: \n" << m2mapconst(2) << endl;

	}

	{
		// changing the mapped array
		int data[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
		Map<RowVectorXi> v(data, 4);
		cout << "The mapped vector v is : " << v << endl;
		new(&v) Map<RowVectorXi>(data + 4, 5);
		cout << "Now v is : " << v << endl;
	}

	{
		// declare a Map obj without first knowing the mapped array's location in memory
		//int n_matrices = 4;
		//Map<Matrix3f> A(NULL);
		//VectorXf b(n_matrices);
		//for (int i = 0; i < n_dims; i++) {
		//	new (&A) Map<Matrix3f>(get_matrix_pointer(i));
		//	b(i) = A.trace();
		//}
	}

	system("pause");
}