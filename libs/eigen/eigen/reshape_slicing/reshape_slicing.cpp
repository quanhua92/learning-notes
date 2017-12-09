#include <iostream>
#include <Eigen\Dense>

using namespace Eigen;
using namespace std;

void main() {
	{
		MatrixXf M1(3, 3); // column-major storage
		M1 << 1, 2, 3, 4, 5, 6, 7, 8, 9;
		cout << "M1: \n" << M1 << endl;
		Map<RowVectorXf> v1(M1.data(), M1.size());
		cout << "v1: \n" << v1 << endl;

		Matrix<float, Dynamic, Dynamic, RowMajor> M2(M1);
		cout << "M2: \n" << M2 << endl;
		Map<RowVectorXf> v2(M2.data(), M2.size());
		cout << "v2: \n" << v2 << endl;
	}
	{
		MatrixXf M1(2, 6);
		M1 << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12;
		cout << "M1: \n" << M1 << endl;
		Map<VectorXf> v1(M1.data(), M1.size());
		cout << "v1: \n" << v1 << endl;
		Map<MatrixXf> M2(M1.data(), 6, 2);
		Map<VectorXf> v2(M2.data(), M2.size());
		cout << "M2: \n" << M2 << endl;
		cout << "v2: \n" << v2 << endl;
	}
	{
		// Slicing
		RowVectorXf v = RowVectorXf::LinSpaced(20, 0, 19);
		cout << "Input: \n" << v << endl;
		Map<RowVectorXf, 0, InnerStride<2>> v2(v.data(), v.size() / 2);
		cout << "Even: \n" << v2 << endl;
	}
	{
		MatrixXf M1 = MatrixXf::Random(3, 8);
		cout << "Column major input: \n" << M1 << endl;
		Map<MatrixXf, 0, OuterStride<>> M2(M1.data(), M1.rows(), 
										  (M1.cols() + 2) / 3, 
										   OuterStride<>(M1.outerStride() * 3));
		cout << "1 column over 3: \n" << M2 << endl;

		typedef Matrix<float, Dynamic, Dynamic, RowMajor> RowMajorMatrixXf;
		RowMajorMatrixXf M3(M1);
		cout << "Row major input:" << endl << M3 << "\n";
		Map<RowMajorMatrixXf, 0, Stride<Dynamic, 3> > M4(M3.data(), M3.rows(), (M3.cols() + 2) / 3,
			Stride<Dynamic, 3>(M3.outerStride(), 3));
		cout << "1 column over 3:" << endl << M4 << "\n";
	}
	system("pause");
}