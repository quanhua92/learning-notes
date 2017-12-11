#include <iostream>
#include <Eigen\Core>
#include <Eigen\SVD>

using namespace Eigen;
using namespace std;

template <typename Derived>
void print_size(const EigenBase<Derived>& b) {
	std::cout << "size (rows, cols): " << b.size() << " (" << b.rows() << ", " << b.cols() << ")" << std::endl;
}

template <typename Derived>
void print_block(const DenseBase<Derived>& b, int x, int y, int r, int c) {
	cout << "block: " << b.block(x, y, r, c) << endl;
}

template <typename Derived>
void print_max_coeff(const ArrayBase<Derived> &a) {
	cout << "max: " << a.maxCoeff() << endl;
}

template <typename Derived>
void print_inv_cond(const MatrixBase<Derived>& a) {
	const typename JacobiSVD<typename Derived::PlainObject>::SingularValuesType &sing_vals = a.jacobiSvd().singularValues();
	cout << "inv cond: " << sing_vals(sing_vals.size() - 1) / sing_vals(0) << endl;
}

template <typename DerivedA, typename DerivedB>
typename DerivedA::Scalar squaredist(const MatrixBase<DerivedA>& p1, const MatrixBase<DerivedB>& p2) {
	return (p1 - p2).squaredNorm();
}

float inv_cond(const Ref<const MatrixXf>& a) {
	const VectorXf sing_vals = a.jacobiSvd().singularValues();
	return sing_vals(sing_vals.size() - 1) / sing_vals(0);
}

void cov(const Ref<const MatrixXf> x, const Ref<const MatrixXf> y, Ref<MatrixXf> C) {
	const float num_observations = static_cast<float>(x.rows());
	const RowVectorXf x_mean = x.colwise().sum() / num_observations;
	const RowVectorXf y_mean = y.colwise().sum() / num_observations;
	C = (x.rowwise() - x_mean).transpose() * (y.rowwise() - y_mean) / num_observations;

}

void main() {
	{
		cout << "EigenBase Example" << endl;
		Vector3f v;
		print_size(v);
		print_size(v.asDiagonal());
	}

	{
		Matrix4f m = Matrix4f::Random();
		cout << "matrix m: \n" << m << endl;
		cout << "inv_cond(m): \n" << inv_cond(m) << endl;
		cout << "inv_cond(m(1:3, 1:3)): \n" << inv_cond(m.topLeftCorner(3, 3)) << endl;
		cout << "inv_cond(m+I):\n" << inv_cond(m + Matrix4f::Identity()) << endl;
	}
	{
		MatrixXf m1, m2, m3;
		m1 = MatrixXf::Random(5, 5);
		m2 = MatrixXf::Random(5, 5);
		m3 = MatrixXf::Random(5, 5);

		cov(m1, m2, m3);
		cov(m1.leftCols<3>(), m2.leftCols<3>(), m3.topLeftCorner<3, 3>());
	}
	system("pause");
}
