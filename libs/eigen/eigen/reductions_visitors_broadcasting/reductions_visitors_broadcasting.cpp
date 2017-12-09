#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;


void main() {

	{
		Eigen::Matrix2d mat;
		mat << 1, 2,
			3, 4;
		cout << "mat: \n" << mat << endl;
		cout << "Here is mat.sum():       " << mat.sum() << endl;
		cout << "Here is mat.prod():      " << mat.prod() << endl;
		cout << "Here is mat.mean():      " << mat.mean() << endl;
		cout << "Here is mat.minCoeff():  " << mat.minCoeff() << endl;
		cout << "Here is mat.maxCoeff():  " << mat.maxCoeff() << endl;
		cout << "Here is mat.trace():     " << mat.trace() << endl;
	}
	{
		VectorXf v(2);
		MatrixXf m(2, 2), n(2, 2);

		v << -1,
			2;

		m << 1, -2,
			-3, 4;
		cout << "v.squaredNorm() = " << v.squaredNorm() << endl;
		cout << "v.norm() = " << v.norm() << endl;
		cout << "v.lpNorm<1>() = " << v.lpNorm<1>() << endl;
		cout << "v.lpNorm<2>() = " << v.lpNorm<2>() << endl;
		cout << "v.lpNorm<Infinity>() = " << v.lpNorm<Infinity>() << endl;
		cout << endl;
		cout << "m.squaredNorm() = " << m.squaredNorm() << endl;
		cout << "m.norm() = " << m.norm() << endl;
		cout << "m.lpNorm<1>() = " << m.lpNorm<1>() << endl;
		cout << "m.lpNorm<2>() = " << m.lpNorm<2>() << endl;
		cout << "m.lpNorm<Infinity>() = " << m.lpNorm<Infinity>() << endl;
	}
	{
		MatrixXf m(2, 2);
		m << 1, -2, -3, 4;
		cout << "1-norm(m) = " << m.cwiseAbs().colwise().sum().maxCoeff() << " == " <<
			m.colwise().lpNorm<1>().maxCoeff() << endl;
		cout << "infty-norm(m) = " << m.cwiseAbs().rowwise().sum().maxCoeff()
			<< " == " << m.rowwise().lpNorm<1>().maxCoeff() << endl;
	}

	{
		ArrayXXf a(2, 2);

		a << 1, 2,
			3, 4;
		cout << "(a > 0).all()   = " << (a > 0).all() << endl;
		cout << "(a > 0).any()   = " << (a > 0).any() << endl;
		cout << "(a > 0).count() = " << (a > 0).count() << endl;
		cout << endl;
		cout << "(a > 2).all()   = " << (a > 2).all() << endl;
		cout << "(a > 2).any()   = " << (a > 2).any() << endl;
		cout << "(a > 2).count() = " << (a > 2).count() << endl;
	}
	
	{
		// Visitors
		Eigen::MatrixXf m(2, 2);

		m << 1, 2,
			4, 4;
		//get location of maximum
		MatrixXf::Index maxRow, maxCol;
		float max = m.maxCoeff(&maxRow, &maxCol);
		//get location of minimum
		MatrixXf::Index minRow, minCol;
		float min = m.minCoeff(&minRow, &minCol);
		cout << "Max: " << max << ", at: " <<
			maxRow << "," << maxCol << endl;
		cout << "Min: " << min << ", at: " <<
			minRow << "," << minCol << endl;
	}
	{
		Eigen::MatrixXf mat(2, 4);
		mat << 1, 2, 6, 9,
			   3, 1, 7, 2;
		cout << "column's maximum : " << endl;
		cout << mat.colwise().maxCoeff() << endl;
	}
	{
		Eigen::MatrixXf mat(2, 4);
		Eigen::VectorXf v(2);

		mat << 1, 2, 6, 9,
			   3, 1, 7, 2;
		v << 0, 
			 1;
		// add v to each column of m
		mat.colwise() += v;
		cout << "Broadcasting colwise: \n" << mat << endl;

		Eigen::VectorXf w(4);
		w << 0, 1, 2, 3;

		mat.rowwise() += w.transpose();

		cout << "Broadcasting rowwise: \n" << mat << endl;
	}
	{
		Eigen::MatrixXf m(2, 4);
		Eigen::VectorXf v(2);
		m << 1, 23, 6, 9,
			 3, 11, 7, 2;
		v << 2,
			 3;
		MatrixXf::Index index;
		// find nearest neighbour
		(m.colwise() - v).colwise().squaredNorm().minCoeff(&index);
		cout << "Nearest neighbour is column " << index << ":" << endl;
		cout << m.col(index) << endl;

	}
	system("pause");
}