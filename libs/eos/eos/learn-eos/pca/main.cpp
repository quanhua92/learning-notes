#include <iostream>
#include <Eigen\Core>
#include "pca.h"

using namespace std;
using Eigen::ArrayXXf;
using Eigen::MatrixXf;
using Eigen::VectorXf;

using namespace eos::pca;

int main(int argc, char** argv) {
	VectorXf eigenvalues;
	MatrixXf eigenvectors;

	MatrixXf meanfree_data = MatrixXf::Random(10, 50); // rows, cols
	cout << "meanfree_data: \n" << meanfree_data << endl;

	Eigen::RowVectorXf mean_data = meanfree_data.colwise().mean();
	cout << "colwise().mean(): \n" << mean_data << endl;

	meanfree_data.rowwise() -= mean_data;
	cout << "meanfree_data: \n" << meanfree_data << endl;

	Covariance covariance_type = Covariance::AtA;

	std::tie(eigenvectors, eigenvalues) = pca(meanfree_data, covariance_type);
	cout << "Eigenvectors: \n" << eigenvectors << endl;
	cout << "Eigenvalues: \n" << eigenvalues << endl;

	system("pause");
	return EXIT_SUCCESS;
}