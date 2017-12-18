#ifndef PCA_H
#define PCA_H
#include <vector>
#include <array>

#include "Eigen\Core"

namespace eos {
namespace pca {

	enum class Covariance {
		AtA, // traditional cov matrix A^t * A
		AAt, // use the inner product A * A^t
	};

	/*
	Compute PCA on the given mean-centred data matrix.
	The function returns n - 1 eigenvectors and eigenvalues, where n is the number of data samples given.
	*/
	std::pair<Eigen::MatrixXf, Eigen::VectorXf> pca(const Eigen::Ref<const Eigen::MatrixXf> data,
		Covariance covariance_type = Covariance::AtA);


} /* namespace pca */
} /* namespace eos */


#endif // !PCA_H
