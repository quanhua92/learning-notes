#include "pca.h"
#include "Eigen\Eigenvalues"

namespace eos {
namespace pca {
	std::pair<Eigen::MatrixXf, Eigen::VectorXf> pca(const Eigen::Ref<const Eigen::MatrixXf> data,
		Covariance covariance_type) {
		using Eigen::MatrixXf;
		using Eigen::VectorXf;

		MatrixXf cov;
		if (covariance_type == Covariance::AtA) {
			cov = data.adjoint() * data;
		}
		else if (covariance_type == Covariance::AAt) {
			cov = data * data.adjoint();
		}
		// the covariance is 1/(n - 1) * AtA (or AAt), so divide by (num_samples - 1)
		cov /= (data.rows() - 1);

		const Eigen::SelfAdjointEigenSolver<MatrixXf> eig(cov);

		const auto num_eigenvectors_to_keep = data.rows() - 1;

		// Select eigenvectors a,d eigenvalues that we want to keep, reverse them (from most significant to least)
		VectorXf eigenvalues = eig.eigenvalues().bottomRows(num_eigenvectors_to_keep).reverse();
		MatrixXf eigenvectors = eig.eigenvectors().rightCols(num_eigenvectors_to_keep).rowwise().reverse();

		if (covariance_type == Covariance::AAt)
		{
			// Bring the AA^t variant in the right form by multiplying with A^t and 1/sqrt(eval):
			// (see e.g. https://math.stackexchange.com/questions/787822/how-do-covariance-matrix-c-aat-justify-the-actual-ata-in-pca)
			// (note the signs might be different from the AtA solution but that's not a problem as the sign of
			// eigenvectors are arbitrary anyway)
			eigenvectors = data.adjoint() * eigenvectors;

			// Multiply each eigenvector (column) with one over the square root of its respective eigenvalue
			// (1/sqrt(eigenvalue(i))): (this is a neat short-hand notation, see
			// https://stackoverflow.com/a/42945996/1345959).
			const VectorXf one_over_sqrt_eigenvalues = eigenvalues.array().rsqrt();
			eigenvectors *= one_over_sqrt_eigenvalues.asDiagonal();

			// Compensate for the covariance division by (n - 1) above:
			eigenvectors /= std::sqrt(data.rows() - 1);
		}

		return { eigenvectors, eigenvalues };
	}

} /*namespace pca*/
} /*namespace eos*/