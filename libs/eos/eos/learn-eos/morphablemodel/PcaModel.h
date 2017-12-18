#pragma once

#ifndef PCAMODEL_H
#define PCAMODEL_H

#include "Eigen\Core"
#include <array>
#include <random>
#include <string>
#include <vector>
#include <fstream>

namespace eos {
namespace morphablemodel {

	class PcaModel
	{
	public:
		PcaModel(Eigen::VectorXf mean, Eigen::MatrixXf orthonormal_pca_basis, Eigen::VectorXf eigenvalues,
			std::vector<std::array<int, 3>> triangle_list)
			: mean(mean), orthonormal_pca_basis(orthonormal_pca_basis), eigenvalues(eigenvalues), triangle_list(triangle_list)
		{

		}
	private:
		Eigen::VectorXf mean; // 3m x 1 col-vector (xyzxyz...) where m is the number of model-vertices
		Eigen::MatrixXf orthonormal_pca_basis; // m x n (rows x cols) = numShapeDims x numShapePcaCoeffs. each column is an eigenvector
		Eigen::MatrixXf rescaled_pca_basis;
		Eigen::VectorXf eigenvalues; // a col-vector of the eigenvalues
		std::vector<std::array<int, 3>> triangle_list; // list of triangles that make up the mesh of the model

		// TODO: serialize using cereal
	};

} // namespace morphablemodel
} // namespace eos

#endif // !PCAMODEL_H

