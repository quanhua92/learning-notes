#pragma once
#ifndef LINEARSHAPEFITTING_H
#define LINEARSHAPEFITTING_H

#include "morphablemodel\PcaModel.h"
#include "Eigen\Core"
#include "Eigen\Sparse"

#include <vector>
#include <optional>

namespace eos {
namespace fitting {

	inline std::vector<float> fit_shape_to_landmarks_linear(
		const morphablemodel::PcaModel& shape_model,
		Eigen::Matrix<float, 3, 4> affine_camera_matrix,
		const std::vector<Eigen::Vector2f>& landmarks,
		const std::vector<int>& vertex_ids,
		Eigen::VectorXf base_face = Eigen::VectorXf(),
		float lambda = 3.0,
		std::optional<int> num_coefficients_to_fit = std::optional<int>(),
		std::optional<float> detector_standard_deviation = std::optional<float>(),
		std::optional<float> model_standard_deviation = std::optional<float>()
	);

} // namespace fitting
} // namespace eos


#endif // !LINEARSHAPEFITTING_H
