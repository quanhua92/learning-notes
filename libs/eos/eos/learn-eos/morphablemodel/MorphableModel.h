#pragma once
#ifndef MORPHABLEMODEL_H
#define MORPHABLEMODEL_H

#include "PcaModel.h"

namespace eos {
namespace morphablemodel {
	class MorphableModel {
	public:
		MorphableModel() = default;
		
		MorphableModel(
			PcaModel shape_model
		)
			:shape_model(shape_model) {

		}
	private:
		PcaModel shape_model;
	};

} // namespace morphablemodel
} // namespace eos


#endif // !MORPHABLEMODEL_H

