#pragma once
#include <Eigen/Core>
#include "./ParticleData.h"

namespace SPH {
	class ParticleCollection {
		public:
			// The size of the collection.
			size_t size;

			// Mass data.
			Eigen::VectorXd mass;

			// Split-out position data.
			Eigen::VectorXd positionX;
			Eigen::VectorXd positionY;
			Eigen::VectorXd positionZ;

			// Split-out position data.
			Eigen::VectorXd velocityX;
			Eigen::VectorXd velocityY;
			Eigen::VectorXd velocityZ;

		public:
			ParticleCollection(const size_t size, const ParticleData* data);
	};
}