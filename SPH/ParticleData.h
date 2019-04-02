#pragma once
#include <Eigen/Core>

namespace SPH {
	struct ParticleData {
		double mass;
		Eigen::Vector3d position;
		Eigen::Vector3d velocity;
	};
}