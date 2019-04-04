#pragma once
#include <Eigen/Core>

namespace FluidSim {
	struct ParticleData {
		double mass;
		Eigen::Vector3d position;
		Eigen::Vector3d velocity;
	};
}