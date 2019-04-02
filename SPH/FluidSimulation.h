#pragma once

namespace SPH {
	class FluidSimulation {
		Eigen::Vector3d gravity; // Gravity as acceleration.

		FluidSimulation(Eigen::Vector3d gravity);

		void update(const double deltaTime, ParticleCollection& particles);
	};
}