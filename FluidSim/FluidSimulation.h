#pragma once
#include <Eigen/Core>
#include "./ParticleCollection.h"

namespace FluidSim {
	class FluidSimulation {
		public:
			double environmentalPressure;
			double gasConstant;
			double resolution;
			Eigen::Vector3d gravity; // Gravity as acceleration.

		public:
			void update(const double deltaTime, ParticleCollection& particles);
			FluidSimulation();

		private:
			double smoothingPressure(const Eigen::Vector3d& distance, const double radius);
			Eigen::Vector3d gradientSmoothingPressure(const Eigen::Vector3d& distance, const double radius);
			double laplacianSmoothingViscosity(const Eigen::Vector3d& distance, const double radius);
	};
}