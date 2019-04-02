#pragma once
#include <Eigen/Core>
#include "./ParticleCollection.h"

namespace SPH {
	class FluidSimulation {
		public:
			double environmentalPressure;
			double gasConstant;
			double resolution;
			Eigen::Vector3d gravity; // Gravity as acceleration.

		public:
			void update(const double deltaTime, ParticleCollection& particles);
			FluidSimulation(): environmentalPressure(0.0), gasConstant(0.0), resolution(0.0), gravity(0.0, 9.81, 0.0) {}

		private:
			double smoothingPressure(const double distance, const double radius);
			Eigen::Vector3d gradientSmoothingPressure(const double distance, const double radius);
	};
}