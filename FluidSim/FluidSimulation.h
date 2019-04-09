#pragma once
#include <Eigen/Core>
#include "./ParticleCollection.h"

namespace FluidSim {
	class FluidSimulation {
	private:
		uint64_t totalTicks;
		double totalAverage;
		double slidingAverage;

		ParticleCollection& particles;
		Eigen::MatrixXd distanceX;
		Eigen::MatrixXd distanceY;
		Eigen::MatrixXd distanceZ;
		Eigen::VectorXd density;
		Eigen::VectorXd pressure;

		public:
			double environmentalPressure;
			double gasConstant;
			double resolution;
			double viscosity;
			double tensionCoefficient;
			double lengthTreshold;
			Eigen::Vector3d gravity; // Gravity as acceleration.

	public:
		void update(const double deltaTime);
		FluidSimulation(ParticleCollection& particles);

	private:
		double smoothingPressure(const Eigen::Vector3d& distance, const double radius);
		Eigen::Vector3d gradientSmoothingPressure(const Eigen::Vector3d& distance, const double radius);
		double laplacianSmoothingPressure(const Eigen::Vector3d& distance, const double radius);

		double laplacianSmoothingViscosity(const Eigen::Vector3d& distance, const double radius);
	};
}