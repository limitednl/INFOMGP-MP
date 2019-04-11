#pragma once
#include <Eigen/Core>
#include "./ParticleCollection.h"

namespace FluidSim {
	class FluidSimulation {
		constexpr static double RESOLUTION = 1.0;

		private:
			uint64_t totalTicks;
			double totalAverage;
			double slidingAverage;

			Eigen::MatrixXd distanceX;
			Eigen::MatrixXd distanceY;
			Eigen::MatrixXd distanceZ;
			Eigen::VectorXd density;
			Eigen::VectorXd pressure;
			std::vector<std::vector<size_t>> particleGrid;

		public:
			bool useViscosity = true;
			bool usePressure = true;
			bool useGravity = true;
			bool useSurfaceTension = true;

			double wallPressure = 3.0;
			double simulationSize = 5.0;
			double environmentalPressure = 2.0;

			double restitutionCoefficient = 0.6;
			double gasConstant = 20.0;
			double viscosity = 0.5;
			double tensionCoefficient = 0.1;
			double lengthTreshold = 1.0;
			Eigen::Vector3d gravity; // Gravity as acceleration.

			ParticleCollection& particles;

		public:
			void update(const double deltaTime);
			FluidSimulation(ParticleCollection& particles);

		private:
			void rebuildParticleGrid();
			std::vector<size_t> getNeighborhoodIndices(size_t index);

			double smoothingPressure(const Eigen::Vector3d& distance);
			Eigen::Vector3d gradientSmoothingPressure(const Eigen::Vector3d& distance);
			double laplacianSmoothingPressure(const Eigen::Vector3d& distance);
			double laplacianSmoothingViscosity(const Eigen::Vector3d& distance);
	};
}