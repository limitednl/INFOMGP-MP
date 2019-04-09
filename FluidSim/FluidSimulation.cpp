#pragma once
#include <cmath>
#include <chrono>
#include <vector>
#include <random>
#include <iostream>
#include "./FluidSimulation.h"
#include "./ParticleCollection.h"

namespace FluidSim {
	FluidSimulation::FluidSimulation(
		ParticleCollection& particles
	) : totalTicks(0), totalAverage(0.0), slidingAverage(0.0), gravity(0.0, -9.81, 0.0),
		particles(particles), density(particles.size), pressure(particles.size),
		distanceX(particles.size, particles.size), distanceY(particles.size, particles.size), distanceZ(particles.size, particles.size) {
	}

	void FluidSimulation::update(const double deltaTime) {
		auto startTime = std::chrono::high_resolution_clock::now();

		// Pre-fill grid with particles in each cell.
		this->rebuildParticleGrid();

		// Calculate local densities.
		this->density.setZero();
		for (size_t i = 0; i < this->particles.size; ++i) {
			// Calculate density for particle i.
			std::vector<size_t> neighbours = this->getNeighborhoodIndices(i);
			for (auto j = neighbours.begin(); j != neighbours.end(); ++j) {
				Eigen::Vector3d distance(
					this->particles.positionX[*j] - this->particles.positionX[i],
					this->particles.positionY[*j] - this->particles.positionY[i],
					this->particles.positionZ[*j] - this->particles.positionZ[i]
				);
				this->density[i] += this->particles.mass[*j] * this->smoothingPressure(distance);
			}
		}

		this->pressure = this->gasConstant * (this->density.array() - this->environmentalPressure);

		for (size_t i = 0; i < this->particles.size; ++i) {
			// Forces that need to be calculated;
			Eigen::Vector3d viscosityForce(0.0, 0.0, 0.0);
			Eigen::Vector3d pressureForce(0.0, 0.0, 0.0);
			Eigen::Vector3d externalForce(0.0, 0.0, 0.0);
			Eigen::Vector3d surfaceForce(0.0, 0.0, 0.0);

			// Get the neighbours.
			std::vector<size_t> neighbours = this->getNeighborhoodIndices(i);

			// Calculate viscosity force for particle i.
			Eigen::Vector3d velocityI(this->particles.velocityX[i], this->particles.velocityY[i], this->particles.velocityZ[i]);
			for (auto j = neighbours.begin(); j != neighbours.end(); ++j) {
				Eigen::Vector3d distance(
					this->particles.positionX[*j] - this->particles.positionX[i],
					this->particles.positionY[*j] - this->particles.positionY[i],
					this->particles.positionZ[*j] - this->particles.positionZ[i]
				);

				Eigen::Vector3d velocityJ(this->particles.velocityX[*j], this->particles.velocityY[*j], this->particles.velocityZ[*j]);
				viscosityForce += this->particles.mass[*j] * (velocityJ - velocityI) / this->density[*j] * this->laplacianSmoothingViscosity(distance);
			}
			viscosityForce *= this->viscosity;

			// Calculate pressure force for particle i.
			for (auto j = neighbours.begin(); j != neighbours.end(); ++j) {
				Eigen::Vector3d distance(
					this->particles.positionX[*j] - this->particles.positionX[i],
					this->particles.positionY[*j] - this->particles.positionY[i],
					this->particles.positionZ[*j] - this->particles.positionZ[i]
				);

				Eigen::Vector3d smoothingFactor = this->gradientSmoothingPressure(distance);
				Eigen::Vector3d partialPressureForce = this->particles.mass[*j] * ((this->pressure[i] + this->pressure[*j]) / (2 * this->density[*j])) * smoothingFactor;
				pressureForce -= partialPressureForce;
			}

			// Wall pressure "hack".
			{
				Eigen::Vector3d smoothingFactor = this->gradientSmoothingPressure(Eigen::Vector3d(-this->simulationSize - this->particles.positionX[i], 0.0, 0.0));
				Eigen::Vector3d partialPressureForce = this->wallPressure * smoothingFactor;
				pressureForce -= partialPressureForce;
			}

			{
				Eigen::Vector3d smoothingFactor = this->gradientSmoothingPressure(Eigen::Vector3d(this->simulationSize - this->particles.positionX[i], 0.0, 0.0));
				Eigen::Vector3d partialPressureForce = this->wallPressure * smoothingFactor;
				pressureForce -= partialPressureForce;
			}

			{
				Eigen::Vector3d smoothingFactor = this->gradientSmoothingPressure(Eigen::Vector3d(0.0, -this->simulationSize - this->particles.positionY[i], 0.0));
				Eigen::Vector3d partialPressureForce = this->wallPressure * smoothingFactor;
				pressureForce -= partialPressureForce;
			}

			{
				Eigen::Vector3d smoothingFactor = this->gradientSmoothingPressure(Eigen::Vector3d(0.0, this->simulationSize - this->particles.positionY[i], 0.0));
				Eigen::Vector3d partialPressureForce = this->wallPressure * smoothingFactor;
				pressureForce -= partialPressureForce;
			}

			{
				Eigen::Vector3d smoothingFactor = this->gradientSmoothingPressure(Eigen::Vector3d(0.0, 0.0, -this->simulationSize - this->particles.positionZ[i]));
				Eigen::Vector3d partialPressureForce = this->wallPressure * smoothingFactor;
				pressureForce -= partialPressureForce;
			}

			{
				Eigen::Vector3d smoothingFactor = this->gradientSmoothingPressure(Eigen::Vector3d(0.0, 0.0, this->simulationSize - this->particles.positionZ[i]));
				Eigen::Vector3d partialPressureForce = this->wallPressure * smoothingFactor;
				pressureForce -= partialPressureForce;
			}


			double cfLaplacian = 0;
			Eigen::Vector3d n = Eigen::Vector3d::Zero();

			//Calculate the surface tension
			for (auto j = neighbours.begin(); j != neighbours.end(); ++j) {
				Eigen::Vector3d distance(
					this->particles.positionX[*j] - this->particles.positionX[i],
					this->particles.positionY[*j] - this->particles.positionY[i],
					this->particles.positionZ[*j] - this->particles.positionZ[i]
				);

				auto weight = particles.mass[*j] * 1 / density[*j];

				//colorfield gradient
				n += weight * gradientSmoothingPressure(distance);
				cfLaplacian += weight * laplacianSmoothingPressure(distance);
			}

			//only apply if the norm is big enough
			if (n.squaredNorm() > lengthTreshold * lengthTreshold)
				surfaceForce = -tensionCoefficient * cfLaplacian * n.normalized();

			// Calculate external forces.
			if (this->useGravity) {
				externalForce += this->gravity * this->density[i];
			}

			if (this->useSurfaceTension) {
				externalForce += surfaceForce;
			}

			// Integrate acceleration for particle.
			Eigen::Vector3d acceleration = externalForce;
			if (this->useViscosity) { acceleration += viscosityForce; }
			if (this->usePressure) { acceleration += pressureForce; }
			acceleration /= this->density[i];

			this->particles.velocityX[i] += deltaTime * acceleration[0];
			this->particles.velocityY[i] += deltaTime * acceleration[1];
			this->particles.velocityZ[i] += deltaTime * acceleration[2];
		}

		// Update positions by integrating velocities.
		this->particles.positionX += deltaTime * particles.velocityX;
		this->particles.positionY += deltaTime * particles.velocityY;
		this->particles.positionZ += deltaTime * particles.velocityZ;

		// TODO: Makeshift boundary condition.
		const double offset = 1e-5;
		const double imperfection = 1e-6;
		std::random_device rd;
		std::mt19937 e2(rd());
		std::uniform_real_distribution<double> dist(0.0, imperfection);

		const double maxBoundary = this->simulationSize - offset - dist(e2);
		const double minBoundary = -this->simulationSize + offset + dist(e2);
		for (size_t i = 0; i < this->particles.size; ++i) {
			if (abs(particles.positionX[i]) > maxBoundary) {
				particles.positionX[i] = std::min(maxBoundary, std::max(minBoundary, particles.positionX[i]));
				particles.velocityX[i] = -this->restitutionCoefficient * particles.velocityX[i];
			}

			if (abs(particles.positionY[i]) > maxBoundary) {
				particles.positionY[i] = std::min(maxBoundary, std::max(minBoundary, particles.positionY[i]));
				particles.velocityY[i] = -this->restitutionCoefficient * particles.velocityY[i];
			}

			if (abs(particles.positionZ[i]) > maxBoundary) {
				particles.positionZ[i] = std::min(maxBoundary, std::max(minBoundary, particles.positionZ[i]));
				particles.velocityZ[i] = -this->restitutionCoefficient * particles.velocityZ[i];
			}
		}

		auto endTime = std::chrono::high_resolution_clock::now();
		double duration = std::chrono::duration_cast<std::chrono::duration<double>>(endTime - startTime).count();

		// Update the total average.
		this->totalAverage = ((totalAverage * this->totalTicks) + duration) / (this->totalTicks + 1);
		++this->totalTicks;

		// Update the sliding average.
		this->slidingAverage = 0.9 * this->slidingAverage + 0.1 * duration;

		printf_s("Fluid update took %.5fs. (%.5f | %.5f for %zu ticks)\n", duration, this->slidingAverage, this->totalAverage, this->totalTicks);
	}

	double FluidSimulation::smoothingPressure(const Eigen::Vector3d & distance) {
		constexpr double radius = FluidSimulation::RESOLUTION;
		constexpr double radiusSq = radius * radius;
		constexpr double radiusP9 = radius * radius * radius * radius * radius * radius * radius * radius * radius;

		if (distance.squaredNorm() > radiusSq) { return 0; }

		constexpr double radiusFactor = 315 / (64 * 3.14 * radiusP9);
		double distanceComponent = pow(radiusSq - distance.squaredNorm(), 3);
		return radiusFactor * distanceComponent;
	}

	Eigen::Vector3d FluidSimulation::gradientSmoothingPressure(const Eigen::Vector3d & distance) {
		//constexpr double radius = FluidSimulation::RESOLUTION;
		//constexpr double radiusSq = radius * radius;
		//constexpr double radiusP9 = radius * radius * radius * radius * radius * radius * radius * radius * radius;

		//if (distance.squaredNorm() > radiusSq) { return Eigen::Vector3d::Zero(); }
		//constexpr double radiusFactor = 945 / (32 * 3.14 * radiusP9);
		//Eigen::Vector3d distanceComponent = distance * pow(radius * radius - distance.squaredNorm(), 2);
		//return radiusFactor * distanceComponent;
		constexpr double radius = FluidSimulation::RESOLUTION;
		constexpr double radiusSq = radius * radius;
		constexpr double radiusP6 = radius * radius * radius * radius * radius * radius;

		if (distance.squaredNorm() <= 0 || distance.squaredNorm() > radiusSq) { return Eigen::Vector3d::Zero(); }
		constexpr double radiusFactor = 45 / (3.14 * radiusP6);
		Eigen::Vector3d distanceComponent = distance.normalized() * pow(radius - distance.norm(), 2);
		return radiusFactor * distanceComponent;
	}

	double FluidSimulation::laplacianSmoothingPressure(const Eigen::Vector3d & distance) {
		constexpr double radius = FluidSimulation::RESOLUTION;
		constexpr double radiusSq = radius * radius;
		constexpr double radiusP9 = radius * radius * radius * radius * radius * radius * radius * radius * radius;

		if (distance.squaredNorm() > radiusSq) { return 0.0; }
		constexpr double radiusFactor = 945 / (32 * 3.14 * radiusP9);
		double distanceComponent = (radius * radius - distance.squaredNorm()) * (3 * radius * radius - 7 * distance.squaredNorm());
		return radiusFactor * distanceComponent;
	}

	double FluidSimulation::laplacianSmoothingViscosity(const Eigen::Vector3d & distance) {
		constexpr double radius = FluidSimulation::RESOLUTION;
		constexpr double radiusSq = radius * radius;
		constexpr double radiusP6 = radius * radius * radius * radius * radius * radius;

		if (distance.squaredNorm() > radiusSq) { return 0.0; }
		constexpr double radiusFactor = 45 / (3.14 * radiusP6);
		double distanceComponent = radius - distance.norm();
		return radiusFactor * distanceComponent;
	}

	void FluidSimulation::rebuildParticleGrid() {
		const double cellSize = FluidSimulation::RESOLUTION;
		const int gridSize = (int) ceil(2 * this->simulationSize / cellSize);
		this->particleGrid = std::vector<std::vector<size_t>>(gridSize * gridSize * gridSize);

		for (size_t i = 0; i < this->particles.size; ++i) {
			const int gridX = (int) ((this->particles.positionX[i] + this->simulationSize) / cellSize);
			const int gridY = (int) ((this->particles.positionY[i] + this->simulationSize) / cellSize);
			const int gridZ = (int) ((this->particles.positionZ[i] + this->simulationSize) / cellSize);
			const int cellIndex = gridSize * gridSize * gridX + gridSize * gridY + gridZ;
			this->particleGrid[cellIndex].push_back(i);
		}
	}

	std::vector<size_t> FluidSimulation::getNeighborhoodIndices(size_t index) {
		std::vector<size_t> indices;

		const double cellSize = FluidSimulation::RESOLUTION;
		const int gridSize = (int) ceil(2 * this->simulationSize / cellSize);
		const int centerX = (int) ((this->particles.positionX[index] + this->simulationSize) / cellSize);
		const int centerY = (int) ((this->particles.positionY[index] + this->simulationSize) / cellSize);
		const int centerZ = (int) ((this->particles.positionZ[index] + this->simulationSize) / cellSize);

		for (int x = -1; x <= 1; ++x) {
			for (int y = -1; y <= 1; ++y) {
				for (int z = -1; z <= 1; ++z) {
					int cellIndex = gridSize * gridSize * (centerX + x) + gridSize * (centerY + y) + (centerZ + z);
					if (cellIndex < 0 || cellIndex >= gridSize * gridSize * gridSize) { continue; }
					std::vector<size_t>& cell = this->particleGrid[cellIndex];

					indices.insert(indices.end(), cell.begin(), cell.end());
				}
			}
		}

		return indices;
	}
}