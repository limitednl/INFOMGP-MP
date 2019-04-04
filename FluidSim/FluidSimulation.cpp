#pragma once
#include <cmath>
#include "./FluidSimulation.h"
#include "./ParticleCollection.h"

namespace FluidSim {
	FluidSimulation::FluidSimulation(): environmentalPressure(100000.0), gasConstant(0.08), 
		                                resolution(0.1), gravity(0.0, -9.81, 0.0) {}

	void FluidSimulation::update(const double deltaTime, ParticleCollection& particles) {
		// Calculate local densities.
		Eigen::VectorXd densities(particles.size); 
		densities.setZero();

		for (size_t i = 0; i < particles.size; ++i) {
			// Calculate density for particle i.
			for (size_t j = 0; j < particles.size; ++j) {
				if (i == j) { continue; }
				double distanceX = particles.positionX[i] - particles.positionX[j];
				double distanceY = particles.positionY[i] - particles.positionY[j];
				double distanceZ = particles.positionZ[i] - particles.positionZ[j];
				Eigen::Vector3d distance(distanceX, distanceY, distanceZ);

				densities[i] += particles.mass[j] * this->smoothingPressure(distance, this->resolution);
			}
		}

		for (size_t i = 0; i < particles.size; ++i) {
			// Forces that need to be calculated;
			Eigen::Vector3d viscosityForce(0.0, 0.0, 0.0);
			Eigen::Vector3d pressureForce(0.0, 0.0, 0.0);
			Eigen::Vector3d externalForce(0.0, 0.0, 0.0);

			// Calculate viscosity force for particle i.
			Eigen::Vector3d velocityI(particles.velocityX[i], particles.velocityY[i], particles.velocityZ[i]);
			for (size_t j = 0; j < particles.size; ++j) {
				if (i == j) { continue; }
				double distanceX = particles.positionX[i] - particles.positionX[j];
				double distanceY = particles.positionY[i] - particles.positionY[j];
				double distanceZ = particles.positionZ[i] - particles.positionZ[j];
				Eigen::Vector3d distance(distanceX, distanceY, distanceZ);

				Eigen::Vector3d velocityJ(particles.velocityX[j], particles.velocityY[j], particles.velocityZ[j]);
				viscosityForce += particles.mass[j] * (velocityJ - velocityI) / densities[j] 
					            * this->laplacianSmoothingViscosity(distance, this->resolution);
			}
			viscosityForce *= 1.0f; // TODO: mu.

			// Calculate pressure force for particle i.
			double pressureI = this->gasConstant * (densities[i] - this->environmentalPressure);
			for (size_t j = 0; j < particles.size; ++j) {
				if (i == j) { continue; }
				double distanceX = particles.positionX[i] - particles.positionX[j];
				double distanceY = particles.positionY[i] - particles.positionY[j];
				double distanceZ = particles.positionZ[i] - particles.positionZ[j];
				Eigen::Vector3d distance(distanceX, distanceY, distanceZ);

				double pressureJ = this->gasConstant * (densities[j] - this->environmentalPressure);
				pressureForce -= particles.mass[j] * ((pressureI + pressureJ) / (2 * pressureJ)) 
					             * this->gradientSmoothingPressure(distance, this->resolution);
			}

			// Calculate external forces.
			externalForce += this->gravity * densities[i];

			// Integrate acceleration for particle.
			Eigen::Vector3d acceleration = (viscosityForce + pressureForce + externalForce) / densities[i];
			particles.velocityX[i] += deltaTime * acceleration[0];
			particles.velocityY[i] += deltaTime * acceleration[1];
			particles.velocityZ[i] += deltaTime * acceleration[2];

			// TODO: Makeshift boundary condition.
			if(abs(particles.positionX[i] + (deltaTime * particles.velocityX[i])) > 1.0) { particles.velocityX[i] = -particles.velocityX[i]; }
			if(abs(particles.positionY[i] + (deltaTime * particles.velocityY[i])) > 1.0) { particles.velocityY[i] = -particles.velocityY[i]; }
			if(abs(particles.positionZ[i] + (deltaTime * particles.velocityZ[i])) > 1.0) { particles.velocityZ[i] = -particles.velocityZ[i]; }
		}

		// Update positions by integrating velocities.
		particles.positionX += deltaTime * particles.velocityX;
		particles.positionY += deltaTime * particles.velocityY;
		particles.positionZ += deltaTime * particles.velocityZ;
	}

	double FluidSimulation::smoothingPressure(const Eigen::Vector3d& distance, const double radius) {
		if (distance.squaredNorm() < radius * radius) { return 0; }
		double distanceComponent = pow(radius * radius - distance.squaredNorm(), 3);
		return 315 / (64 * 3.14 * pow(radius, 9)) * distanceComponent;
	}

	Eigen::Vector3d FluidSimulation::gradientSmoothingPressure(const Eigen::Vector3d& distance, const double radius) {
		if (distance.squaredNorm() < radius * radius) { return Eigen::Vector3d::Zero(); }
		Eigen::Vector3d distanceComponent = distance * pow(radius * radius - distance.squaredNorm(), 2);
		return 945 / (32 * 3.14 * pow(radius, 9)) * distanceComponent;
	}

	double FluidSimulation::laplacianSmoothingViscosity(const Eigen::Vector3d& distance, const double radius) {
		if (distance.squaredNorm() < radius * radius) { return 0.0; }
		double distanceComponent = radius - distance.norm();
		return 45 / (3.14 * pow(radius, 6)) * distanceComponent;
	}
}