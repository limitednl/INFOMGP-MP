#pragma once
#include "./FluidSimulation.h"
#include "./ParticleCollection.h"

namespace SPH {
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
				double distance = sqrt(distanceX * distanceX + distanceY * distanceY + distanceZ * distanceZ);

				densities[i] += particles.mass[j] * this->smoothingPressure(distance, this->resolution);
			}
		}

		for (size_t i = 0; i < particles.size; ++i) {
			// Forces that need to be calculated;
			Eigen::Vector3d viscosityForce(0.0, 0.0, 0.0);
			Eigen::Vector3d pressureForce(0.0, 0.0, 0.0);
			Eigen::Vector3d externalForce(0.0, 0.0, 0.0);

			// Calculate viscosity force for particle i.
			// TODO!

			// Calculate pressure force for particle i.
			double pressureI = this->gasConstant * (densities[i] - this->environmentalPressure);
			for (size_t j = 0; j < particles.size; ++j) {
				if (i == j) { continue; }
				double distanceX = particles.positionX[i] - particles.positionX[j];
				double distanceY = particles.positionY[i] - particles.positionY[j];
				double distanceZ = particles.positionZ[i] - particles.positionZ[j];
				double distance = sqrt(distanceX * distanceX + distanceY * distanceY + distanceZ * distanceZ);

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
			if(abs(particles.positionX[i] + (deltaTime * particles.velocityX[i])) > 5.0) { particles.velocityX[i] = -particles.velocityX[i]; }
			if (abs(particles.positionY[i] + (deltaTime * particles.velocityY[i])) > 5.0) { particles.velocityY[i] = -particles.velocityY[i]; }
			if (abs(particles.positionZ[i] + (deltaTime * particles.velocityZ[i])) > 5.0) { particles.velocityZ[i] = -particles.velocityZ[i]; }
		}

		// Update positions by integrating velocities.
		particles.positionX += deltaTime * particles.velocityX;
		particles.positionY += deltaTime * particles.velocityY;
		particles.positionZ += deltaTime * particles.velocityZ;
	}

	double FluidSimulation::smoothingPressure(const double distance, const double radius) {
		return 0.0;
	}

	Eigen::Vector3d FluidSimulation::gradientSmoothingPressure(const double distance, const double radius) {
		return Eigen::Vector3d::Zero();
	}
}