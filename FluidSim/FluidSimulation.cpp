#pragma once
#include <cmath>
#include "./FluidSimulation.h"
#include "./ParticleCollection.h"

namespace FluidSim {
	FluidSimulation::FluidSimulation(
		ParticleCollection& particles
	) : environmentalPressure(100000.0), gasConstant(0.08), resolution(0.1), gravity(0.0, -9.81, 0.0),
		particles(particles), density(particles.size), pressure(particles.size),
	    distanceX(particles.size, particles.size), distanceY(particles.size, particles.size), distanceZ(particles.size, particles.size) {
	}

	void FluidSimulation::update(const double deltaTime) {
		// Calculate all particle distances.
		for (size_t i = 0; i < this->particles.size; ++i) {
			for (size_t j = i + 1; j < this->particles.size; ++j) {
				this->distanceX(i, j) = this->particles.positionX[i] - this->particles.positionX[j];
				this->distanceX(j, i) = this->particles.positionX[j] - this->particles.positionX[i];

				this->distanceY(i, j) = this->particles.positionY[i] - this->particles.positionY[j];
				this->distanceY(j, i) = this->particles.positionY[j] - this->particles.positionY[i];

				this->distanceZ(i, j) = this->particles.positionZ[i] - this->particles.positionZ[j];
				this->distanceZ(j, i) = this->particles.positionZ[j] - this->particles.positionZ[i];
			}
		}

		// Calculate local densities.
		for (size_t i = 0; i < this->particles.size; ++i) {
			// Calculate density for particle i.
			for (size_t j = 0; j < this->particles.size; ++j) {
				if (i == j) { continue; }
				Eigen::Vector3d distance(this->distanceX(i, j), this->distanceY(i, j), this->distanceZ(i, j));
				this->density[i] += this->particles.mass[j] * this->smoothingPressure(distance, this->resolution);
			}
		}

		this->pressure = this->gasConstant * (this->density.array() - this->environmentalPressure);
		for (size_t i = 0; i < this->particles.size; ++i) {
			// Forces that need to be calculated;
			Eigen::Vector3d viscosityForce(0.0, 0.0, 0.0);
			Eigen::Vector3d pressureForce(0.0, 0.0, 0.0);
			Eigen::Vector3d externalForce(0.0, 0.0, 0.0);

			// Calculate viscosity force for particle i.
			Eigen::Vector3d velocityI(this->particles.velocityX[i], this->particles.velocityY[i], this->particles.velocityZ[i]);
			for (size_t j = 0; j < this->particles.size; ++j) {
				if (i == j) { continue; }
				Eigen::Vector3d distance(this->distanceX(i, j), this->distanceY(i, j), this->distanceZ(i, j));

				Eigen::Vector3d velocityJ(this->particles.velocityX[j], this->particles.velocityY[j], this->particles.velocityZ[j]);
				viscosityForce += this->particles.mass[j] * (velocityJ - velocityI) / this->density[j]
					            * this->laplacianSmoothingViscosity(distance, this->resolution);
			}
			viscosityForce *= 1.0f; // TODO: mu.

			// Calculate pressure force for particle i.
			for (size_t j = 0; j < this->particles.size; ++j) {
				if (i == j) { continue; }
				Eigen::Vector3d distance(this->distanceX(i, j), this->distanceY(i, j), this->distanceZ(i, j));

				double pressureJ = this->gasConstant * (this->density[j] - this->environmentalPressure);
				pressureForce -= this->particles.mass[j] * ((this->pressure[i] + this->pressure[j]) / (2 * this->pressure[j]))
					             * this->gradientSmoothingPressure(distance, this->resolution);
			}

			// Calculate external forces.
			externalForce += this->gravity * this->density[i];

			// Integrate acceleration for particle.
			Eigen::Vector3d acceleration = (viscosityForce + pressureForce + externalForce) / this->density[i];
			this->particles.velocityX[i] += deltaTime * acceleration[0];
			this->particles.velocityY[i] += deltaTime * acceleration[1];
			this->particles.velocityZ[i] += deltaTime * acceleration[2];

			// TODO: Makeshift boundary condition.
			if(abs(particles.positionX[i] + (deltaTime * particles.velocityX[i])) > 1.0) { particles.velocityX[i] = -particles.velocityX[i]; }
			if(abs(particles.positionY[i] + (deltaTime * particles.velocityY[i])) > 1.0) { particles.velocityY[i] = -particles.velocityY[i]; }
			if(abs(particles.positionZ[i] + (deltaTime * particles.velocityZ[i])) > 1.0) { particles.velocityZ[i] = -particles.velocityZ[i]; }
		}

		// Update positions by integrating velocities.
		this->particles.positionX += deltaTime * particles.velocityX;
		this->particles.positionY += deltaTime * particles.velocityY;
		this->particles.positionZ += deltaTime * particles.velocityZ;
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