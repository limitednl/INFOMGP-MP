#pragma once
#include "./FluidSimulation.h"
#include "./ParticleCollection.h"

namespace SPH {
	FluidSimulation::FluidSimulation(Eigen::Vector3d gravity): gravity(gravity) {}

	void FluidSimulation::update(const double deltaTime, ParticleCollection& particles) {
		// Update velocity with gravity.
		particles.velocityX.array() += deltaTime * this->gravity[0];
		particles.velocityY.array() += deltaTime * this->gravity[1];
		particles.velocityZ.array() += deltaTime * this->gravity[2];

		// Update positions by integrating velocities.
		particles.positionX += deltaTime * particles.velocityX;
		particles.positionY += deltaTime * particles.velocityY;
		particles.positionZ += deltaTime * particles.velocityZ;
	}
}