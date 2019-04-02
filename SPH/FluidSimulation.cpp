#pragma once
#include "./FluidSimulation.h"
#include "./ParticleCollection.h"

namespace SPH {
	void FluidSimulation::update(const double deltaTime, ParticleCollection& particles) {
		// Update positions by integrating velocities.
		particles.positionX += deltaTime * particles.velocityX;
		particles.positionY += deltaTime * particles.velocityY;
		particles.positionZ += deltaTime * particles.velocityZ;
	}
}