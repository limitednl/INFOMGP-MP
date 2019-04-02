#pragma once

namespace SPH {
	class FluidSimulation {
		void update(const double deltaTime, ParticleCollection& particles);
	};
}