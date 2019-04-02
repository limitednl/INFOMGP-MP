#include "./ParticleCollection.h"

namespace SPH {
	ParticleCollection::ParticleCollection(
		const size_t size,
		const ParticleData* data
	) : size(size), mass(size);
		positionX(size), positionY(size), positionZ(size),
		velocityX(size), velocityY(size), velocityZ(size) {
		for (size_t i = 0; i < size; ++i) {
			// Write mass data.
			this->mass[i] = data[i].mass;
			
			// Write position data.
			this->positionX[i] = data[i].position[0];
			this->positionY[i] = data[i].position[1];
			this->positionZ[i] = data[i].position[2];

			// Write velocity data.
			this->velocityX[i] = data[i].velocity[0];
			this->velocityY[i] = data[i].velocity[1];
			this->velocityZ[i] = data[i].velocity[2];
		}
	}
}