#pragma once
#include "SPH/FluidSimulation.h"
#include "SPH/ParticleData.h"
#include "SPH/ParticleCollection.h"

//This class contains the entire scene operations, and the engine time loop.
class Scene{
	private:
		SPH::FluidSimulation fluidSimulation;
		SPH::ParticleCollection particles;

	public:  
	  void updateScene(double timeStep) {
		  // Update the fluid simulation.
		  this->fluidSimulation.update(timeStep, this->particles);
	  }
  
	  //loading a scene from the scene .txt files
	  //you do not need to update this function
	  bool loadScene(const std::string dataFolder, const std::string sceneFileName) {
		  SPH::ParticleData data[] = { 
			  { 1.0, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d::Zero() },
			  { 1.0, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d::Zero() },
			  { 1.0, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d::Zero() },
			  { 1.0, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d::Zero() },
			  { 1.0, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d::Zero() },
			  { 1.0, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d::Zero() },
			  { 1.0, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d::Zero() },
			  { 1.0, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d::Zero() },
			  { 1.0, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d::Zero() },
			  { 1.0, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d::Zero() }
		  };
		  this->particles = SPH::ParticleCollection(10, data);
	  }

	  Scene(): particles(0, NULL) {}
};