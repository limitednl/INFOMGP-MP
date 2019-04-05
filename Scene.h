#pragma once
#include <Eigen/Core>
#include <igl/opengl/glfw/Viewer.h>
#include "./FluidSim/FluidSimulation.h"
#include "./FluidSim/ParticleData.h"
#include "./FluidSim/ParticleCollection.h"

//This class contains the entire scene operations, and the engine time loop.
class Scene{
	private:
		double particleRadius;
		Eigen::MatrixXd particleVertices;
		Eigen::MatrixXi particleFaces;

	public:
		FluidSim::FluidSimulation* fluidSimulation;
		FluidSim::ParticleCollection* particles;

	public:
		size_t getMeshCount();
		void draw(igl::opengl::glfw::Viewer& viewer);

		void updateScene(double timeStep);
 
		//loading a scene from the scene .txt files
		//you do not need to update this function
		bool loadScene(const std::string dataFolder, const std::string sceneFileName);
		
		Scene();
		~Scene();
};