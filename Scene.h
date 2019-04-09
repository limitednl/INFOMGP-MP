#pragma once
#include <vector>
#include <utility>
#include <Eigen/Core>
#include <igl/opengl/glfw/Viewer.h>
#include "./FluidSim/FluidSimulation.h"
#include "./FluidSim/ParticleData.h"
#include "./FluidSim/ParticleCollection.h"

//This class contains the entire scene operations, and the engine time loop.
class Scene{
	private:
		bool loaded;
		double particleRadius;
		Eigen::MatrixXd particleVertices;
		Eigen::MatrixXi particleFaces;
		std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> meshes;

	public:
		FluidSim::FluidSimulation* fluidSimulation;
		FluidSim::ParticleCollection* particles;

	public:
		size_t getMeshCount();
		void draw(igl::opengl::glfw::Viewer& viewer);

		void updateScene(double timeStep);
 
		//loading a scene from the scene .txt files
		//you do not need to update this function
		bool loadScene(const std::string dataFolder, const std::string sceneFileName, igl::opengl::glfw::Viewer& viewer);
		
		Scene();
		~Scene();

	private:
		void generateParticleMesh();
};