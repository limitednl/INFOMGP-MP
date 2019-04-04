#include "./Scene.h"
#include <cmath>
#include <random>
#include <iostream>
#include <Eigen/Core>

Scene::Scene() : particles(0, NULL), particleRadius(0.1) {}

size_t Scene::getMeshCount() { 
	return this->particles.size; 
}

void Scene::draw(igl::opengl::glfw::Viewer& viewer) {
	Eigen::RowVector3d color; color << 0.1, 0.1, 0.8; //blue-ish?
	for (size_t i = 0; i < this->particles.size; ++i) {
		Eigen::Matrix4d transform; transform << 
			this->particleRadius,                  0.0,                  0.0, this->particles.positionX[i],
			                 0.0, this->particleRadius,                  0.0, this->particles.positionY[i],
			                 0.0,                  0.0, this->particleRadius, this->particles.positionZ[i],
			                 0.0,                  0.0,                  0.0,                          1.0;

		Eigen::MatrixXd transformedVertices = (transform * this->particleVertices.transpose()).transpose();
		Eigen::MatrixXd transformedMesh = transformedVertices.block(0, 0, transformedVertices.rows(), 3);
		viewer.data_list[i].clear();
		viewer.data_list[i].set_mesh(transformedMesh, this->particleFaces);
		viewer.data_list[i].set_face_based(true);
		viewer.data_list[i].show_lines = false;
		viewer.data_list[i].set_colors(color);
	}

	Eigen::RowVector3d edgeColor; edgeColor << 0.8, 0.1, 0.1; //red-ish?
	Eigen::MatrixXd p1Edges(12, 3); p1Edges << 
		 1.0,  1.0,  1.0,
		 1.0,  1.0, -1.0,
		-1.0,  1.0, -1.0,
		-1.0,  1.0,  1.0,

		 1.0, -1.0,  1.0,
		 1.0, -1.0, -1.0,
		-1.0, -1.0, -1.0,
		-1.0, -1.0,  1.0,

		 1.0,  1.0,  1.0,
		 1.0,  1.0, -1.0,
		-1.0,  1.0,  1.0,
		-1.0,  1.0, -1.0;
	Eigen::MatrixXd p2Edges(12, 3); p2Edges << 
		 1.0,  1.0, -1.0,
		-1.0,  1.0, -1.0,
		-1.0,  1.0,  1.0,
		 1.0,  1.0,  1.0,

		 1.0, -1.0, -1.0,
		-1.0, -1.0, -1.0,
		-1.0, -1.0,  1.0,
		 1.0, -1.0,  1.0,

		 1.0, -1.0,  1.0,
		 1.0, -1.0, -1.0,
		-1.0, -1.0,  1.0,
		-1.0, -1.0, -1.0;
	viewer.data().add_edges(p1Edges, p2Edges, edgeColor);
}

void Scene::updateScene(double timeStep) {
	// Update the fluid simulation.
	this->fluidSimulation.update(timeStep, this->particles);
}

bool Scene::loadScene(const std::string dataFolder, const std::string sceneFileName) {
	std::random_device rd;
	std::mt19937 e2(rd());
	std::uniform_real_distribution<> dist(-0.5, 0.5);

	const size_t count = 400;
	const double particleSize = 2 * this->particleRadius;
	const double areaSize = 2.0;

	double ppd = areaSize / particleSize;
	FluidSim::ParticleData data[count];
	for (size_t i = 0; i < count; ++i) {
		double y = -0.5 * areaSize * particleSize * floor(i / (ppd * ppd));
		double z = -0.5 * areaSize + particleSize * floor((i % (int) (ppd * ppd)) / ppd);
		double x = -0.5 * areaSize + particleSize * (i % (int) ppd);

		data[i] = { 0.0997, Eigen::Vector3d(x + this->particleRadius, y + this->particleRadius, z + this->particleRadius), Eigen::Vector3d::Zero() };
	}

	this->particles = FluidSim::ParticleCollection(count, data);

	// Generate meshes.
	const int sphereResolution = 10;
	const int spherePoints = (sphereResolution + 1) * sphereResolution;
	this->particleVertices.resize(spherePoints, 4);
	for (size_t m = 0; m <= sphereResolution; ++m) {
		for (size_t n = 0; n < sphereResolution; ++n) {
			size_t index = sphereResolution * m + n;
			const double PI = 3.14;
			Eigen::Vector4d pointPos(
				sinf(PI * m / sphereResolution) * cosf(2 * PI * n / sphereResolution),
				sinf(PI * m / sphereResolution) * sinf(2 * PI * n / sphereResolution),
				cosf(PI * m / sphereResolution),
				1.0f
			);
			std::stringstream slabel; slabel << index;
			//viewer.data().add_label(pointPos, slabel.str());

			this->particleVertices.row(index) = pointPos;
		}
	}

	//viewer.data().add_points(baseVertices.block(0, 0, spherePoints, 3), color);
	//return;

	this->particleFaces.resize(2 * sphereResolution * sphereResolution, 3);
	for (int m = 0; m < sphereResolution; ++m) {
		for (int n = 0; n < sphereResolution; ++n) {
			const int faceIndex = 2 * (sphereResolution * m + n);

			const int colOffset = sphereResolution * m;
			const int rowOffset = n;

			const int k1 = colOffset + rowOffset;
			const int k2 = colOffset + sphereResolution + rowOffset;
			const int k11 = colOffset + ((rowOffset + 1) % sphereResolution);
			const int k21 = colOffset + sphereResolution + ((rowOffset + 1) % sphereResolution);

			this->particleFaces.row(faceIndex) = Eigen::Vector3i(k1, k2, k11);
			this->particleFaces.row(faceIndex + 1) = Eigen::Vector3i(k11, k2, k21);
		}
	}

	return true;
}