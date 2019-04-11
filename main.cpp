#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <iostream>
#include "scene.h"
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

igl::opengl::glfw::Viewer globalViewer;

float currTime = 0;
float timeStep = 0.02;

//initial values

Scene scene;

bool relaunch = true;
bool fScreen = false;

bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier) {
	if (key == ' ') {
		viewer.core.is_animating = !viewer.core.is_animating;
		if (viewer.core.is_animating)
			cout << "Simulation running" << endl;
		else
			cout << "Simulation paused" << endl;
		return true;
	}

	if (key == 'S') {
		if (!viewer.core.is_animating) {
			scene.updateScene(timeStep);
			currTime += timeStep;

			viewer.data().clear();
			scene.draw(viewer);
			std::cout << "currTime: " << currTime << std::endl;
			return true;
		}
	}

	if (key == 'V') {
		glfwSetWindowShouldClose(viewer.window, GLFW_TRUE);
		fScreen = !fScreen;
		relaunch = true;
	}

	return false;
}

bool pre_draw(igl::opengl::glfw::Viewer& viewer) {
	using namespace Eigen;
	using namespace std;

	if (viewer.core.is_animating) {
		scene.updateScene(timeStep);
		currTime += timeStep;

		scene.draw(viewer);
	}

	return false;
}

class CustomMenu : public igl::opengl::glfw::imgui::ImGuiMenu
{

	virtual void draw_viewer_menu() override {
		// Draw parent menu
		//ImGuiMenu::draw_viewer_menu();

		// Add new group
		if (ImGui::CollapsingHeader("Simulation Options", ImGuiTreeNodeFlags_DefaultOpen)) {

			if (ImGui::Button("Reset"))
			{
				//reset world
				globalViewer.core.is_animating = false;
				scene.loadScene(globalViewer);
				scene.updateScene(timeStep);
				scene.draw(globalViewer);
			}

			ImGui::Checkbox("Use Gravity", &scene.fluidSimulation->useGravity);
			ImGui::Checkbox("Use Viscosity", &scene.fluidSimulation->useViscosity);
			ImGui::Checkbox("Use Pressure", &scene.fluidSimulation->usePressure);
			ImGui::Checkbox("Use Surface Tension", &scene.fluidSimulation->useSurfaceTension);

			ImGui::InputDouble("COR", &scene.fluidSimulation->restitutionCoefficient, 0, 0);
			ImGui::InputDouble("Wall Pressure", &scene.fluidSimulation->wallPressure, 0, 0);
			ImGui::InputDouble("Gravity X", &scene.fluidSimulation->gravity[0], 0, 0);
			ImGui::InputDouble("Gravity Y", &scene.fluidSimulation->gravity[1], 0, 0);
			ImGui::InputDouble("Gravity Z", &scene.fluidSimulation->gravity[2], 0, 0);


			if (ImGui::InputFloat("Time Step", &timeStep)) {
				globalViewer.core.animation_max_fps = (((int)1.0 / timeStep));
			}
		}

		if (ImGui::CollapsingHeader("Fluid Options", ImGuiTreeNodeFlags_DefaultOpen)) {
			ImGui::InputDouble("Environmental Pressure", &scene.fluidSimulation->environmentalPressure, 0, 0);
			ImGui::InputDouble("Gas Constant", &scene.fluidSimulation->gasConstant, 0, 0);
			ImGui::InputDouble("Viscosity", &scene.fluidSimulation->viscosity, 0, 0);
		}
	}
};

int main(int argc, char* argv[]) {
	using namespace Eigen;
	using namespace std;

	// Load scene
	//if (argc < 1) {
	//	//cout << "Please provide path (argument 1 and name of scene file (argument 2)!" << endl;
	//	return 0;
	//}

	/*
	TODO: setup world here
	1. load world
	2. add default meshes
	*/
	scene.loadScene(globalViewer);
	scene.updateScene(timeStep);
	scene.draw(globalViewer);

	//add a menu
	globalViewer.callback_key_down = &key_down;
	globalViewer.callback_pre_draw = &pre_draw;

	CustomMenu menu;
	globalViewer.plugins.push_back(&menu);

	while (relaunch) {
		//handle fullscreen
		relaunch = false;
		globalViewer.core.viewport[2] = 1200;
		globalViewer.core.viewport[3] = 800;
		globalViewer.launch(!fScreen, fScreen);
	}
}


