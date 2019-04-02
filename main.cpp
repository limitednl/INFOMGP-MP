#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <iostream>
#include "scene.h"

igl::opengl::glfw::Viewer viewer;

float currTime = 0;

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
			//scene.updateScene();
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
		//scene.updateScene();
	}

	return false;
}

int main(int argc, char* argv[]) {
	using namespace Eigen;
	using namespace std;

	// Load scene
	if (argc < 3) {
		cout << "Please provide path (argument 1 and name of scene file (argument 2)!" << endl;
		return 0;
	}

	/*
	TODO: setup world here
	1. load world
	2. add default meshes
	*/

	//add a menu
	igl::opengl::glfw::imgui::ImGuiMenu menu;
	viewer.plugins.push_back(&menu);
	viewer.callback_key_down = &key_down;

	while (relaunch) {
		//handle fullscreen
		relaunch = false;
		viewer.core.viewport[2] = 1200;
		viewer.core.viewport[3] = 800;
		viewer.launch(!fScreen, fScreen);
	}
}