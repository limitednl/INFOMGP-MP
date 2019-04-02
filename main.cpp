#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <iostream>
#include "scene.h"
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

igl::opengl::glfw::Viewer viewer;

float currTime = 0;

//initial values

Scene scene;

bool relaunch = true;
bool fScreen = false;

bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier)
{
	if (key == ' ')
	{
		viewer.core.is_animating = !viewer.core.is_animating;
		if (viewer.core.is_animating)
			cout << "Simulation running" << endl;
		else
			cout << "Simulation paused" << endl;
		return true;
	}

	if (key == 'S')
	{
		if (!viewer.core.is_animating) {
			//scene.updateScene();
			return true;
		}
	}

	if (key == 'V')
	{
		glfwSetWindowShouldClose(viewer.window, GLFW_TRUE);
		fScreen = !fScreen;
		relaunch = true;
	}

	return false;
}

void GetSphereMesh(double radius, MatrixXd& vertices, MatrixXi& faces) {
	vertices.resize(8, 3);
	vertices <<
		-radius, radius, -radius,
		-radius, radius, radius,
		radius, radius, radius,
		radius, radius, -radius,
		-radius, -radius, -radius,
		-radius, -radius, radius,
		radius, -radius, radius,
		radius, -radius, -radius;

	faces.resize(12, 3);
	faces <<
		0, 1, 2,
		2, 3, 0,
		6, 5, 4,
		4, 7, 6,
		1, 0, 5,
		0, 4, 5,
		2, 1, 6,
		1, 5, 6,
		3, 2, 7,
		2, 6, 7,
		0, 3, 4,
		3, 7, 4;
}

bool pre_draw(igl::opengl::glfw::Viewer& viewer)
{
	using namespace Eigen;
	using namespace std;

	if (viewer.core.is_animating) {
		RowVector3d color; color << 0.8, 0.1, 0.1; //red-ish?

		//todo: set;
		MatrixXd vertices;
		MatrixXi faces;

		//x,y,z,radius
		Vector4d particle; particle << 0, 0, 0, 20;

 		GetSphereMesh(1, vertices, faces);

		viewer.data_list[0].clear();
		viewer.data_list[0].set_mesh(vertices, faces);
		viewer.data_list[0].set_face_based(true);
		viewer.data_list[0].show_lines = false;
		viewer.data_list[0].set_colors(color);
		viewer.core.align_camera_center(vertices);
	}

	return false;
}

int main(int argc, char* argv[])
{
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
	viewer.callback_pre_draw = &pre_draw;

	while (relaunch)
	{
		//handle fullscreen
		relaunch = false;
		viewer.core.viewport[2] = 1200;
		viewer.core.viewport[3] = 800;
		viewer.launch(!fScreen, fScreen);
	}
}


