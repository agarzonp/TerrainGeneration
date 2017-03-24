#include <iostream>

#define GLEW_STATIC
#include "GL/glew.h"
#include "GLFW/glfw3.h"
#include "glm/glm.hpp"

#include "Input/Input.h"
#include "TerrainGeneration/TerrainGeneration.h"

#include <chrono>

int main()
{
	// init glfw
	if (!glfwInit())
	{
		std::cerr << "GLFW init failed!" << std::endl;
		return -1;
	}

	// set window configuration
	//glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	//glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	//glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // only support modern opengl

	// create the window
	GLFWwindow* window = glfwCreateWindow(1024, 768, "SplineCam", nullptr, nullptr);
	if (!window)
	{
		std::cerr << "GLFW failed to create the window!" << std::endl;
		return -1;
	}

	glfwMakeContextCurrent(window);

	// Init glew
	glewExperimental = GL_TRUE;
	if (glewInit() != GLEW_OK)
	{
		std::cerr << "GLEW init failed!" << std::endl;
		return -1;
	}

	glEnable(GL_DEPTH_TEST);

	// init SplineCam
	TerrainGeneration splineCam;

	// init input
	Input::SetWindow(window);
	Input::SetListener(static_cast<InputListener*>(&splineCam));
	glfwSetKeyCallback(window, Input::OnKeyCallback);
	glfwSetMouseButtonCallback(window, Input::OnMouseButtonCallback);
	glfwSetScrollCallback(window, Input::OnMouseScrollCallback);
	glfwSetCursorPosCallback(window, Input::OnMouseMoveCallback);

	const auto frameCap = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<long, std::ratio<1, 60>>(1));
	auto deltaTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::microseconds::zero());
	auto previousTime = std::chrono::steady_clock::now();

	// main loop
	while (!glfwWindowShouldClose(window))
	{
		glfwPollEvents();

		auto currentTime = std::chrono::steady_clock::now();
		deltaTime += std::chrono::duration_cast<std::chrono::microseconds>(currentTime - previousTime);
		previousTime = currentTime;

		if (deltaTime > frameCap) 
		{
			glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			splineCam.Update((float)deltaTime.count() / 1000000);
			splineCam.Render();

			glfwSwapBuffers(window);

			deltaTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::microseconds::zero());
		}
	}

	glfwTerminate();

	return 0;
}



