// http://www.opengl-tutorial.org/beginners-tutorials/tutorial-1-opening-a-window/
#include <iostream>
#include <GL\glew.h>

#include <GLFW\glfw3.h>

#include <glm\glm.hpp>

using namespace glm;
using namespace std;

int main(int argc, char** argv) {

	// Initialize GLFW
	if (!glfwInit()) {
		cout << "Failed to initialize GLFW" << endl;
		return EXIT_FAILURE;
	}

	glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // to make MacOS happy
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);


	// Open a window & create its OpenGL Context
	GLFWwindow* window;
	window = glfwCreateWindow(1024, 768, "Tutorial 01", NULL, NULL);
	if (window == NULL) {
		cout << "Failed to open GLFW window" << endl;
		glfwTerminate();
		return EXIT_FAILURE;
	}

	glfwMakeContextCurrent(window); // Initialize GLEW
	glewExperimental = true; // need in core profile
	if (glewInit() != GLEW_OK) {
		cout << "Failed to initialize GLEW" << endl;
		return EXIT_FAILURE;
	}

	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE); // capture keys

	do {
		// draw nothing
		// swap buffers
		glfwSwapBuffers(window);
		glfwPollEvents();
	} while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
		glfwWindowShouldClose(window) == 0);

	// Close OpenGL window and terminate GLFW
	glfwTerminate();
	return EXIT_SUCCESS;
}
