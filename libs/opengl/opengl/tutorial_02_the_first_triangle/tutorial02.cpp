// http://www.opengl-tutorial.org/beginners-tutorials/tutorial-2-the-first-triangle/
#include <iostream>
#include <GL\glew.h>

#include <GLFW\glfw3.h>

#include <glm\glm.hpp>

#include "common\shader.h"

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
	window = glfwCreateWindow(1024, 768, "Tutorial 02", NULL, NULL);
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



	GLuint VertexArrayID;
	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);

	// an array of 3 vectors which represents 3 vertices
	static const GLfloat g_vertex_buffer_data[] = {
		-1.0f, -1.0f, 0.0f,
		 1.0f, -1.0f, 0.0f,
		 0.0f,  1.0f, 0.0f,
	};

	// this will identify our vertex buffer
	GLuint vertexbuffer;

	// generate 1 buffer, put the resulting identifier in vertexbuffer
	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

	// Create and compile our GLSL program from the shaders
	GLuint programID = LoadShaders("SimpleVertexShader.vertexshader", "SimpleFragmentShader.fragmentshader");
	glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

	do {
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glUseProgram(programID);

		// draw the triangle
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
		glVertexAttribPointer(
			0, // attribute 0
			3, // size
			GL_FLOAT, // type
			GL_FALSE, // normalized?
			0, // stride,
			(void*)0// offset
		);

		glDrawArrays(GL_TRIANGLES, 0, 3); // start from vertex 0; 3 vertices total -> 1 triangle
		glDisableVertexAttribArray(0);
		// swap buffers
		glfwSwapBuffers(window);
		glfwPollEvents();
	} while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
		glfwWindowShouldClose(window) == 0);

	// Cleanup VBO
	glDeleteBuffers(1, &vertexbuffer);
	glDeleteVertexArrays(1, &VertexArrayID);
	glDeleteProgram(programID);

	// Close OpenGL window and terminate GLFW
	glfwTerminate();
	return EXIT_SUCCESS;
}
