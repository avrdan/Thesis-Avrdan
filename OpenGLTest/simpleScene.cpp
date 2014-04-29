#include "simpleScene.h"
//#include "common/loadShader.h"

SimpleScene::SimpleScene()
{
	vertexShaderSource = "./assets/shaders/shader.vert";
	fragmentShaderSource = "./assets/shaders/shader.frag";
}

void SimpleScene::initScene(GLFWwindow *window)
{
	glClearColor(0.0f, 0.5f, 1.0f, 1.0f);

	// setup triangle vertices
	fTriangle[0] = -0.4f; fTriangle[1] = 0.1f; fTriangle[2] = 0.0f;
	fTriangle[3] = 0.4f;  fTriangle[4] = 0.1f; fTriangle[5] = 0.0f;
	fTriangle[6] = 0.0f;  fTriangle[7] = 0.7f; fTriangle[8] = 0.0f;

	// setup triangle color
	fTriangleColor[0] = 1.0f; fTriangleColor[1] = 0.0f; fTriangleColor[2] = 0.0f;
	fTriangleColor[3] = 0.0f; fTriangleColor[4] = 1.0f; fTriangleColor[5] = 0.0f;
	fTriangleColor[6] = 0.0f; fTriangleColor[7] = 0.0f; fTriangleColor[8] = 1.0f;

	// setup quad vertices
	fQuad[0] = -0.2f; fQuad[1] = -0.1f; fQuad[2] = 0.0f;
	fQuad[3] = -0.2f; fQuad[4] = -0.6f; fQuad[5] = 0.0f;
	fQuad[6] = 0.2f; fQuad[7] = -0.1f; fQuad[8] = 0.0f;
	fQuad[9] = 0.2f; fQuad[10] = -0.6f; fQuad[11] = 0.0f;

	// setup quad color
	fQuadColor[0] = 1.0f; fQuadColor[1] = 0.0f; fQuadColor[2] = 0.0f;
	fQuadColor[3] = 0.0f; fQuadColor[4] = 1.0f; fQuadColor[8] = 0.0f;
	fQuadColor[6] = 0.0f; fQuadColor[7] = 0.0f; fQuadColor[5] = 1.0f;
	fQuadColor[9] = 1.0f; fQuadColor[10] = 1.0f; fQuadColor[11] = 0.0f;

	// generate two VAOs
	glGenVertexArrays(2, uiVAO);
	// create vbos
	glGenBuffers(4, uiVBO);

	// setup whole triangle
	glBindVertexArray(uiVAO[0]);

	glBindBuffer(GL_ARRAY_BUFFER, uiVBO[0]);
	glBufferData(GL_ARRAY_BUFFER, 9 * sizeof(float), fTriangle, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, uiVBO[1]);
	glBufferData(GL_ARRAY_BUFFER, 9 * sizeof(float), fTriangleColor, GL_STATIC_DRAW);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
	
	// setup whole quad
	glBindVertexArray(uiVAO[1]);

	glBindBuffer(GL_ARRAY_BUFFER, uiVBO[2]);
	glBufferData(GL_ARRAY_BUFFER, 12 * sizeof(float), fQuad, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, uiVBO[3]);
	glBufferData(GL_ARRAY_BUFFER, 12 * sizeof(float), fQuadColor, GL_STATIC_DRAW);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);

	programID = LoadShaders(vertexShaderSource, fragmentShaderSource);
	glUseProgram(programID);
}

void SimpleScene::releaseScene()
{
	// Cleanup VBO & Shader
	glDeleteBuffers(4, uiVBO);
	glDeleteProgram(programID);
	glDeleteVertexArrays(2, uiVAO);

	// Close OpenGL window and terminate GLFW
	glfwTerminate();
}

void SimpleScene::renderScene(GLFWwindow *window)
{
	glClear(GL_COLOR_BUFFER_BIT);
	/*
	glEnableVertexAttribArray(0);
	//Triangle render
	glBindBuffer(GL_ARRAY_BUFFER, uiVBO[0]);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glDrawArrays(GL_TRIANGLES, 0, 3);

	//Quad render using triangle strip
	glBindBuffer(GL_ARRAY_BUFFER, uiVBO[1]);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	*/

	glBindVertexArray(uiVAO[0]);
	glDrawArrays(GL_TRIANGLES, 0, 3);

	glBindVertexArray(uiVAO[1]);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

	// Swap front and back buffers
	glfwSwapBuffers(window);

	// Poll for and process events
	glfwPollEvents();

	//glDisableVertexAttribArray(0);
}