#ifndef SIMPLE_SCENE_H
#define SIMPLE_SCENE_H

#include <Windows.h>
// OGL via glew
#include <GL/glew.h>
// include GLFW as the windowing context handling library
#include <GLFW/glfw3.h>
// include GLM math library
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/euler_angles.hpp>

#include "common/loadShader.h"
#include "common/pipeline.h"
#include "common/texture.h"
#include "common/vertexBufferObject.h"
#include "common/skybox.h"
#include "common/assimpModel.h"
#include "common/controls.h"
#include "common/camera.h"

#include "static_geometry.h"

#include <iostream>
#include <stdlib.h>
class AbstractScene
{
public:
	virtual void initScene(GLFWwindow *window) = 0;
	virtual void renderScene(GLFWwindow *window) = 0;
	virtual void releaseScene() = 0;
};

class SimpleScene : public AbstractScene
{
public:
	SimpleScene();
	void initScene(GLFWwindow *window);
	void renderScene(GLFWwindow *window);
	void releaseScene();
private:
	float fTriangle[9];
	float fQuad[12];
	float fTriangleColor[9];
	float fQuadColor[12];

	GLuint uiVBO[4];
	GLuint uiVAO[2];

	const char* vertexShaderSource;
	const char* fragmentShaderSource;
	GLuint programID;
};

#endif // SIMPLE_SCENE_H