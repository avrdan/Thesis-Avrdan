#ifndef PYRAMID_SCENE_H
#define PYRAMID_SCENE_H

#include "simpleScene.h"
#include "common/loadShader.h"
#include "common/pipeline.h"

class PyramidScene : public AbstractScene
{
public:
	PyramidScene();
	void initScene(GLFWwindow *window);
	void renderScene(GLFWwindow *window);
	void releaseScene();
private:
	float fPyramid[36]; // 4 triangles of 3 verts of 3 floats
	float fPyramidColor[36];

	GLuint uiVBO[2]; // pos + color
	GLuint uiVAO[1];

	bool bShowFPS;
	bool bVerticalSync;

	const char* vertexShaderSource;
	const char* fragmentShaderSource;
	GLuint programID;
	Pipeline *pipeline;
	float fRotationAngle;
	float PIover180;
};

#endif // PYRAMID_SCENE_H