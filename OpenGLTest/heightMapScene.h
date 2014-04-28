#pragma once
#include "simpleScene.h"
#include "common/pipeline.h"

#define HM_SIZE_X 4
#define HM_SIZE_Y 4

class HeightMapScene :
	public AbstractScene
{
public:
	HeightMapScene();
	~HeightMapScene();
	void initScene(GLFWwindow *window);
	void renderScene(GLFWwindow *window);
	void releaseScene();
private:
	const char* vertexShaderSource;
	const char* fragmentShaderSource;
	GLuint programID;
	Pipeline *pipeline;
	float fRotationAngle;
	float PIover180;
};

