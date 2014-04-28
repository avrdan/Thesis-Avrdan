#pragma once
#include "simpleScene.h"

class AssimpScene : public AbstractScene
{
public:
	AssimpScene();
	~AssimpScene();

	void initScene(GLFWwindow *window);
	void renderScene(GLFWwindow *window);
	void releaseScene();
private:
	VertexBufferObject vboSceneObjects;
	GLuint uiVAOSceneObjects;
	Skybox skybox;
	AssimpModel amModels[3];

	const char* vertexShaderSource;
	const char* fragmentShaderSource;
	const char* texSnowSource;
	const char* texGoldSource;

	const char* skyboxPath;
	const char* skyFront;
	const char* skyDown;
	const char* skyUp;
	const char* skyRight;
	const char* skyBack;
	const char* skyLeft;

	char* wolfModel;
	char* houseModel;

	GLuint programID;
	Pipeline *pipeline;
};



