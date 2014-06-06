#pragma once
#include "simpleScene.h"
class TextureScene :
	public AbstractScene
{
public:
	TextureScene();
	~TextureScene();

	void initScene(GLFWwindow *window);
	void renderScene(GLFWwindow *window);
	void releaseScene();
private:
	const char* vertexShaderSource;
	const char* fragmentShaderSource;

	const char* hmVertexShaderSource;
	const char* hmFragmentShaderSource;

	const char* texSnowSource;
	const char* texGoldSource;

	const char* skyboxPath;
	const char* skyFront;
	const char* skyDown;
	const char* skyUp;
	const char* skyRight;
	const char* skyBack;
	const char* skyLeft;

	GLuint programID;
	GLuint programID_HeatMap;
	Pipeline *pipeline;
	float fRotationAngle;
	float PIover180;

	VertexBufferObject vboSceneObjects;
	GLuint uiVAO; // only one VAO
	Texture tGold, tSnow;

	Skybox skybox;

	float timer;
	bool bShowHeatMap;
};

