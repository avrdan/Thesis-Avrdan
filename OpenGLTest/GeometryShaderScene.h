#pragma once
#include "simpleScene.h"
class GeometryShaderScene :
	public AbstractScene
{
public:
	GeometryShaderScene();
	~GeometryShaderScene();

	void initScene(GLFWwindow *window);
	void renderScene(GLFWwindow *window);
	void releaseScene();
private:
	VertexBufferObject vboSceneObjects;
	GLuint uiVAO; // Only one VAO now

	GLuint geometryProgramID;
	GLuint lightProgramID;
	GLuint programID;

	//Texture tTextures[4];

	const char* vertexShaderSource;
	const char* fragmentShaderSource;

	const char* vertexGeometryShaderSource;
	const char* geometryShaderSource;
	const char* fragmentGeometryShaderSource;

	int iTorusFaces;
	float fGlobalAngle;
	float fSunAngle;
	float fBender;
	float timer;
	bool bWireFrame;

	Pipeline *pipeline;
	Skybox skybox;
};

