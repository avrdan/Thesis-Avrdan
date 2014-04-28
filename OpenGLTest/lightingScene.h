#pragma once
#include "simpleScene.h"
class LightingScene :
	public AbstractScene
{
public:
	LightingScene();
	~LightingScene();

	void initScene(GLFWwindow *window);
	void renderScene(GLFWwindow *window);
	void releaseScene();
private:
	/* One VBO, where all static data are stored now,
	in this tutorial vertex is stored as 3 floats for
	position, 2 floats for texture coordinate and
	3 floats for normal vector. */

	VertexBufferObject vboSceneObjects;
	GLuint uiVAO; // Only one VAO now

	GLuint lightProgramID;
	GLuint programID;

	//Texture tTextures[4];

	const char* vertexShaderSource;
	const char* fragmentShaderSource;

	const char* vertexLightShaderSource;
	const char* fragmentLightShaderSource;

	int iTorusFaces1, iTorusFaces2;
	float fGlobalAngle;
	float fSunAngle;

	Pipeline *pipeline;
};

