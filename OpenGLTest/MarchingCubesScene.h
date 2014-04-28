#pragma once
#include "simpleScene.h"
#include "MarchingCubes.h"

class MarchingCubesScene :
	public AbstractScene
{
public:
	MarchingCubesScene();
	~MarchingCubesScene();

	void initScene(GLFWwindow *window);
	void renderScene(GLFWwindow *window);
	void releaseScene();
private:
	MarchingCubes mc;
	GLuint edgeTableTex;
	GLuint triTableTex;
	GLuint dataFieldTex[3];

	VertexBufferObject vboSceneObjects;
	GLuint uiVAO; // Only one VAO now
	GLuint geometryProgramID;
	GLuint programID;

	const char* mcVertexShaderSource;
	const char* mcGeometryShaderSource;
	const char* mcFragmentShaderSource;

	const char* texVertexShaderSource;
	const char* texFragmentShaderSource;

	float fGlobalAngle;
	float fSunAngle;
	float timer;
	bool bWireFrame;

	Pipeline *pipeline;
	Skybox skybox;

	float *dataField[3];
	glm::ivec3 dataSize;
	int curData;
	float isoLevel;
	glm::vec3 cubeSize;
	glm::vec3 cubeStep;
	float *gridData;
	GLuint gridDataBuffId;
};

