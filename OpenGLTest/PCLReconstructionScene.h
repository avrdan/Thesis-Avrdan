#pragma once
#include "simpleScene.h"
#include "PCLGreedyReconstruction.h"
class PCLReconstructionScene :
	public AbstractScene
{
public:
	PCLReconstructionScene();
	~PCLReconstructionScene();

	void initScene(GLFWwindow *window);
	void renderScene(GLFWwindow *window);
	void releaseScene();
private:
	VertexBufferObject vboSceneObjects;
	VertexBufferObject ibo;
	GLuint uiVAO; // Only one VAO now
	GLuint programID;
	GLuint programIDTex;

	const char* vertexShaderSource;
	const char* fragmentShaderSource;

	const char* vertexShaderSourceTex;
	const char* fragmentShaderSourceTex;

	float fGlobalAngle;
	float fSunAngle;
	float timer;
	bool bWireFrame;

	Pipeline *pipeline;
	Skybox skybox;

	PCLGreedyReconstruction pclGR;
};

