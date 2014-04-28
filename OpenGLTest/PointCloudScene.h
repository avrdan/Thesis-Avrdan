#pragma once
#include "simpleScene.h"
#include "RawDepthPipeline.h"
class PointCloudScene :
	public AbstractScene
{
public:
	PointCloudScene();
	~PointCloudScene();

	void initScene(GLFWwindow *window);
	void renderScene(GLFWwindow *window);
	void releaseScene();
private:
	VertexBufferObject vboSceneObjects;
	GLuint uiVAOSceneObjects;
	Skybox skybox;
	AssimpModel amModels[1];

	/*const char* vertexShaderLightSource = "./assets/shaders/dirLight.vert";
	const char* fragmentShaderLightSource = "./assets/shaders/dirLight.frag";*/
	const char* vertexShaderTexSource;
	const char* fragmentShaderTexSource;
	const char* vertexShaderSource;
	const char* fragmentShaderSource;

	char* sphereModel;
	char* sphereHalfModelPro;

	GLuint programID;
	GLuint programColorID;

	Pipeline *pipeline;
	RawDepthPipeline* rdp;

	// test camera
	glm::mat4 testCamV;

	glm::mat4 mModel;
	glm::mat4 texGenMatrix;
	glm::mat4 invViewMatrix;

	float fSunAngle;
	bool  useMainCam;
	bool  invertView;
	bool  lockMainCam;
	float  timer;
	float zoomFactor;
	float fRotationAngle;
};

