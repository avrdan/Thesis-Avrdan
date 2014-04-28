#pragma once
#include "simpleScene.h"
class PTMTestScene :
	public AbstractScene
{
public:
	PTMTestScene();
	~PTMTestScene();

	void initScene(GLFWwindow *window);
	void renderScene(GLFWwindow *window);
	void releaseScene();
private:
	void computeFrustumCoords(glm::mat4 mProjection);

	VertexBufferObject vboSceneObjects;
	GLuint uiVAOSceneObjects;
	Skybox skybox;
	AssimpModel amModels[1];

	const char* vertexShaderLightSource;
	const char* fragmentShaderLightSource;
	const char* vertexShaderPTMSource;
	const char* fragmentShaderPTMSource;
	const char* vertexShaderSimpleSource;
	const char* fragmentShaderSimpleSource;

	char* sphereModel;
	char* sphereHalfModel;
	char* sphereHalfModelPro;
	char* sphereHalfModelCut;

	GLuint programID;
	GLuint programPTM_ID;
	GLuint programSimpleID;

	Pipeline *pipeline;

	// projector frustum details
	float pFovY;
	float pAR;
	float pNear;
	float pFar;
	glm::vec3 pUp;

	glm::mat4 biasMatrix;
	glm::mat4 projectorP;
	glm::mat4 projectorV;
	glm::vec3 projectorOrigin;
	glm::vec3 projectorTarget;

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
};

