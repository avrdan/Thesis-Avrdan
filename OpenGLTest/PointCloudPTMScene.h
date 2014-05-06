#pragma once
#include "simpleScene.h"
#include "RawDepthPipeline.h"

class PointCloudPTMScene :
	public AbstractScene
{
public:
	PointCloudPTMScene();
	~PointCloudPTMScene();

	void initScene(GLFWwindow *window);
	void renderScene(GLFWwindow *window);
	void releaseScene();
	
private:
	VertexBufferObject vboSceneObjects;
	VertexBufferObject vboSceneObjects2;
	VertexBufferObject ibo;
	GLuint uiVAOSceneObjects;
	Skybox skybox;
	AssimpModel amModels[1];

	/*const char* vertexShaderLightSource = "./assets/shaders/dirLight.vert";
	const char* fragmentShaderLightSource = "./assets/shaders/dirLight.frag";*/
	const char* vertexShaderTexSource;
	const char* fragmentShaderTexSource;
	const char* vertexShaderSource;
	const char* fragmentShaderSource;
	const char* vertexShaderPTMSource;
	const char* fragmentShaderPTMSource;

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
	bool bWireFrame;
	bool bUseMainCam;

	//glm::mat4 projectorV;
	//bool swap;

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

	void computeFrustumCoords(glm::mat4 mProjection);
};

