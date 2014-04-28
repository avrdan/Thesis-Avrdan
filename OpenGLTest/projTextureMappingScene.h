#pragma once
#include "simpleScene.h"
class ProjTextureMappingScene :
	public AbstractScene
{
public:
	ProjTextureMappingScene();
	~ProjTextureMappingScene();

	void initScene(GLFWwindow *window);
	void renderScene(GLFWwindow *window);
	void releaseScene();
private:
	void computeFrustumCoords(glm::mat4 mProjection);

	VertexBufferObject vboSceneObjects;
	GLuint uiVAOSceneObjects;
	Skybox skybox;
	AssimpModel amModels[3];

	const char* vertexShaderSource;
	const char* fragmentShaderSource;
	const char* vertexShaderLightSource;
	const char* fragmentShaderLightSource;
	const char* vertexShaderPTMSource;
	const char* fragmentShaderPTMSource;
	const char* vertexShaderSimpleSource;
	const char* fragmentShaderSimpleSource;

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
	//char* torusModel;
	//char* torusModel;
	char* torusModel;
	char* roomModel;
	char* tableModel;
	char* chairModel;
	char* bowlModel;
	char* suzanneModel;

	GLuint programID;
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

	GLuint programPTM_ID;
	GLuint programNoLightID;
	GLuint programSimpleID;

	float fSunAngle;
	bool  useMainCam;
	float  timer;
	float zoomFactor;

	Camera *cam;
	//glm::vec3 frustum_coords[];
};

