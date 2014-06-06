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
	VertexBufferObject vboDebug;
	GLuint uiVAOSceneObjects;
	GLuint uiVAODebug;
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
	const char* hmVertexShaderSource;
	const char* hmFragmentShaderSource;
	const char* vertexShaderQuadSource;
	const char* fragmentShaderQuadSource;
	const char* frustumVertexShaderSource;
	const char* frustumFragmentShaderSource;

	char* sphereModel;
	char* sphereHalfModelPro;

	GLuint programID;
	GLuint programColorID;
	GLuint programQuadID;
	GLuint programFrustumID;
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
	glm::vec3 finalProjectorOrigin;

	void computeFrustumCoords(glm::mat4 mProjection);
	void build_opengl_projection_for_intrinsics(glm::mat4 &frustum, int *viewport, 
		double alpha, double beta, double skew, double u0, double v0, 
		int img_width, int img_height, double near_clip, double far_clip);
	
	float fu;
	float fv;
	float u0;
	float v0;
	float nearP;
	float farP;
	float left;
	float right;
	float bottom;
	float top;

	glm::mat4 calibratedView;
	glm::mat3 projectorRotation;
	glm::vec3 projectorTranslation;

	//heatmap
	GLuint programID_HeatMap;
	bool bShowHeatMap;

	glm::mat4 mExtrinsicMatrix;
	float aspect;
	float fovy;

	bool bDebugTexture;
	bool bMirrorView;
	bool bUseCalibParams;
	bool bShowFrustum;

	glm::mat4 frustum;
};
