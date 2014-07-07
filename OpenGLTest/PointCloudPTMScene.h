#pragma once
#include "simpleScene.h"
#include "RawDepthPipeline.h"

#include "text.h"

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
	const char* textVertexShaderSource;
	const char* textFragmentShaderSource;

	char* sphereModel;
	char* sphereHalfModelPro;

	GLuint programID;
	GLuint programColorID;
	GLuint programQuadID;
	GLuint programFrustumID;
	GLuint programTextID;
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
	bool bUseCustomMatrices;
	bool bFragmentShaderDebug;
	bool bInitFrustum;

	glm::mat4 frustum;

	double frustum_height;
	double frustum_width;
	double offset_x;
	double offset_y;

	float zRotation, yRotation, xRotation;
	float zRotationCam, yRotationCam, xRotationCam;
	float zTranslationCam, yTranslationCam, xTranslationCam;
	float zTranslation, yTranslation, xTranslation;
	float zTranslationPrjTotal, yTranslationPrjTotal, xTranslationPrjTotal;
	float xRotationPrjTotal, yRotationPrjTotal, zRotationPrjTotal;
	float zRotationPrj, yRotationPrj, xRotationPrj;
	float zTranslationPrj, yTranslationPrj, xTranslationPrj;

	glm::mat4 rotationMatrix;
	glm::mat4 translationMatrix;

	void printGLMMatrix(glm::mat4 matrix, string name);

	int activeTextureIndex;
	int nFrames;
	int currFrameIndex;

	glm::mat3 rotationMatDepthToColor;
	glm::vec3 translationVecDepthToColor;
	glm::mat4 mDepthToColor;

	glm::mat4 kRGB, kDepth;
	glm::mat4 calibRotMat;

	GLfloat getGLDepth(int x, int y, glm::mat4 mModelView, glm::mat4 frustum);

	bool runVolumeSlicing;
	bool initDepthState;
	float initDepth;
	bool continuousMode;
	float volumeSliceStep;

	glm::mat4 savedMV;
	glm::mat4 savedP;

	bool runGoogleMap;
	bool runDeformScene;
	bool runDeformText;
	bool runDeformGrid;

	int getRandomNumber(int n);
	void randomizeDistortionState(int bound);

	ofstream outputMesh;

	std::vector<PXCPoint3DF32> storedPos;
	bool saveMesh;

	vector<string> s1TextureNamesStr;
	vector<string> s2TextureNamesStr;
	vector<string> s3TextureNamesStr;
	vector<string> s4TextureNamesStr;
	vector<string> sVolumeSliceTextureNames;

	// dynamic textures
	vector<string> s1MainNamesStr;
	vector<string> s1CenterNamesStr;
	vector<string> s1LeftNamesStr;
	vector<string> s1RightNamesStr;

	vector<string> s2MainNamesStr;
	vector<string> s2CenterNamesStr;
	vector<string> s2LeftNamesStr;
	vector<string> s2RightNamesStr;

	vector<string> s3MainNamesStr;
	vector<string> s3CenterNamesStr;
	vector<string> s3LeftNamesStr;
	vector<string> s3RightNamesStr;

	vector<string> s4MainNamesStr;


	//vector<string> sVolumeSliceTextureNames2;

	int s2Counter;
	int s3Counter;
	int s1Counter;

	bool useMedianFiltering;
	bool useWeightedMovingAverage;

	Text text;
	bool bShowText;

	void readTextureNames(string baseName, const char* target[], int iter);
	void loopFrames(int begin, int end);

	int dynamicTextureOffset;
	int dynamicScenarioOffset;


	int s1Cnt;
	int s2Cnt;
	int s3Cnt;
	int s4Cnt;
	int s1MainCnt;
	int s1CenterCnt;
	int s1LeftCnt;
	int s1RightCnt;
	int s2MainCnt;
	int s2CenterCnt;
	int s2LeftCnt;
	int s2RightCnt;
	int s3MainCnt;
	int s3CenterCnt;
	int s3LeftCnt;
	int s3RightCnt;
	int s4MainCnt;

	bool bAIsFirst;
	int slowdown;
	bool bRenderFrame;

	bool started;
};

