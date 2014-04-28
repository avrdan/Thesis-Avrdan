#pragma once
#include "simpleScene.h"
class ShadowMappingScene :
	public AbstractScene
{
public:
	ShadowMappingScene();
	~ShadowMappingScene();

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

	//GLuint lightProgramID;
	//GLuint programID;
	GLuint depthProgramID;
	GLuint shadowMapProgramID;
	GLuint quadprogramID;

	//Texture tTextures[4];

	const char* vertexShaderSource;
	const char* fragmentShaderSource;

	const char* vertexLightShaderSource;
	const char* fragmentLightShaderSource;

	const char* vertexShaderDepthRTTSource;
	const char* fragmentShaderDepthRTTSource;


	const char* vertexShaderShadowMapSource;
	const char* fragmentShaderShadowMapSource;

	const char* vertexShaderQuadSource;
	const char* fragmentShaderQuadSource;

	int iTorusFaces1, iTorusFaces2;
	float fGlobalAngle;
	float fSunAngle;

	Pipeline *pipeline;

	GLuint fb;
	GLuint depthTexture;
	int width;
	int height;

	glm::mat4 depthProjectionMatrix;
	glm::mat4 depthViewMatrix;
	glm::mat4 depthModelMatrix;
	glm::mat4 depthMVP;
	glm::mat4 biasMatrix;
	glm::mat4 depthBiasMVP;

	// uniforms
	GLuint matrixID;
	GLuint depthBiasID;
	GLuint shadowMapID;
	GLuint depthMatrixID;

	// light
	glm::vec3 vSunPos;
	glm::vec3 lightInvDir;
};

