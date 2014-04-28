#pragma once
#include "simpleScene.h"
class FrameBuffersScene :
	public AbstractScene
{
public:
	FrameBuffersScene();
	~FrameBuffersScene();

	void initScene(GLFWwindow *window);
	void renderScene(GLFWwindow *window);
	void releaseScene();
private:
	VertexBufferObject vboSceneObjects;
	GLuint uiVAOSceneObjects;

	const char* vertexShaderSource;
	const char* fragmentShaderSource;
	const char* vertexShaderPostProcessSource;
	const char* fragmentShaderPostProcessSource;
	GLuint programPostProcessID;
	GLuint programID;
	Pipeline *pipeline;

	// frame buffer stuff
	int width;
	int height;
	GLuint fb;
	GLuint fb_tex;
	GLuint rb;

	// texture
	const char* texSource;
	//const char* texSource = "./assets/textures/biohazard_512x512.jpg";
	//const char* texSource = "./assets/textures/MiLn4eyia_512k.png";
	Texture texture;
	GLuint samplerFB;
};

