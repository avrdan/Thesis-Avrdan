#include <Windows.h>


#include<stdlib.h>
#include<stdio.h>
// OGL via glew
#include <GL/glew.h>
// include GLFW as the windowing context handling library
#include <GLFW/glfw3.h>
// include GLM math library
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include<iostream>

#include "common/controls.h"

// scenes
#include "simpleScene.h"
#include "pyramidScene.h"
#include "heightMapScene.h"
#include "textureScene.h"
#include "assimpScene.h"
#include "projTextureMappingScene.h"
#include "lightingScene.h"
#include "frameBuffersScene.h"
#include "shadowMappingScene.h"
#include "PTMTestScene.h"
//#include <Windows.h>
#include "PointCloudScene.h"
#include "GeometryShaderScene.h"
#include "MarchingCubesScene.h"
#include "PCLReconstructionScene.h"
#include "HPMCScene.h"
#include "PointCloudPCLScene.h"
#include "PointCloudPTMScene.h"

//#include <Windows.h>

using namespace std;

//int main(int argc, char* argv[])
int wmain(int argc, WCHAR* argv[])
{
	
	GLFWwindow *window;
	//HWND x;
	/* Initialize the library*/
	if (!glfwInit())
	{
		printf("GLFW INIT ERROR..\n");
		cout << "GLFW INIT ERROR.." << endl;
		return -1;
	}

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_SAMPLES, 0); // 4x Antialiasing
	//glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
		
	
	/* Create a windowed mode window and its OpenGL Context*/
	// get the current Desktop screen resolution and colour depth
	/*GLFWvidmode desktop;
	
	glfwGetDesktopMode(&desktop);

	// open the window at the current Desktop resolution and colour depth
	if (!glfwOpenWindow(
		desktop.Width,
		desktop.Height,
		desktop.RedBits,
		desktop.GreenBits,
		desktop.BlueBits,
		8,          // alpha bits
		32,         // depth bits
		0,          // stencil bits
		GLFW_FULLSCREEN
		)) {
		// failed to open window: handle it here
	}*/
	//window = glfwCreateWindow(1024, 768, "3D SCENE", NULL, NULL);
	//window = glfwCreateWindow(320, 240, "Deformable Display", NULL, NULL);
	//window = glfwCreateWindow(1920, 1080, "Deformable Display", NULL, NULL);
	//window = glfwCreateWindow(1280, 720, "Deformable Display", glfwGetPrimaryMonitor(), NULL);
	//window = glfwCreateWindow(1920, 1080, "Deformable Display", glfwGetPrimaryMonitor(), NULL);
	window = glfwCreateWindow(1919, 1079, "Deformable Display", glfwGetPrimaryMonitor(), NULL);
	//glfwSetWindowSize(window, 1920, 1080);
	if (!window)
	{
		glfwTerminate();
		printf("GLFW WINDOW ERROR..\n");
		cout << "GLFW WINDOW ERROR.." << endl;
		return -1;
	}

	/* Make the window's context current */
	glfwMakeContextCurrent(window);

	// init glew (must be done after glfw initializes opengl version)
	glewExperimental = true;
	GLenum res = glewInit();
	if (res != GLEW_OK)
	{
		fprintf(stderr, "Error: '%s'\n", glewGetErrorString(res));
		cout << "GLEW ERROR" << endl;
		return -1;
	}
	//std::cout << "OPENGL GLFW INIT error: " << gluErrorString(glGetError()) << "\n";

	// keyboard input
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
	
	// clear to dark blue
	glClearColor(0.0f, 0.0f, 0.4f, 1.0f);
	// Enable depth test
	glEnable(GL_DEPTH_TEST);
	glClearDepth(1.0f);
	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS);

	// Cull triangles which normal is not towards the camera
	//glEnable(GL_CULL_FACE);

	// create scene
	AbstractScene *scene;
	//scene = new PyramidScene();
	//scene = new HeightMapScene();
	//scene = new TextureScene();
	//scene = new AssimpScene();
	//scene = new ProjTextureMappingScene();
	//scene = new LightingScene();
	//scene = new FrameBuffersScene();	
	//scene = new ShadowMappingScene();
	//scene = new PTMTestScene();
	//scene = new PointCloudScene();
	//scene = new GeometryShaderScene();
	//scene = new MarchingCubesScene();
	//scene = new PCLReconstructionScene();
	//scene = new PointCloudPCLScene();
	//scene = new HPMCScene();
	scene = new PointCloudPTMScene();

	scene->initScene(window);

	/* Loop until the user closes the window */
	while (!glfwWindowShouldClose(window) && glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS)
	{
		//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		scene->renderScene(window);
	}

	scene->releaseScene();
	glfwTerminate();
	return 0;
}