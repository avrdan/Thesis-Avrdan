#pragma once
#include "simpleScene.h"
#include "hpmc/include/hpmc.h"
#include "hpmc/include/hpmc_internal.h"
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>

using std::ifstream;
using std::vector;
using std::string;
using std::cerr;
using std::endl;

class HPMCScene :
	public AbstractScene
{
public:
	HPMCScene();
	~HPMCScene();

	void initScene(GLFWwindow *window);
	void renderScene(GLFWwindow *window);
	void releaseScene();
private:
	int volume_size_x;
	int volume_size_y;
	int volume_size_z;
	vector<GLubyte> dataset;

	GLuint volume_tex;

	struct HPMCConstants* hpmc_c;
	struct HPMCHistoPyramid* hpmc_h;

	// -----------------------------------------------------------------------------
	GLuint shaded_v;
	GLuint shaded_f;
	GLuint shaded_p;
	struct HPMCTraversalHandle* hpmc_th_flat;

	const char* shaded_vertex_shader;
	const char* shaded_fragment_shader;
	const char* vertexShaderSourceTex;
	const char* fragmentShaderSourceTex;

	GLuint flat_v;
	GLuint flat_p;
	struct HPMCTraversalHandle* hpmc_th_shaded;

	const char* flat_vertex_shader;

	float fGlobalAngle;
	float fSunAngle;
	float timer;
	bool bWireFrame;

	Pipeline *pipeline;
	Skybox skybox;

	VertexBufferObject vboSceneObjects;
	VertexBufferObject ibo;
	GLuint uiVAO; // Only one VAO now
	GLuint programID;
	GLuint programIDTex;
};

