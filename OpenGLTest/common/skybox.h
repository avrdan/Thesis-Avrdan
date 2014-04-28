#pragma once

#include <string>
#include <GL/glew.h>
#include <glm/glm.hpp>

#include "vertexBufferObject.h"
#include "texture.h"


using namespace std;

class Skybox
{
public:
	Skybox();
	~Skybox();

	void loadSkybox(string a_sDirectory, string a_sFront, string a_sBack, string a_sLeft, string a_sRight, string a_sTop, string a_sBottom);
	void renderSkybox();
	void releaseSkybox();
private:
	GLuint uiVAO;
	VertexBufferObject vboRenderData;
	Texture tTextures[6];
	string sDirectory;
	string sFront, sBack, sLeft, sRight, sTop, sBottom;
};

