#include "assimpScene.h"
#include "static_geometry.h"
#include "common/controls.h"
#include <iostream>

AssimpScene::AssimpScene()
{
	vertexShaderSource = "./assets/shaders/shader_tex.vert";
	fragmentShaderSource = "./assets/shaders/shader_tex.frag";
	texSnowSource = "./assets/textures/snow.jpg";
	texGoldSource = "./assets/textures/golddiag.jpg";

	skyboxPath = "./assets/textures/skybox/elbrus/";
	skyFront = "elbrus_front.jpg";
	skyDown = "elbrus_top.jpg";
	skyUp = "elbrus_top.jpg";
	skyRight = "elbrus_left.jpg";
	skyBack = "elbrus_back.jpg";
	skyLeft = "elbrus_right.jpg";

	wolfModel = "./assets/models/Wolf/wolf.obj";
	houseModel = "./assets/models/house/house.3ds";
}


AssimpScene::~AssimpScene()
{
}

void AssimpScene::initScene(GLFWwindow *window)
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

	CreateStaticSceneObjects(&uiVAOSceneObjects, vboSceneObjects);

	const char *cTextureNames[] = { "sand_grass_02.jpg" };
	vector<string> sTextureNames(cTextureNames, &cTextureNames[sizeof(cTextureNames) / sizeof(cTextureNames[0])]);

	loadAllTextures(sTextureNames);

	skybox.loadSkybox(skyboxPath, skyFront, skyBack, skyLeft, skyRight, skyUp, skyDown);

	amModels[0].LoadModelFromFile(wolfModel);
	amModels[1].LoadModelFromFile(houseModel);

	AssimpModel::FinalizeVBO();

	programID = LoadShaders(vertexShaderSource, fragmentShaderSource);
	glUseProgram(programID);

	// pipeline
	pipeline = new Pipeline();
	pipeline->resetTimer();
	pipeline->setProjection3D(45.0f, 4.0f / 3.0f, 0.1f, 1000.0f);
}

void AssimpScene::renderScene(GLFWwindow *window)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	computeMatricesFromInputs(window);

	int iModelViewLoc = glGetUniformLocation(programID, "MV");
	int iProjectionLoc = glGetUniformLocation(programID, "P");
	glUniformMatrix4fv(iProjectionLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));

	//glm::mat4 mModelView = glm::lookAt(glm::vec3(0, 12, 27), glm::vec3(0, 0, 0), glm::vec3(0.0f, 1.0f, 0.0f));
	glm::mat4 mModelView = getViewMatrix();
	glm::mat4 mCurrent;

	// render ground
	glBindVertexArray(uiVAOSceneObjects);

	tTextures[0].bindTexture();
	//glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mModelView));
	glDrawArrays(GL_TRIANGLES, 0, 6);
	// render some houses
	AssimpModel::BindModelsVAO();
	for (int i = 0; i < 6; i++)
	{
		float xPosHouse = -80.0f + i*30.0f;
		mCurrent = glm::translate(mModelView, glm::vec3(xPosHouse, 0, 0));
		mCurrent = glm::scale(mCurrent, glm::vec3(3, 3, 3));
	
		glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));
		amModels[1].RenderModel();
	}


	// render wolves
	AssimpModel::BindModelsVAO();

	for (int i = 0; i < 7; i++)
	{
		float xPosWolf = -75.0f + i*30.0f;
		mCurrent = glm::translate(mModelView, glm::vec3(xPosWolf, 0, 0));
		mCurrent = glm::scale(mCurrent, glm::vec3(1.8f, 1.8f, 1.8f));
		
		glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));
		amModels[0].RenderModel();
	}
	

	// render skybox
	skybox.renderSkybox();

	// Swap front and back buffers
	glfwSwapBuffers(window);
	// Poll for and process events
	glfwPollEvents();

	pipeline->updateTimer();
}

void AssimpScene::releaseScene()
{
	for (int i = 0; i < Texture::NUMTEXTURES; i++)
	{
		tTextures[i].releaseTexture();
	}
	skybox.releaseSkybox();

	glDeleteProgram(programID);
	glDeleteVertexArrays(1, &uiVAOSceneObjects);
	vboSceneObjects.releaseVBO();
}
