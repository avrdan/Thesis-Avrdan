#include "lightingScene.h"
#include "common/controls.h"
#include <iostream>

LightingScene::LightingScene()
{
	vertexShaderSource = "./assets/shaders/shader_tex.vert";
	fragmentShaderSource = "./assets/shaders/shader_tex.frag";

	vertexLightShaderSource = "./assets/shaders/dirLight.vert";
	fragmentLightShaderSource = "./assets/shaders/dirLight.frag";

	fGlobalAngle = 0;
	fSunAngle = 45.0f;
}


LightingScene::~LightingScene()
{
}

void LightingScene::initScene(GLFWwindow *window)
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

	vboSceneObjects.createVBO();
	glGenVertexArrays(1, &uiVAO); // create a VAO
	glBindVertexArray(uiVAO);

	vboSceneObjects.bindVBO();

	// add cube to VBO
	for (int i = 0; i < 36; i++)
	{
		vboSceneObjects.addData(&vCubeVerticesL[i], sizeof(glm::vec3));
		vboSceneObjects.addData(&vCubeTexCoordsL[i % 6], sizeof(glm::vec2));
		vboSceneObjects.addData(&vCubeNormalsL[i / 6], sizeof(glm::vec3));
	}

	// add ground to vbo
	for (int i = 0; i < 6; i++)
	{
		vboSceneObjects.addData(&vGroundL[i], sizeof(glm::vec3));
		vCubeTexCoordsL[i] *= 10.0f;
		vboSceneObjects.addData(&vCubeTexCoordsL[i % 6], sizeof(glm::vec2));
		glm::vec3 vGroundNormal(0.0f, 1.0f, 0.0f);
		vboSceneObjects.addData(&vGroundNormal, sizeof(glm::vec3));
	}

	// add torus to VBO and store the number of faces of this torus
	iTorusFaces1 = GenerateTorus(vboSceneObjects, 7.0f, 2.0f, 20, 20);
	// add sun torus to VBO
	iTorusFaces2 = GenerateTorus(vboSceneObjects, 10.0f, 6.0f, 10, 10);

	vboSceneObjects.uploadDataToGPU(GL_STATIC_DRAW);

	// Vertex positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3) + sizeof(glm::vec2), 0);
	// Texture coordinates
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3) + sizeof(glm::vec2), (void*)sizeof(glm::vec3));
	// Normal vectors
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3) + sizeof(glm::vec2), (void*)(sizeof(glm::vec3) + sizeof(glm::vec2)));

	lightProgramID = LoadShaders(vertexLightShaderSource, fragmentLightShaderSource);
	
	glUseProgram(lightProgramID);

	// load textures
	const char *cTextureNames[] = { "ground.jpg", "box.jpg", "rust.jpg", "sun.jpg" };
	vector<string> sTextureNames(cTextureNames, &cTextureNames[sizeof(cTextureNames) / sizeof(cTextureNames[0])]);

	loadAllTextures(sTextureNames);
	glEnable(GL_TEXTURE_2D);
	/*for (int i = 0; i < 4; i++)
	{
		tTextures[i].loadTexture2D("");
		tTextures[i].setFiltering(TEXTURE_FILTER_MAG_BILINEAR, TEXTURE_FILTER_MIN_BILINEAR_MIPMAP);
	}*/

	// pipeline
	pipeline = new Pipeline();
	pipeline->resetTimer();
	pipeline->setProjection3D(45.0f, 4.0f / 3.0f, 0.1f, 1000.0f);
}

void LightingScene::renderScene(GLFWwindow *window)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	computeMatricesFromInputs(window);

	glBindVertexArray(uiVAO);

	// set light properties
	float fSine = sinf(fSunAngle*3.1415f / 180.0f);
	glm::vec3 vSunPos(cosf(fSunAngle*3.1415f / 180.0f) * 70, sinf(fSunAngle*3.1415f / 180.0f) * 70, 0.0f);

	// change the color of the sky
	// depending on the sun's position
	glClearColor(0.0f, max(0.0f, 0.9f*fSine), max(0.0f, 0.9f*fSine), 1.0f);

	// set uniforms
	int iLightColorLoc = glGetUniformLocation(lightProgramID, "sunLight.vColor");
	glUniform3fv(iLightColorLoc, 1, glm::value_ptr(glm::vec3(1.0f, 1.0f, 1.0f)));
	int iLightAmbientLoc = glGetUniformLocation(lightProgramID, "sunLight.fAmbientIntensity");
	float ambientIntensity = 0.25f;
	glUniform1fv(iLightAmbientLoc, 1, &ambientIntensity);
	int iLightDirectionLoc = glGetUniformLocation(lightProgramID, "sunLight.vDirection");
	glUniform3fv(iLightDirectionLoc, 1, glm::value_ptr(-glm::normalize(vSunPos)));

	int iModelViewLoc = glGetUniformLocation(lightProgramID, "MV");
	int iProjectionLoc = glGetUniformLocation(lightProgramID, "P");
	int iNormalMatrixLoc = glGetUniformLocation(lightProgramID, "N");

	glUniformMatrix4fv(iProjectionLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));
	glm::mat4 mModelView = getViewMatrix();
	glm::mat4 mCurrent;
	
	int iSamplerLoc = glGetUniformLocation(lightProgramID, "gSampler");
	int samplerVal = 0;
	//glUniform1iv(iSamplerLoc, 1, &samplerVal);
	glUniform1i(iSamplerLoc, 0);

	glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mModelView));
	int iColorLoc = glGetUniformLocation(lightProgramID, "vColor");
	glUniform4fv(iColorLoc, 1, glm::value_ptr(glm::vec4(1.0f, 1.0f, 1.0f, 1.0f)));

	// render ground
	tTextures[0].bindTexture();
	//cout << "tex height: " << tTextures[1].getHeight() << endl;
	//cout << "tex width: " << tTextures[1].getWidth() << endl;
	mCurrent = glm::translate(mModelView, glm::vec3(0.0f, 0.0f, 0.0f));
	glUniformMatrix4fv(iNormalMatrixLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mCurrent))));
	glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mModelView*mCurrent));

	glDrawArrays(GL_TRIANGLES, 36, 6);

	// render 5 cubes
	tTextures[1].bindTexture();
	
	for (int i = 0; i < 5; i++)
	{
		float fSign = -1.0f + float(i % 2)*2.0f; // returns -1 or 1
		glm::vec3 vPos = glm::vec3(fSign*15.0f, 0.0f, 50.0f - float(i)*25.0f);
		//mCurrent = glm::translate(glm::mat4(1.0), vPos);
		mCurrent = glm::translate(mModelView, vPos);
		mCurrent = glm::scale(mCurrent, glm::vec3(8.0f, 8.0f, 8.0f));
		// transform normals
		// by transpose of inverse matrix
		// of transforms(except translation)
		glUniformMatrix4fv(iNormalMatrixLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mCurrent))));
		glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mModelView*mCurrent));

		glDrawArrays(GL_TRIANGLES, 0, 36);
	}

	// render 5 tori
	tTextures[2].bindTexture();

	for (int i = 0; i < 5; i++)
	{
		float fSign = 1.0f - float(i % 2)*2.0f; // This just returns -1.0f or 1.0f (try to examine this)
		glm::vec3 vPos = glm::vec3(fSign*15.0f, 0.0f, 50.0f - float(i)*25.0f);
		//mCurrent = glm::translate(glm::mat4(1.0), vPos);
		mCurrent = glm::translate(mModelView, vPos);
		mCurrent = glm::rotate(mCurrent, fGlobalAngle + i*30.0f, glm::vec3(0.0f, 1.0f, 0.0f));

		glUniformMatrix4fv(iNormalMatrixLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mCurrent))));
		glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mModelView*mCurrent));
		glDrawArrays(GL_TRIANGLES, 42, iTorusFaces1 * 3);
	}

	// render the sun
	tTextures[3].bindTexture();
	//mCurrent = glm::translate(glm::mat4(1.0f), vSunPos);
	mCurrent = glm::translate(mModelView, vSunPos);
	glUniformMatrix4fv(iNormalMatrixLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mCurrent))));
	glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mModelView*mCurrent));
	// make it shine
	float intensity = 0.8f;
	glUniform1fv(iLightAmbientLoc, 1, &intensity);

	glDrawArrays(GL_TRIANGLES, 42 + iTorusFaces1 * 3, iTorusFaces2 * 3);
	
	// update stuff
	fGlobalAngle += pipeline->sof(100.0f);
	if (fGlobalAngle >= 360)
		fGlobalAngle -= 360;

	if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS)
	{
		fSunAngle -= pipeline->sof(45.0f);
	}

	if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS)
	{
		fSunAngle += pipeline->sof(45.0f);
	}
	
	// Swap front and back buffers
	glfwSwapBuffers(window);
	// Poll for and process events
	glfwPollEvents();

	pipeline->updateTimer();
}

void LightingScene::releaseScene()
{
	for (int i = 0; i < 4; i++)
	{
		tTextures[i].releaseTexture();
	}

	glDeleteProgram(lightProgramID);
	glDeleteProgram(programID);

	glDeleteVertexArrays(1, &uiVAO);
	vboSceneObjects.releaseVBO();
}