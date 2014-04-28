#include "GeometryShaderScene.h"
#include <iostream>

using namespace std;

GeometryShaderScene::GeometryShaderScene()
{
	vertexShaderSource = "./assets/shaders/shader_tex.vert";
	fragmentShaderSource = "./assets/shaders/shader_tex.frag";

	vertexGeometryShaderSource   = "./assets/shaders/geometry.vert";
	geometryShaderSource	     = "./assets/shaders/geometry.geom";
	fragmentGeometryShaderSource = "./assets/shaders/geometry.frag";

	fGlobalAngle = 0;
	fSunAngle = 45.0f;

	fBender = 0.75f;
	bWireFrame = false;
	timer = 0;
}


GeometryShaderScene::~GeometryShaderScene()
{
}

void GeometryShaderScene::initScene(GLFWwindow *window)
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

	for (int i = 0; i < 24; i++)
	{
		vboSceneObjects.addData(&vBuilding[i], sizeof(glm::vec3));
		glm::vec2 vCoord = vCubeTexCoords[i % 6] * 10.0f;
		vboSceneObjects.addData(&vCoord, sizeof(glm::vec2));
		vboSceneObjects.addData(&vBuildingNormals[i / 6], sizeof(glm::vec3));
	}

	// add torus to VBO and store the number of faces of this torus
	iTorusFaces = GenerateTorus(vboSceneObjects, 7.0f, 2.0f, 20, 20);

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

	geometryProgramID = LoadShaders(vertexGeometryShaderSource, geometryShaderSource,
		fragmentGeometryShaderSource);
	programID = LoadShaders(vertexShaderSource, fragmentShaderSource);
	glUseProgram(geometryProgramID);

	// load textures
	const char *cTextureNames[] = { "Tile41a.jpg", "crate.jpg", "metalplate.jpg", "stone_wall.jpg" };
	vector<string> sTextureNames(cTextureNames, &cTextureNames[sizeof(cTextureNames) / sizeof(cTextureNames[0])]);

	loadAllTextures(sTextureNames);
	glEnable(GL_TEXTURE_2D);
	/*for (int i = 0; i < 4; i++)
	{
	tTextures[i].loadTexture2D("");
	tTextures[i].setFiltering(TEXTURE_FILTER_MAG_BILINEAR, TEXTURE_FILTER_MIN_BILINEAR_MIPMAP);
	}*/

	glEnable(GL_DEPTH_TEST);
	glClearDepth(1.0);
	glClearColor(0.0f, 0.26f, 0.48f, 1.0f);

	skybox.loadSkybox("./assets/textures/skybox/jajsundown1/", "jajsundown1_ft.jpg", "jajsundown1_bk.jpg", "jajsundown1_lf.jpg", "jajsundown1_rt.jpg", "jajsundown1_up.jpg", "jajsundown1_dn.jpg");

	// pipeline
	pipeline = new Pipeline();
	pipeline->resetTimer();
	pipeline->setProjection3D(45.0f, 4.0f / 3.0f, 0.1f, 1000.0f);

	///////////////
	//Final error testing//
	std::cout << "Init end error: " << gluErrorString(glGetError()) << "\n";
}

void GeometryShaderScene::renderScene(GLFWwindow *window)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	computeMatricesFromInputs(window);
	timer += getDeltaTime();

	// set light properties
	float fSine = sinf(fSunAngle*3.1415f / 180.0f);
	glm::vec3 vSunPos(cosf(fSunAngle*3.1415f / 180.0f) * 70, sinf(fSunAngle*3.1415f / 180.0f) * 70, 0.0f);

	// set uniforms
	int iBenderLoc = glGetUniformLocation(geometryProgramID, "fBender");
	glUniform1f(iBenderLoc, fBender);
	int iProjectionLoc = glGetUniformLocation(geometryProgramID, "matrices.projMatrix");
	glUniformMatrix4fv(iProjectionLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));
	int iSamplerLoc = glGetUniformLocation(geometryProgramID, "gSampler");
	glUniform1i(iSamplerLoc, 0);

	glm::mat4 mView = getViewMatrix();
	
	int iViewLoc = glGetUniformLocation(geometryProgramID, "matrices.viewMatrix");
	glUniformMatrix4fv(iViewLoc, 1, GL_FALSE, glm::value_ptr(mView));
	int iModelLoc = glGetUniformLocation(geometryProgramID, "matrices.modelMatrix");
	glUniformMatrix4fv(iModelLoc, 1, GL_FALSE, glm::value_ptr(glm::mat4(1.0f)) );

	glm::mat4 mModelMatrix = glm::translate(glm::mat4(1.0f), getEyeVector());

	//cout << getEyeVector().x << ", " << getEyeVector().y << ", " << getEyeVector().z << endl;
	//cout << getEyeVector().x << ", " << getEyeVector().y << ", " << getLookVector().z << endl;

	int iNormalMatrixLoc = glGetUniformLocation(geometryProgramID, "matrices.normalMatrix");
	glUniformMatrix4fv(iNormalMatrixLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mView*mModelMatrix))));

	int iLightColorLoc = glGetUniformLocation(geometryProgramID, "sunLight.vColor");
	glUniform3fv(iLightColorLoc, 1, glm::value_ptr(glm::vec3(1.0f, 1.0f, 1.0f)));
	int iLightAmbientLoc = glGetUniformLocation(geometryProgramID, "sunLight.fAmbientIntensity");
	float ambientIntensity = 0.25f;
	glUniform1fv(iLightAmbientLoc, 1, &ambientIntensity);
	int iLightDirectionLoc = glGetUniformLocation(geometryProgramID, "sunLight.vDirection");
	glUniform3fv(iLightDirectionLoc, 1, glm::value_ptr(-glm::normalize(vSunPos)));

	int iColorLoc = glGetUniformLocation(geometryProgramID, "vColor");
	glUniform4fv(iColorLoc, 1, glm::value_ptr(glm::vec4(1.0f, 1.0f, 1.0f, 1.0f)));

	int iSkybox = glGetUniformLocation(geometryProgramID, "sunLight.iSkybox");
	glUniform1i(iSkybox, 1);
	skybox.renderSkybox();
	glUniform1i(iSkybox, 0);

	glBindVertexArray(uiVAO);

	// render ground
	if (bWireFrame)
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	else
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glUniformMatrix4fv(iModelLoc, 1, GL_FALSE, glm::value_ptr(glm::mat4(1.0f)));
	glUniformMatrix4fv(iNormalMatrixLoc, 1, GL_FALSE, glm::value_ptr(glm::mat4(1.0f)));
	tTextures[0].bindTexture();

	glDrawArrays(GL_TRIANGLES, 36, 6);

	// render building
	tTextures[3].bindTexture();
	glDrawArrays(GL_TRIANGLES, 42, 24);

	// create a box pile inside "building"
	tTextures[1].bindTexture();
	for (int nb = 1; nb <= 9; nb++)
	{
		int iCnt = nb > 5 ? 10 - nb : nb;

		for (int i = 0; i < iCnt; i++)
		{
			glm::vec3 vPos = glm::vec3(-25.0f + nb*8.02f, 4.0f + i*8.02f, 0.0f);
			mModelMatrix = glm::translate(glm::mat4(1.0), vPos);
			mModelMatrix = glm::scale(mModelMatrix, glm::vec3(8.0f, 8.0f, 8.0f));
			// we need to transform normals properly
			glUniformMatrix4fv(iModelLoc, 1, GL_FALSE, glm::value_ptr(mModelMatrix));
			glUniformMatrix4fv(iNormalMatrixLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mModelMatrix))));
		
			glDrawArrays(GL_TRIANGLES, 0, 36);
		}
	}

	// render 3 rotated tori
	tTextures[2].bindTexture();

	// Now it's gonna float in the air
	glm::vec3 vPos = glm::vec3(0.0f, 50.0, 0.0f);
	mModelMatrix = glm::translate(glm::mat4(1.0), vPos);
	mModelMatrix = glm::rotate(mModelMatrix, fGlobalAngle, glm::vec3(0.0f, 1.0f, 0.0f));
	glUniformMatrix4fv(iNormalMatrixLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mModelMatrix))));
	glUniformMatrix4fv(iModelLoc, 1, GL_FALSE, glm::value_ptr(mModelMatrix));

	glDrawArrays(GL_TRIANGLES, 66, iTorusFaces * 3);

	mModelMatrix = glm::translate(glm::mat4(1.0), vPos);
	mModelMatrix = glm::rotate(mModelMatrix, fGlobalAngle, glm::vec3(0.0f, 1.0f, 0.0f));
	mModelMatrix = glm::rotate(mModelMatrix, 90.0f, glm::vec3(1.0f, 0.0f, 0.0f));
	glUniformMatrix4fv(iNormalMatrixLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mModelMatrix))));
	glUniformMatrix4fv(iModelLoc, 1, GL_FALSE, glm::value_ptr(mModelMatrix));
	glDrawArrays(GL_TRIANGLES, 66, iTorusFaces * 3);

	mModelMatrix = glm::translate(glm::mat4(1.0), vPos);
	mModelMatrix = glm::rotate(mModelMatrix, fGlobalAngle + 90.0f, glm::vec3(0.0f, 1.0f, 0.0f));
	glUniformMatrix4fv(iNormalMatrixLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mModelMatrix))));
	glUniformMatrix4fv(iModelLoc, 1, GL_FALSE, glm::value_ptr(mModelMatrix));
	glDrawArrays(GL_TRIANGLES, 66, iTorusFaces * 3);
	
	// make it shine
	float intensity = 0.8f;
	glUniform1fv(iLightAmbientLoc, 1, &intensity);

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

	if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS)
	{
		fBender += pipeline->sof(2.0f);
	}

	if (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS)
	{
		fBender -= pipeline->sof(2.0f);
	}

	if (timer >= 0.25f)
	{
		if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_M) != GLFW_RELEASE)
		{
			bWireFrame = !bWireFrame;
			timer = 0;
		}
	}


	glEnable(GL_DEPTH_TEST);

	// Swap front and back buffers
	glfwSwapBuffers(window);
	// Poll for and process events
	glfwPollEvents();

	pipeline->updateTimer();
}

void GeometryShaderScene::releaseScene()
{
	for (int i = 0; i < 4; i++)
	{
		tTextures[i].releaseTexture();
	}

	skybox.releaseSkybox();

	glDeleteProgram(geometryProgramID);
	glDeleteProgram(programID);

	glDeleteVertexArrays(1, &uiVAO);
	vboSceneObjects.releaseVBO();
}