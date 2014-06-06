#include "shadowMappingScene.h"

ShadowMappingScene::ShadowMappingScene()
{
	vertexShaderSource = "./assets/shaders/shader_tex.vert";
	fragmentShaderSource = "./assets/shaders/shader_tex.frag";

	vertexLightShaderSource = "./assets/shaders/dirLight.vert";
	fragmentLightShaderSource = "./assets/shaders/dirLight.frag";

	vertexShaderDepthRTTSource = "./assets/shaders/depthRTT.vert";
	fragmentShaderDepthRTTSource = "./assets/shaders/depthRTT.frag";


	vertexShaderShadowMapSource = "./assets/shaders/shadow_map.vert";
	fragmentShaderShadowMapSource = "./assets/shaders/shadow_map.frag";

	vertexShaderQuadSource = "./assets/shaders/passthrough.vert";
	fragmentShaderQuadSource = "./assets/shaders/simpleTexture.frag";

	fGlobalAngle = 0;
	fSunAngle = 45.0f;
}


ShadowMappingScene::~ShadowMappingScene()
{
}

void ShadowMappingScene::initScene(GLFWwindow *window)
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

	// add DEBUG QUAD
	for (int i = 0; i < 6; i++)
	{
		//vboSceneObjects.addData(&ss_quad_pos[i], sizeof(glm::vec3));
		vboSceneObjects.addData(&g_quad_vertex_buffer_data[i], sizeof(glm::vec3));

		// dummy data to keep structure (NOT USED)
		vCubeTexCoordsL[i] *= 10.0f;
		vboSceneObjects.addData(&vCubeTexCoordsL[i % 6], sizeof(glm::vec2));
		glm::vec3 vGroundNormal(0.0f, 1.0f, 0.0f);
		vboSceneObjects.addData(&vGroundNormal, sizeof(glm::vec3));
	}

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

	//lightProgramID		= LoadShaders(vertexLightShaderSource, fragmentLightShaderSource);
	shadowMapProgramID  = LoadShaders(vertexShaderShadowMapSource, fragmentShaderShadowMapSource);
	depthProgramID		= LoadShaders(vertexShaderDepthRTTSource, fragmentShaderDepthRTTSource);
	quadprogramID		= LoadShaders(vertexShaderQuadSource, fragmentShaderQuadSource);
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

	// render to texture
	glfwGetWindowSize(window, &width, &height);

	glGenFramebuffers(1, &fb);
	glBindFramebuffer(GL_FRAMEBUFFER, fb);

	//glClearColor(0.2f, 0.2f, 0.4f, 0.5f);
	glClearDepth(1.0f);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	glGenTextures(1, &depthTexture);
	glBindTexture(GL_TEXTURE_2D, depthTexture);
	// IMPORTANT: texture must be COMPLETE (mipmaps must be specified..
	// or the following parameters can be used if there are none)
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
	glTexImage2D(GL_TEXTURE_2D,
		0,
		GL_DEPTH_COMPONENT16,
		width,
		height,
		0,
		GL_DEPTH_COMPONENT,
		GL_FLOAT,
		NULL);

	// texture filtering
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);
	//DEBUG
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);

	// bind fb
	glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depthTexture, 0);

	// no color ouput, only depth
	glDrawBuffer(GL_NONE);

	// Always check that our framebuffer is ok
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	{
		cout << "Framebuffer error" << endl;
	}

	// set up shadow projection

	// correct the light...needs to be updated
	// based on the sun's position
	lightInvDir = glm::vec3(0.5f, 2, 2);

	// Compute the MVP matrix from the light's point of view
	depthProjectionMatrix = glm::ortho<float>(-100, 100, -100, 100, -100, 200);
	depthViewMatrix = glm::lookAt(lightInvDir, glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
	depthModelMatrix = glm::mat4(1.0);
	depthMVP = depthProjectionMatrix * depthViewMatrix * depthModelMatrix;
	biasMatrix = glm::mat4(
		0.5, 0.0, 0.0, 0.0,
		0.0, 0.5, 0.0, 0.0,
		0.0, 0.0, 0.5, 0.0,
		0.5, 0.5, 0.5, 1.0
		);
	depthBiasMVP = biasMatrix*depthMVP;

	// Get a handle for our "MVP" uniform
	//matrixID = glGetUniformLocation(programID, "MVP");
	depthBiasID = glGetUniformLocation(shadowMapProgramID, "depthBiasMVP");
	shadowMapID = glGetUniformLocation(shadowMapProgramID, "shadowMap");

	depthMatrixID = glGetUniformLocation(depthProgramID, "depthMVP");

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glUseProgram(shadowMapProgramID);
}

void ShadowMappingScene::renderScene(GLFWwindow *window)
{
	// set light properties
	float fSine = sinf(fSunAngle*3.1415f / 180.0f);
	vSunPos = glm::vec3(cosf(fSunAngle*3.1415f / 180.0f) * 70, sinf(fSunAngle*3.1415f / 180.0f) * 70, 0.0f);

	// change the color of the sky
	// depending on the sun's position
	glClearColor(0.0f, max(0.0f, 0.9f*fSine), max(0.0f, 0.9f*fSine), 1.0f);

	lightInvDir = (-1.0f) * vSunPos;

	// PASS I : render to the custom framebuffer
	// -----------------------------------------

	glBindFramebuffer(GL_FRAMEBUFFER, fb);
	glViewport(0, 0, width, height);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(depthProgramID);

	// recompute matrices which change
	depthViewMatrix = glm::lookAt(lightInvDir, glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
	depthMVP = depthProjectionMatrix * depthViewMatrix * depthModelMatrix;
	

	glBindVertexArray(uiVAO);

	// CREATE DEPTH MAP
	/*
	// render ground
	glDrawArrays(GL_TRIANGLES, 36, 6);
	// render cubes
	glDrawArrays(GL_TRIANGLES, 0, 36);
	// render tori
	glDrawArrays(GL_TRIANGLES, 42, iTorusFaces1 * 3);
	*/

	// render ground
	/*depthModelMatrix = glm::translate(depthViewMatrix, glm::vec3(0.0f, 0.0f, 0.0f));
	// model is now MV due to prev computations
	depthMVP = depthProjectionMatrix * depthModelMatrix;
	//depthMVP = depthProjectionMatrix * depthViewMatrix * depthModelMatrix;
	glUniformMatrix4fv(depthMatrixID, 1, GL_FALSE, glm::value_ptr(depthMVP));
	glDrawArrays(GL_TRIANGLES, 36, 6);*/
	
	// render cubes
	for (int i = 0; i < 5; i++)
	{
		float fSign = -1.0f + float(i % 2)*2.0f; // returns -1 or 1
		glm::vec3 vPos = glm::vec3(fSign*15.0f, 0.0f, 50.0f - float(i)*25.0f);
		//mCurrent = glm::translate(glm::mat4(1.0), vPos);
		depthModelMatrix = glm::translate(depthViewMatrix, vPos);
		depthModelMatrix = glm::scale(depthModelMatrix, glm::vec3(8.0f, 8.0f, 8.0f));
		// model is now MV due to prev computations
		depthMVP = depthProjectionMatrix * depthModelMatrix;
		glUniformMatrix4fv(depthMatrixID, 1, GL_FALSE, glm::value_ptr(depthMVP));
		glDrawArrays(GL_TRIANGLES, 0, 36);
	}
	// render tori
	for (int i = 0; i < 5; i++)
	{
		float fSign = 1.0f - float(i % 2)*2.0f; // This just returns -1.0f or 1.0f (try to examine this)
		glm::vec3 vPos = glm::vec3(fSign*15.0f, 0.0f, 50.0f - float(i)*25.0f);
		//mCurrent = glm::translate(glm::mat4(1.0), vPos);
		depthModelMatrix = glm::translate(depthViewMatrix, vPos);
		depthModelMatrix = glm::rotate(depthModelMatrix, fGlobalAngle + i*30.0f, glm::vec3(0.0f, 1.0f, 0.0f));
		// model is now MV due to prev computations
		depthMVP = depthProjectionMatrix * depthModelMatrix;
		glUniformMatrix4fv(depthMatrixID, 1, GL_FALSE, glm::value_ptr(depthMVP));
		glDrawArrays(GL_TRIANGLES, 42, iTorusFaces1 * 3);
	}
	
	// DONE CREATING DEPTH MAP


	// PASS II :Render to the screen
	// -----------------------------------------


	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glViewport(0, 0, width, height);

	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(shadowMapProgramID);

	computeMatricesFromInputs(window);

	glBindVertexArray(uiVAO);

	// set uniforms
	int iLightColorLoc = glGetUniformLocation(shadowMapProgramID, "sunLight.vColor");
	glUniform3fv(iLightColorLoc, 1, glm::value_ptr(glm::vec3(1.0f, 1.0f, 1.0f)));
	int iLightAmbientLoc = glGetUniformLocation(shadowMapProgramID, "sunLight.fAmbientIntensity");
	float ambientIntensity = 0.25f;
	glUniform1fv(iLightAmbientLoc, 1, &ambientIntensity);
	int iLightDirectionLoc = glGetUniformLocation(shadowMapProgramID, "sunLight.vDirection");
	glUniform3fv(iLightDirectionLoc, 1, glm::value_ptr(-glm::normalize(vSunPos)));

	int iModelViewLoc = glGetUniformLocation(shadowMapProgramID, "MV");
	int iProjectionLoc = glGetUniformLocation(shadowMapProgramID, "P");
	int iNormalMatrixLoc = glGetUniformLocation(shadowMapProgramID, "N");
	
	

	glUniformMatrix4fv(iProjectionLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));
	glm::mat4 mModelView = getViewMatrix();
	glm::mat4 mCurrent;

	int iSamplerLoc = glGetUniformLocation(shadowMapProgramID, "gSampler");
	int samplerVal = 0;
	//glUniform1iv(iSamplerLoc, 1, &samplerVal);
	glUniform1i(iSamplerLoc, 0);

	glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mModelView));
	int iColorLoc = glGetUniformLocation(shadowMapProgramID, "vColor");
	glUniform4fv(iColorLoc, 1, glm::value_ptr(glm::vec4(1.0f, 1.0f, 1.0f, 1.0f)));

	// render ground
	tTextures[0].bindTexture();

	// bind shadow map
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, depthTexture);
	glUniform1i(shadowMapID, 1);

	mCurrent = glm::translate(mModelView, glm::vec3(0.0f, 0.0f, 0.0f));
	glUniformMatrix4fv(iNormalMatrixLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mCurrent))));
	glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));

	depthModelMatrix = glm::translate(depthViewMatrix, glm::vec3(0.0f, 0.0f, 0.0f));
	// model is now MV due to prev computations
	depthMVP = depthProjectionMatrix * depthModelMatrix;

	// compute depth bias matrix
	depthBiasMVP = biasMatrix*depthMVP;
	// pass depth bias matrix
	glUniformMatrix4fv(depthBiasID, 1, GL_FALSE, glm::value_ptr(depthBiasMVP));

	glDrawArrays(GL_TRIANGLES, 36, 6);
	// render 5 cubes
	tTextures[1].bindTexture();
	// bind shadow map
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, depthTexture);
	glUniform1i(shadowMapID, 1);
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
		glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));

		depthModelMatrix = glm::translate(depthViewMatrix, vPos);
		depthModelMatrix = glm::scale(depthModelMatrix, glm::vec3(8.0f, 8.0f, 8.0f));
		// model is now MV due to prev computations
		depthMVP = depthProjectionMatrix * depthModelMatrix;

		// compute depth bias matrix
		depthBiasMVP = biasMatrix*depthMVP;
		// pass depth bias matrix
		glUniformMatrix4fv(depthBiasID, 1, GL_FALSE, glm::value_ptr(depthBiasMVP));

		glDrawArrays(GL_TRIANGLES, 0, 36);
	}

	// render 5 tori
	tTextures[2].bindTexture();
	// bind shadow map
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, depthTexture);
	glUniform1i(shadowMapID, 1);
	for (int i = 0; i < 5; i++)
	{
		float fSign = 1.0f - float(i % 2)*2.0f; // This just returns -1.0f or 1.0f (try to examine this)
		glm::vec3 vPos = glm::vec3(fSign*15.0f, 0.0f, 50.0f - float(i)*25.0f);
		//mCurrent = glm::translate(glm::mat4(1.0), vPos);
		mCurrent = glm::translate(mModelView, vPos);
		mCurrent = glm::rotate(mCurrent, fGlobalAngle + i*30.0f, glm::vec3(0.0f, 1.0f, 0.0f));

		glUniformMatrix4fv(iNormalMatrixLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mCurrent))));
		glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));

		depthModelMatrix = glm::translate(depthViewMatrix, vPos);
		depthModelMatrix = glm::rotate(depthModelMatrix, fGlobalAngle + i*30.0f, glm::vec3(0.0f, 1.0f, 0.0f));
		// model is now MV due to prev computations
		depthMVP = depthProjectionMatrix * depthModelMatrix;

		// compute depth bias matrix
		depthBiasMVP = biasMatrix*depthMVP;
		// pass depth bias matrix
		glUniformMatrix4fv(depthBiasID, 1, GL_FALSE, glm::value_ptr(depthBiasMVP));

		glDrawArrays(GL_TRIANGLES, 42, iTorusFaces1 * 3);
	}

	// render the sun
	tTextures[3].bindTexture();
	//mCurrent = glm::translate(glm::mat4(1.0f), vSunPos);
	mCurrent = glm::translate(mModelView, vSunPos);
	glUniformMatrix4fv(iNormalMatrixLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mCurrent))));
	glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));
	// make it shine
	float intensity = 0.8f;
	glUniform1fv(iLightAmbientLoc, 1, &intensity);

	glDrawArrays(GL_TRIANGLES, 42 + iTorusFaces1 * 3, iTorusFaces2 * 3);

	// PART III: Optionally render the shadowmap (for debug only)
	// ----------------------------------------------------------
	
	// Render only on a corner of the window (or we we won't see the real rendering...)
	glViewport(0, 0, 512, 512);
	GLuint texID = glGetUniformLocation(quadprogramID, "tex");
	// Use our shader
	glUseProgram(quadprogramID);

	// Bind our texture in Texture Unit 0
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, depthTexture);
	// Set our "renderedTexture" sampler to user Texture Unit 0
	glUniform1i(texID, 0);

	glBindVertexArray(uiVAO);
	glDrawArrays(GL_TRIANGLES, 42 + iTorusFaces1 * 3 + iTorusFaces2 * 3, 42 + iTorusFaces1 * 3 + iTorusFaces2 * 3 + 6);
	

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

void ShadowMappingScene::releaseScene()
{
	for (int i = 0; i < 4; i++)
	{
		tTextures[i].releaseTexture();
	}

	glDeleteProgram(depthProgramID);
	glDeleteProgram(shadowMapProgramID);

	glDeleteVertexArrays(1, &uiVAO);
	vboSceneObjects.releaseVBO();
	glDeleteFramebuffers(1, &fb);
	glDeleteTextures(1, &depthTexture);
}