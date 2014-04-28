#include "PTMTestScene.h"

PTMTestScene::PTMTestScene()
{
	vertexShaderLightSource = "./assets/shaders/dirLight.vert";
	fragmentShaderLightSource = "./assets/shaders/dirLight.frag";
	vertexShaderPTMSource = "./assets/shaders/ptm.vert";
	fragmentShaderPTMSource = "./assets/shaders/ptm.frag";
	vertexShaderSimpleSource = "./assets/shaders/simple.vert";
	fragmentShaderSimpleSource = "./assets/shaders/simple.frag";

	sphereModel = "./assets/models/blue_sphere/blueSphere.obj";
	sphereHalfModel = "./assets/models/blue_half_sphere/blueHalfSphere.obj";
	sphereHalfModelPro = "./assets/models/blue_half_sphere_pro/blueHalfSpherePro.obj";
	sphereHalfModelCut = "./assets/models/blue_half_sphere_cut/blueHalfSphereCut.obj";

	pFovY = 45.0;
	pAR = 4.0f / 3.0f;
	pNear = 0.1f;
	pFar = 5.0f;
	pUp = glm::vec3(0, 1.0f, 0);

	fSunAngle = 45.0f;
	useMainCam = true;
	invertView = false;
	lockMainCam = true;
	timer = 0;
	zoomFactor = 1;
}


PTMTestScene::~PTMTestScene()
{
}

void PTMTestScene::initScene(GLFWwindow *window)
{
	glClearColor(0.0f, 0.0f, 0.4f, 1.0f);

	// set up pipeline
	pipeline = new Pipeline();
	pipeline->resetTimer();
	pipeline->setProjection3D(45.0f, 4.0f / 3.0f, 0.1f, 1000.0f);

	// matrix data
	biasMatrix = glm::mat4(
		0.5, 0.0, 0.0, 0.0,
		0.0, 0.5, 0.0, 0.0,
		0.0, 0.0, 0.5, 0.0,
		0.5, 0.5, 0.5, 1.0
		);

	//biasMatrix = glm::mat4(1.0f);

	// 4:3 perspective with 45 fov
	//projectorP = glm::perspective(45.0f * zoomFactor, 4.0f / 3.0f, 0.1f, 1000.0f);
	//projectorP = glm::perspective(pFovY, pAR, pNear, pFar);
	projectorP = glm::perspective(30.0f, 1.0f, 0.2f, 1000.0f);
	//projectorP = glm::ortho<float>(-100, 100, -100, 100, -100, 200);
	//projectorOrigin = glm::vec3(0.0f, 0.0f, 3.0f);
	projectorOrigin = glm::vec3(0.0f, 0.0f, 8.0f);
	projectorTarget = glm::vec3(0.0f, 0.0f, 0.0f);
	projectorV = glm::lookAt(projectorOrigin, // projector origin
		projectorTarget,	 // project on object at origin 
		glm::vec3(0.0f, 1.0f, 0.0f)   // Y axis is up
		);
	mModel = glm::mat4(1.0f);


	// compute frustum coords
	computeFrustumCoords(projectorP);

	//vector<string> sTextureNames = { "ground.jpg", "box.jpg", "biohazard_512x512.jpg" };
	//vector<string> sTextureNames = { "A_Smiley.jpg" };
	const char *cTextureNames[] = { "biohazard_512x512.jpg", "A_Smiley.jpg", "3dcarrush.jpg",
		"3D_Scene_011.jpg", "outdoor_free_3d_scene_by_djeric.jpg", "grid_512k.jpg",
		"grid-xxl.jpg", "grid-green.jpg", "tuscany.jpg" };
	vector<string> sTextureNames(cTextureNames, &cTextureNames[sizeof(cTextureNames) / sizeof(cTextureNames[0])]);
	loadAllTextures(sTextureNames);
	glEnable(GL_TEXTURE_2D);

	//amModels[0].LoadModelFromFile(sphereModel);
	//amModels[0].LoadModelFromFile(sphereHalfModel);
	amModels[0].LoadModelFromFile(sphereHalfModelPro);
	//amModels[0].LoadModelFromFile(sphereHalfModelCut);

	AssimpModel::FinalizeVBO();

	programID = LoadShaders(vertexShaderLightSource, fragmentShaderLightSource);
	programPTM_ID = LoadShaders(vertexShaderPTMSource, fragmentShaderPTMSource);
	programSimpleID = LoadShaders(vertexShaderSimpleSource, fragmentShaderSimpleSource);

	glUseProgram(programID);
}

void PTMTestScene::renderScene(GLFWwindow *window)
{

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	computeMatricesFromInputs(window);

	timer += getDeltaTime();
	// set light properties
	float fSine = sinf(fSunAngle*3.1415f / 180.0f);
	glm::vec3 vSunPos(cosf(fSunAngle*3.1415f / 180.0f) * 70, sinf(fSunAngle*3.1415f / 180.0f) * 70, 0.0f);

	// change the color of the sky
	// depending on the sun's position
	//glClearColor(0.0f, max(0.0f, 0.9f*fSine), max(0.0f, 0.9f*fSine), 1.0f);

	// set uniforms
	int iLightColorLoc = glGetUniformLocation(programID, "sunLight.vColor");
	int iLightAmbientLoc = glGetUniformLocation(programID, "sunLight.fAmbientIntensity");
	int iLightDirectionLoc = glGetUniformLocation(programID, "sunLight.vDirection");
	float ambientIntensity = 0.25f;
	int iSkybox = glGetUniformLocation(programID, "sunLight.iSkybox");

	int iModelViewLoc = glGetUniformLocation(programID, "MV");
	int iProjectionLoc = glGetUniformLocation(programID, "P");
	int iNormalMatrixLoc = glGetUniformLocation(programID, "N");

	int iSamplerLoc = glGetUniformLocation(programID, "gSampler");
	int iColorLoc = glGetUniformLocation(programID, "vColor");

	glm::mat4 mModelView;
	glm::mat4 mCurrent;
	glm::mat4 mProjection;

	if (useMainCam)
	{
		if (lockMainCam)
		{
			if (invertView)
				mModelView = glm::lookAt(glm::vec3(0.0f, 0.0f, 6.0f), glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
			else
				mModelView = glm::lookAt(glm::vec3(0.0f, 0.0f, -6.0f), glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
		}
		else
		{
			mModelView = getViewMatrix();
		}
	
		mCurrent = glm::mat4(1.0f);
		mModelView = mModelView;
		mProjection = *pipeline->getProjectionMatrix();
	}
	else
	{
		mModelView = projectorV;
		mProjection = projectorP;
	}

	mModel = glm::mat4(1.0f);

	// PROJECTIVE TEXTURE MAPPING

	// RENDER PROJECTOR FRUSTUM
	glUseProgram(programSimpleID);
	glBindVertexArray(uiVAOSceneObjects);
	int iStaticColor = glGetUniformLocation(programSimpleID, "theColor");
	glUniform3fv(iStaticColor, 1, glm::value_ptr(glm::vec3(0, 1, 0)));

	int iFrustumModelViewLoc = glGetUniformLocation(programSimpleID, "MV");
	int iFrustumProjectionLoc = glGetUniformLocation(programSimpleID, "P");
	int iFrustumInvProjectionLoc = glGetUniformLocation(programSimpleID, "invP");
	int iFrustumInvMVLoc = glGetUniformLocation(programSimpleID, "invMV");

	glUniformMatrix4fv(iFrustumInvProjectionLoc, 1, GL_FALSE, glm::value_ptr(glm::inverse(projectorP)));
	glUniformMatrix4fv(iFrustumInvMVLoc, 1, GL_FALSE, glm::value_ptr(glm::inverse(projectorV)));

	glUniformMatrix4fv(iFrustumProjectionLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));
	//glUniformMatrix4fv(iFrustumProjectionLoc, 1, GL_FALSE, glm::value_ptr(glm::inverse(projectorP)));
	mCurrent = glm::translate(mModelView, projectorOrigin);
	glUniformMatrix4fv(iFrustumModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));

	// wireframe
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glDisable(GL_CULL_FACE);
	glDrawArrays(GL_TRIANGLES, 0, 36);
	// fill polygons (normal)
	//glEnable(GL_CULL_FACE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glUseProgram(programPTM_ID);
	glClearDepth(1.0f);
	tTextures[5].bindTexture(1);
	//tTextures[0].setFiltering(TEXTURE_FILTER_MAG_BILINEAR, TEXTURE_FILTER_MIN_TRILINEAR);
	/*glTexParameteri(GL_TEXTURE_2D,
		GL_TEXTURE_WRAP_S,
		GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D,
		GL_TEXTURE_WRAP_T,
		GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D,
		GL_TEXTURE_WRAP_R,
		GL_CLAMP_TO_BORDER);*/

	int iPTMModelViewLoc = glGetUniformLocation(programPTM_ID, "MV");
	int iPTMProjectionLoc = glGetUniformLocation(programPTM_ID, "P");
	int iPTMNormalLoc = glGetUniformLocation(programPTM_ID, "N");
	glUniformMatrix4fv(iPTMProjectionLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));
	glUniformMatrix4fv(iPTMModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));
	glUniformMatrix4fv(iPTMNormalLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mCurrent))));

	int iTexGenMatLoc = glGetUniformLocation(programPTM_ID, "TexGenMat");
	glUniformMatrix4fv(iTexGenMatLoc, 1, GL_FALSE, glm::value_ptr(texGenMatrix));
	int iInvViewMatrix = glGetUniformLocation(programPTM_ID, "InvViewMat");
	glUniformMatrix4fv(iInvViewMatrix, 1, GL_FALSE, glm::value_ptr(invViewMatrix));

	iSamplerLoc = glGetUniformLocation(programPTM_ID, "gSampler");
	int iProjSamplerLoc = glGetUniformLocation(programPTM_ID, "projMap");
	glUniform1i(iSamplerLoc, 0);
	glUniform1i(iProjSamplerLoc, 1);

	iLightColorLoc = glGetUniformLocation(programPTM_ID, "sunLight.vColor");
	iLightAmbientLoc = glGetUniformLocation(programPTM_ID, "sunLight.fAmbientIntensity");
	iLightDirectionLoc = glGetUniformLocation(programPTM_ID, "sunLight.vDirection");
	iColorLoc = glGetUniformLocation(programPTM_ID, "vColor");

	glUniform3fv(iLightColorLoc, 1, glm::value_ptr(glm::vec3(1.0f, 1.0f, 1.0f)));
	glUniform1fv(iLightAmbientLoc, 1, &ambientIntensity);
	glUniform3fv(iLightDirectionLoc, 1, glm::value_ptr(-glm::normalize(vSunPos)));
	glUniform4fv(iColorLoc, 1, glm::value_ptr(glm::vec4(1.0f, 1.0f, 1.0f, 1.0f)));


	// SPHERE

	AssimpModel::BindModelsVAO();
	//glBindVertexArray(uiVAOSceneObjects);
	mCurrent = glm::translate(mModelView, glm::vec3(0, 0, 0));

	mCurrent = glm::rotate(mCurrent, 90.0f, glm::vec3(0, 1, 0));
	//mCurrent = glm::scale(mCurrent, glm::vec3(2, 2, 2));
	mModel = glm::mat4(1.0);
	//mModel = glm::rotate(mCurrent, 90.0f, glm::vec3(0, 1, 0));
	//mModel = glm::scale(mCurrent, glm::vec3(2, 2, 2));
	//texGenMatrix = biasMatrix * projectorP * projectorV * mModel;
	texGenMatrix = biasMatrix * projectorP * projectorV;
	// inverse of the scene's view camera
	invViewMatrix = glm::inverse(mModelView);
	//invViewMatrix = glm::inverse(projectorV);

	glUniformMatrix4fv(iPTMProjectionLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));
	glUniformMatrix4fv(iPTMModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));
	glUniformMatrix4fv(iPTMNormalLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mCurrent))));
	glUniformMatrix4fv(iTexGenMatLoc, 1, GL_FALSE, glm::value_ptr(texGenMatrix));
	glUniformMatrix4fv(iInvViewMatrix, 1, GL_FALSE, glm::value_ptr(invViewMatrix));
	glUniform1i(iSamplerLoc, 0);
	glUniform1i(iProjSamplerLoc, 1);

	//amModels[0].RenderModel();
	amModels[0].RenderModel();
	//glDrawArrays(GL_TRIANGLES, 0, 6);

	// END PROJECTIVE TEXTURE MAPPING

	glUseProgram(programID);
	// interaction
	if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS)
	{
		//cout << "Changing angle --" << endl;
		fSunAngle -= pipeline->sof(45.0f);
	}

	if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS)
	{
		//cout << "Changing angle ++" << endl;
		fSunAngle += pipeline->sof(45.0f);
	}

	// zoom
	if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS)
	{

		zoomFactor -= pipeline->sof(1.0f);
		projectorP = glm::perspective(45.0f * zoomFactor, 4.0f / 3.0f, 0.1f, 1000.0f);
	}

	if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS)
	{

		zoomFactor += pipeline->sof(1.0f);
		projectorP = glm::perspective(45.0f * zoomFactor, 4.0f / 3.0f, 0.1f, 1000.0f);
	}

	if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS)
	{
		zoomFactor -= pipeline->sof(1.0f);
		projectorV = glm::translate(projectorV, glm::vec3(0.0f, 0.0f, -0.1f));
		//projectorV = glm::translate(projectorV, glm::vec3(0.0f, -0.1f, 0.0f));
	}

	if (glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS)
	{

		zoomFactor += pipeline->sof(1.0f);
		projectorV = glm::translate(projectorV, glm::vec3(0.0f, 0.0f, 0.1f));
		//projectorV = glm::translate(projectorV, glm::vec3(0.0f, 0.1f, 0.0f));
	}

	//cout << "timer:" << timer << endl;
	if (timer >= 0.25f)
	{

		if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_P) != GLFW_RELEASE)
		{
			cout << "PRESSED" << endl;
			// change main camera to 
			// the projector camera
			// and conversely
			useMainCam = !useMainCam;

			timer = 0;
		}

		if (glfwGetKey(window, GLFW_KEY_V) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_V) != GLFW_RELEASE)
		{
			cout << "PRESSED" << endl;
			// change main camera to 
			// the projector camera
			// and conversely
			invertView = !invertView;

			timer = 0;
		}

		if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_L) != GLFW_RELEASE)
		{
			cout << "PRESSED" << endl;
			// change main camera to 
			// the projector camera
			// and conversely
			lockMainCam = !lockMainCam;

			timer = 0;
		}

	}

	// Swap front and back buffers
	glfwSwapBuffers(window);
	// Poll for and process events
	glfwPollEvents();

	pipeline->updateTimer();
}

void PTMTestScene::releaseScene()
{
	for (int i = 0; i < Texture::NUMTEXTURES; i++)
	{
		tTextures[i].releaseTexture();
	}

	skybox.releaseSkybox();

	glDeleteProgram(programID);
	glDeleteProgram(programPTM_ID);
	glDeleteVertexArrays(1, &uiVAOSceneObjects);
	vboSceneObjects.releaseVBO();
}

void PTMTestScene::computeFrustumCoords(glm::mat4 mProjection)
{
	glm::mat4 invProj = glm::inverse(mProjection);

	glm::vec3 ftl = glm::vec3(-1, +1, pFar); //far top left
	glm::vec3 fbr = glm::vec3(+1, -1, pFar); //far bottom right
	glm::vec3 fbl = glm::vec3(-1, -1, pFar); //far bottom left
	glm::vec3 ftr = glm::vec3(+1, +1, pFar); //far top right
	glm::vec3 ntl = glm::vec3(-1, +1, pNear); //near top left
	glm::vec3 nbr = glm::vec3(+1, -1, pNear); //near bottom right
	glm::vec3 nbl = glm::vec3(-1, -1, pNear); //near bottom left
	glm::vec3 ntr = glm::vec3(+1, +1, pNear); //near top right

	glm::vec3	frustum_coords[36] = {
		/*
		//near
		glm::vec3(-1, -1, pNear), glm::vec3(+1, +1, pNear), glm::vec3(-1, +1, pNear),
		glm::vec3(+1, +1, pNear), glm::vec3(-1, -1, pNear), glm::vec3(+1, -1, pNear),
		//far
		glm::vec3(-1, -1, pFar), glm::vec3(+1, +1, pFar), glm::vec3(-1, +1, pFar),
		glm::vec3(+1, +1, pFar), glm::vec3(-1, -1, pFar), glm::vec3(+1, -1, pFar),
		// left
		nbl, ftl, ntl,
		ftl, nbl, fbl,
		// right
		// bottom
		// top
		*/
		// near
		ntl, nbl, ntr, // 1 triangle (9 verts)
		ntr, nbl, nbr,
		// right
		nbr, ftr, ntr,
		ftr, nbr, fbr,
		// left
		nbl, ftl, ntl,
		ftl, nbl, fbl,
		// far
		ftl, fbl, fbr,
		fbr, ftr, ftl,
		//bottom
		nbl, fbr, fbl,
		fbr, nbl, nbr,
		//top
		ntl, ftr, ftl,
		ftr, ntl, ntr
	};


	cout << "frustum coord 0: " << frustum_coords[0].x << ", " << frustum_coords[0].y << ", " << frustum_coords[0].z << endl;
//	CreateStaticSceneObjects2(&uiVAOSceneObjects, vboSceneObjects, frustum_coords);
	CreateFrustumOutlines(&uiVAOSceneObjects, vboSceneObjects, frustum_coords);
}