#include "projTextureMappingScene.h"
#include "static_geometry.h"
#include "common/controls.h"
#include <iostream>

ProjTextureMappingScene::ProjTextureMappingScene()
{
	vertexShaderSource = "./assets/shaders/shader_tex.vert";
	fragmentShaderSource = "./assets/shaders/shader_tex.frag";
	vertexShaderLightSource = "./assets/shaders/dirLight.vert";
	fragmentShaderLightSource = "./assets/shaders/dirLight.frag";
	vertexShaderPTMSource = "./assets/shaders/ptm.vert";
	fragmentShaderPTMSource = "./assets/shaders/ptm.frag";
	vertexShaderSimpleSource = "./assets/shaders/simple.vert";
	fragmentShaderSimpleSource = "./assets/shaders/simple.frag";

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
	//torusModel = "./assets/models/blueTorusT.obj";
	//torusModel = "./assets/models/Torus.3ds";
	torusModel = "./assets/models/redTorus.obj";
	roomModel = "./assets/models/blueRoom.obj";
	tableModel = "./assets/models/table_cloth/Table with cloth.3DS";
	chairModel = "./assets/models/chair/chair.obj";
	bowlModel = "./assets/models/glass_bowl/Glass Bowl with Cloth Towel.fbx";
	suzanneModel = "./assets/models/suzanne/suzanne.obj";

	pFovY = 45.0;
	pAR = 4.0f / 3.0f;
	pNear = 0.1f;
	pFar = 2.0f;
	pUp = glm::vec3(0, 1.0f, 0);

	fSunAngle = 45.0f;
	useMainCam = true;
	timer = 0;
	zoomFactor = 1;
}


ProjTextureMappingScene::~ProjTextureMappingScene()
{
}

void ProjTextureMappingScene::initScene(GLFWwindow *window)
{
	glClearColor(0.0f, 0.0f, 0.4f, 1.0f);

	// set up pipeline
	pipeline = new Pipeline();
	pipeline->resetTimer();
	pipeline->setProjection3D(45.0f, 4.0f / 3.0f, 0.1f, 1000.0f);

	// set up camera
	//camera = new Camera(pipeline);


	// matrix data
	/*biasMatrix = glm::mat4(0.5f, 0, 0, 0.5f,
						   0, 0.5f, 0, 0.5f,
						   0, 0, 0.5f, 0.5f,
						   0, 0, 0,    1.0f);*/
	biasMatrix = glm::mat4(
		0.5, 0.0, 0.0, 0.0,
		0.0, 0.5, 0.0, 0.0,
		0.0, 0.0, 0.5, 0.0,
		0.5, 0.5, 0.5, 1.0
		);

	// 4:3 perspective with 45 fov
	//projectorP = glm::perspective(45.0f * zoomFactor, 4.0f / 3.0f, 0.1f, 1000.0f);
	projectorP = glm::perspective(pFovY, pAR, pNear, pFar);
	//projectorP = glm::ortho<float>(-100, 100, -100, 100, -100, 200);
	projectorOrigin = glm::vec3(-3.0f, 3.0f, 0.0f);
	projectorTarget = glm::vec3(0.0f, 0.0f, 0.0f);
	projectorV = glm::lookAt(projectorOrigin, // projector origin
		projectorTarget,	 // project on object at origin 
		glm::vec3(0.0f, 1.0f, 0.0f)   // Y axis is up
		);
	mModel = glm::mat4(1.0f);

	// test
	testCamV = glm::lookAt(glm::vec3(-3.0f, 3.0f, 0.0f), // projector origin
		glm::vec3(3.0f, -3.0f, 0.0f),	 // project on object at origin 
		glm::vec3(0.0f, 1.0f, 0.0f)   // Y axis is up
		);

	// compute frustum coords
	computeFrustumCoords(projectorP);
	//cout << "frustum coord 0: " << frustum_coords[0].x << ", " << frustum_coords[0].y << ", " << frustum_coords[0].z << endl;
	//CreateStaticSceneObjects2(&uiVAOSceneObjects, vboSceneObjects, frustum_coords);

	const char *cTextureNames[] = { "ground.jpg", "box.jpg", "biohazard_512x512.jpg", "A_Smiley.jpg" };
	vector<string> sTextureNames(cTextureNames, &cTextureNames[sizeof(cTextureNames) / sizeof(cTextureNames[0])]);
	
	//vector<string> sTextureNames = { "ground.jpg", "box.jpg", "A_Smiley.jpg" };
	//vector<string> sTextureNames = { "ground.jpg", "box.jpg", "matrix1k.jpg" };
	//vector<string> sTextureNames = { "ground.jpg", "box.jpg", "MiLn4eyia_512k.png" };
	loadAllTextures(sTextureNames);
	glEnable(GL_TEXTURE_2D);

	
	/*vboSceneObjects.createVBO();
	glGenVertexArrays(1, &uiVAOSceneObjects); // Create one VAO
	glBindVertexArray(uiVAOSceneObjects);

	vboSceneObjects.bindVBO();

	// geometry

	// Add ground to VBO
	for (int i = 0; i < 6; i++)
	{
		vboSceneObjects.addData(&vGround[i], sizeof(glm::vec3));
		glm::vec2 vCoord = vCubeTexCoordsAS[i] * 50.0f;
		vboSceneObjects.addData(&vCoord, sizeof(glm::vec2));
		glm::vec3 vGroundNormal(0.0f, 1.0f, 0.0f);
		vboSceneObjects.addData(&vGroundNormal, sizeof(glm::vec3));
	}


	// add cube to VBO
	for (int i = 0; i < 36; i++)
	{
		vboSceneObjects.addData(&vCubeVerticesL[i], sizeof(glm::vec3));
		vboSceneObjects.addData(&vCubeTexCoordsL[i % 6], sizeof(glm::vec2));
		vboSceneObjects.addData(&vCubeNormalsL[i / 6], sizeof(glm::vec3));
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
	*/
	
	amModels[0].LoadModelFromFile(torusModel);
	amModels[1].LoadModelFromFile(roomModel);
	amModels[2].LoadModelFromFile(suzanneModel);

	AssimpModel::FinalizeVBO();

	//programID = LoadShaders(vertexShaderSource, fragmentShaderSource);
	programID		= LoadShaders(vertexShaderLightSource, fragmentShaderLightSource);
	programPTM_ID   = LoadShaders(vertexShaderPTMSource, fragmentShaderPTMSource);
	programSimpleID = LoadShaders(vertexShaderSimpleSource, fragmentShaderSimpleSource);
	//programNoLightID = LoadShaders(vertexShaderSource, fragmentShaderSource);

	skybox.loadSkybox(skyboxPath, skyFront, skyBack, skyLeft, skyRight, skyUp, skyDown);
	glUseProgram(programID);
}

void ProjTextureMappingScene::renderScene(GLFWwindow *window)
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
	int iColorLoc   = glGetUniformLocation(programID, "vColor");


	glm::mat4 mModelView;
	glm::mat4 mCurrent;
	glm::mat4 mProjection;

	if (useMainCam)
	{
		mModelView = getViewMatrix();
		//mModelView = camera->Look();
		//mModelView = glm::lookAt(glm::vec3(3, 0, 0), glm::vec3(-3, 0, 0), glm::vec3(0.0f, 1.0f, 0.0f));
		mCurrent   = glm::mat4(1.0f);
		//mCurrent = glm::translate(mModelView, glm::vec3(3, -3, 0));
		mModelView = mModelView*mCurrent;
		mProjection = *pipeline->getProjectionMatrix();
	}
	else
	{
		
	
		//mCurrent = glm::translate(mModelView, projectorOrigin);
		//mCurrent = glm::rotate(mCurrent, 90.0f, glm::vec3(1, 0, 0));
		//mModelView = projectorV*glm::inverse(mModelView);
		//projectorV = glm::lookAt(glm::vec3(3, 0, 0), glm::vec3(-3, 0, 0), glm::vec3(0.0f, 1.0f, 0.0f));
		mModelView = projectorV;
		//mModelView = testCamV;
		//mModelView = glm::translate(mModelView, projectorOrigin);
		mProjection = projectorP;
	}
	


	glUniform3fv(iLightColorLoc, 1, glm::value_ptr(glm::vec3(1.0f, 1.0f, 1.0f)));
	glUniform1fv(iLightAmbientLoc, 1, &ambientIntensity);
	glUniform3fv(iLightDirectionLoc, 1, glm::value_ptr(-glm::normalize(vSunPos)));

	glUniformMatrix4fv(iProjectionLoc, 1, GL_FALSE, glm::value_ptr(mProjection));
	//glm::mat4 mModelView = glm::lookAt(glm::vec3(0, 12, 27), glm::vec3(0, 0, 0), glm::vec3(0.0f, 1.0f, 0.0f));
	glUniform1i(iSamplerLoc, 0);
	glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mModelView));
	glUniform4fv(iColorLoc, 1, glm::value_ptr(glm::vec4(1.0f, 1.0f, 1.0f, 1.0f)));
	//glUniform4fv(iColorLoc, 1, glm::value_ptr(glm::vec4(10.0f, 1.0f, 1.0f, 1.0f)));

	// init PTM
	int iPTMModelViewLoc = glGetUniformLocation(programPTM_ID, "MV");
	int iPTMProjectionLoc = glGetUniformLocation(programPTM_ID, "P");
	int iPTMNormalLoc     = glGetUniformLocation(programPTM_ID, "N");
	int iTexGenMatLoc     = glGetUniformLocation(programPTM_ID, "TexGenMat");
	int iInvViewMatrix    = glGetUniformLocation(programPTM_ID, "InvViewMat");
	int iProjSamplerLoc   = glGetUniformLocation(programPTM_ID, "projMap");
	int iSamplerPTMLoc    = glGetUniformLocation(programPTM_ID, "gSampler");

	// render ground
	glUseProgram(programPTM_ID);
	glBindVertexArray(uiVAOSceneObjects);
	tTextures[0].bindTexture();
	tTextures[3].bindTexture(1);
	//tTextures[2].setFiltering(TEXTURE_FILTER_MAG_BILINEAR, TEXTURE_FILTER_MIN_TRILINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);

	/*glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);*/
	mCurrent = glm::translate(mModelView, glm::vec3(0.0f, 0.0f, 0.0f));
	//glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mModelView));
	glUniformMatrix4fv(iNormalMatrixLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mCurrent))));
	glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));

	// PTM GROUND
	mCurrent = glm::translate(mModelView, glm::vec3(0, 0, 0));
	//mCurrent = glm::scale(mCurrent, glm::vec3(15.0f, 15.0f, 15.0f));
	mModel = glm::mat4(1.0);
	//mModel = glm::translate(mModel, projectorOrigin);
	//mModel = glm::translate(mModelView, glm::vec3(3.0f, -3.0f, 0.0f));

	texGenMatrix = biasMatrix * projectorP * projectorV * mModel;
	//texGenMatrix = biasMatrix * projectorP * projectorV;
	// inverse of the scene's view camera
	invViewMatrix = glm::inverse(mModelView);
	//invViewMatrix = glm::inverse(projectorV);

	glUniformMatrix4fv(iPTMProjectionLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));
	glUniformMatrix4fv(iPTMModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));
	glUniformMatrix4fv(iPTMNormalLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mCurrent))));
	glUniformMatrix4fv(iTexGenMatLoc, 1, GL_FALSE, glm::value_ptr(texGenMatrix));
	glUniformMatrix4fv(iInvViewMatrix, 1, GL_FALSE, glm::value_ptr(invViewMatrix));
	glUniform1i(iSamplerPTMLoc, 0);
	glUniform1i(iProjSamplerLoc, 1);
	// END PTM GROUND

	glDrawArrays(GL_TRIANGLES, 0, 6);

	//glUseProgram(programID);
	
	
	// render cube
	tTextures[1].bindTexture();

	glm::vec3 vPos = glm::vec3(2.5f, -9.5f, 0.0f);
	mModel = glm::mat4(1.0f);

	//mCurrent = glm::translate(glm::mat4(1.0), vPos);
	//mCurrent = glm::translate(getViewMatrix(), vPos);
	mCurrent = glm::translate(mModelView, vPos);
	//mCurrent = glm::scale(mCurrent, glm::vec3(1.0f, 1.0f, 1.0f));
	// transform normals
	// by transpose of inverse matrix
	// of transforms(except translation)
	texGenMatrix = biasMatrix * projectorP * projectorV;
	//texGenMatrix = biasMatrix * projectorP * projectorV;
	// inverse of the scene's view camera
	invViewMatrix = glm::inverse(mModelView);

	//glUniformMatrix4fv(iNormalMatrixLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mCurrent))));
	//glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mModelView*mCurrent));
	glUniformMatrix4fv(iPTMProjectionLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));
	glUniformMatrix4fv(iPTMNormalLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mCurrent))));
	glUniformMatrix4fv(iPTMModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));
	glUniformMatrix4fv(iTexGenMatLoc, 1, GL_FALSE, glm::value_ptr(texGenMatrix));
	glUniformMatrix4fv(iInvViewMatrix, 1, GL_FALSE, glm::value_ptr(invViewMatrix));
	glDrawArrays(GL_TRIANGLES, 6, 36);
	

	// render torus/table
	AssimpModel::BindModelsVAO();
	

	//mCurrent = glm::translate(getViewMatrix(), projectorOrigin);
	mCurrent = glm::translate(mModelView, glm::vec3(0, 0, 0));
	//mCurrent = glm::rotate(mCurrent, 90.0f, glm::vec3(1, 0, 0));
	//glUniformMatrix4fv(iNormalMatrixLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mCurrent))));
	//glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mModelView*mCurrent));
	//mCurrent = glm::scale(mCurrent, glm::vec3(1.0f, 1.0f, 1.0f));

	texGenMatrix = biasMatrix * projectorP * projectorV * mModel;
	//texGenMatrix = biasMatrix * projectorP * projectorV;
	// inverse of the scene's view camera
	invViewMatrix = glm::inverse(mModelView);

	//glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));
	glUniformMatrix4fv(iNormalMatrixLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mCurrent))));
	//glUniformMatrix4fv(iNormalMatrixLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));
	glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));

	//amModels[0].RenderModel();
	//amModels[2].RenderModel();
	
	// render room
	//AssimpModel::BindModelsVAO();

	mCurrent = glm::translate(mModelView, glm::vec3(3.0f, 0, 0));
	mCurrent = glm::scale(mCurrent, glm::vec3(1.0f, 1.0f, 1.0f));

	
	glUniformMatrix4fv(iNormalMatrixLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mCurrent))));
	glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));
	//glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));
	//amModels[1].RenderModel();
	//amModels[1].RenderModel(iModelViewLoc, iNormalMatrixLoc, mCurrent, mModelView);
	
	mModel = glm::mat4(1.0f);


	// PROJECTIVE TEXTURE MAPPING

	// RENDER PROJECTOR FRUSTUM
	glUseProgram(programSimpleID);
	glBindVertexArray(uiVAOSceneObjects);
	int iStaticColor = glGetUniformLocation(programSimpleID, "theColor");
	glUniform3fv(iStaticColor, 1, glm::value_ptr( glm::vec3(0, 1, 0) ));

	int iFrustumModelViewLoc = glGetUniformLocation(programSimpleID, "MV");
	int iFrustumProjectionLoc = glGetUniformLocation(programSimpleID, "P");
	int iFrustumInvProjectionLoc = glGetUniformLocation(programSimpleID, "invP");
	int iFrustumInvMVLoc = glGetUniformLocation(programSimpleID, "invMV");

	glUniformMatrix4fv(iFrustumInvProjectionLoc, 1, GL_FALSE, glm::value_ptr(glm::inverse(projectorP)));
	glUniformMatrix4fv(iFrustumInvMVLoc, 1, GL_FALSE, glm::value_ptr(glm::inverse(projectorV)));

	glUniformMatrix4fv(iFrustumProjectionLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));
	//glUniformMatrix4fv(iFrustumProjectionLoc, 1, GL_FALSE, glm::value_ptr(glm::inverse(projectorP)));
	mCurrent = glm::translate(mModelView, projectorOrigin);
	glUniformMatrix4fv(iFrustumModelViewLoc, 1, GL_FALSE, glm::value_ptr(mModelView));

	// wireframe
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glDisable(GL_CULL_FACE);
	glDrawArrays(GL_TRIANGLES, 42, 36 + 42);
	// fill polygons (normal)
	glEnable(GL_CULL_FACE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	//glEnable(GL_DEPTH_TEST);
	//glDepthFunc(GL_LEQUAL);

	glUseProgram(programPTM_ID);
	glClearDepth(1.0f);

	/*glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);*/

	// CUBE
	//mCurrent = glm::translate(mModelView, projectorOrigin);
	mCurrent = glm::translate(mModelView, projectorTarget);
	mModel = glm::mat4(1.0);
	//mModel = glm::translate(mModel, projectorOrigin);
	//mModel = glm::translate(mModel, glm::vec3(3.0f, -3.0f, 0));
	//mModel = glm::translate(projectorV, glm::vec3(-3, 0, 0));
	//mModel = glm::translate(projectorV, projectorOrigin);
	texGenMatrix = biasMatrix * projectorP * projectorV * mModel;
	// inverse of the scene's view camera
	invViewMatrix = glm::inverse(projectorV);
	//invViewMatrix = glm::inverse(mModelView);

	tTextures[1].bindTexture();
	tTextures[2].bindTexture(1);
	tTextures[2].setFiltering(TEXTURE_FILTER_MAG_BILINEAR, TEXTURE_FILTER_MIN_TRILINEAR);
	/*glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);*/

	glUniformMatrix4fv(iPTMProjectionLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));
	glUniformMatrix4fv(iPTMModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));
	glUniformMatrix4fv(iPTMNormalLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mCurrent))));


	glUniformMatrix4fv(iTexGenMatLoc, 1, GL_FALSE, glm::value_ptr(texGenMatrix));

	glUniformMatrix4fv(iInvViewMatrix, 1, GL_FALSE, glm::value_ptr(invViewMatrix));

	iSamplerLoc = glGetUniformLocation(programPTM_ID, "gSampler");
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
	

	//glBindVertexArray(uiVAOSceneObjects);
	//glDrawArrays(GL_TRIANGLES, 6, 36);

	// TORUS/table
	
	AssimpModel::BindModelsVAO();
	//glBindVertexArray(uiVAOSceneObjects);
	mCurrent = glm::translate(mModelView, glm::vec3(0, 0, 0));
	mModel = glm::mat4(1.0);
	texGenMatrix = biasMatrix * projectorP * projectorV * mModel;
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
	amModels[2].RenderModel();
	//glDrawArrays(GL_TRIANGLES, 0, 6);

	// END PROJECTIVE TEXTURE MAPPING

	glUseProgram(programID);
	//glUseProgram(programNoLightID);
	//glDisable(GL_DEPTH_TEST);
	// render skybox

	glUniform1i(iSkybox, 1);

	glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mModelView*glm::mat4(1.0)));
	skybox.renderSkybox();
	glUniform1i(iSkybox, 0);
	glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(glm::mat4(1.0)));

	//glEnable(GL_DEPTH_TEST);
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

	
	}

	//camera->Update(window);

	// Swap front and back buffers
	glfwSwapBuffers(window);
	// Poll for and process events
	glfwPollEvents();

	pipeline->updateTimer();
}

void ProjTextureMappingScene::releaseScene()
{
	for (int i = 0; i < Texture::NUMTEXTURES; i++)
	{
		tTextures[i].releaseTexture();
	}

	skybox.releaseSkybox();

	glDeleteProgram(programID);
	glDeleteProgram(programPTM_ID);
	glDeleteProgram(programNoLightID);
	glDeleteVertexArrays(1, &uiVAOSceneObjects);
	vboSceneObjects.releaseVBO();
}

void ProjTextureMappingScene::computeFrustumCoords(glm::mat4 mProjection)
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
	CreateStaticSceneObjects2(&uiVAOSceneObjects, vboSceneObjects, frustum_coords);

	// Get near and far from the Projection matrix.
	//float near = mProjection[1][1] / (mProjection[1][0] – 1.0f);
	//float far = mProjection[1][1] / (1.0f + mProjection[1][0]);
	/*

	// Get the sides of the near plane.
	float nLeft = near * (proj2 – 1.0) / proj0;
	float nRight = near * (1.0 + proj2) / proj0;
	float nTop = near * (1.0 + proj6) / proj5;
	float nBottom = near * (proj6 – 1.0) / proj5;


	// Get the sides of the far plane.
	float fLeft = far * (proj2 – 1.0) / proj0;
	float fRight = far * (1.0 + proj2) / proj0;
	float fTop = far * (1.0 + proj6) / proj5;
	float fBottom = far * (proj6 – 1.0) / proj5;*/
}