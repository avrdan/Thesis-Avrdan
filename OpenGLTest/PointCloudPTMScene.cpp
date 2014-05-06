#include "PointCloudPTMScene.h"
#include "static_geometry.h"

PointCloudPTMScene::PointCloudPTMScene()
{
	vertexShaderTexSource = "./assets/shaders/color.vert";
	fragmentShaderTexSource = "./assets/shaders/color.frag";

	vertexShaderSource = "./assets/shaders/color.vert";
	fragmentShaderSource = "./assets/shaders/color.frag";

	sphereModel = "./assets/models/blue_sphere/blueSphere.obj";
	sphereHalfModelPro = "./assets/models/blue_half_sphere_pro/blueHalfSpherePro.obj";

	fSunAngle = 45.0f;
	useMainCam = true;
	invertView = false;
	lockMainCam = true;
	timer = 0;
	zoomFactor = 1;
	fRotationAngle = 0.0f;
	bWireFrame = false;
	bUseMainCam = true;
	
	vertexShaderPTMSource = "./assets/shaders/ptm_color.vert";
	fragmentShaderPTMSource = "./assets/shaders/ptm_color.frag";

	// frustum params
	pFovY = 45.0;
	pAR = 4.0f / 3.0f;
	pNear = 0.1f;
	pFar = 2.0f;
	pUp = glm::vec3(0, 1.0f, 0);
}


PointCloudPTMScene::~PointCloudPTMScene()
{
}

void PointCloudPTMScene::initScene(GLFWwindow *window)
{
	glClearColor(0.0f, 0.0f, 0.4f, 1.0f);

	// set up pipeline
	pipeline = new Pipeline();
	pipeline->resetTimer();
	pipeline->setProjection3D(45.0f, 4.0f / 3.0f, 0.1f, 1000.0f);

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
	projectorOrigin = glm::vec3(0.0f, 0.0f, 0.002f);
	projectorTarget = glm::vec3(0.0f, 0.0f, 0.0f);
	projectorV = glm::lookAt(projectorOrigin, // projector origin
		projectorTarget,	 // project on object at origin 
		glm::vec3(0.0f, 1.0f, 0.0f)   // Y axis is up
		);
	mModel = glm::mat4(1.0f);

	// compute frustum coords
	computeFrustumCoords(projectorP);

	// init depth cam and compute
	// point cloud data
	// frame by frame
	rdp = new RawDepthPipeline();

	// load data
	glGenVertexArrays(1, &uiVAOSceneObjects);
	glBindVertexArray(uiVAOSceneObjects);

	// create and bind IBO
	ibo.createVBO();
	ibo.bindVBO(GL_ELEMENT_ARRAY_BUFFER);

	ibo.addData(&(rdp->indices[0]), rdp->indices.size() * sizeof(unsigned int));
	ibo.uploadDataToGPU(GL_STATIC_DRAW);
	//ibo.uploadDataToGPU(GL_STREAM_DRAW);

	// create and bind VBO
	vboSceneObjects.createVBO();
	vboSceneObjects.bindVBO();

	glBufferData(GL_ARRAY_BUFFER, rdp->nPoints * sizeof(PXCPoint3DF32), NULL, GL_STREAM_DRAW);
	// vertex positions start at index 0
	// the distance between two consecutive vertices is
	// sizeof whole vertex data
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(PXCPoint3DF32), 0);

	const char *cTextureNames[] = { "biohazard_512x512.jpg", "A_Smiley.jpg", "3dcarrush.jpg",
		"3D_Scene_011.jpg", "outdoor_free_3d_scene_by_djeric.jpg", "grid_512k.jpg",
		"grid-xxl.jpg", "grid-green.jpg", "tuscany.jpg" };
	vector<string> sTextureNames(cTextureNames, &cTextureNames[sizeof(cTextureNames) / sizeof(cTextureNames[0])]);
	loadAllTextures(sTextureNames);
	glEnable(GL_TEXTURE_2D);

	programID = LoadShaders(vertexShaderPTMSource, fragmentShaderPTMSource);
	//programID = LoadShaders(vertexShaderTexSource, fragmentShaderTexSource);
	programColorID = LoadShaders(vertexShaderSource, fragmentShaderSource);

	glPointSize(2.0f);
	glUseProgram(programID);


	std::cout << "Init end error..: " << gluErrorString(glGetError()) << "\n";
	std::cout << "Init end error.. " << gluErrorString(glGetError()) << "\n";
}

void PointCloudPTMScene::renderScene(GLFWwindow *window)
{
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	rdp->renderFrame();
	
	vboSceneObjects.addData(rdp->worldPos.data(), rdp->nPoints * sizeof(PXCPoint3DF32));
	vboSceneObjects.uploadDataToGPU(GL_STREAM_DRAW);
	
	computeMatricesFromInputs(window);
	timer += getDeltaTime();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//int iModelViewLoc = glGetUniformLocation(programID, "MV");
	//int iProjectionLoc = glGetUniformLocation(programID, "P");
	
	//glUniformMatrix4fv(iProjectionLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));

	glm::mat4 mModelView;
	if (bUseMainCam)
	{
		mModelView = getViewMatrix();
	}
	else
	{
		mModelView = projectorV;
	}

	glm::mat4 mCurrent;

	glUseProgram(programColorID);

	//int iColorLoc = glGetUniformLocation(programColorID, "inColor");
	//int iModelViewColorLoc = glGetUniformLocation(programColorID, "MV");
	//int iProjectionColorLoc = glGetUniformLocation(programColorID, "P");
	//glUniform4fv(iColorLoc, 1, glm::value_ptr(glm::vec4(0.0f, 1.0f, 0.0f, 1.0f)));
	//glUniformMatrix4fv(iProjectionColorLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));

	// render point cloud
	// how many points?
	if (bWireFrame)
	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	}
	else
	{

		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}
	mCurrent = glm::translate(mModelView, glm::vec3(0.0f, 0.0f, 0.0f));
	mCurrent = glm::rotate(mCurrent, 180.0f, glm::vec3(1.0f, 0.0f, 0.0f));
	//mCurrent = glm::scale(mCurrent, glm::vec3(10.0f, 10.0f, 10.0f));

	//glUniformMatrix4fv(iModelViewColorLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));

	glUseProgram(programID);


	// init PTM
	int iPTMModelViewLoc = glGetUniformLocation(programID, "MV");
	int iPTMProjectionLoc = glGetUniformLocation(programID, "P");
	int iPTMNormalLoc = glGetUniformLocation(programID, "N");
	int iTexGenMatLoc = glGetUniformLocation(programID, "TexGenMat");
	int iInvViewMatrix = glGetUniformLocation(programID, "InvViewMat");
	int iProjSamplerLoc = glGetUniformLocation(programID, "projMap");
	int iSamplerPTMLoc = glGetUniformLocation(programID, "gSampler");
	int iColorLoc = glGetUniformLocation(programID, "vColor");

		// set texture
	glClearDepth(1.0f);
	tTextures[5].bindTexture(1);


	// PTM MESH
	//mCurrent = glm::translate(mModelView, glm::vec3(0, 0, 0));
	//mModel = glm::mat4(1.0);
	texGenMatrix = biasMatrix * projectorP * projectorV;
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
	glUniform4fv(iColorLoc, 1, glm::value_ptr(glm::vec4(0.0f, 1.0f, 0.0f, 1.0f)));

	// END PTM MESH
	
	// draw mesh
	glDrawElements(
		GL_TRIANGLE_STRIP,      // mode
		rdp->indices.size(),    // count
		GL_UNSIGNED_INT,   // type
		(void*)0           // element array buffer offset
		);

	//std::cout << "Looping..: " << gluErrorString(glGetError()) << "\n";
	//std::cout << "Looping.. " << gluErrorString(glGetError()) << "\n";
	// render skybox
	//skybox.renderSkybox();
	fRotationAngle += pipeline->sof(30.0f);

	// make sure the float does not increase
	// drastically with time
	if (fRotationAngle >= 360)
		fRotationAngle -= 360;

	// user interaction
	if (timer >= 0.25f)
	{
		if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_M) != GLFW_RELEASE)
		{
			bWireFrame = !bWireFrame;
			timer = 0;
		}
	}

	if (timer >= 0.25f)
	{
		if (glfwGetKey(window, GLFW_KEY_V) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_V) != GLFW_RELEASE)
		{
			bUseMainCam = !bUseMainCam;
			timer = 0;
		}
	}
	// end user interaction

	// Swap front and back buffers
	glfwSwapBuffers(window);
	// Poll for and process events
	glfwPollEvents();

	pipeline->updateTimer();
}

void PointCloudPTMScene::releaseScene()
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

void PointCloudPTMScene::computeFrustumCoords(glm::mat4 mProjection)
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