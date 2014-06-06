#pragma warning(disable:4996) 
#include "PointCloudPCLScene.h"

PointCloudPCLScene::PointCloudPCLScene()
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
}


PointCloudPCLScene::~PointCloudPCLScene()
{
}

void PointCloudPCLScene::initScene(GLFWwindow *window)
{
	glClearColor(0.0f, 0.0f, 0.4f, 1.0f);

	// set up pipeline
	pipeline = new Pipeline();
	pipeline->resetTimer();
	pipeline->setProjection3D(45.0f, 4.0f / 3.0f, 0.1f, 1000.0f);

	// load data
	vboSceneObjects.createVBO();
	glGenVertexArrays(1, &uiVAOSceneObjects);
	glBindVertexArray(uiVAOSceneObjects);

	vboSceneObjects.bindVBO();

	// add cube to vbo
	for (int i = 0; i < 36; i++)
	{
		// vertices for one face
		vboSceneObjects.addData(&vCubeVertices[i], sizeof(glm::vec3));
		// texture coords for one face
		//vboSceneObjects.addData(&vCubeTexCoords[i % 6], sizeof(glm::vec2));
	}

	// add ground to VBO
	for (int i = 0; i < 6; i++)
	{
		vboSceneObjects.addData(&vGround[i], sizeof(glm::vec3));
		// scale cube coords
		// or provide different ground tex coords
		vCubeTexCoords[i] *= 5.0f;
		//vboSceneObjects.addData(&vCubeTexCoords[i % 6], sizeof(glm::vec2));
	}

	for (int i = 0; i < (int)320 * 240 / 3; i++)
	{
		glm::vec3 zero = glm::vec3(0, 0, 0);
		//glm::vec2 zero_uv = glm::vec2(0, 0);

		vboSceneObjects.addData(&zero, sizeof(glm::vec3));
		//vboSceneObjects.addData(&zero_uv, sizeof(glm::vec2));
	}
	vboSceneObjects.uploadDataToGPU(GL_STREAM_DRAW);

	// vertex positions start at index 0
	// the distance between two consecutive vertices is
	// sizeof whole vertex data
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), 0);

	const char *cTextureNames[] = { "biohazard_512x512.jpg", "A_Smiley.jpg", "3dcarrush.jpg",
		"3D_Scene_011.jpg", "outdoor_free_3d_scene_by_djeric.jpg", "grid_512k.jpg",
		"grid-xxl.jpg", "grid-green.jpg", "tuscany.jpg" };
	vector<string> sTextureNames(cTextureNames, &cTextureNames[sizeof(cTextureNames) / sizeof(cTextureNames[0])]);
	loadAllTextures(sTextureNames);
	glEnable(GL_TEXTURE_2D);

	programID = LoadShaders(vertexShaderTexSource, fragmentShaderTexSource);
	programColorID = LoadShaders(vertexShaderSource, fragmentShaderSource);

	glPointSize(2.0f);
	glDisable(GL_POINT_SMOOTH);
	glUseProgram(programID);

	// init depth cam and compute
	// point cloud data
	// frame by frame
	rdp = new RawDepthPipeline();

	// initialize mesh array
	// not correct for triangles..
	pgr.verticesUnshaded.resize((int)rdp->depthCamWidth*rdp->depthCamWidth / 3);
}

void PointCloudPCLScene::renderScene(GLFWwindow *window)
{
	cout << "Looping.." << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr frameCloud = rdp->renderFramePCL();
	if (frameCloud != NULL)
		pgr.reconstructFromCloud(frameCloud);
	else
		cout << "FRAME CLOUD INVALID!!" << endl;
	// convert pvertexmem to glm
	// update vertex data	
	//rdp->pVertexMem->
	// to use this I need to supply initial values
	// for the vertices and uvs
	//

	//glBindVertexArray(uiVAOSceneObjects);
	vboSceneObjects.bindVBO();

	// map the buffer
	/*	glm::vec3 *mapped =
	reinterpret_cast<glm::vec3*>(
	glMapBufferRange(GL_ARRAY_BUFFER, 0,
	sizeof(glm::vec3)*rdp->pVertexMem->positions.size(),
	GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT
	)
	);
	*/
	// explicitly invalidate the buffer
	//glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3)*rdp->pVertexMem->positions.size(), 0, GL_DYNAMIC_DRAW);

	glm::vec3 *mapped = reinterpret_cast<glm::vec3*>(vboSceneObjects.mapSubBufferToMemory(GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_RANGE_BIT | GL_MAP_FLUSH_EXPLICIT_BIT,
		sizeof(glm::vec3) * 42, sizeof(glm::vec3)*pgr.verticesUnshaded.size()));

	// copy data into the mapped memory
	std::vector<glm::vec3> data = pgr.verticesUnshaded;

	std::copy(data.begin(), data.end(), mapped);
	//memcpy(mapped, data.data(), data.size());

	// unmap the buffer
	//glUnmapBuffer(GL_ARRAY_BUFFER);
	//glFlushMappedBufferRange(GL_ARRAY_BUFFER, sizeof(glm::vec3) * 42, sizeof(glm::vec3)*rdp->pVertexMem->positions.size());
	glFlushMappedBufferRange(GL_ARRAY_BUFFER, sizeof(glm::vec3) * 42, sizeof(glm::vec3)*pgr.verticesUnshaded.size());
	vboSceneObjects.unmapBuffer();

	// clear first
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//vboSceneObjects.addData(&rdp->pVertexMem->positions, rdp->depthCamHeight * rdp->depthCamWidth * sizeof(glm::vec3));
	//vboSceneObjects.updateGPUData(42*sizeof(glm::vec3));

	//glEnableVertexAttribArray(0);
	//glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, rdp->depthCamHeight * rdp->depthCamWidth * sizeof(glm::vec3), &(rdp->pVertexMem->positions)[0]);
	//vboSceneObjects.bindVBO();

	glBindVertexArray(uiVAOSceneObjects);



	//glBufferSubData(GL_ARRAY_BUFFER, 42, rdp->depthCamHeight * rdp->depthCamWidth * sizeof(glm::vec3), &(rdp->pVertexMem->positions));
	//glBufferSubData(GL_ARRAY_BUFFER, 42 + rdp->depthCamWidth * rdp->depthCamHeight, sizeof(glm::vec2), &(rdp->pVertexMem->uvs));

	computeMatricesFromInputs(window);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	int iModelViewLoc = glGetUniformLocation(programID, "MV");
	int iProjectionLoc = glGetUniformLocation(programID, "P");
	int iColorLocX = glGetUniformLocation(programID, "inColor");
	glUniformMatrix4fv(iProjectionLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));

	//glm::mat4 mModelView = glm::lookAt(glm::vec3(0, 12, 27), glm::vec3(0, 0, 0), glm::vec3(0.0f, 1.0f, 0.0f));
	glm::mat4 mModelView = getViewMatrix();
	glm::mat4 mCurrent;



	// texture binding
	// set GL_ACTIVE_TEXTURE0
	// pass sampler to fragment shader
	int iSamplerLoc = glGetUniformLocation(programID, "gSampler");
	glUniform1i(iSamplerLoc, 0);

	// ground
	tTextures[1].bindTexture();
	glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mModelView));
	glUniform4fv(iColorLocX, 1, glm::value_ptr(glm::vec4(0.0f, 1.0f, 1.0f, 1.0f)));
	glDrawArrays(GL_TRIANGLES, 36, 6);

	tTextures[0].bindTexture(0);

	// rendering
	// cube
	mCurrent = glm::translate(mModelView, glm::vec3(-8.0f, 0.0f, 0.0f));
	mCurrent = glm::scale(mCurrent, glm::vec3(10.0f, 10.0f, 10.0f));
	//mCurrent = glm::rotate(mCurrent, fRotationAngle, glm::vec3(1.0f, 0.0f, 0.0f));
	glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));
	glUniform4fv(iColorLocX, 1, glm::value_ptr(glm::vec4(1.0f, 0.0f, 0.0f, 1.0f)));
	glDrawArrays(GL_TRIANGLES, 0, 36);

	glUseProgram(programColorID);

	int iColorLoc = glGetUniformLocation(programColorID, "inColor");
	int iModelViewColorLoc = glGetUniformLocation(programColorID, "MV");
	int iProjectionColorLoc = glGetUniformLocation(programColorID, "P");
	glUniform4fv(iColorLoc, 1, glm::value_ptr(glm::vec4(0.0f, 1.0f, 0.0f, 1.0f)));
	glUniformMatrix4fv(iProjectionColorLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));

	// render unshaded mesh
	mCurrent = glm::translate(mModelView, glm::vec3(0.0f, 0.0f, 0.0f));
	mCurrent = glm::rotate(mCurrent, 180.0f, glm::vec3(1.0f, 0.0f, 0.0f));
	glDrawArrays(GL_TRIANGLES, 42, 42 + pgr.verticesUnshaded.size());
	/*
	// render point cloud
	// how many points?
	mCurrent = glm::translate(mModelView, glm::vec3(0.0f, 0.0f, 0.0f));
	mCurrent = glm::rotate(mCurrent, 180.0f, glm::vec3(1.0f, 0.0f, 0.0f));
	glUniformMatrix4fv(iModelViewColorLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));
	glDrawArrays(GL_POINTS, 42, 42 + rdp->depthCamWidth * rdp->depthCamHeight);
	*/
	glUseProgram(programID);


	// render skybox
	skybox.renderSkybox();

	// user interaction
	// end user interaction

	fRotationAngle += pipeline->sof(30.0f);

	// make sure the float does not increase
	// drastically with time
	if (fRotationAngle >= 360)
		fRotationAngle -= 360;

	// Swap front and back buffers
	glfwSwapBuffers(window);
	// Poll for and process events
	glfwPollEvents();

	pipeline->updateTimer();

}

void PointCloudPCLScene::releaseScene()
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

