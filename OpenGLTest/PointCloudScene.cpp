#pragma warning(disable:4996) 
#include "PointCloudScene.h"

PointCloudScene::PointCloudScene()
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
	swap = false;
}


PointCloudScene::~PointCloudScene()
{
}

void PointCloudScene::initScene(GLFWwindow *window)
{
	glClearColor(0.0f, 0.0f, 0.4f, 1.0f);

	// set up pipeline
	pipeline = new Pipeline();
	pipeline->resetTimer();
	pipeline->setProjection3D(45.0f, 4.0f / 3.0f, 0.1f, 1000.0f);

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
	
/*	for (int i = 0; i < 320 * 240; i++)
	{
		unsigned int zero = 0;

		ibo.addData(&zero, sizeof(unsigned int));
	}*/

	ibo.addData(&(rdp->indices[0]), rdp->indices.size() * sizeof(unsigned int));
	ibo.uploadDataToGPU(GL_STATIC_DRAW);
	//ibo.uploadDataToGPU(GL_STREAM_DRAW);

	// create and bind VBO
	vboSceneObjects.createVBO();
	vboSceneObjects.bindVBO();

	/*for (int i = 0; i < 320 * 240; i++)
	{
		glm::vec3 zero    = glm::vec3(0, 0, 0);
		vboSceneObjects.addData(&zero, sizeof(glm::vec3));
	}

	vboSceneObjects.uploadDataToGPU(GL_STREAM_DRAW);*/
	glBufferData(GL_ARRAY_BUFFER, rdp->nPoints * sizeof(PXCPoint3DF32), NULL, GL_STREAM_DRAW);
	// vertex positions start at index 0
	// the distance between two consecutive vertices is
	// sizeof whole vertex data
	glEnableVertexAttribArray(0);
	//glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), 0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(PXCPoint3DF32), 0);
	//glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3) + sizeof(glm::vec2), 0);

	/*vboSceneObjects2.createVBO();
	vboSceneObjects2.bindVBO();
	glBufferData(GL_ARRAY_BUFFER, rdp->nPoints * sizeof(PXCPoint3DF32), NULL, GL_STREAM_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(PXCPoint3DF32), 0);*/

	// texture coordinates start right after the position (stride = size glm3)
	// distance is again whole vertex data
	//glEnableVertexAttribArray(1);
	//glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec3) + sizeof(glm::vec2), (void*)sizeof(glm::vec3));
	const char *cTextureNames[] = { "biohazard_512x512.jpg", "A_Smiley.jpg", "3dcarrush.jpg",
		"3D_Scene_011.jpg", "outdoor_free_3d_scene_by_djeric.jpg", "grid_512k.jpg",
		"grid-xxl.jpg", "grid-green.jpg", "tuscany.jpg" };
	vector<string> sTextureNames(cTextureNames, &cTextureNames[sizeof(cTextureNames) / sizeof(cTextureNames[0])]);
	loadAllTextures(sTextureNames);
	glEnable(GL_TEXTURE_2D);


	programID		= LoadShaders(vertexShaderTexSource, fragmentShaderTexSource);
	programColorID  = LoadShaders(vertexShaderSource, fragmentShaderSource);
	
	glPointSize(2.0f);
	//glDisable(GL_POINT_SMOOTH);
	glUseProgram(programID);

	glm::vec3 projectorOrigin = glm::vec3(0.0f, 0.0f, 3.0f);
	glm::vec3 projectorTarget = glm::vec3(0.0f, 0.0f, 0.0f);
	projectorV = glm::lookAt(projectorOrigin, // projector origin
		projectorTarget,	 // project on object at origin 
		glm::vec3(0.0f, 1.0f, 0.0f)   // Y axis is up
		);


	std::cout << "Init end error..: " << gluErrorString(glGetError()) << "\n";
	std::cout << "Init end error.. " << gluErrorString(glGetError()) << "\n";
}

void PointCloudScene::renderScene(GLFWwindow *window)
{
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	rdp->renderFrame(false, false);
	// convert pvertexmem to glm
	// update vertex data	
	//rdp->pVertexMem->
	// to use this I need to supply initial values
	// for the vertices and uvs
	//

	//glBindVertexArray(uiVAOSceneObjects);
	//glBindVertexArray(uiVAOSceneObjects);
	//vboSceneObjects.bindVBO();

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
	/*
	PXCPoint3DF32 *mapped = reinterpret_cast<PXCPoint3DF32*>(vboSceneObjects.mapSubBufferToMemory(GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_RANGE_BIT | GL_MAP_FLUSH_EXPLICIT_BIT, 
		0, sizeof(PXCPoint3DF32)*rdp->nPoints));

	// copy data into the mapped memory
	//std::vector<PXCPoint3DF32> data = &rdp->pos3d[0];

	//std::copy(&rdp->pos3d[0], &rdp->pos3d[rdp->nPoints], &mapped[0]);
	//std::copy(data.begin(), data.end(), mapped);
	
	//std::copy(std::begin(rdp->pos3d), std::end(rdp->pos3d), mapped);
	std::copy(&rdp->pos3d[0], &rdp->pos3d[0] + (rdp->nPoints-1) / sizeof(PXCPoint3DF32), &mapped[0]);
	//memcpy(mapped, data.data(), data.size());
//	mapped = rdp->pos3d;
	// unmap the buffer
	//glUnmapBuffer(GL_ARRAY_BUFFER);
	glFlushMappedBufferRange(GL_ARRAY_BUFFER, 0, sizeof(PXCPoint3DF32)*rdp->nPoints);
	vboSceneObjects.unmapBuffer();
	*/
	// clear first
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//vboSceneObjects.addData(NULL, rdp->depthCamHeight * rdp->depthCamWidth * sizeof(glm::vec3));

	/*
	//vboSceneObjects.updateGPUData(42 * sizeof(glm::vec3));
	vboSceneObjects.addData(&(rdp->pVertexMem->positions[0]), rdp->depthCamHeight * rdp->depthCamWidth * sizeof(glm::vec3));
	//vboSceneObjects.updateGPUData(42 * sizeof(glm::vec3));
	vboSceneObjects.updateGPUData(0);

	ibo.addData(&(rdp->pVertexMem->indices[0]), rdp->depthCamHeight * rdp->depthCamWidth * sizeof(unsigned int));
	ibo.updateGPUData(0);
	*/
	//vboSceneObjects.uploadDataToGPU(GL_STREAM_DRAW);
	//glBufferData(GL_ARRAY_BUFFER, rdp->nPoints * sizeof(PXCPoint3DF32), NULL, GL_STREAM_DRAW);
	//glBufferData(GL_ARRAY_BUFFER, rdp->nPoints * sizeof(PXCPoint3DF32), NULL, GL_STREAM_DRAW);
	/*if (swap)
	{
		swap = !swap;
		vboSceneObjects.bindVBO();
		glBufferData(GL_ARRAY_BUFFER, rdp->nPoints * sizeof(PXCPoint3DF32), NULL, GL_STREAM_DRAW);
		vboSceneObjects2.bindVBO();
		//vboSceneObjects.addData(&(rdp->pos3d[0]), rdp->nPoints * sizeof(PXCPoint3DF32));
		vboSceneObjects2.addData(rdp->worldPos.data(), rdp->nPoints * sizeof(PXCPoint3DF32));
		//vboSceneObjects.addData(&(rdp->worldPos[0]), rdp->nPoints * sizeof(PXCPoint3DF32));
		vboSceneObjects2.uploadDataToGPU(GL_STREAM_DRAW);
	}
	else
	{
		swap = !swap;
		vboSceneObjects2.bindVBO();
		glBufferData(GL_ARRAY_BUFFER, rdp->nPoints * sizeof(PXCPoint3DF32), NULL, GL_STREAM_DRAW);
		vboSceneObjects.bindVBO();
		//vboSceneObjects.addData(&(rdp->pos3d[0]), rdp->nPoints * sizeof(PXCPoint3DF32));
		vboSceneObjects.addData(rdp->worldPos.data(), rdp->nPoints * sizeof(PXCPoint3DF32));
		//vboSceneObjects.addData(&(rdp->worldPos[0]), rdp->nPoints * sizeof(PXCPoint3DF32));
		vboSceneObjects.uploadDataToGPU(GL_STREAM_DRAW);
	}*/
	//glBindVertexArray(uiVAOSceneObjects);
	//glBufferData(GL_ARRAY_BUFFER, 0, NULL, GL_STREAM_DRAW);
	//glInvalidateBufferData(vboSceneObjects.getBufferID());
	
	//vboSceneObjects.addData(&(rdp->pos3d[0]), rdp->nPoints * sizeof(PXCPoint3DF32));
	vboSceneObjects.addData(rdp->worldPos.data(), rdp->nPoints * sizeof(PXCPoint3DF32));
	//vboSceneObjects.addData(&(rdp->worldPos[0]), rdp->nPoints * sizeof(PXCPoint3DF32));
	vboSceneObjects.uploadDataToGPU(GL_STREAM_DRAW);

	//glBufferData(GL_ELEMENT_ARRAY_BUFFER, 0, NULL, GL_STREAM_DRAW);
	//ibo.addData(&(rdp->indices[0]), rdp->indices.size() * sizeof(unsigned int));
	//ibo.uploadDataToGPU(GL_STATIC_DRAW);
	//ibo.uploadDataToGPU(GL_STREAM_DRAW);
	
	//glBufferData(GL_ELEMENT_ARRAY_BUFFER, rdp->indices.size() * sizeof(unsigned int), NULL, GL_STREAM_DRAW);
	//ibo.addData(&(rdp->indices[0]), rdp->indices.size() * sizeof(unsigned int));
	//vboSceneObjects.addData(&(rdp->worldPos[0]), rdp->nPoints * sizeof(PXCPoint3DF32));
	//ibo.uploadDataToGPU(GL_STREAM_DRAW);

	//glBufferData(GL_ARRAY_BUFFER, rdp->nPoints * sizeof(PXCPoint3DF32), NULL, GL_STREAM_DRAW);
	
	//vboSceneObjects.addData(&(rdp->pVertexMem->positions[0]), rdp->nPoints * sizeof(glm::vec3));
	//vboSceneObjects.updateGPUData(0);

	

	computeMatricesFromInputs(window);
	timer += getDeltaTime();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	int iModelViewLoc = glGetUniformLocation(programID, "MV");
	int iProjectionLoc = glGetUniformLocation(programID, "P");
	int iColorLocX = glGetUniformLocation(programID, "inColor");
	glUniformMatrix4fv(iProjectionLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));

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

	int iColorLoc = glGetUniformLocation(programColorID, "inColor");
	int iModelViewColorLoc = glGetUniformLocation(programColorID, "MV");
	int iProjectionColorLoc = glGetUniformLocation(programColorID, "P");
	glUniform4fv(iColorLoc, 1, glm::value_ptr(glm::vec4(0.0f, 1.0f, 0.0f, 1.0f)));
	glUniformMatrix4fv(iProjectionColorLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));

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

	glUniformMatrix4fv(iModelViewColorLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));
	//glDrawArrays(GL_POINTS, 0, rdp->nPoints);

	glDrawElements(
		GL_TRIANGLE_STRIP,      // mode
		rdp->indices.size(),    // count
		GL_UNSIGNED_INT,   // type
		(void*)0           // element array buffer offset
		);
		
	glUseProgram(programID);

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

void PointCloudScene::releaseScene()
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
