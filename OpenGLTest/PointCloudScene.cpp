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

/*	// add cube to vbo
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
	}*/

	/*for (int i = 0; i < 320 * 240; i++)
	{
		glm::vec3 zero    = glm::vec3(0, 0, 0);
		//glm::vec2 zero_uv = glm::vec2(0, 0);

		vboSceneObjects.addData(&zero, sizeof(glm::vec3));
		//vboSceneObjects.addData(&zero_uv, sizeof(glm::vec2));
	}*/

	//vboSceneObjects.uploadDataToGPU(GL_STREAM_DRAW);

	// vertex positions start at index 0
	// the distance between two consecutive vertices is
	// sizeof whole vertex data
	glEnableVertexAttribArray(0);
	//glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), 0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(PXCPoint3DF32), 0);
	//glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3) + sizeof(glm::vec2), 0);

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

	rdp->renderFrame();
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

	/*glm::vec3 *mapped = reinterpret_cast<glm::vec3*>(vboSceneObjects.mapSubBufferToMemory(GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_RANGE_BIT | GL_MAP_FLUSH_EXPLICIT_BIT, 
		sizeof(glm::vec3) * 42, sizeof(glm::vec3)*rdp->pVertexMem->positions.size()));

	// copy data into the mapped memory
	std::vector<glm::vec3> data = rdp->pVertexMem->positions;

	std::copy(data.begin(), data.end(), mapped);
	//memcpy(mapped, data.data(), data.size());

	// unmap the buffer
	//glUnmapBuffer(GL_ARRAY_BUFFER);
	glFlushMappedBufferRange(GL_ARRAY_BUFFER, sizeof(glm::vec3) * 42, sizeof(glm::vec3)*rdp->pVertexMem->positions.size());
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
	vboSceneObjects.addData(&(rdp->pos3d[0]), rdp->nPoints * sizeof(PXCPoint3DF32));
	//vboSceneObjects.updateGPUData(0);
	vboSceneObjects.uploadDataToGPU(GL_STREAM_DRAW);
	glBindVertexArray(uiVAOSceneObjects);

	computeMatricesFromInputs(window);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	int iModelViewLoc = glGetUniformLocation(programID, "MV");
	int iProjectionLoc = glGetUniformLocation(programID, "P");
	int iColorLocX = glGetUniformLocation(programID, "inColor");
	glUniformMatrix4fv(iProjectionLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));

	//glm::mat4 mModelView = glm::lookAt(glm::vec3(0, 12, 27), glm::vec3(0, 0, 0), glm::vec3(0.0f, 1.0f, 0.0f));
	//glm::mat4 mModelView = getViewMatrix();
	glm::mat4 mModelView = projectorV;
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
	//glDrawArrays(GL_TRIANGLES, 36, 6);

	tTextures[0].bindTexture(0);

	// rendering
	// cube
	mCurrent = glm::translate(mModelView, glm::vec3(-8.0f, 0.0f, 0.0f));
	//mCurrent = glm::rotate(mCurrent, fRotationAngle, glm::vec3(1.0f, 0.0f, 0.0f));
	mCurrent = glm::scale(mCurrent, glm::vec3(10.0f, 10.0f, 10.0f));
	
	glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));
	glUniform4fv(iColorLocX, 1, glm::value_ptr(glm::vec4(1.0f, 0.0f, 0.0f, 1.0f)));
	//glDrawArrays(GL_TRIANGLES, 0, 36);

	glUseProgram(programColorID);

	int iColorLoc = glGetUniformLocation(programColorID, "inColor");
	int iModelViewColorLoc = glGetUniformLocation(programColorID, "MV");
	int iProjectionColorLoc = glGetUniformLocation(programColorID, "P");
	glUniform4fv(iColorLoc, 1, glm::value_ptr(glm::vec4(0.0f, 1.0f, 0.0f, 1.0f)));
	glUniformMatrix4fv(iProjectionColorLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));

	// render point cloud
	// how many points?
	mCurrent = glm::translate(mModelView, glm::vec3(0.0f, 0.0f, 0.0f));
	mCurrent = glm::rotate(mCurrent, 180.0f, glm::vec3(1.0f, 0.0f, 0.0f));
	//mCurrent = glm::scale(mCurrent, glm::vec3(10.0f, 10.0f, 10.0f));

	glUniformMatrix4fv(iModelViewColorLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));
	//glDrawArrays(GL_POINTS, 42, 42 + rdp->depthCamWidth * rdp->depthCamHeight);
	//glDrawArrays(GL_POINTS, 0, rdp->depthCamWidth * rdp->depthCamHeight);
	//glDrawArrays(GL_TRIANGLES, 0, rdp->nPoints);

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
