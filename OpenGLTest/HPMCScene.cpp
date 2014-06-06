#include "HPMCScene.h"


HPMCScene::HPMCScene()
{
	shaded_vertex_shader = "./assets/shaders/hpmc_th_shaded.vert";
	shaded_fragment_shader = "./assets/shaders/hpmc_th_shaded.frag";

	vertexShaderSourceTex = "./assets/shaders/shader_tex.vert";
	fragmentShaderSourceTex = "./assets/shaders/shader_tex.frag";

	fGlobalAngle = 0;
	fSunAngle = 45.0f;
	bWireFrame = false;
	timer = 0;

	volume_size_x = 256;
	volume_size_y = 256;
	volume_size_z = 256;

	dataset.resize(volume_size_x * volume_size_y * volume_size_z);
	ifstream datafile("./assets/volume/bonsai.raw", std::ios::in | std::ios::binary);
	if (datafile.good()) {
		datafile.read(reinterpret_cast<char*>(&dataset[0]),
			dataset.size());
	}
	else {
		cerr << "Error opening \"" << "./volume/bonsai.raw" << "\" for reading." << endl;
		getchar();
		exit(EXIT_FAILURE);
	}
}


HPMCScene::~HPMCScene()
{
}

void HPMCScene::initScene(GLFWwindow *window)
{
	glClearColor(0.0f, 0.0f, 0.4f, 1.0f);
	std::cout << "Init begin error: " << gluErrorString(glGetError()) << "\n";
	std::cout << "Init begin error: " << gluErrorString(glGetError()) << "\n";

	// --- upload volume ------------------------------------------------------

	GLint alignment;
	glGetIntegerv(GL_UNPACK_ALIGNMENT, &alignment);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glGenTextures(1, &volume_tex);
	glBindTexture(GL_TEXTURE_3D, volume_tex);
	glTexImage3D(GL_TEXTURE_3D, 0, GL_ALPHA,
		volume_size_x, volume_size_y, volume_size_z, 0,
		GL_ALPHA, GL_UNSIGNED_BYTE, &dataset[0]);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glBindTexture(GL_TEXTURE_3D, 0);
	glPixelStorei(GL_UNPACK_ALIGNMENT, alignment);

	// --- create HistoPyramid -------------------------------------------------
	hpmc_c = HPMCcreateConstants();
	hpmc_h = HPMCcreateHistoPyramid(hpmc_c);

	HPMCsetLatticeSize(hpmc_h,
		volume_size_x,
		volume_size_y,
		volume_size_z);

	HPMCsetGridSize(hpmc_h,
		volume_size_x - 1,
		volume_size_y - 1,
		volume_size_z - 1);

	float max_size = max(volume_size_x, max(volume_size_y, volume_size_z));
	HPMCsetGridExtent(hpmc_h,
		volume_size_x / max_size,
		volume_size_y / max_size,
		volume_size_z / max_size);

	HPMCsetFieldTexture3D(hpmc_h,
		volume_tex,
		GL_FALSE);

	// load shaders
	programID = LoadShaders(shaded_vertex_shader, shaded_fragment_shader);
	programIDTex = LoadShaders(vertexShaderSourceTex, fragmentShaderSourceTex);

	glUseProgram(programID);

	hpmc_th_shaded = HPMCcreateTraversalHandle(hpmc_h);
	HPMCsetTraversalHandleProgram(hpmc_th_shaded,
		programID,
		0, 1, 2);

	skybox.loadSkybox("./assets/textures/skybox/jajsundown1/", "jajsundown1_ft.jpg", "jajsundown1_bk.jpg", "jajsundown1_lf.jpg", "jajsundown1_rt.jpg", "jajsundown1_up.jpg", "jajsundown1_dn.jpg");


	// 1. Gen and bind VAO
	// A VAO can store the binding information for a single IBO, and at least 16 VBOs.
	glGenVertexArrays(1, &uiVAO); // create a VAO
	glBindVertexArray(uiVAO);

	// 2. Gen and bind IBO and BufferData
	/*ibo.createVBO();
	ibo.bindVBO(GL_ELEMENT_ARRAY_BUFFER);
	ibo.addData(&pclGR.indices, sizeof(pclGR.indices[0])*pclGR.indices.size());
	ibo.uploadDataToGPU(GL_STATIC_DRAW);*/

	//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	// 3. Gen and bind VBO and BufferData
	vboSceneObjects.createVBO();
	vboSceneObjects.bindVBO(GL_ARRAY_BUFFER);
	//glBufferData(vboSceneObjects.getBufferID(), sizeof(pclGR.test_verts[0])*pclGR.test_verts.size(), &pclGR.test_verts[0], GL_STATIC_DRAW);
	//glBufferData(vboSceneObjects.getBufferID(), sizeof(glm::vec3) * 36, &vCubeVerticesL[0], GL_STATIC_DRAW);
	//vboSceneObjects.addData(&pclGR.vertices, sizeof(pclGR.vertices[0])*pclGR.vertices.size());
	
	//vboSceneObjects.addData(&pclGR.test_verts, sizeof(glm::vec3)*pclGR.test_verts.size());
	
	//vboSceneObjects.addData(&vCubeVerticesL, sizeof(glm::vec3)*36);
	//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	// add cube to VBO
	/*for (int i = 0; i < 36; i++)
	{
	vboSceneObjects.addData(&vCubeVerticesL[i], sizeof(glm::vec3));
	//vboSceneObjects.addData(&vCubeTexCoordsL[i % 6], sizeof(glm::vec2));
	//vboSceneObjects.addData(&vCubeNormalsL[i / 6], sizeof(glm::vec3));
	}*/
	vboSceneObjects.uploadDataToGPU(GL_STATIC_DRAW);
	// 4. EnableVertexAttribArrays that I need and set VertexAttribPointers
	glEnableVertexAttribArray(0);
	//glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(pclGR.vertices[0]), 0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), 0);
	//glEnableVertexAttribArray(1);
	//glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3), (void*)sizeof(glm::vec3));

	// 5. Unbind VAO (binding to 0)
	glBindVertexArray(0);

	// pipeline
	pipeline = new Pipeline();
	pipeline->resetTimer();
	pipeline->setProjection3D(45.0f, 4.0f / 3.0f, 0.1f, 1000.0f);

	glPolygonOffset(1.0, 1.0);
	///////////////
	//Final error testing//
	std::cout << "Init end error: " << gluErrorString(glGetError()) << "\n";
	std::cout << "Init end error: " << gluErrorString(glGetError()) << "\n";
}

void HPMCScene::renderScene(GLFWwindow *window)
{
	// --- clear screen and set up view ----------------------------------------
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	//States setting
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_STENCIL_TEST);
	computeMatricesFromInputs(window);
	timer += getDeltaTime();

	// set light properties
	//float fSine = sinf(fSunAngle*3.1415f / 180.0f);
	glm::vec3 vSunPos(cosf(fSunAngle*3.1415f / 180.0f) * 70, sinf(fSunAngle*3.1415f / 180.0f) * 70, 0.0f);

	// set uniforms
	int iProjectionLoc = glGetUniformLocation(programID, "P");
	glUniformMatrix4fv(iProjectionLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));

	glm::mat4 mView = getViewMatrix();
	glm::mat4 mCurrent;
	int iModelViewLoc = glGetUniformLocation(programID, "MV");
	glm::mat4 mModelMatrix = glm::translate(glm::mat4(1.0f), getEyeVector());
	glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mView*mModelMatrix));


	int iNormalMatrixLoc = glGetUniformLocation(programID, "N");

	int iColorLoc = glGetUniformLocation(programID, "inColor");
	glUniform4fv(iColorLoc, 1, glm::value_ptr(glm::vec4(0.0f, 1.0f, 0.0f, 1.0f)));

	int iLightColorLoc = glGetUniformLocation(programID, "sunLight.vColor");
	int iLightAmbientLoc = glGetUniformLocation(programID, "sunLight.fAmbientIntensity");
	int iLightDirectionLoc = glGetUniformLocation(programID, "sunLight.vDirection");

	glUniform3fv(iLightColorLoc, 1, glm::value_ptr(glm::vec3(1.0f, 1.0f, 1.0f)));
	float ambientIntensity = 0.25f;
	glUniform1fv(iLightAmbientLoc, 1, &ambientIntensity);
	glUniform3fv(iLightDirectionLoc, 1, glm::value_ptr(-glm::normalize(vSunPos)));
	

	glUseProgram(programIDTex);
	glDisable(GL_DEPTH_TEST);
	// uniforms for normal tex program
	int iModelViewLocTex = glGetUniformLocation(programIDTex, "MV");
	int iProjTexLoc = glGetUniformLocation(programIDTex, "P");
	glUniformMatrix4fv(iProjTexLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));
	glUniformMatrix4fv(iModelViewLocTex, 1, GL_FALSE, glm::value_ptr(mView));
	int iSamplerLoc = glGetUniformLocation(programIDTex, "gSampler");
	glUniform1i(iSamplerLoc, 0);
	skybox.renderSkybox();
	glEnable(GL_DEPTH_TEST);
	glUseProgram(programID);



	float max_size = max(volume_size_x, max(volume_size_y, volume_size_z));
	mModelMatrix = glm::translate(glm::mat4(1.0), glm::vec3(-0.5f*volume_size_x / max_size,
															-0.5f*volume_size_y / max_size,
															-0.5f*volume_size_z / max_size));
	// --- build HistoPyramid --------------------------------------------------
	//float iso = 0.5 + 0.48*cosf(pipeline->sof(45.0f));
	float iso = 0.5f;
	HPMCbuildHistopyramid(hpmc_h, iso);

	// render mesh
	HPMCextractVertices(hpmc_th_shaded);
	glBindVertexArray(uiVAO);
	/*if (bWireFrame)
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	else
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		*/

	
	//mModelMatrix = glm::scale(mModelMatrix, glm::vec3(0.01f, 0.01f, 0.01f));
	mCurrent = mView*mModelMatrix;
	glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));
	glUniformMatrix4fv(iNormalMatrixLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mCurrent))));

	//glDrawArrays(GL_TRIANGLES, 0, 36);

	
	glBindVertexArray(0);
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

	if (timer >= 0.25f)
	{
		if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_M) != GLFW_RELEASE)
		{
			bWireFrame = !bWireFrame;
			timer = 0;
		}
	}

	// Swap front and back buffers
	glfwSwapBuffers(window);
	// Poll for and process events
	glfwPollEvents();

	pipeline->updateTimer();
}

void HPMCScene::releaseScene()
{
	skybox.releaseSkybox();

	glDeleteProgram(programIDTex);
	glDeleteProgram(programID);

	glDeleteVertexArrays(1, &uiVAO);
	vboSceneObjects.releaseVBO();
	//ibo.releaseVBO();
}