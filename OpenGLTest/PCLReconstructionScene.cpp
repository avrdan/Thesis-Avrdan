#include "PCLReconstructionScene.h"


PCLReconstructionScene::PCLReconstructionScene()
{
	vertexShaderSource = "./assets/shaders/dirLight_simple.vert";
	fragmentShaderSource = "./assets/shaders/dirLight_simple.frag";

	vertexShaderSourceTex = "./assets/shaders/shader_tex.vert";
	fragmentShaderSourceTex = "./assets/shaders/shader_tex.frag";

	fGlobalAngle = 0;
	fSunAngle = 45.0f;
	bWireFrame = false;
	timer = 0;
}


PCLReconstructionScene::~PCLReconstructionScene()
{
}

void PCLReconstructionScene::initScene(GLFWwindow *window)
{
	glClearColor(0.0f, 0.0f, 0.4f, 1.0f);
	std::cout << "Init begin error: " << gluErrorString(glGetError()) << "\n";
	std::cout << "Init begin error: " << gluErrorString(glGetError()) << "\n";
	programID = LoadShaders(vertexShaderSource, fragmentShaderSource);
	programIDTex = LoadShaders(vertexShaderSourceTex, fragmentShaderSourceTex);

	glUseProgram(programID);

	skybox.loadSkybox("./assets/textures/skybox/jajsundown1/", "jajsundown1_ft.jpg", "jajsundown1_bk.jpg", "jajsundown1_lf.jpg", "jajsundown1_rt.jpg", "jajsundown1_up.jpg", "jajsundown1_dn.jpg");

	// load in data from PCL
	pclGR.reconstruct();

	// 1. Gen and bind VAO
	// A VAO can store the binding information for a single IBO, and at least 16 VBOs.
	glGenVertexArrays(1, &uiVAO); // create a VAO
	glBindVertexArray(uiVAO);

	// 2. Gen and bind IBO and BufferData
	ibo.createVBO();
	ibo.bindVBO(GL_ELEMENT_ARRAY_BUFFER);
	ibo.addData(&(pclGR.indices[0]), sizeof(pclGR.indices[0])*pclGR.indices.size());
	ibo.uploadDataToGPU(GL_STATIC_DRAW);
	
	//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	// 3. Gen and bind VBO and BufferData
	vboSceneObjects.createVBO();
	vboSceneObjects.bindVBO(GL_ARRAY_BUFFER);
	//glBufferData(vboSceneObjects.getBufferID(), sizeof(pclGR.test_verts[0])*pclGR.test_verts.size(), &pclGR.test_verts[0], GL_STATIC_DRAW);
	//glBufferData(vboSceneObjects.getBufferID(), sizeof(glm::vec3) * 36, &vCubeVerticesL[0], GL_STATIC_DRAW);
	//vboSceneObjects.addData(&pclGR.vertices, sizeof(pclGR.vertices[0])*pclGR.vertices.size());

	vboSceneObjects.addData(&(pclGR.vertices[0]), sizeof(pclGR.vertices[0])*pclGR.vertices.size());

	//vboSceneObjects.addData(&(vCubeVerticesL[0]), sizeof(glm::vec3)*36);
	//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	// add cube to VBO
	/*for (int i = 0; i < 36; i++)
	{
		vboSceneObjects.addData(&vCubeVerticesL[i], sizeof(glm::vec3));
		//vboSceneObjects.addData(&vCubeTexCoordsL[i % 6], sizeof(glm::vec2));
		vboSceneObjects.addData(&vCubeNormalsL[i / 6], sizeof(glm::vec3));
	}*/
	vboSceneObjects.uploadDataToGPU(GL_STATIC_DRAW);
	// 4. EnableVertexAttribArrays that I need and set VertexAttribPointers
	glEnableVertexAttribArray(0);
	//glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(pclGR.vertices[0]), 0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3), 0);
	
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3), (void*)sizeof(glm::vec3));

	// 5. Unbind VAO (binding to 0)
	//glBindVertexArray(0);
	
	// pipeline
	pipeline = new Pipeline();
	pipeline->resetTimer();
	pipeline->setProjection3D(45.0f, 4.0f / 3.0f, 0.1f, 1000.0f);

	///////////////
	//Final error testing//
	std::cout << "Init end error: " << gluErrorString(glGetError()) << "\n";
	std::cout << "Init end error: " << gluErrorString(glGetError()) << "\n";
}

void PCLReconstructionScene::renderScene(GLFWwindow *window)
{
	//std::cout << "Init end error: " << gluErrorString(glGetError()) << "\n";
	//std::cout << "Init end error: " << gluErrorString(glGetError()) << "\n";
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

	int iColorLoc = glGetUniformLocation(programID, "vColor");
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
	

	// render mesh
	glBindVertexArray(uiVAO);
	if (bWireFrame)
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	else
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);


	mModelMatrix = glm::translate(glm::mat4(1.0), glm::vec3(0, 0, 0));
	mModelMatrix = glm::scale(mModelMatrix, glm::vec3(10.0f, 10.0f, 10.0f));
	mCurrent = mView*mModelMatrix;
	glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));
	glUniformMatrix4fv(iNormalMatrixLoc, 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(mCurrent))));

	//glDrawArrays(GL_POINTS, 0, cubeSize.x*cubeSize.y*cubeSize.z);
	//glDrawArrays(GL_TRIANGLES, 0, 36);
	//glDrawArrays(GL_TRIANGLES, 0, pclGR.vertices.size());
	//glDrawArrays(GL_TRIANGLES, 0, pclGR.test_verts.size());
	// Draw the triangles !
	glDrawElements(
		GL_TRIANGLES,      // mode
		pclGR.indices.size(),    // count
		GL_UNSIGNED_INT,   // type
		(void*)0           // element array buffer offset
		);
		
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

void PCLReconstructionScene::releaseScene()
{
	skybox.releaseSkybox();

	glDeleteProgram(programIDTex);
	glDeleteProgram(programID);

	glDeleteVertexArrays(1, &uiVAO);
	vboSceneObjects.releaseVBO();
	ibo.releaseVBO();
}
