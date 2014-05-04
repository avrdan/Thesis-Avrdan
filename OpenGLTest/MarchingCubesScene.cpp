#include "MarchingCubesScene.h"


MarchingCubesScene::MarchingCubesScene()
{
	mc = MarchingCubes();
	
	mcVertexShaderSource   = "./assets/shaders/marchingCubes.vert";
	mcGeometryShaderSource = "./assets/shaders/marchingCubes.geom";
	mcFragmentShaderSource = "./assets/shaders/marchingCubes.frag";
	
	/*mcVertexShaderSource = "./assets/shaders/geometry.vert";
	mcGeometryShaderSource = "./assets/shaders/geometry.geom";
	mcFragmentShaderSource = "./assets/shaders/geometry.frag";*/

	fGlobalAngle = 0;
	fSunAngle = 45.0f;
	bWireFrame = false;
	timer = 0;

	isoLevel = 0.5f;
	curData = 0;
	cubeSize = glm::vec3(32, 32, 32);
	cubeStep = glm::vec3(2.0f, 2.0f, 2.0f) / cubeSize;
	dataSize = glm::ivec3(128, 128, 128);

	texVertexShaderSource    =  "./assets/shaders/shader_tex.vert";
	texFragmentShaderSource  =  "./assets/shaders/shader_tex.frag";
}


MarchingCubesScene::~MarchingCubesScene()
{
}

void MarchingCubesScene::initScene(GLFWwindow *window)
{
	glClearColor(0.0f, 0.0f, 0.4f, 1.0f);
	std::cout << "Init begin error: " << gluErrorString(glGetError()) << "\n";
	std::cout << "Init begin error: " << gluErrorString(glGetError()) << "\n";
	geometryProgramID = LoadShaders(mcVertexShaderSource, mcGeometryShaderSource,
		mcFragmentShaderSource);
	programID = LoadShaders(texVertexShaderSource, texFragmentShaderSource);

	glUseProgram(geometryProgramID);
	//glUseProgram(programID);
	
	skybox.loadSkybox("./assets/textures/skybox/jajsundown1/", "jajsundown1_ft.jpg", "jajsundown1_bk.jpg", "jajsundown1_lf.jpg", "jajsundown1_rt.jpg", "jajsundown1_up.jpg", "jajsundown1_dn.jpg");




	//Edge Table texture//
	//This texture store the 256 different configurations of a marching cube.
	//This is a table accessed with a bitfield of the 8 cube edges states
	//(edge cut by isosurface or totally in or out).
	//(cf. MarchingCubes.cpp)
	glGenTextures(1, &edgeTableTex);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, edgeTableTex);
	
	//Integer textures must use nearest filtering mode
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

	// IMPORTANT: texture must be COMPLETE (mipmaps must be specified..
	// or the following parameters can be used if there are none)
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);

	//We create an integer texture with new GL_EXT_texture_integer formats
	// SOME ALPHA STUFF WAS REMOVED FROM OPENGL
	//glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA16I_EXT, 256, 1, 0,
	//	GL_ALPHA_INTEGER_EXT, GL_INT, &mc.edgeTable);
	// USE THE "RED" CHANNEL INSTEAD
	glTexImage2D(GL_TEXTURE_2D, 0, GL_R16I, 256, 1, 0,
		GL_RED_INTEGER, GL_INT, &(mc.edgeTable[0]));
	glGenerateMipmap(GL_TEXTURE_2D);
	//Triangle Table texture//
	//This texture store the vertex index list for
	//generating the triangles of each configurations.
	//(cf. MarchingCubes.cpp)
	glGenTextures(1, &triTableTex);
	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, triTableTex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

	// IMPORTANT: texture must be COMPLETE (mipmaps must be specified..
	// or the following parameters can be used if there are none)
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_R16I, 16, 256, 0,
		GL_RED_INTEGER, GL_INT, &(mc.triTable[0]));
	glGenerateMipmap(GL_TEXTURE_2D);
	//Datafield//
	//Store the volume data to polygonise
	glGenTextures(3, dataFieldTex);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_3D, dataFieldTex[0]);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

	//Generate a distance field to the center of the cube
	dataField[0] = new float[dataSize.x*dataSize.y*dataSize.z];
	for (int k = 0; k<dataSize.z; k++)
	for (int j = 0; j<dataSize.y; j++)
	for (int i = 0; i<dataSize.x; i++){
		//float d = glm::vec3(i, j, k).distance(glm::vec3(dataSize.x / 2, dataSize.y / 2, dataSize.z / 2)) / (float)(dataSize.length()*0.4);
		float d = glm::distance(glm::vec3(i, j, k), glm::vec3(dataSize.x / 2, dataSize.y / 2, dataSize.z / 2)) / (float)(dataSize.length()*0.4);
		dataField[0][i + j*dataSize.x + k*dataSize.x*dataSize.y] = d;//+(rand()%100-50)/200.0f*d;
	}

	// IMPORTANT: texture must be COMPLETE (mipmaps must be specified..
	// or the following parameters can be used if there are none)
	//glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_BASE_LEVEL, 0);
	//glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAX_LEVEL, 0);
	
	// same alpha problem, use RED channel instead
	glTexImage3D(GL_TEXTURE_3D, 0, GL_R32F, dataSize.x, dataSize.y, dataSize.z, 0, GL_RED, GL_FLOAT, dataField[0]);
	glGenerateMipmap(GL_TEXTURE_3D);
	//glTexImage3D(GL_TEXTURE_3D, 0, GL_ALPHA32F_ARB, dataSize.x, dataSize.y, dataSize.z, 0, GL_ALPHA, GL_FLOAT, dataField[0]);

	//Datafield Perturbated//
	dataField[1] = new float[dataSize.x*dataSize.y*dataSize.z];
	//perturb
	for (int k = 0; k<dataSize.z; k++)
	for (int j = 0; j<dataSize.y; j++)
	for (int i = 0; i<dataSize.x; i++){
		float d = dataField[0][i + j*dataSize.x + k*dataSize.x*dataSize.y];
		dataField[1][i + j*dataSize.x + k*dataSize.x*dataSize.y] = d + (rand() % 100 - 50) / 100.0f*d;
	}

	//Smooth
	for (int l = 0; l<4; l++)
	for (int k = 1; k<dataSize.z - 1; k++)
	for (int j = 1; j<dataSize.y - 1; j++)
	for (int i = 1; i<dataSize.x - 1; i++){
		dataField[1][i + j*dataSize.x + k*dataSize.x*dataSize.y] = (dataField[1][i + 1 + j*dataSize.x + k*dataSize.x*dataSize.y] + dataField[1][i - 1 + j*dataSize.x + k*dataSize.x*dataSize.y] + dataField[1][i + (j + 1)*dataSize.x + k*dataSize.x*dataSize.y] + dataField[1][i + (j - 1)*dataSize.x + k*dataSize.x*dataSize.y] + dataField[1][i + j*dataSize.x + (k + 1)*dataSize.x*dataSize.y] + dataField[1][i + j*dataSize.x + (k - 1)*dataSize.x*dataSize.y]) / 6.0f;
	}

	//Store the volume data to polygonise
	glBindTexture(GL_TEXTURE_3D, dataFieldTex[1]);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

	// IMPORTANT: texture must be COMPLETE (mipmaps must be specified..
	// or the following parameters can be used if there are none)
	//glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_BASE_LEVEL, 0);
	//glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAX_LEVEL, 0);


	glTexImage3D(GL_TEXTURE_3D, 0, GL_R32F, dataSize.x, dataSize.y, dataSize.z, 0, GL_RED, GL_FLOAT, dataField[1]);
	glGenerateMipmap(GL_TEXTURE_3D);

	//Cayley-polynomial//
	dataField[2] = new float[dataSize.x*dataSize.y*dataSize.z];

	for (int k = 0; k<dataSize.z; k++)
	for (int j = 0; j<dataSize.y; j++)
	for (int i = 0; i<dataSize.x; i++){
		float x = 2.0f / dataSize.x*i - 1.0f;
		float y = 2.0f / dataSize.y*j - 1.0f;
		float z = 2.0f / dataSize.z*k - 1.0f;
		dataField[2][i + j*dataSize.x + k*dataSize.x*dataSize.y] = 16.0f*x*y*z + 4.0f*x*x + 4.0f*y*y + 4.0f*z*z - 1.0f;
	}

	glBindTexture(GL_TEXTURE_3D, dataFieldTex[2]);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

	// IMPORTANT: texture must be COMPLETE (mipmaps must be specified..
	// or the following parameters can be used if there are none)
	//glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_BASE_LEVEL, 0);
	//glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAX_LEVEL, 0);


	glTexImage3D(GL_TEXTURE_3D, 0, GL_R32F, dataSize.x, dataSize.y, dataSize.z, 0, GL_RED, GL_FLOAT, dataField[2]);
	glGenerateMipmap(GL_TEXTURE_3D);

	//Set current texture//
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_3D, dataFieldTex[curData]);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, edgeTableTex);
	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, triTableTex);
	
	////Samplers assignment///
	glUniform1i(glGetUniformLocation(geometryProgramID, "dataFieldTex"), 0);
	glUniform1i(glGetUniformLocation(geometryProgramID, "edgeTableTex"), 1);
	glUniform1i(glGetUniformLocation(geometryProgramID, "triTableTex"), 2);

	////Uniforms parameters////
	//Initial isolevel
	glUniform1f(glGetUniformLocation(geometryProgramID, "isoLevel"), isoLevel);

	//Step in data 3D texture for gradient computation (lighting)
	glUniform3f(glGetUniformLocation(geometryProgramID, "dataStep"), 1.0f / dataSize.x, 1.0f / dataSize.y, 1.0f / dataSize.z);

	//Decal for each vertex in a marching cube
	glUniform3fv(glGetUniformLocation(geometryProgramID, "vertDecals[0]"), 1, glm::value_ptr(glm::vec3(0.0f, 0.0f, 0.0f)));
	glUniform3fv(glGetUniformLocation(geometryProgramID, "vertDecals[1]"), 1, glm::value_ptr(glm::vec3(cubeStep.x, 0.0f, 0.0f)));
	glUniform3fv(glGetUniformLocation(geometryProgramID, "vertDecals[2]"), 1, glm::value_ptr(glm::vec3(cubeStep.x, cubeStep.y, 0.0f)));
	glUniform3fv(glGetUniformLocation(geometryProgramID, "vertDecals[3]"), 1, glm::value_ptr(glm::vec3(0.0f, cubeStep.y, 0.0f)));
	glUniform3fv(glGetUniformLocation(geometryProgramID, "vertDecals[4]"), 1, glm::value_ptr(glm::vec3(0.0f, 0.0f, cubeStep.z)));
	glUniform3fv(glGetUniformLocation(geometryProgramID, "vertDecals[5]"), 1, glm::value_ptr(glm::vec3(cubeStep.x, 0.0f, cubeStep.z)));
	glUniform3fv(glGetUniformLocation(geometryProgramID, "vertDecals[6]"), 1, glm::value_ptr(glm::vec3(cubeStep.x, cubeStep.y, cubeStep.z)));
	glUniform3fv(glGetUniformLocation(geometryProgramID, "vertDecals[7]"), 1, glm::value_ptr(glm::vec3(0.0f, cubeStep.y, cubeStep.z)));
	/////

	glPointSize(2.0f);

	//////Grid data construction
	//Linear Walk
	gridData = new float[(int)(cubeSize.x*cubeSize.y*cubeSize.z * 3)];
	//gridData = new float[1];
	int ii = 0;
	for (float k = -1; k<1.0f; k += cubeStep.z)
	for (float j = -1; j<1.0f; j += cubeStep.y)
	for (float i = -1; i<1.0f; i += cubeStep.x){
		gridData[ii] = i;
		gridData[ii + 1] = j;
		gridData[ii + 2] = k;

		ii += 3;
	}

	// add geometry and vertex attrib pointers
	glGenVertexArrays(1, &uiVAO); // create a VAO
	glBindVertexArray(uiVAO);
	vboSceneObjects.createVBO();
	vboSceneObjects.bindVBO();

	vboSceneObjects.addData(&(gridData[0]), sizeof(glm::vec3)*cubeSize.x*cubeSize.y*cubeSize.z);
	vboSceneObjects.uploadDataToGPU(GL_STATIC_DRAW);

	// Vertex positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), 0);

	// pipeline
	pipeline = new Pipeline();
	pipeline->resetTimer();
	pipeline->setProjection3D(45.0f, 4.0f / 3.0f, 0.1f, 1000.0f);

	///////////////
	//Final error testing//
	std::cout << "Init end error: " << gluErrorString(glGetError()) << "\n";
	std::cout << "Init end error: " << gluErrorString(glGetError()) << "\n";
}

void MarchingCubesScene::renderScene(GLFWwindow *window)
{


	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	//States setting
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_STENCIL_TEST);
	//glDisable(GL_ALPHA_TEST);
	computeMatricesFromInputs(window);
	timer += getDeltaTime();

	// set uniforms
	int iProjectionLoc = glGetUniformLocation(geometryProgramID, "matrices.projMatrix");
	glUniformMatrix4fv(iProjectionLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));

	glm::mat4 mView = getViewMatrix();

	int iViewLoc = glGetUniformLocation(geometryProgramID, "matrices.viewMatrix");
	glUniformMatrix4fv(iViewLoc, 1, GL_FALSE, glm::value_ptr(mView));
	int iModelLoc = glGetUniformLocation(geometryProgramID, "matrices.modelMatrix");
	glUniformMatrix4fv(iModelLoc, 1, GL_FALSE, glm::value_ptr(glm::mat4(1.0f)));
	glm::mat4 mModelMatrix = glm::translate(glm::mat4(1.0f), getEyeVector());
	glUniformMatrix4fv(iModelLoc, 1, GL_FALSE, glm::value_ptr(mModelMatrix));
	int iColorLoc = glGetUniformLocation(geometryProgramID, "vColor");
	glUniform4fv(iColorLoc, 1, glm::value_ptr(glm::vec4(0.0f, 1.0f, 0.0f, 1.0f)));
	
	glUseProgram(programID);
	glDisable(GL_DEPTH_TEST);
	// uniforms for normal tex program
	int iModelViewLoc = glGetUniformLocation(programID, "MV");
	int iProjTexLoc = glGetUniformLocation(programID, "P");
	glUniformMatrix4fv(iProjTexLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));
	glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mView));
	int iSamplerLoc = glGetUniformLocation(programID, "gSampler");
	glUniform1i(iSamplerLoc, 0);
	skybox.renderSkybox();
	glUseProgram(geometryProgramID);
	glEnable(GL_DEPTH_TEST);

	// render mesh
	glBindVertexArray(uiVAO);
	if (bWireFrame)
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	else
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);


	mModelMatrix = glm::translate(glm::mat4(1.0), glm::vec3(0, 0, 0));
	mModelMatrix = glm::scale(mModelMatrix, glm::vec3(8.0f, 8.0f, 8.0f));

	glUniformMatrix4fv(iModelLoc, 1, GL_FALSE, glm::value_ptr(mModelMatrix));

	//glEnableClientState(GL_VERTEX_ARRAY);
	//glVertexPointer(3, GL_FLOAT, 0, NULL);
	//glDrawArrays(GL_POINTS, 0, cubeSize.x*cubeSize.y*cubeSize.z);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, cubeSize.x*cubeSize.y*cubeSize.z);
	//glDisableClientState(GL_VERTEX_ARRAY);
	//glDrawArrays(GL_POINTS, 0, (int)(cubeSize.x*cubeSize.y*cubeSize.z));

	// Swap front and back buffers
	glfwSwapBuffers(window);
	// Poll for and process events
	glfwPollEvents();

	pipeline->updateTimer();

	//std::cout << "Loop error: " << gluErrorString(glGetError()) << "\n";
	//std::cout << "Loop error: " << gluErrorString(glGetError()) << "\n";
}

void MarchingCubesScene::releaseScene()
{
	skybox.releaseSkybox();

	glDeleteProgram(geometryProgramID);

	glDeleteVertexArrays(1, &uiVAO);
	vboSceneObjects.releaseVBO();
}
