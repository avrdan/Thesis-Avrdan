#include "textureScene.h"
#include "common/controls.h"

TextureScene::TextureScene()
{
	vertexShaderSource   = "./assets/shaders/shader_tex.vert";
	fragmentShaderSource = "./assets/shaders/shader_tex.frag";
	texSnowSource        = "./assets/textures/snow.jpg";
	texGoldSource        = "./assets/textures/golddiag.jpg";

	hmVertexShaderSource   = "./assets/shaders/heatMap.vert";
	hmFragmentShaderSource = "./assets/shaders/heatMap.frag";

	skyboxPath = "./assets/textures/skybox/jajlands1/";
	skyFront   = "jajlands1_ft.jpg";
	skyDown    = "jajlands1_dn.jpg";
	skyUp      = "jajlands1_up.jpg";
	skyRight   = "jajlands1_rt.jpg";
	skyBack    = "jajlands1_bk.jpg";
	skyLeft    = "jajlands1_lf.jpg";

	fRotationAngle = 0.0f;
	PIover180 = 3.1415f / 180.0f;

	timer = 0;
	bShowHeatMap = false;
}


TextureScene::~TextureScene()
{
}

void TextureScene::initScene(GLFWwindow *window)
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

	vboSceneObjects.createVBO();
	glGenVertexArrays(1, &uiVAO);
	glBindVertexArray(uiVAO);

	vboSceneObjects.bindVBO();

	// add cube to vbo
	for (int i = 0; i < 36; i++)
	{
		// vertices for one face
		vboSceneObjects.addData(&vCubeVertices[i], sizeof(glm::vec3));
		// texture coords for one face
		vboSceneObjects.addData(&vCubeTexCoords[i % 6], sizeof(glm::vec2));
	}

	// add pyramid to vbo
	for (int i = 0; i < 12; i++)
	{
		vboSceneObjects.addData(&vPyramidVertices[i], sizeof(glm::vec3));
		vboSceneObjects.addData(&vPyramidTexCoords[i % 3], sizeof(glm::vec2));
	}

	// add ground to VBO
	for (int i = 0; i < 6; i++)
	{
		vboSceneObjects.addData(&vGround[i], sizeof(glm::vec3));
		// scale cube coords
		// or provide different ground tex coords
		vCubeTexCoords[i] *= 5.0f;
		vboSceneObjects.addData(&vCubeTexCoords[i % 6], sizeof(glm::vec2));
	}

	vboSceneObjects.uploadDataToGPU(GL_STATIC_DRAW);

	// vertex positions start at index 0
	// the distance between two consecutive vertices is
	// sizeof whole vertex data
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3) + sizeof(glm::vec2), 0);

	// texture coordinates start right after the position (stride = size glm3)
	// distance is again whole vertex data
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec3) + sizeof(glm::vec2), (void*)sizeof(glm::vec3));

	programID			= LoadShaders(vertexShaderSource, fragmentShaderSource);
	programID_HeatMap   = LoadShaders(hmVertexShaderSource, hmFragmentShaderSource);
	glUseProgram(programID);

	// load textures
	tGold.loadTexture2D(texGoldSource, true);
	tGold.setFiltering(TEXTURE_FILTER_MAG_BILINEAR, TEXTURE_FILTER_MIN_BILINEAR_MIPMAP);

	tSnow.loadTexture2D(texSnowSource, true);
	tSnow.setFiltering(TEXTURE_FILTER_MAG_BILINEAR, TEXTURE_FILTER_MIN_BILINEAR_MIPMAP);

	glEnable(GL_TEXTURE_2D);

	skybox.loadSkybox(skyboxPath, skyFront, skyBack, skyLeft, skyRight, skyUp, skyDown);

	// pipeline
	pipeline = new Pipeline();
	pipeline->resetTimer();
	pipeline->setProjection3D(45.0f, 4.0f / 3.0f, 0.1f, 1000.0f);
}

void TextureScene::renderScene(GLFWwindow *window)
{
	computeMatricesFromInputs(window);
	timer += getDeltaTime();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	int iModelViewLoc;
	int iProjectionLoc;
	//int iColorLoc;
	if(bShowHeatMap)
	{
		glUseProgram(programID_HeatMap);
		iModelViewLoc  = glGetUniformLocation(programID_HeatMap, "MV");
		iProjectionLoc = glGetUniformLocation(programID_HeatMap, "P");
		//iColorLoc = glGetUniformLocation(programID_HeatMap, "inColor");
		//cout << "Showing heatmap!!" << endl;
	}
	else
	{
		glUseProgram(programID);
		iModelViewLoc = glGetUniformLocation(programID, "MV");
		iProjectionLoc = glGetUniformLocation(programID, "P");

		// texture binding
		// set GL_ACTIVE_TEXTURE0
		// pass sampler to fragment shader
		int iSamplerLoc = glGetUniformLocation(programID, "gSampler");
		glUniform1i(iSamplerLoc, 0);
		tGold.bindTexture(0);
	}


	glUniformMatrix4fv(iProjectionLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));

	//glm::mat4 mModelView = glm::lookAt(glm::vec3(0, 12, 27), glm::vec3(0, 0, 0), glm::vec3(0.0f, 1.0f, 0.0f));
	glm::mat4 mModelView = getViewMatrix();
	glm::mat4 mCurrent;

	glBindVertexArray(uiVAO);



// rendering
	// cube
	mCurrent = glm::translate(mModelView, glm::vec3(-8.0f, 0.0f, 0.0f));
	mCurrent = glm::scale(mCurrent, glm::vec3(10.0f, 10.0f, 10.0f));
	mCurrent = glm::rotate(mCurrent, fRotationAngle, glm::vec3(1.0f, 0.0f, 0.0f));
	glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));
	
	//if (bShowHeatMap)
	//	glUniform4fv(iColorLoc, 1, glm::value_ptr(glm::vec4(0.0f, 1.0f, 0.0f, 1.0f)));
	glDrawArrays(GL_TRIANGLES, 0, 36);
	// pyramid
	mCurrent = glm::translate(mModelView, glm::vec3(8.0f, 0.0f, 0.0f));
	mCurrent = glm::scale(mCurrent, glm::vec3(10.0f, 10.0f, 10.0f));
	mCurrent = glm::rotate(mCurrent, fRotationAngle, glm::vec3(0.0f, 1.0f, 0.0f));
	glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));
	//if (bShowHeatMap)
	//	glUniform4fv(iColorLoc, 1, glm::value_ptr(glm::vec4(0.0f, 1.0f, 0.0f, 1.0f)));
	glDrawArrays(GL_TRIANGLES, 36, 12);
	// ground
	glUseProgram(programID);
	//if (!bShowHeatMap)
		tSnow.bindTexture();
	//if (bShowHeatMap)
	//	glUniform4fv(iColorLoc, 1, glm::value_ptr(glm::vec4(0.0f, 1.0f, 0.0f, 1.0f)));
	glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mModelView));
	glDrawArrays(GL_TRIANGLES, 48, 6);

	
	// render skybox
	skybox.renderSkybox();

	// user interaction
	// end user interaction

	fRotationAngle += pipeline->sof(30.0f);
	// make sure the float does not increase
	// drastically with time
	if (fRotationAngle >= 360)
		fRotationAngle -= 360;

	if (timer >= 0.25f)
	{
		if (glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_H) != GLFW_RELEASE)
		{
			bShowHeatMap = !bShowHeatMap;
			timer = 0;
		}
	}
		
	// Swap front and back buffers
	glfwSwapBuffers(window);
	// Poll for and process events
	glfwPollEvents();

	pipeline->updateTimer();
}

void TextureScene::releaseScene()
{
	skybox.releaseSkybox();

	glDeleteProgram(programID);

	vboSceneObjects.releaseVBO();
	glDeleteVertexArrays(1, &uiVAO);

	tGold.releaseTexture();
	tSnow.releaseTexture();
}