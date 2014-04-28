#include "heightMapScene.h"

GLuint uiVBOHeightmapData; // Here are stored heightmap data (vertices)
GLuint uiVBOIndices; // And here indices for rendering heightmap

GLuint uiVAOHeightmap; // One VAO for heightmap

HeightMapScene::HeightMapScene()
{
	vertexShaderSource = "./assets/shaders/shader_hm.vert";
	fragmentShaderSource = "./assets/shaders/shader_hm.frag";
	fRotationAngle = 0.0f;
	PIover180 = 3.1415f / 180.0f;
}

HeightMapScene::~HeightMapScene()
{
}

void HeightMapScene::initScene(GLFWwindow *window)
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

	// setup heightmap
	glGenVertexArrays(1, &uiVAOHeightmap);
	glGenBuffers(1, &uiVBOHeightmapData);
	glGenBuffers(1, &uiVBOIndices);

	glBindVertexArray(uiVAOHeightmap);
	glBindBuffer(GL_ARRAY_BUFFER, uiVBOHeightmapData);

	glm::vec3 vHeightMapData[HM_SIZE_X*HM_SIZE_Y];

	float fHeights[HM_SIZE_X*HM_SIZE_Y] =
	{
		4.0f, 2.0f, 3.0f, 1.0f,
		3.0f, 5.0f, 8.0f, 2.0f,
		7.0f, 10.0f, 12.0f, 6.0f,
		4.0f, 6.0f, 8.0f, 3.0f
	};

	float fSizeX = 40.0f, fSizeZ = 40.0f;

	for (int i = 0; i < HM_SIZE_X*HM_SIZE_Y; i++)
	{
		float column = float(i % HM_SIZE_X),
			row = float(i / HM_SIZE_X);
		vHeightMapData[i] = glm::vec3(
			-fSizeX / 2 + fSizeX*column / float(HM_SIZE_X - 1),
			fHeights[i],
			-fSizeZ / 2 + fSizeZ*row / float(HM_SIZE_Y - 1)
			);
	}

	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3)*HM_SIZE_X*HM_SIZE_Y, vHeightMapData, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glGenBuffers(1, &uiVBOIndices);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, uiVBOIndices);
	int iIndices[] =
	{
		0, 4, 1, 5, 2, 6, 3, 7, 16, // First row, then restart
		4, 8, 5, 9, 6, 10, 7, 11, 16, // Second row, then restart
		8, 12, 9, 13, 10, 14, 11, 15, // Third row, no restart
	};
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(iIndices), iIndices, GL_STATIC_DRAW);
	glEnable(GL_PRIMITIVE_RESTART);
	glPrimitiveRestartIndex(HM_SIZE_X*HM_SIZE_Y);

	programID = LoadShaders(vertexShaderSource, fragmentShaderSource);
	glUseProgram(programID);

	// pipeline
	pipeline = new Pipeline();
	pipeline->resetTimer();
	pipeline->setProjection3D(45.0f, 4.0f / 3.0f, 0.001f, 1000.0f);
}

void HeightMapScene::renderScene(GLFWwindow *window)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glBindVertexArray(uiVAOHeightmap);

	int iModelViewLoc  = glGetUniformLocation(programID, "MV");
	int iProjectionLoc = glGetUniformLocation(programID, "P");
	glUniformMatrix4fv(iProjectionLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));

	glm::mat4 mModelView = glm::lookAt(glm::vec3(0, 60, 30), glm::vec3(0, 0, 0), glm::vec3(0.0f, 1.0f, 0.0f));

	glm::mat4 mCurrent = glm::rotate(mModelView, fRotationAngle, glm::vec3(0.0f, 1.0f, 0.0f));
	glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));
	glBindVertexArray(uiVAOHeightmap);
	glDrawElements(GL_TRIANGLE_STRIP, HM_SIZE_X*(HM_SIZE_Y - 1) * 2 + HM_SIZE_Y - 2, GL_UNSIGNED_INT, 0);

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

void HeightMapScene::releaseScene()
{
	glDeleteProgram(programID);
	
	// delete shaders??

	glDeleteBuffers(1, &uiVBOHeightmapData);
	glDeleteBuffers(1, &uiVBOIndices);
	glDeleteVertexArrays(1, &uiVAOHeightmap);
}