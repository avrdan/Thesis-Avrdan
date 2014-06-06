#include "static_geometry.h"

glm::vec3 vCubeVertices[] =
{
	// Front face
	glm::vec3(-0.5f, 0.5f, -0.5f), glm::vec3(0.5f, 0.5f, -0.5f), glm::vec3(0.5f, -0.5f, -0.5f), glm::vec3(0.5f, -0.5f, -0.5f), glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(-0.5f, 0.5f, -0.5f),
	// Back face
	glm::vec3(0.5f, 0.5f, 0.5f), glm::vec3(-0.5f, 0.5f, 0.5f), glm::vec3(-0.5f, -0.5f, 0.5f), glm::vec3(-0.5f, -0.5f, 0.5f), glm::vec3(0.5f, -0.5f, 0.5f), glm::vec3(0.5f, 0.5f, 0.5f),
	// Left face
	glm::vec3(-0.5f, 0.5f, 0.5f), glm::vec3(-0.5f, 0.5f, -0.5f), glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(-0.5f, -0.5f, 0.5f), glm::vec3(-0.5f, 0.5f, 0.5f),
	// Right face
	glm::vec3(0.5f, 0.5f, -0.5f), glm::vec3(0.5f, 0.5f, 0.5f), glm::vec3(0.5f, -0.5f, 0.5f), glm::vec3(0.5f, -0.5f, 0.5f), glm::vec3(0.5f, -0.5f, -0.5f), glm::vec3(0.5f, 0.5f, -0.5f),
	// Top face
	glm::vec3(-0.5f, 0.5f, 0.5f), glm::vec3(0.5f, 0.5f, 0.5f), glm::vec3(0.5f, 0.5f, -0.5f), glm::vec3(0.5f, 0.5f, -0.5f), glm::vec3(-0.5f, 0.5f, -0.5f), glm::vec3(-0.5f, 0.5f, 0.5f),
	// Bottom face
	glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(0.5f, -0.5f, -0.5f), glm::vec3(0.5f, -0.5f, 0.5f), glm::vec3(0.5f, -0.5f, 0.5f), glm::vec3(-0.5f, -0.5f, 0.5f), glm::vec3(-0.5f, -0.5f, -0.5f),
};
glm::vec2 vCubeTexCoords[] = { glm::vec2(0.0f, 1.0f), glm::vec2(1.0f, 1.0f), glm::vec2(1.0f, 0.0f), glm::vec2(1.0f, 0.0f), glm::vec2(0.0f, 0.0f), glm::vec2(0.0f, 1.0f) };

glm::vec3 vPyramidVertices[] =
{
	// Front face
	glm::vec3(0.0f, 0.5f, 0.0f), glm::vec3(-0.5f, -0.5f, 0.5f), glm::vec3(0.5f, -0.5f, 0.5f),
	// Back face
	glm::vec3(0.0f, 0.5f, 0.0f), glm::vec3(0.5f, -0.5f, -0.5f), glm::vec3(-0.5f, -0.5f, -0.5f),
	// Left face
	glm::vec3(0.0f, 0.5f, 0.0f), glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(-0.5f, -0.5f, 0.5f),
	// Right face
	glm::vec3(0.0f, 0.5f, 0.0f), glm::vec3(0.5f, -0.5f, 0.5f), glm::vec3(0.5f, -0.5f, -0.5f)
};

glm::vec2 vPyramidTexCoords[] = { glm::vec2(0.5f, 1.0f), glm::vec2(0.0f, 0.0f), glm::vec2(1.0f, 0.0f) };

glm::vec3 vGround[] =
{
	glm::vec3(-50, -10, 50), glm::vec3(50, -10, 50), glm::vec3(50, -10, -50), glm::vec3(50, -10, -50), glm::vec3(-50, -10, -50), glm::vec3(-50, -10, 50)
};

// lighting scene
glm::vec3 vCubeVerticesL[36] =
{
	// Front face
	glm::vec3(-0.5f, 0.5f, -0.5f), glm::vec3(0.5f, 0.5f, -0.5f), glm::vec3(0.5f, -0.5f, -0.5f), glm::vec3(0.5f, -0.5f, -0.5f), glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(-0.5f, 0.5f, -0.5f),
	// Back face
	glm::vec3(0.5f, 0.5f, 0.5f), glm::vec3(-0.5f, 0.5f, 0.5f), glm::vec3(-0.5f, -0.5f, 0.5f), glm::vec3(-0.5f, -0.5f, 0.5f), glm::vec3(0.5f, -0.5f, 0.5f), glm::vec3(0.5f, 0.5f, 0.5f),
	// Left face
	glm::vec3(-0.5f, 0.5f, 0.5f), glm::vec3(-0.5f, 0.5f, -0.5f), glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(-0.5f, -0.5f, 0.5f), glm::vec3(-0.5f, 0.5f, 0.5f),
	// Right face
	glm::vec3(0.5f, 0.5f, -0.5f), glm::vec3(0.5f, 0.5f, 0.5f), glm::vec3(0.5f, -0.5f, 0.5f), glm::vec3(0.5f, -0.5f, 0.5f), glm::vec3(0.5f, -0.5f, -0.5f), glm::vec3(0.5f, 0.5f, -0.5f),
	// Top face
	glm::vec3(-0.5f, 0.5f, 0.5f), glm::vec3(0.5f, 0.5f, 0.5f), glm::vec3(0.5f, 0.5f, -0.5f), glm::vec3(0.5f, 0.5f, -0.5f), glm::vec3(-0.5f, 0.5f, -0.5f), glm::vec3(-0.5f, 0.5f, 0.5f),
	// Bottom face
	glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(0.5f, -0.5f, -0.5f), glm::vec3(0.5f, -0.5f, 0.5f), glm::vec3(0.5f, -0.5f, 0.5f), glm::vec3(-0.5f, -0.5f, 0.5f), glm::vec3(-0.5f, -0.5f, -0.5f),
};
glm::vec2 vCubeTexCoordsL[6] = { glm::vec2(0.0f, 1.0f), glm::vec2(1.0f, 1.0f), glm::vec2(1.0f, 0.0f), glm::vec2(1.0f, 0.0f), glm::vec2(0.0f, 0.0f), glm::vec2(0.0f, 1.0f) };

glm::vec3 vCubeNormalsL[6] =
{
	glm::vec3(0.0f, 0.0f, 1.0f),
	glm::vec3(0.0f, 0.0f, -1.0f),
	glm::vec3(-1.0f, 0.0f, 0.0f),
	glm::vec3(1.0f, 0.0f, 0.0f),
	glm::vec3(0.0f, 1.0f, 0.0f),
	glm::vec3(0.0f, -1.0f, 0.0f)
};

glm::vec3 vGroundL[6] =
{
	glm::vec3(-100, -10, 100), glm::vec3(50, -10, 100), glm::vec3(100, -10, -100), glm::vec3(100, -10, -100), glm::vec3(-100, -10, -100), glm::vec3(-100, -10, 100)
};

/*-----------------------------------------------

Name:	GenerateTorus

Params:	vboDest - VBO where to store torus
fRadius - outer radius
fTubeRadius - inner radius
iSubDivAround - subdivisions around torus
iSubDivTube - subdivisions of tube

Result: Generates centered torus with specified
parameters and stores it in VBO.

/*---------------------------------------------*/

int GenerateTorus(VertexBufferObject &vboDest, float fRadius, float fTubeRadius, int iSubDivAround, int iSubDivTube)
{
	float fAddAngleAround = 360.0f / (float)iSubDivAround;
	float fAddAngleTube = 360.0f / (float)iSubDivTube;

	float fCurAngleAround = 0.0f;
	int iStepsAround = 1;
	const float PI = float(atan(1.0)*4.0);

	int iFacesAdded = 0;

	while (iStepsAround <= iSubDivAround)
	{
		float fSineAround = sin(fCurAngleAround / 180.0f*PI);
		float fCosineAround = cos(fCurAngleAround / 180.0f*PI);
		glm::vec3 vDir1(fSineAround, fCosineAround, 0.0f);
		float fNextAngleAround = fCurAngleAround + fAddAngleAround;
		float fNextSineAround = sin(fNextAngleAround / 180.0f*PI);
		float fNextCosineAround = cos(fNextAngleAround / 180.0f*PI);
		glm::vec3 vDir2(fNextSineAround, fNextCosineAround, 0.0f);
		float fCurAngleTube = 0.0f;
		int iStepsTube = 1;
		while (iStepsTube <= iSubDivTube)
		{
			float fSineTube = sin(fCurAngleTube / 180.0f*PI);
			float fCosineTube = cos(fCurAngleTube / 180.0f*PI);
			float fNextAngleTube = fCurAngleTube + fAddAngleTube;
			float fNextSineTube = sin(fNextAngleTube / 180.0f*PI);
			float fNextCosineTube = cos(fNextAngleTube / 180.0f*PI);
			glm::vec3 vMid1 = vDir1*(fRadius - fTubeRadius / 2), vMid2 = vDir2*(fRadius - fTubeRadius / 2);
			glm::vec3 vQuadPoints[] =
			{
				vMid1 + glm::vec3(0.0f, 0.0f, -fNextSineTube*fTubeRadius) + vDir1*fNextCosineTube*fTubeRadius,
				vMid1 + glm::vec3(0.0f, 0.0f, -fSineTube*fTubeRadius) + vDir1*fCosineTube*fTubeRadius,
				vMid2 + glm::vec3(0.0f, 0.0f, -fSineTube*fTubeRadius) + vDir2*fCosineTube*fTubeRadius,
				vMid2 + glm::vec3(0.0f, 0.0f, -fNextSineTube*fTubeRadius) + vDir2*fNextCosineTube*fTubeRadius
			};

			glm::vec3 vNormals[] =
			{
				glm::vec3(0.0f, 0.0f, -fNextSineTube) + vDir1*fNextCosineTube,
				glm::vec3(0.0f, 0.0f, -fSineTube) + vDir1*fCosineTube,
				glm::vec3(0.0f, 0.0f, -fSineTube) + vDir2*fCosineTube,
				glm::vec3(0.0f, 0.0f, -fNextSineTube) + vDir2*fNextCosineTube
			};

			glm::vec2 vTexCoords[] =
			{
				glm::vec2(fCurAngleAround / 360.0f, fNextAngleTube / 360.0f),
				glm::vec2(fCurAngleAround / 360.0f, fCurAngleTube / 360.0f),
				glm::vec2(fNextAngleAround / 360.0f, fCurAngleTube / 360.0f),
				glm::vec2(fNextAngleAround / 360.0f, fNextAngleTube / 360.0f)
			};

			int iIndices[] = { 0, 1, 2, 2, 3, 0 };

			for (int i = 0; i < 6; i++)
			{
				int index = iIndices[i];
				vboDest.addData(&vQuadPoints[index], sizeof(glm::vec3));
				vboDest.addData(&vTexCoords[index], sizeof(glm::vec2));
				vboDest.addData(&vNormals[index], sizeof(glm::vec3));
			}
			iFacesAdded += 2; // Keep count of added faces
			fCurAngleTube += fAddAngleTube;
			iStepsTube++;
		}
		fCurAngleAround += fAddAngleAround;
		iStepsAround++;
	}
	return iFacesAdded;
}

/*-----------------------------------------------

Name:	GenerateCylinder

Params:	vboDest - VBO where to store torus
fRadius - outer radius
fHeight - height of cylinder
iSubDivAround - subdivisions around cylinder

Result: Generates centered opened cylinder and stores
it in VBO.

/*---------------------------------------------*/

int GenerateCylinder(VertexBufferObject &vboDest, float fRadius, float fHeight, int iSubDivAround, float fMapU, float fMapV)
{
	float fAddAngleAround = 360.0f / (float)(iSubDivAround - 1);

	float fCurAngleAround = 0.0f;
	int iStepsAround = 1;
	const float PI = float(atan(1.0)*4.0);

	int iFacesAdded = 0;

	while (iStepsAround <= iSubDivAround)
	{
		float fSineAround = sin(fCurAngleAround / 180.0f*PI);
		float fCosineAround = cos(fCurAngleAround / 180.0f*PI);
		glm::vec3 vDir1(fCosineAround, 0.0f, fSineAround);
		float fNextAngleAround = fCurAngleAround + fAddAngleAround;
		float fNextSineAround = sin(fNextAngleAround / 180.0f*PI);
		float fNextCosineAround = cos(fNextAngleAround / 180.0f*PI);
		glm::vec3 vDir2(fNextCosineAround, 0.0f, fNextSineAround);

		glm::vec3 vQuadPoints[] =
		{
			glm::vec3(0.0f, fHeight, 0.0f) + vDir1*fRadius,
			vDir1*fRadius,
			vDir2*fRadius,
			glm::vec3(0.0f, fHeight, 0.0f) + vDir2*fRadius
		};


		glm::vec2 vTexCoords[] =
		{
			glm::vec2(fMapU*fCurAngleAround / 360.0f, fMapV),
			glm::vec2(fMapU*fCurAngleAround / 360.0f, 0.0f),
			glm::vec2(fMapU*fNextAngleAround / 360.0f, 0.0f),
			glm::vec2(fMapU*fNextAngleAround / 360.0f, fMapV)
		};

		glm::vec3 vNormals[] =
		{
			vDir1,
			vDir1,
			vDir2,
			vDir2
		};

		int iIndices[] = { 0, 1, 2, 2, 3, 0 };

		for (int i = 0; i < 6; i++)
		{
			int index = iIndices[i];
			vboDest.addData(&vQuadPoints[index], sizeof(glm::vec3));
			vboDest.addData(&vTexCoords[index], sizeof(glm::vec2));
			vboDest.addData(&vNormals[index], sizeof(glm::vec3));
		}
		iFacesAdded += 2; // Keep count of added faces

		fCurAngleAround += fAddAngleAround;
		iStepsAround++;
	}
	return iFacesAdded;
}


/*-----------------------------------------------

Name:	GenerateSphere

Params:	vboDest - VBO where to store sphere
fRadius - sphere radius

Result: Generates centered sphere.

/*---------------------------------------------*/

int GenerateSphere(VertexBufferObject &vboDest, float fRadius, int iSubDivY, int iSubDivZ)
{
	float fAddAngleY = 360.0f / float(iSubDivY), fAddAngleZ = 180.0f / float(iSubDivZ);
	float fCurAngleY = 0.0f;
	float fTexU = 1.0f / float(iSubDivY), fTexV = 1.0f / float(iSubDivZ);
	int iStepsY = 1;

	const float PI = float(atan(1.0)*4.0);

	int iFacesAdded = 0;

	while (iStepsY <= iSubDivY)
	{
		float fNextAngleY = fCurAngleY + fAddAngleY;
		float fSineY = sin(fCurAngleY / 180.0f*PI), fCosY = cos(fCurAngleY / 180.0f*PI);
		float fNextSineY = sin(fNextAngleY / 180.0f*PI), fNextCosY = cos(fNextAngleY / 180.0f*PI);
		glm::vec3 vDirY(fCosY, 0.0f, -fSineY), vNextDirY(fNextCosY, 0.0f, -fNextSineY);
		float fCurAngleZ = 0.0f;
		int iStepsZ = 1;
		while (iStepsZ <= iSubDivZ)
		{
			float fNextAngleZ = fCurAngleZ + fAddAngleZ;

			float fSineZ = sin(fCurAngleZ / 180.0f*PI), fCosZ = cos(fCurAngleZ / 180.0f*PI);
			float fNextSineZ = sin(fNextAngleZ / 180.0f*PI), fNextCosZ = cos(fNextAngleZ / 180.0f*PI);

			glm::vec3 vQuadPoints[] =
			{
				glm::vec3(vDirY.x*fSineZ*fRadius, fCosZ*fRadius, vDirY.z*fSineZ*fRadius),
				glm::vec3(vDirY.x*fNextSineZ*fRadius, fNextCosZ*fRadius, vDirY.z*fNextSineZ*fRadius),
				glm::vec3(vNextDirY.x*fNextSineZ*fRadius, fNextCosZ*fRadius, vNextDirY.z*fNextSineZ*fRadius),
				glm::vec3(vNextDirY.x*fSineZ*fRadius, fCosZ*fRadius, vNextDirY.z*fSineZ*fRadius),
			};
			/*
			TODO: I'm not sure why this doesn't work at all, would like to kno
			SECRET QUEST FOR THIS TUTORIAL: Find out what's wrong and let me know
			REWARD: 10000 EXP
			glm::vec2 vTexCoords[] =
			{
			glm::vec2(fTexU*float(iStepsY-1), 1.0f-fTexV*float(iStepsZ-1)),
			glm::vec2(fTexU*float(iStepsY), 1.0f-fTexV*float(iStepsZ)),
			glm::vec2(fTexU*float(iStepsY), 1.0f-fTexV*float(iStepsZ)),
			glm::vec2(fTexU*float(iStepsY-1), 1.0f-fTexV*float(iStepsZ-1))
			};*/

			glm::vec3 vNormals[] =
			{
				glm::normalize(vQuadPoints[0]),
				glm::normalize(vQuadPoints[1]),
				glm::normalize(vQuadPoints[2]),
				glm::normalize(vQuadPoints[3])
			};

			glm::vec2 vTexCoords[] =
			{
				glm::vec2(asin(vNormals[0].x) / PI + 0.5f, asin(vNormals[0].y) / PI + 0.5f),
				glm::vec2(asin(vNormals[1].x) / PI + 0.5f, asin(vNormals[1].y) / PI + 0.5f),
				glm::vec2(asin(vNormals[2].x) / PI + 0.5f, asin(vNormals[2].y) / PI + 0.5f),
				glm::vec2(asin(vNormals[3].x) / PI + 0.5f, asin(vNormals[3].y) / PI + 0.5f),
			};

			int iIndices[] = { 0, 1, 2, 2, 3, 0 };

			for (int i = 0; i < 6; i++)
			{
				int index = iIndices[i];
				vboDest.addData(&vQuadPoints[index], sizeof(glm::vec3));
				vboDest.addData(&vTexCoords[index], sizeof(glm::vec2));
				vboDest.addData(&vNormals[index], sizeof(glm::vec3));
			}
			iFacesAdded += 2; // Keep count of added faces

			iStepsZ++;
			fCurAngleZ += fAddAngleZ;
		}
		iStepsY++;
		fCurAngleY += fAddAngleY;
	}

	return iFacesAdded;
}

glm::vec2 vCubeTexCoordsAS[6] = { glm::vec2(0.0f, 1.0f), glm::vec2(1.0f, 1.0f), glm::vec2(1.0f, 0.0f), glm::vec2(1.0f, 0.0f), glm::vec2(0.0f, 0.0f), glm::vec2(0.0f, 1.0f) };

glm::vec3 vGroundAS[6] =
{
	glm::vec3(-1000, 0, -1000), glm::vec3(-1000, 0, 1000), glm::vec3(1000, 0, 1000), glm::vec3(1000, 0, 1000), glm::vec3(1000, 0, -1000), glm::vec3(-1000, 0, -1000)
};

int iSphereFaces;

/*-----------------------------------------------

Name:	CreateStaticSceneObjects

Params:	uiVAO - Vertex Array Object with VBO assigned
vboDest - VBO where to store objects

Result: Adds all static objects to scene.

/*---------------------------------------------*/


void CreateStaticSceneObjects(GLuint* uiVAO, VertexBufferObject& vboDest)
{
	vboDest.createVBO();
	glGenVertexArrays(1, uiVAO); // Create one VAO
	glBindVertexArray(*uiVAO);

	vboDest.bindVBO();

	// Add ground to VBO
	for (int i = 0; i < 6; i++)
	{
		vboDest.addData(&vGround[i], sizeof(glm::vec3));
		glm::vec2 vCoord = vCubeTexCoordsAS[i] * 50.0f;
		vboDest.addData(&vCoord, sizeof(glm::vec2));
		glm::vec3 vGroundNormal(0.0f, 1.0f, 0.0f);
		vboDest.addData(&vGroundNormal, sizeof(glm::vec3));
	}

	vboDest.uploadDataToGPU(GL_STATIC_DRAW);

	// Vertex positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3) + sizeof(glm::vec2), 0);
	// Texture coordinates
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3) + sizeof(glm::vec2), (void*)sizeof(glm::vec3));
	// Normal vectors
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3) + sizeof(glm::vec2), (void*)(sizeof(glm::vec3) + sizeof(glm::vec2)));
}

void CreateFrustumSimple(GLuint* uiVAO, VertexBufferObject& vboDest, glm::vec3 frustum_coords[], int index)
{
	vboDest.createVBO();
	//glGenVertexArrays(1, uiVAO); // Create one VAO
	//glBindVertexArray(*uiVAO);

	vboDest.bindVBO();

	// add frustum to VBO
	for (int i = index; i < 36 + index; i++)
	{
		vboDest.addData(&frustum_coords[i], sizeof(glm::vec3));
	}

	vboDest.uploadDataToGPU(GL_STATIC_DRAW);

	// Vertex positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), 0);
}

void CreateStaticSceneObjects2(GLuint* uiVAO, VertexBufferObject& vboDest, glm::vec3 frustum_coords[])
{
	vboDest.createVBO();
	glGenVertexArrays(1, uiVAO); // Create one VAO
	glBindVertexArray(*uiVAO);

	vboDest.bindVBO();

	// Add ground to VBO
	for (int i = 0; i < 6; i++)
	{
		vboDest.addData(&vGround[i], sizeof(glm::vec3));
		glm::vec2 vCoord = vCubeTexCoordsAS[i] * 50.0f;
		vboDest.addData(&vCoord, sizeof(glm::vec2));
		glm::vec3 vGroundNormal(0.0f, 1.0f, 0.0f);
		vboDest.addData(&vGroundNormal, sizeof(glm::vec3));
	}


	// add cube to VBO
	for (int i = 0; i < 36; i++)
	{
		vboDest.addData(&vCubeVerticesL[i], sizeof(glm::vec3));
		vboDest.addData(&vCubeTexCoordsL[i % 6], sizeof(glm::vec2));
		vboDest.addData(&vCubeNormalsL[i / 6], sizeof(glm::vec3));
	}

	// add frustum to VBO
	for (int i = 0; i < 36; i++)
	{
		vboDest.addData(&frustum_coords[i], sizeof(glm::vec3));
		// add dummy data for UV and normals as they will not be used
		vboDest.addData(&glm::vec2(0, 0), sizeof(glm::vec2));
		vboDest.addData(&glm::vec3(0, 0, 0), sizeof(glm::vec3));
	}

	vboDest.uploadDataToGPU(GL_STATIC_DRAW);

	// Vertex positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3) + sizeof(glm::vec2), 0);
	// Texture coordinates
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3) + sizeof(glm::vec2), (void*)sizeof(glm::vec3));
	// Normal vectors
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3) + sizeof(glm::vec2), (void*)(sizeof(glm::vec3) + sizeof(glm::vec2)));
}

void CreateFrustumOutlines(GLuint* uiVAO, VertexBufferObject& vboDest, glm::vec3 frustum_coords[])
{
	vboDest.createVBO();
	glGenVertexArrays(1, uiVAO); // Create one VAO
	glBindVertexArray(*uiVAO);

	vboDest.bindVBO();

	// add frustum to VBO
	for (int i = 0; i < 36; i++)
	{
		vboDest.addData(&frustum_coords[i], sizeof(glm::vec3));
		// add dummy data for UV and normals as they will not be used
		vboDest.addData(&glm::vec2(0, 0), sizeof(glm::vec2));
		vboDest.addData(&glm::vec3(0, 0, 0), sizeof(glm::vec3));
	}

	vboDest.uploadDataToGPU(GL_STATIC_DRAW);

	// Vertex positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3) + sizeof(glm::vec2), 0);
	// Texture coordinates
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3) + sizeof(glm::vec2), (void*)sizeof(glm::vec3));
	// Normal vectors
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3) + sizeof(glm::vec2), (void*)(sizeof(glm::vec3) + sizeof(glm::vec2)));
}

// x,y vertex positions
glm::vec3 ss_quad_pos[] = {
	glm::vec3(-1.0f, -1.0f, 0.0f),
	glm::vec3(1.0f, -1.0f, 0.0f),
	glm::vec3(1.0f, 1.0f, 0.0f),
	glm::vec3(1.0f, 1.0f, 0.0f),
	glm::vec3(-1.0f, 1.0f, 0.0f),
	glm::vec3(-1.0f, -1.0f, 0.0f)
};
// per-vertex texture coordinates
glm::vec2 ss_quad_st[] = {
	glm::vec2(0.0f, 0.0f),
	glm::vec2(1.0f, 0.0f),
	glm::vec2(1.0f, 1.0f),
	glm::vec2(1.0f, 1.0f),
	glm::vec2(0.0f, 1.0f),
	glm::vec2(0.0f, 0.0f)
};

// The quad's FBO. Used only for visualizing the shadowmap.
glm::vec3 g_quad_vertex_buffer_data[] = {
	glm::vec3(-1.0f, -1.0f, 0.0f),
	glm::vec3(1.0f, -1.0f, 0.0f),
	glm::vec3(-1.0f, 1.0f, 0.0f),
	glm::vec3(-1.0f, 1.0f, 0.0f),
	glm::vec3(1.0f, -1.0f, 0.0f),
	glm::vec3(1.0f, 1.0f, 0.0f)
};

void CreateStaticSceneObjects3(GLuint* uiVAO, VertexBufferObject& vboDest)
{
	vboDest.createVBO();
	glGenVertexArrays(1, uiVAO); // Create one VAO
	glBindVertexArray(*uiVAO);

	vboDest.bindVBO();

	// Add ground to VBO
	for (int i = 0; i < 6; i++)
	{
		vboDest.addData(&ss_quad_pos[i], sizeof(glm::vec3));

		vboDest.addData(&ss_quad_st[i], sizeof(glm::vec2));
		//glm::vec3 vGroundNormal(0.0f, 1.0f, 0.0f);
		//vboDest.addData(&vGroundNormal, sizeof(glm::vec3));
	}

	vboDest.uploadDataToGPU(GL_STATIC_DRAW);

	// Vertex positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, /*2 * */sizeof(glm::vec3) + sizeof(glm::vec2), 0);
	// Texture coordinates
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, /*2 * */ sizeof(glm::vec3) + sizeof(glm::vec2), (void*)sizeof(glm::vec3));
	// Normal vectors
	//glEnableVertexAttribArray(2);
	//glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3) + sizeof(glm::vec2), (void*)(sizeof(glm::vec3) + sizeof(glm::vec2)));
}

glm::vec3 vBuildingNormals[4] =
{
	glm::vec3(0.0f, 0.0f, 1.0f),
	glm::vec3(-1.0f, 0.0f, 0.0f),
	glm::vec3(1.0f, 0.0f, 0.0f),
	glm::vec3(0.0f, -1.0f, 0.0f)
};

glm::vec3 vBuilding[24] =
{
	glm::vec3(-75.0f, 75.0f, -75.0f), glm::vec3(75.0f, 75.0f, -75.0f), glm::vec3(75.0f, 0.0f, -75.0f), glm::vec3(75.0f, 0.0f, -75.0f), glm::vec3(-75.0f, 0.0f, -75.0f), glm::vec3(-75.0f, 75.0f, -75.0f),
	glm::vec3(-75.0f, 75.0f, -75.0f), glm::vec3(-75.0f, 75.0f, 75.0f), glm::vec3(-75.0f, 0.0f, 75.0f), glm::vec3(-75.0f, 0.0f, 75.0f), glm::vec3(-75.0f, 0.0f, -75.0f), glm::vec3(-75.0f, 75.0f, -75.0f),
	glm::vec3(75.0f, 75.0f, 75.0f), glm::vec3(75.0f, 75.0f, -75.0f), glm::vec3(75.0f, 0.0f, -75.0f), glm::vec3(75.0f, 0.0f, -75.0f), glm::vec3(75.0f, 0.0f, 75.0f), glm::vec3(75.0f, 75.0f, 75.0f),
	glm::vec3(-75.0f, 75.0f, -75.0f), glm::vec3(75.0f, 75.0f, -75.0f), glm::vec3(75.0f, 75.0f, 75.0f), glm::vec3(75.0f, 75.0f, 75.0f), glm::vec3(-75.0f, 75.0f, 75.0f), glm::vec3(-75.0f, 75.0f, -75.0f)
};