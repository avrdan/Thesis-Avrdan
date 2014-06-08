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
	bShowHeatMap = false;
	
	vertexShaderPTMSource = "./assets/shaders/ptm_color.vert";
	fragmentShaderPTMSource = "./assets/shaders/ptm_color.frag";

	// heat map shader
	hmVertexShaderSource = "./assets/shaders/heatMap.vert";
	hmFragmentShaderSource = "./assets/shaders/heatMap.frag";

	// quad shader
	vertexShaderQuadSource = "./assets/shaders/passthrough.vert";
	fragmentShaderQuadSource = "./assets/shaders/simpleTexture.frag";

	// frustum shader
	frustumVertexShaderSource   = "./assets/shaders/simple.vert";
	frustumFragmentShaderSource = "./assets/shaders/simple.frag";

	// frustum params
	pFovY = 60.0;
	pAR = 16.0f / 9.0f;
	pNear = 0.1f;
	pFar = 2.0f;
	pUp = glm::vec3(0, 1.0f, 0);

	// CALIBRATION

	// INTRINSIC PARAMS
	// projector calibration perspective params
	/*fu = 1.431257f * 1000;
	fv = 1.431257f * 1000;
	u0 = 9.595000f * 100;
	v0 = 8.955700f * 100;
	nearP = 0.1f;
	farP = 1000.0f;*/

	/*fu = 9.645909f * 10;
	fv = 9.645909f * 10;
	u0 = 9.595000f * 100;
	v0 = 8.955700f * 100;
	nearP = 0.1f;
	farP = 1000.0f;*/

	/*fu = 6.145110f;
	fv = 6.145110f;
	u0 = 6.395000f * 100;
	v0 = 5.967700f * 100;
	nearP = 0.1f;
	farP = 1000.0f;*/

	/*fu = 1.739954f * 100;
	fv = 1.739954f * 100;
	u0 = 3.195000f * 100;
	v0 = 2.395000f * 100;
	nearP = 0.1f;
	farP = 1000.0f;*/

	/*fu = 1000.145110f;
	fv = 1000.145110f;
	u0 = 6.395000f * 100;
	v0 = 5.967700f * 100;
	nearP = 0.1f;
	farP = 1000.0f;*/


	// EXTRINSIC PARAMS
	/*projectorRotation = glm::mat3(1, -6.831473f * 0.000000000001f, -2.304257f * 0.00000000001f,
		3.765715f * 0.000000000001f, 9.914472f * 0.1f, 1.305087f * 0.1f,
		-1.305087f * 0.01f, 2.373711 * 0.00000000001f, 9.914472f * 0.1f);
	projectorTranslation = glm::vec3(1.276965f * 0.0000000001f, -5.647171f * 0.01f, -8.252846f * 0.1f);*/
	//projectorTranslation = glm::vec3(-1.305087f * 0.1f, -9.914472f * 0.1f, 8.252846f * 0.1f);
	
	/*[slib::EstimateRelativePoseByEssentialMatrix] matR =:
        [  0.991884  0.0657448  0.108825 ]
        [ -0.0869922  0.975146  0.203772 ]
        [ -0.092723 -0.211586  0.972951 ]
[slib::EstimateRelativePoseByEssentialMatrix] vecT =:
        [ -0.191535 ]
        [ -0.654035 ]
        [ -0.731815 ]*/
	/*projectorRotation = glm::mat3(9.918845f * 0.1f, 6.574481f * 0.01f, 1.088248f * 0.1f,
		-1.915351f * 0.1f, -8.699224f * 0.01f, 9.751457f * 0.1f,
		2.037724f * 0.1f, -6.540347f * 0.1f, -9.272302f * 0.01f);
	projectorTranslation = glm::vec3(-2.115856f * 0.1f, 9.729512f * 0.1f, -7.318148f * 0.1f);*/
	
	/*projectorRotation = glm::mat3(0.991884f, 0.0657448f, 0.108825f,
		-0.0869922f, 0.975146f, 0.203772f,
		-0.092723f, -0.211586f, 0.972951f);
	projectorTranslation = glm::vec3(-0.191535f, -0.654035f, -0.731815f);*/

	/*projectorRotation = glm::mat3(-0.991884f, -0.0657448f, -0.108825f,
		0.0869922f, -0.975146f, -0.203772f,
		0.092723f, 0.211586f, -0.972951f);*/
	//projectorTranslation = glm::vec3(0.191535f, 0.654035f, 0.731815f);

	/* Correct resolution
	
	[slib::EstimateRelativePoseByEssentialMatrix] matR =:
        [ -0.992275 -0.0724406 -0.100716 ]
        [  0.0851535 -0.988077 -0.12827 ]
        [ -0.0902228 -0.135855  0.986612 ]
[slib::EstimateRelativePoseByEssentialMatrix] vecT =:
        [  0.182916 ]
        [  0.630361 ]
        [ -0.754444 ]*/
/*	projectorRotation = glm::mat3(-0.992275f, - 0.0724406f, - 0.100716f,
		0.0851535f, - 0.988077f, - 0.12827f,
		-0.0902228f, - 0.135855f,  0.986612f);
	projectorTranslation = glm::vec3(0.182916f, 0.630361f, -0.754444f);
	*/
	/* transpose of R
	[slib::EstimateRelativePoseByEssentialMatrix] matR =:
        [ -0.992275  0.0851535 -0.0902228   ]
        [ -0.0724406 -0.988077 -0.135855  ]
        [ -0.100716 -0.12827   0.986612 ]
	*/
	/*projectorRotation = glm::mat3(-0.992275f,  0.0851535f, -0.0902228f,
		-0.0724406f, -0.988077f, -0.135855f,
		-0.100716f,  -0.12827f,   0.986612f);*/
	/*projectorRotation = glm::mat3(0.992275f, -0.0851535f, 0.0902228f,
		0.0724406f, 0.988077f, 0.135855f,
		0.100716f, 0.12827f, -0.986612f);*/
	//projectorTranslation = glm::vec3(0.182916f, 0.630361f, -0.754444f);

	// custom values from manual homography process
	/* Pose =
		 [0.98989952, 0.04134931, -0.00043529339, 71.581383;
		  0.1417698, 0.99914461, -0.00044309642, 108.5266;
		  0.00050215528, 0.00046859321, 0.98319072, 2.3090768]

	*/

/*	projectorRotation = glm::mat3(0.98989952f, 0.04134931f, -0.00043529339,
		0.1417698f, 0.99914461f, -0.00044309642f,
		0.00050215528f, 0.00046859321f, 0.98319072f);
	projectorTranslation = glm::vec3(71.581383f, 108.5266f, 2.3090768f);
	*/

	// alpha = fu = beta = fv
	// aspect = beta/alpha * width/height
	aspect = 1920 / 1080.0f;
	//aspect = 1280 / 720.0f;
	// fovy = 2 arctan(height/2*beta);
	//fovy = 2 * glm::atan(1080 / 2 * fv);
	//fovy = 2 * glm::atan(720 / 2 * fv);
	//fovy = 2 * glm::atan(1080 / 2 * 9.645909f * 10);
	//fovy = 2 * glm::atan(1080 / 2 * 23.6f);
	//fovy = 2 * glm::atan(v0 / 2 * fv) * (180 / 3.14f);
	fovy = 2 * glm::atan(glm::tan(1080.0f / 2) * 1080.0f / 1920) * (180 / 3.14f);

	

	/* THE MODELVIEW MATRIX: FRONT CENTERED - MIRRORED IMAGE
		0.994276, -0.00854613, -0.106502, 0
		-6.39558e-005, 0.996748, -0.0805799, 0
		0.106844, 0.0801254, 0.991042, 0
		-0.0471905, -0.0128643, -1.2705, 1
	*/

	bDebugTexture		 = false;
	bMirrorView			 = false;
	bUseCalibParams		 = false;
	bShowFrustum		 = false;
	bUseCustomMatrices   = false;
	bFragmentShaderDebug = false;
	/* CALIB 1080p 5 JUNE
		[slib::EstimateRelativePoseByEssentialMatrix] matR =:
        [  0.999679 -0.0048328 -0.0248841 ]
        [  0.00615097  0.998566  0.0531716 ]
        [  0.0245914 -0.0533075  0.998275 ]
[slib::EstimateRelativePoseByEssentialMatrix] vecT =:
        [  0.0340259 ]
        [ -0.526122 ]
        [ -0.849728 ]
	*/
	projectorRotation = glm::mat3(0.999679f, -0.0048328f, 0.0248841f,
		0.00615097f, 0.998566f, 0.0531716f,
		0.0245914f, -0.0533075f, 0.998275f);
	projectorTranslation = glm::vec3(0.0340259f, -0.526122f, -0.849728f);

	mExtrinsicMatrix = glm::mat4(projectorRotation[0][0], projectorRotation[0][1], projectorRotation[0][2], projectorTranslation.x,
		projectorRotation[1][0], projectorRotation[1][1], projectorRotation[1][2], projectorTranslation.y,
		projectorRotation[2][0], projectorRotation[2][1], projectorRotation[2][2], projectorTranslation.z,
		0, 0, 0, 1);
	//mExtrinsicMatrix = glm::transpose(mExtrinsicMatrix);
	mExtrinsicMatrix = glm::inverse(mExtrinsicMatrix);
	
	printGLMMatrix(mExtrinsicMatrix, "EXTRINSIC MATRIX");

	fu = 4.035944f * 10;
	fv = 4.035944f * 10;
	u0 = 9.595000f * 100;
	v0 = 8.955700f * 100;
	nearP = 0.1f;
	farP = 1000.0f;


	// cam intrinsic
	/*fu = 1.534547f * 100;
	fv = 1.534547f * 100;
	u0 = 3.195000f * 100;
	v0 = 2.395000f * 100;
	nearP = 0.1f;
	farP = 1000.0f;*/

	int viewport[4] = { 0, 0, 0, 0 };
	
	build_opengl_projection_for_intrinsics(frustum, viewport,
		fu, fv, 0, u0, v0,
		1920, 1080, nearP, farP);

	left = -1 * (u0 / fu) * nearP;
	right = ((1920 - u0) / fu) * nearP;
	top = (v0 / fv) * nearP;
	bottom = -1 * ((1080 - v0) / fv) * nearP;


	frustum = glm::mat4(0);
	frustum[0][0] = fu / u0;
	frustum[1][1] = fv / v0;
	frustum[2][2] = -(farP + nearP) / (farP - nearP);
	frustum[2][3] = (-2) * farP * nearP / (farP - nearP);
	frustum[3][2] = -1;

	printGLMMatrix(frustum, "FRUSTUM MATRIX");


	/*fovy   = 1.0f / (fu / 1080.0f * 2);
	aspect = 1980.0f / 1080 * fv / fu;

	cout << "Aspect: " << aspect << endl;
	cout << "FOV Y:" << fovy << endl;

	frustum_height = nearP * fovy;
	frustum_width = frustum_height * aspect;
	offset_x = (1920 / 2 - u0) / 1920 * frustum_width * 2;
	offset_y = (1080 / 2 - v0) / 1080 * frustum_height * 2;*/

	fovy = 1.0f / (fu/2.5f / 1080.0f * 2);
	aspect = 1980.0f / 1080 * fv / fu;

	cout << "Aspect: " << aspect << endl;
	cout << "FOV Y:" << fovy << endl;

	frustum_height = nearP * fovy;
	frustum_width = frustum_height * aspect;
	offset_x = (1920 / 2 - u0) / 1920 * frustum_width * 2;
	offset_y = (1080 / 2 - v0) / 1080 * frustum_height * 2;

	// Build and apply the projection matrix
	//glFrustumf(-frustum_width - offset_x, frustum_width - offset_x, -frustum_height - offset_y, frustum_height - offset_y, nearP, farP);
	xRotation = 0;
	yRotation = 0;
	zRotation = 0;
	xRotationCam = 0;
	yRotationCam = 0;
	zRotationCam = 0;

	activeTextureIndex = 5;
}


PointCloudPTMScene::~PointCloudPTMScene()
{
}

void PointCloudPTMScene::initScene(GLFWwindow *window)
{
	//glClearColor(0.0f, 0.0f, 0.4f, 1.0f);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

	// set up pipeline
	pipeline = new Pipeline();
	pipeline->resetTimer();
	//pipeline->setProjection3D(45.0f, 4.0f / 3.0f, 0.1f, 1000.0f);
	

	pipeline->setProjection3D(pFovY, 16.0f / 9.0f, 0.1f, 1000.0f);
	
	//pipeline->setProjection3D(fovy - 7, 16.0f / 9.0f, 0.1f, 10000.0f);
	glm::mat4 *projMatrix = pipeline->getProjectionMatrix();
	//*projMatrix = glm::scale(*projMatrix, glm::vec3(1.0f, -1.0f, 1.0f));
	*projMatrix  = glm::scale(*projMatrix, glm::vec3(-1.0f, 1.0f, 1.0f));
	pipeline->setProjectionMatrix(*projMatrix);
	//pipeline->setProjectionMatrix(glm::inverse(frustum));
	//pipeline->setProjectionMatrix(frustum);
	
	//pipeline->setProjection3D(fovy, aspect, 0.1f, 1000.0f);
	// set calibrated frustum
	//pipeline->setFrustum(left, right, bottom, top, nearP, farP);
	// set calibrated view matrix
	/*calibratedView = glm::lookAt(projectorTranslation, // projector origin
		glm::vec3(projectorTranslation.x + 100 * projectorRotation[2][0], 
		projectorTranslation.y + 100 * projectorRotation[2][1], 
		projectorTranslation.z + 100 * projectorRotation[2][2]),	 // project on object.. 
		glm::vec3(projectorRotation[1][0], projectorRotation[1][1], projectorRotation[1][2])   // Y axis is up
		);*/
	/*calibratedView = glm::lookAt(projectorTranslation, // projector origin
		glm::vec3(projectorTranslation.x + 100 * projectorRotation[0][2],
		projectorTranslation.y + 100 * projectorRotation[1][2],
		projectorTranslation.z + 100 * projectorRotation[2][2]),	 // project on object.. 
		glm::vec3(projectorRotation[0][1], projectorRotation[1][1], projectorRotation[2][1])   // Y axis is up
		);*/
	biasMatrix = glm::mat4(
		0.5, 0.0, 0.0, 0.0,
		0.0, 0.5, 0.0, 0.0,
		0.0, 0.0, 0.5, 0.0,
		0.5, 0.5, 0.5, 1.0
		);

	// 4:3 perspective with 45 fovv
	//projectorP = glm::perspective(45.0f * zoomFactor, 4.0f / 3.0f, 0.1f, 1000.0f);
	//projectorP = glm::perspective(fovy + 5, aspect, pNear, pFar);
	//projectorP = glm::perspective(fovy, aspect, pNear, pFar);
	projectorP = glm::perspective(30.0f, aspect, pNear, pFar);
	projectorP = glm::scale(projectorP, glm::vec3(-1.0f, 1.0f, 1.0f));
	//projectorP = glm::scale(projectorP, glm::vec3(1.0f, -1.0f, 1.0f));
	//projectorP = glm::perspective(fovy, aspect, pNear, pFar);
	//projectorP = glm::ortho<float>(-100, 100, -100, 100, -100, 200);
	//projectorOrigin = glm::vec3(0.0f, -0.01f, 0.10f);
	//projectorOrigin = glm::vec3(0.0f, -0.015f, 0.1f);
	projectorOrigin = glm::vec3(0.0f, -0.002f, 0.1f);
	//projectorOrigin = glm::vec3(0.0f, -0.0f, 0.2f);
	//projectorOrigin = glm::vec3(0.0f, -0.01f, 0.002f);
	finalProjectorOrigin = glm::vec3(0.0f, 0.0f, -0.1f);
	projectorTarget = glm::vec3(0.0f, 0.0f, 0.0f);
	projectorV = glm::lookAt(projectorOrigin, // projector origin
		projectorTarget,	 // project on object at origin 
		glm::vec3(0.0f, 1.0f, 0.0f)   // Y axis is up
		);
	//projectorV = mExtrinsicMatrix;
	//projectorV = getViewMatrix();
	mModel = glm::mat4(1.0f);

	

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

	//glGenVertexArrays(1, &uiVAODebug);
	//glBindVertexArray(uiVAODebug);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	vboDebug.createVBO();
	vboDebug.bindVBO();
	// add DEBUG QUAD
	for (int i = 0; i < 6; i++)
	{
		//vboSceneObjects.addData(&ss_quad_pos[i], sizeof(glm::vec3));
		vboDebug.addData(&g_quad_vertex_buffer_data[i], sizeof(glm::vec3));

		// dummy data to keep structure (NOT USED)
		//vCubeTexCoordsL[i] *= 10.0f;
		//vboSceneObjects.addData(&vCubeTexCoordsL[i % 6], sizeof(glm::vec2));
		//glm::vec3 vGroundNormal(0.0f, 1.0f, 0.0f);
		//vboSceneObjects.addData(&vGroundNormal, sizeof(glm::vec3));
	}

	//vboDebug.uploadDataToGPU(GL_STATIC_DRAW);
	// compute frustum coords
	computeFrustumCoords(frustum);
	glEnableVertexAttribArray(0);
	//glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), 0);

	const char *cTextureNames[] = { "biohazard_512x512.jpg", "A_Smiley.jpg", "3dcarrush.jpg",
		"3D_Scene_011.jpg", "outdoor_free_3d_scene_by_djeric.jpg", "grid_512k.jpg",
		"grid-xxl.jpg", "grid-green.jpg", "tuscany.jpg" };
	vector<string> sTextureNames(cTextureNames, &cTextureNames[sizeof(cTextureNames) / sizeof(cTextureNames[0])]);
	loadAllTextures(sTextureNames);
	glEnable(GL_TEXTURE_2D);

	programID		  = LoadShaders(vertexShaderPTMSource, fragmentShaderPTMSource);
	//programID		  = LoadShaders(vertexShaderTexSource, fragmentShaderTexSource);
	programColorID	  = LoadShaders(vertexShaderSource, fragmentShaderSource);
	programID_HeatMap = LoadShaders(hmVertexShaderSource, hmFragmentShaderSource);
	programQuadID	  = LoadShaders(vertexShaderQuadSource, fragmentShaderQuadSource);
	programFrustumID  = LoadShaders(frustumVertexShaderSource, frustumFragmentShaderSource);

	glPointSize(2.0f);
	glUseProgram(programID);

	std::cout << "Init end error..: " << gluErrorString(glGetError()) << "\n";
	std::cout << "Init end error.. " << gluErrorString(glGetError()) << "\n";
}

void PointCloudPTMScene::renderScene(GLFWwindow *window)
{
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	rdp->renderFrame();
	timer += getDeltaTime();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glm::mat4 mModelView;
	glm::mat4 mCurrent;
	computeMatricesFromInputs(window);

	if (bDebugTexture)
	{
		// only diplay texture
		// DEFORMATION OFF
		glUseProgram(programQuadID);
		GLuint texID = glGetUniformLocation(programQuadID, "tex");
		// Bind our texture in Texture Unit 0
		tTextures[activeTextureIndex].bindTexture();
		// Set our "renderedTexture" sampler to user Texture Unit 0
		glUniform1i(texID, 0);

		vboDebug.bindVBO();
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), 0);
		//glBindVertexArray(uiVAODebug);
		glDrawArrays(GL_TRIANGLES, 0, 6);
	}
	else
	{
		
		vboSceneObjects.bindVBO();
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(PXCPoint3DF32), 0);
		//glBindVertexArray(uiVAOSceneObjects);
		vboSceneObjects.addData(rdp->worldPos.data(), rdp->nPoints * sizeof(PXCPoint3DF32));
		vboSceneObjects.uploadDataToGPU(GL_STREAM_DRAW);

		if (bUseMainCam)
		{
			mModelView = getViewMatrix();
			if (bUseCalibParams)
			{
				//pipeline->setProjection3D(fovy, 16.0f / 9.0f, 0.1f, 1000.0f);
				//pipeline->setProjectionMatrix(glm::inverse(frustum));
				//pipeline->setProjectionMatrix(frustum);

				/*pipeline->setFrustum(left, right, bottom, top, nearP, farP);
				frustum = *pipeline->getProjectionMatrix();
				pipeline->setProjectionMatrix(glm::inverse(frustum));*/

				pipeline->setProjection3D(fovy, aspect, 0.1f, 1000.0f);
				//pipeline->setFrustum(-frustum_width - offset_x, frustum_width - offset_x, -frustum_height - offset_y, frustum_height - offset_y, nearP, farP);
				frustum = *pipeline->getProjectionMatrix();
				frustum = glm::scale(frustum, glm::vec3(-1.0f, 1.0f, 1.0f));
				pipeline->setProjectionMatrix(frustum);
				mModelView = mExtrinsicMatrix;
			}
			else if (bUseCustomMatrices)
			{
				/* RED DEPTH DEBUG MATRICES
				--===========================================--
				THE PERSPECTIVE MATRIX
				-3.0229, -0, -0, -0
				0, 5.54199, 0, 0
				0, 0, -1.0002, -1
				0, 0, -0.20002, 0

				THE MODELVIEW MATRIX
				0.99998, 0.000654481, 0.00628154, 0
				-7.30413e-005, 0.995748, -0.0921205, 0
				-0.00631512, 0.0921182, 0.995728, 0
				-0.00421113, -0.0437065, -0.472429, 1
				--===========================================--

				PRETTY WELL CALIBRATED
				DISTORTION IS WRONG!! - it should be the other way around
				--===========================================--
				THE PERSPECTIVE MATRIX
				-2.99261, -0, -0, -0
				0, 5.48645, 0, 0
				0, 0, -1.0002, -1
				0, 0, -0.20002, 0

				THE MODELVIEW MATRIX
				0.999999, -0.000101662, -0.00157952, 0
				-0.00010083, 0.991817, -0.127672, 0
				0.00157957, 0.127672, 0.991815, 0
				3.65805e-005, -0.0139091, -0.455075, 1
				--===========================================--

				HOW IS THE DISTORTION CORRECT????
				--===========================================--
				THE PERSPECTIVE MATRIX
				-2.93013, -0, -0, -0
				0, 5.37191, 0, 0
				0, 0, -1.0002, -1
				0, 0, -0.20002, 0

				THE MODELVIEW MATRIX
				0.999999, -0.000116628, -0.00157537, 0
				-0.00011537, 0.989216, -0.146467, 0
				0.00157547, 0.146467, 0.989214, 0
				0.0019247, -0.000493295, -0.45061, 1
				--===========================================--

				*/

				/*frustum = glm::mat4(-3.0229, -0, -0, -0,
									0, 5.54199, 0, 0,
									0, 0, -1.0002, -1,
									0, 0, -0.20002, 0);
				pipeline->setProjectionMatrix(frustum);

				mModelView = glm::mat4(0.99998, 0.000654481, 0.00628154, 0,
									- 7.30413e-005, 0.995748, -0.0921205, 0,
									- 0.00631512, 0.0921182, 0.995728, 0,
									- 0.00421113, -0.0437065, -0.472429, 1);

				NEW ALIGNMENT: NOT PERFECT

				--===========================================--
				THE PERSPECTIVE MATRIX
				-2.87296, -0, -0, -0
				0, 5.2671, 0, 0
				0, 0, -1.0002, -1
				0, 0, -0.20002, 0

				THE MODELVIEW MATRIX
				0.998122, -0.00426727, -0.061106, 0
				-5.60573e-005, 0.997507, -0.0705755, 0
				0.0612547, 0.0704464, 0.995633, 0
				0.0531064, -0.0214702, -0.427705, 1
				--===========================================--

				NEW ALIGNMENT: BETTER
				--===========================================--
				THE PERSPECTIVE MATRIX
				-2.81337, -0, -0, -0
				0, 5.15784, 0, 0
				0, 0, -1.0002, -1
				0, 0, -0.20002, 0

				THE MODELVIEW MATRIX
				0.99993, 0.00172356, 0.011729, 0
				-0.000107598, 0.990654, -0.136402, 0
				-0.0118545, 0.136391, 0.990584, 0
				-0.0148524, 0.0282155, -0.403769, 1
				--===========================================--


				NEW ALIGNMENT - ALIGNED TO 1 FINGER (PROJECTION AREA IS WRONG, TEXTURE IS MIRRORED)
				--===========================================--
				THE PERSPECTIVE MATRIX
				-2.96728, 0, 0, 0
				0, 5.44002, 0, 0
				0, 0, -1.0002, -1
				0, 0, -0.20002, 0

				THE MODELVIEW MATRIX
				0.999843, 0.000360266, 0.0177194, 0
				-1.54878e-005, 0.999811, -0.019454, 0
				-0.017723, 0.0194507, 0.999654, 0
				-0.0183076, 0.000230341, -0.465685, 1

				THE CURRENT MATRIX (TRANSFORMED MV)
				0.998323, -0.00100735, 0.0578822, 0
				0.000962591, -0.999422, -0.0339958, 0
				0.0578829, 0.0339945, -0.997745, 0
				-0.0183076, 0.000230341, -0.465685, 1
				--===========================================--

									*/
				frustum = glm::mat4(-2.96728, -0, -0, -0,
									0, 5.44002, 0, 0,
									0, 0, -1.0002, -1,
									0, 0, -0.20002, 0);
				pipeline->setProjectionMatrix(frustum);

				// DOES NOT WORK BECAUSE THE CURRENT MATRIX IS INVERTED AGAIN 180 degrees!!!!!
				// need to go around this, so I can save the correct matrix!!
				mModelView = glm::mat4(0.998323, -0.00100735, 0.0578822, 0,
									0.000962591, -0.999422, -0.0339958, 0,
									0.0578829, 0.0339945, -0.997745, 0,
									- 0.0183076, 0.000230341, -0.465685, 1);
			}
			else
			{
				pipeline->setProjection3D(pFovY, aspect, nearP, farP);
				frustum = *pipeline->getProjectionMatrix();
				frustum = glm::scale(frustum, glm::vec3(-1.0f, 1.0f, 1.0f));
				//frustum = glm::scale(frustum, glm::vec3(1.0f, -1.0f, 1.0f));
				pipeline->setProjectionMatrix(frustum);
			}
			//mModelView = calibratedView;
			
			if (bMirrorView)
			{
				//glm::mat4* projMatrix = pipeline->getProjectionMatrix();
				//*projMatrix = glm::scale(*projMatrix, glm::vec3(1.0f, -1.0f, 1.0f));
				//*projMatrix = glm::scale(*projMatrix, glm::vec3(-1.0f, 1.0f, 1.0f));
				//pipeline->setProjectionMatrix(*projMatrix);

				frustum = glm::scale(frustum, glm::vec3(-1.0f, 1.0f, 1.0f));
				pipeline->setProjectionMatrix(frustum);
				//mModelView = glm::scale(mModelView, glm::vec3(1.0f, 1.0f, -1.0f));
			}
		}
		else
		{
			if (invertView)
			{
				mModelView = glm::inverse(getViewMatrix());
				//mModelView = glm::lookAt(finalProjectorOrigin, projectorTarget, glm::vec3(0, 1, 0));
				//mModelView = glm::inverse(calibratedView);
				//mModelView = glm::inverse(mExtrinsicMatrix);
			}
			else
				mModelView = projectorV;
		}


		rotationMatrix = glm::mat4(1.0f);
		rotationMatrix = glm::rotate(rotationMatrix, xRotationCam, glm::vec3(1.0f, 0.0f, 0.0f));
		rotationMatrix = glm::rotate(rotationMatrix, yRotationCam, glm::vec3(0.0f, 1.0f, 0.0f));
		rotationMatrix = glm::rotate(rotationMatrix, zRotationCam, glm::vec3(0.0f, 0.0f, 1.0f));

		// apply model view to rotation!!
		mModelView = rotationMatrix * mModelView;

		mCurrent = mModelView;
		//mCurrent = glm::scale(mCurrent, glm::vec3(-1.0f, 1.0f, 1.0f));
		//mCurrent = glm::translate(mModelView, glm::vec3(0.0f, 0.0f, 0.0f));
		//mCurrent = glm::scale(mCurrent, glm::vec3(400.0f, 400.0f, 400.0f));
		//mCurrent = glm::translate(mModelView, glm::vec3(0.0f, 0.0f, -1.0f));
		//mCurrent = glm::rotate(mCurrent, 180.0f, glm::vec3(-1.0f, 0.0f, 0.0f));
		
		mCurrent = glm::rotate(mCurrent, -180.0f, glm::vec3(1.0f, 0.0f, 0.0f));
		
		// also allow rotation
		mCurrent = glm::rotate(mCurrent, xRotation, glm::vec3(1.0f, 0.0f, 0.0f));
		mCurrent = glm::rotate(mCurrent, yRotation, glm::vec3(0.0f, 1.0f, 0.0f));
		mCurrent = glm::rotate(mCurrent, zRotation, glm::vec3(0.0f, 0.0f, 1.0f));
		

		//mCurrent = glm::rotate(mCurrent, 180.0f, glm::vec3(0.0f, 1.0f, 0.0f));
		//mCurrent = glm::scale(mCurrent, glm::vec3(10.0f, 10.0f, 10.0f));

		//glUniformMatrix4fv(iModelViewColorLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));

		if (bShowFrustum)
		{
			// RENDER PROJECTOR FRUSTUM
			glUseProgram(programFrustumID);
			vboDebug.bindVBO();
			glEnableVertexAttribArray(0);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), 0);

			int iStaticColor = glGetUniformLocation(programFrustumID, "theColor");
			glUniform3fv(iStaticColor, 1, glm::value_ptr(glm::vec3(1, 1, 1)));

			int iFrustumModelViewLoc = glGetUniformLocation(programFrustumID, "MV");
			int iFrustumProjectionLoc = glGetUniformLocation(programFrustumID, "P");
			int iFrustumInvProjectionLoc = glGetUniformLocation(programFrustumID, "invP");
			int iFrustumInvMVLoc = glGetUniformLocation(programFrustumID, "invMV");

			// calibrated camera
			//glUniformMatrix4fv(iFrustumInvProjectionLoc, 1, GL_FALSE, glm::value_ptr(frustum));
			//glUniformMatrix4fv(iFrustumInvMVLoc, 1, GL_FALSE, glm::value_ptr(mExtrinsicMatrix));
			glUniformMatrix4fv(iFrustumInvProjectionLoc, 1, GL_FALSE, glm::value_ptr(glm::inverse(frustum)));
			glUniformMatrix4fv(iFrustumInvMVLoc, 1, GL_FALSE, glm::value_ptr(glm::inverse(mExtrinsicMatrix)));
			//glUniformMatrix4fv(iFrustumInvProjectionLoc, 1, GL_FALSE, glm::value_ptr(glm::inverse(projectorP)));
			//glUniformMatrix4fv(iFrustumInvMVLoc, 1, GL_FALSE, glm::value_ptr(glm::inverse(projectorV)));
			// world view
			glUniformMatrix4fv(iFrustumProjectionLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));
			//glUniformMatrix4fv(iFrustumProjectionLoc, 1, GL_FALSE, glm::value_ptr(glm::inverse(projectorP)));
			//mCurrent = glm::translate(mModelView, projectorOrigin);
			glUniformMatrix4fv(iFrustumModelViewLoc, 1, GL_FALSE, glm::value_ptr(mModelView));

			// wireframe
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			glDisable(GL_CULL_FACE);
			glDrawArrays(GL_TRIANGLES, 6, 42);
			// fill polygons (normal)
			//glEnable(GL_CULL_FACE);
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

			glUseProgram(programID);
			vboSceneObjects.bindVBO();
			glEnableVertexAttribArray(0);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(PXCPoint3DF32), 0);
		}

		if (bShowHeatMap)
		{
			glUseProgram(programID_HeatMap);
			int iModelViewLoc = glGetUniformLocation(programID_HeatMap, "MV");
			int iProjectionLoc = glGetUniformLocation(programID_HeatMap, "P");

			glUniformMatrix4fv(iProjectionLoc, 1, GL_FALSE, glm::value_ptr(*pipeline->getProjectionMatrix()));
			glUniformMatrix4fv(iModelViewLoc, 1, GL_FALSE, glm::value_ptr(mCurrent));
		}
		else
		{
			//glUseProgram(programColorID);

			// PROJECTIVE TEXTURE MAPPING
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
			int iDebugLoc = glGetUniformLocation(programID, "debug");

			// set texture
			glClearDepth(1.0f);
			tTextures[activeTextureIndex].bindTexture(1);


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
			
			// update debug state
			glUniform1i(iDebugLoc, bFragmentShaderDebug);

			// END PTM MESH
		}


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
		//vboSceneObjects.bindVBO();
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

	}
	/*
	fRotationAngle += pipeline->sof(30.0f);

	// make sure the float does not increase
	// drastically with time
	if (fRotationAngle >= 360)
		fRotationAngle -= 360;
		*/
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

	if (timer >= 0.25f)
	{
		if (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_N) != GLFW_RELEASE)
		{
			invertView = !invertView;
			timer = 0;
		}
	}

	if (timer >= 0.25f)
	{
		if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_R) != GLFW_RELEASE)
		{
			setResetMouseLook(true);
			timer = 0;
		}
	}

	if (timer >= 0.25f)
	{
		if (glfwGetKey(window, GLFW_KEY_TAB) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_TAB) != GLFW_RELEASE)
		{
			// mirror view
			bMirrorView = !bMirrorView;
			timer = 0;
		}
	}


	if (timer >= 0.25f)
	{
		if (glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_H) != GLFW_RELEASE)
		{
			bShowHeatMap = !bShowHeatMap;
			timer = 0;
		}
	}

	if (timer >= 0.25f)
	{
		if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_1) != GLFW_RELEASE)
		{
			decreaseSpeedFactor();
			timer = 0;
		}
	}

	if (timer >= 0.25f)
	{
		if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_2) != GLFW_RELEASE)
		{
			increaseSpeedFactor();
			timer = 0;
		}
	}

	if (timer >= 0.25f)
	{
		if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_P) != GLFW_RELEASE)
		{
			cout << endl << "--===========================================--" << endl;
			printGLMMatrix(frustum, "THE PERSPECTIVE MATRIX");
			cout << endl;
			printGLMMatrix(mModelView, "THE MODELVIEW MATRIX");
			cout << endl;
			printGLMMatrix(mCurrent, "THE CURRENT MATRIX (TRANSFORMED MV)");
			cout << "--===========================================--" << endl << endl;
			timer = 0;
		}
	}

	if (timer >= 0.25f)
	{
		if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_T) != GLFW_RELEASE)
		{
			bDebugTexture = !bDebugTexture;
			timer = 0;
		}
	}

	if (timer >= 0.25f)
	{
		if (glfwGetKey(window, GLFW_KEY_0) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_0) != GLFW_RELEASE)
		{
			bool sml = isStartMouseLook();
			setStartMouseLook(!sml);
			timer = 0;
		}
	}

	if (timer >= 0.25f)
	{
		if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_K) != GLFW_RELEASE)
		{
			bUseCalibParams = !bUseCalibParams;
			timer = 0;
		}
	}

	if (timer >= 0.25f)
	{
		if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_F) != GLFW_RELEASE)
		{
			bShowFrustum = !bShowFrustum;
			timer = 0;
		}
	}

	if (timer >= 0.25f)
	{
		if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_F) != GLFW_RELEASE)
		{
			bShowFrustum = !bShowFrustum;
			timer = 0;
		}
	}


		if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_O) != GLFW_RELEASE)
		{

			pFovY += 10*getDeltaTime();
			pipeline->setProjection3D(pFovY, aspect, nearP, farP);
			frustum = *pipeline->getProjectionMatrix();
			frustum = glm::scale(frustum, glm::vec3(-1.0f, 1.0f, 1.0f));
			pipeline->setProjectionMatrix(frustum);
			
		}



		if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_I) != GLFW_RELEASE)
		{

			pFovY -= 10 * getDeltaTime();
			pipeline->setProjection3D(pFovY, aspect, nearP, farP);
			frustum = *pipeline->getProjectionMatrix();
			frustum = glm::scale(frustum, glm::vec3(-1.0f, 1.0f, 1.0f));
			pipeline->setProjectionMatrix(frustum);
		
		}

		if (glfwGetKey(window, GLFW_KEY_INSERT) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_INSERT) != GLFW_RELEASE)
		{
			//	zRotation += 3.14f / 180 * getDeltaTime();
			zRotation += 10 * getDeltaTime();
		}

		if (glfwGetKey(window, GLFW_KEY_DELETE) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_DELETE) != GLFW_RELEASE)
		{
			//	zRotation -= 3.14f / 180 * getDeltaTime();
			zRotation -= 10 * getDeltaTime();
		}

		if (glfwGetKey(window, GLFW_KEY_PAGE_UP) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_PAGE_UP) != GLFW_RELEASE)
		{
			//	zRotation += 3.14f / 180 * getDeltaTime();
			yRotation += 10 * getDeltaTime();
		}

		if (glfwGetKey(window, GLFW_KEY_PAGE_DOWN) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_PAGE_DOWN) != GLFW_RELEASE)
		{
			//	zRotation -= 3.14f / 180 * getDeltaTime();
			yRotation -= 10 * getDeltaTime();
		}

		if (glfwGetKey(window, GLFW_KEY_HOME) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_HOME) != GLFW_RELEASE)
		{
		//	zRotation += 3.14f / 180 * getDeltaTime();
			xRotation += 10 * getDeltaTime();
		}

		if (glfwGetKey(window, GLFW_KEY_END) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_END) != GLFW_RELEASE)
		{
		//	zRotation -= 3.14f / 180 * getDeltaTime();
			xRotation -= 10 * getDeltaTime();
		}

		// Rotate X
		if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS)
		{

			xRotationCam += 10 * getDeltaTime();
		}


		if (glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS)
		{
			xRotationCam -= 10 * getDeltaTime();
		}

		// Rotate Y
		if (glfwGetKey(window, GLFW_KEY_5) == GLFW_PRESS)
		{

			yRotationCam += 10 * getDeltaTime();
		}

		if (glfwGetKey(window, GLFW_KEY_6) == GLFW_PRESS)
		{
			yRotationCam -= 10 * getDeltaTime();
		}

		// Rotate Z
		if (glfwGetKey(window, GLFW_KEY_7) == GLFW_PRESS)
		{

			zRotationCam += 10 * getDeltaTime();
		}

		if (glfwGetKey(window, GLFW_KEY_8) == GLFW_PRESS)
		{
			zRotationCam -= 10 * getDeltaTime();
		}


		if (timer >= 0.25f)
		{
			if (glfwGetKey(window, GLFW_KEY_Y) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_Y) != GLFW_RELEASE)
			{
				bUseCustomMatrices = !bUseCustomMatrices;
				timer = 0;
			}
		}

		if (timer >= 0.25f)
		{
			if (glfwGetKey(window, GLFW_KEY_U) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_U) != GLFW_RELEASE)
			{
				bFragmentShaderDebug = !bFragmentShaderDebug;
				timer = 0;
			}
		}

		if (timer >= 0.25f)
		{
			if (glfwGetKey(window, GLFW_KEY_LEFT_BRACKET) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_LEFT_BRACKET) != GLFW_RELEASE)
			{
				// prev tex
				if (activeTextureIndex > 0)
					activeTextureIndex--;
				else
					activeTextureIndex = tTextures.size() - 1;
				timer = 0;
			}
		}

		if (timer >= 0.25f)
		{
			if (glfwGetKey(window, GLFW_KEY_RIGHT_BRACKET) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_RIGHT_BRACKET) != GLFW_RELEASE)
			{
				// next tex
				if (activeTextureIndex < tTextures.size() - 1)
				{
					activeTextureIndex++;
				}
				else
					activeTextureIndex = 0;
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


	//cout << "frustum coord 0: " << frustum_coords[0].x << ", " << frustum_coords[0].y << ", " << frustum_coords[0].z << endl;
	CreateFrustumSimple(&uiVAOSceneObjects, vboDebug, frustum_coords);

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

/**
@brief basic function to produce an OpenGL projection matrix and associated viewport parameters
which match a given set of camera intrinsics. This is currently written for the Eigen linear
algebra library, however it should be straightforward to port to any 4x4 matrix class.
@param[out] frustum Eigen::Matrix4d projection matrix.  Eigen stores these matrices in column-major (i.e. OpenGL) order.
@param[out] viewport 4-component OpenGL viewport values, as might be retrieved by glGetIntegerv( GL_VIEWPORT, &viewport[0] )
@param[in]  alpha x-axis focal length, from camera intrinsic matrix
@param[in]  alpha y-axis focal length, from camera intrinsic matrix
@param[in]  skew  x and y axis skew, from camera intrinsic matrix
@param[in]  u0 image origin x-coordinate, from camera intrinsic matrix
@param[in]  v0 image origin y-coordinate, from camera intrinsic matrix
@param[in]  img_width image width, in pixels
@param[in]  img_height image height, in pixels
@param[in]  near_clip near clipping plane z-location, can be set arbitrarily > 0, controls the mapping of z-coordinates for OpenGL
@param[in]  far_clip  far clipping plane z-location, can be set arbitrarily > near_clip, controls the mapping of z-coordinate for OpenGL
*/
void PointCloudPTMScene::build_opengl_projection_for_intrinsics(glm::mat4 &frustum, int *viewport, 
	double alpha, double beta, double skew, double u0, double v0, 
	int img_width, int img_height, double near_clip, double far_clip){

	// These parameters define the final viewport that is rendered into by
	// the camera.
	double L = 0;
	double R = img_width;
	double B = 0;
	double T = img_height;

	// near and far clipping planes, these only matter for the mapping from
	// world-space z-coordinate into the depth coordinate for OpenGL
	double N = near_clip;
	double F = far_clip;

	// set the viewport parameters
	viewport[0] = L;
	viewport[1] = B;
	viewport[2] = R - L;
	viewport[3] = T - B;

	// construct an orthographic matrix which maps from projected
	// coordinates to normalized device coordinates in the range
	// [-1, 1].  OpenGL then maps coordinates in NDC to the current
	// viewport
	glm::mat4 ortho = glm::mat4(0);
	
	ortho[0][0] = 2.0 / (R - L); ortho[0][3] = -(R + L) / (R - L);
	ortho[1][1] = 2.0 / (T - B); ortho[1][3] = -(T + B) / (T - B);
	ortho[2][2] = -2.0 / (F - N); ortho[2][3] = -(F + N) / (F - N);
	ortho[3][3] = 1.0;

	/*ortho[0][0] = 2.0 / (R - L); ortho[3][0] = -(R + L) / (R - L);
	ortho[1][1] = 2.0 / (T - B); ortho[3][1] = -(T + B) / (T - B);
	ortho[2][2] = -2.0 / (F - N); ortho[3][2] = -(F + N) / (F - N);
	ortho[3][3] = 1.0;*/


	printGLMMatrix(ortho, "ORTHO MATRIX");

	// construct a projection matrix, this is identical to the 
	// projection matrix computed for the intrinsicx, except an
	// additional row is inserted to map the z-coordinate to
	// OpenGL. 
	glm::mat4 tproj = glm::mat4(0);
	
	tproj[0][0] = alpha; tproj[0][1] = skew; tproj[0][2] = u0;
	tproj[1][1] = beta; tproj[1][2] = v0;
	tproj[2][2] = -(N + F); tproj[2][3] = -N*F;
	tproj[3][2] = 1.0;

	/*tproj[0][0] = alpha; tproj[1][0] = skew; tproj[0][2] = u0;
	tproj[1][1] = beta; tproj[2][1] = v0;
	tproj[2][2] = -(N + F); tproj[3][2] = -N*F;
	tproj[2][3] = 1.0;*/

	//tproj[0][0] = alpha;
	//tproj[1][1] = beta;
	
	//tproj[2][2] = -(N + F); tproj[2][3] = -N*F;
	//tproj[3][2] = 1.0;
	
	//tproj[2][2] = (N + F); tproj[2][3] = N*F;
	//tproj[3][2] = -1.0;


	printGLMMatrix(tproj, "PROJ MATRIX");
	// resulting OpenGL frustum is the product of the orthographic
	// mapping to normalized device coordinates and the augmented
	// camera intrinsic matrix
	frustum = ortho*tproj;
	//frustum = glm::inverse(ortho*tproj);
	//frustum = glm::transpose(ortho*tproj);
}

void PointCloudPTMScene::printGLMMatrix(glm::mat4 matrix, string name)
{
	cout << name << endl;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			cout << matrix[i][j];

			if (j == 3)
			{
				cout << endl;
			}
			else
				cout << ", ";
		}

	}
}