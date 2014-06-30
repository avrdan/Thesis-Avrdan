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
	aspect = 1920 / 1080.0f;
	fovy = 2 * glm::atan(glm::tan(1080.0f / 2) * 1080.0f / 1920) * (180 / 3.14f);

	bDebugTexture		 = false;
	bMirrorView			 = false;
	bUseCalibParams		 = false;
	bShowFrustum		 = false;
	bUseCustomMatrices   = false;
	bFragmentShaderDebug = false;
	bInitFrustum = true;
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

	
	fu = 10027.93410;
	fv = 15699.67859;
	u0 = 959.50000;
	v0 = 539.50000;
	nearP = 0.1f;
	farP = 1000.0f;

	/*
		STRUCTURED LIGHT CALIB (WITH TRIPOD)

		PRO INTRINSIC:
		8.237734e+001  0.000000e+000  9.595000e+002
		0.000000e+000  8.237734e+001  8.955700e+002
		0.000000e+000  0.000000e+000  1.000000e+000

		PRO EXTRINSIC:
		9.999946e-001 -2.607081e-003 -1.982735e-003  2.482408e-003 
		2.972938e-003  4.683975e-001  8.835129e-001  5.700335e-001 
		-1.374682e-003 -8.835140e-001  4.684027e-001  8.216177e-001 

		CAM INTRINSIC:
		2.879222e+002  0.000000e+000  3.195000e+002
		0.000000e+000  2.879222e+002  2.395000e+002
		0.000000e+000  0.000000e+000  1.000000e+000

	*/

	fu = 8.237734f * 10;
	fv = 8.237734f * 10;
	u0 = 9.595000f * 100;
	v0 = 8.955700f * 100;
	nearP = 0.1f;
	farP = 1000.0f;

		/*
		%YAML:1.0
	camIntrinsic: !!opencv-matrix
	   rows: 3
	   cols: 3
	   dt: d
	   data: [ 8.6130405325614177e+001, 0., 3.1950000000000000e+002, 0.,
		   8.6130405325614177e+001, 2.3950000000000000e+002, 0., 0., 1. ]
	camDistortion: -2.3384479671009299e-007
	proIntrinsic: !!opencv-matrix
	   rows: 3
	   cols: 3
	   dt: d
	   data: [ 3.1183991748016678e+002, 0., 9.5950000000000000e+002, 0.,
		   3.1183991748016678e+002, 5.3950000000000000e+002, 0., 0., 1. ]
	proDistortion: -2.9923238202665282e-007
	proExtrinsic: !!opencv-matrix
	   rows: 3
	   cols: 4
	   dt: d
	   data: [ -6.3130502809375055e-001, 4.5280180168695242e-002,
		   7.7421164211566795e-001, -9.0500915844368901e-001,
		   -3.4976844531106721e-002, -9.9894065805775833e-001,
		   2.9902876546890243e-002, -2.5473025906725736e-002,
		   7.7474549488860900e-001, -8.2016439219341399e-003,
		   6.3222001802124395e-001, 4.2462871792214074e-001 ]
	   */
	fu = 3.1183991748016678f * 100;
	fv = 3.1183991748016678f * 100;
	u0 = 9.595000f * 100;
	v0 = 5.395000f * 100;
	nearP = 0.01f;
	farP = 1000.0f;

	fu = 1.3800428341496229f * 1000;
	fv = 1.3800428341496229f * 1000;
	u0 = 9.595000f * 100;
	v0 = 5.395000f * 100;
	nearP = 0.01f;
	farP = 1000.0f;

	/* OFX CALIBRATION
		4.0364508071020787e+003, 0., 9.0808415334558583e+002, 0.,
		3.8705795848373373e+003, 5.9102917701675472e+002, 0., 0., 1.*/
	fu = 4.0364508071020787e+003;
	fv = 3.8705795848373373e+003;
	u0 = 9.0808415334558583e+002;
	v0 = 5.9102917701675472e+002;
	nearP = 0.01f;
	farP = 1000.0f;

	/*
		// rgb FOVs (H and V) in radians (default is for the Creative Gesture Cam)
		rgbFOV.x = 1.003888951f; // 57.5186 degrees
		rgbFOV.y = 0.76542287f; // 43.8555 degrees

		// principal point (image center)
		rgbU = 320;
		dgbV = 240;
		// depth FOVs (H and V) in radians (default is for the Creative Gesture Cam)
		depthFOV.x = 1.238419315f; // 70.9562
		depthFOV.y = 0.960044535f; // 55.0065

		// principal point (image center)
		depthU = 160;
		depthV = 120;
	*/
	
	/*kRGB = glm::mat3(1.003888951f, 0, 320,
		0, 0.76542287f, 240,
		0, 0, 1);*/

	//kRGB = glm::frustum(-1 * (320 / 1.003888951f) * nearP, ((640 - 320) / 1.003888951f) * nearP, -1 * ((480 - 240) / 0.76542287f) * nearP,
	//	(240 / 0.76542287f) * nearP, nearP, farP);

	/*kDepth = glm::mat3(1.238419315f, 0, 160,
		0, 0.960044535f, 120,
		0, 0, 1);
	*/

	//kDepth = glm::frustum(-1 * (160 / 1.238419315f) * nearP, ((320 - 160) / 1.238419315f) * nearP, -1 * ((240 - 120) / 0.960044535f) * nearP,
	//	(120 / 0.960044535f) * nearP, nearP, farP);
	kRGB = glm::perspective(0.76542287f * 180 / glm::pi<float>(), 640 / 480.0f, nearP, farP);
	kDepth = glm::perspective(0.960044535f * 180 / glm::pi<float>(), 320 / 240.0f, nearP, farP);


	int viewport[4] = { 0, 0, 0, 0 };
	
	build_opengl_projection_for_intrinsics(frustum, viewport,
		fu, fv, 0, u0, v0,
		1920, 1080, nearP, farP);

	left = -1 * (u0 / fu) * nearP;
	right = ((1920 - u0) / fu) * nearP;
	top = (v0 / fv) * nearP;
	bottom = -1 * ((1080 - v0) / fv) * nearP;


	/*frustum = glm::mat4(0);
	frustum[0][0] = fu / u0;
	frustum[1][1] = fv / v0;
	frustum[2][2] = -(farP + nearP) / (farP - nearP);
	frustum[2][3] = (-2) * farP * nearP / (farP - nearP);
	frustum[3][2] = -1;*/

	printGLMMatrix(frustum, "FRUSTUM MATRIX");

	//fovy = 1.0f / (fu/2.5f / 1080.0f * 2);
	fovy = 2 * glm::atan(1080.0f / 2 * fu) * 180/glm::pi<float>();
	aspect = 1980.0f / 1080 * fv / fu;

	//fovy = 2 * glm::atan(1080 / 2.0f * fv);
	//aspect = 1980.0f / 1080 * fv / fu;

	cout << "Aspect: " << aspect << endl;
	cout << "FOV Y:" << fovy << endl;

	/*frustum_height = nearP * fovy;
	frustum_width = frustum_height * aspect;
	offset_x = (1920 / 2 - u0) / 1920 * frustum_width * 2;
	offset_y = (1080 / 2 - v0) / 1080 * frustum_height * 2;*/

	// Build and apply the projection matrix
	//glFrustumf(-frustum_width - offset_x, frustum_width - offset_x, -frustum_height - offset_y, frustum_height - offset_y, nearP, farP);
	xRotation = 0;
	yRotation = 0;
	zRotation = 0;
	xRotationCam = 0;
	yRotationCam = 0;
	zRotationCam = 0;
	zTranslationCam = 0;
	yTranslationCam = 0;
	xTranslationCam = 0;
	xTranslation = 0;
	yTranslation = 0;
	zTranslation = 0;
	xTranslationPrjTotal = 0;
	yTranslationPrjTotal = 0;
	zTranslationPrjTotal = 0;

	activeTextureIndex = 5;
	nFrames = 10;
	currFrameIndex = 0;

	/* 3D Depth to 3D Color
		0.999997 - 0.001236 0.002095 - 0.001285 - 0.999726 0.023357 - 0.002065 0.023359 0.999725 0.026000 - 0.000508 - 0.000863
	*/
	/*mDepthToColor = glm::mat4(0.999997, -0.001236, 0.002095, -0.001285,
		-0.999726, 0.023357, -0.002065, 0.023359,
		0.999725, 0.026000, -0.000508, -0.000863,
		0,				0,		0,			1);*/
	mDepthToColor = glm::mat4(0.999997, -0.001236, 0.002095, 0.026000,
		-0.001285, -0.999726, 0.023357, -0.000508,
		-0.002065, 0.023359, 0.999725, -0.000863,
		0, 0, 0, 1);
	//mDepthToColor = glm::rotate(mDepthToColor, 180.0f, glm::vec3(1.0f, 0.0f, 0.0f));
	//mDepthToColor = glm::transpose(mDepthToColor);

	runVolumeSlicing = false;
	initDepthState   = false;
	continuousMode   = false;
	volumeSliceStep  = 0.1f;
	runGoogleMap     = 0;
	runDeformScene   = false;
	runDeformText    = false;
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
	
	
	//pipeline->setProjection3D(pFovY, 16.0f / 9.0f, 0.1f, 1000.0f);
	//pipeline->setProjection3D(120, 16.0f / 9.0f, 0.01f, 1000.0f);
	//frustum = *pipeline->getProjectionMatrix();
	//pipeline->setProjectionMatrix(frustum);

	//pipeline->setProjection3D(110, aspect, 0.1f, 1000.0f);
	//pipeline->setProjection3D(15.23f, aspect, 0.1f, 1000.0f);
	pipeline->setProjection3D(15.23f, aspect, 0.1f, 1000.0f);
	//pipeline->setFrustum(left, right, bottom, top, nearP, farP);
	frustum = *pipeline->getProjectionMatrix();

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

	//projectorP = glm::perspective(30.5f, aspect, pNear, pFar);
	//projectorP = glm::perspective(-10.0f, aspect, pNear, pFar);
	projectorP = glm::perspective(11.4f, aspect, pNear, pFar);
	
	/*projectorP = glm::mat4(-2.86342, -0, -0, -0,
		0, 5.2496, 0, 0,
		0, 0, -1.0002, -1,
		0, 0, -0.20002, 0);*/
	
	//projectorP = glm::scale(projectorP, glm::vec3(-1.0f, 1.0f, 1.0f));
	//projectorP = glm::scale(projectorP, glm::vec3(1.0f, 1.0f, -1.0f));

	//projectorP = glm::scale(projectorP, glm::vec3(1.0f, -1.0f, 1.0f));
	
	//projectorP = glm::perspective(fovy, aspect, pNear, pFar);
	//projectorP = glm::ortho<float>(-100, 100, -100, 100, -100, 200);
	//projectorOrigin = glm::vec3(0.0f, -0.01f, 0.10f);
	//projectorOrigin = glm::vec3(0.0f, -0.015f, 0.1f);
	//projectorOrigin = glm::vec3(0.0f, -0.002f, 0.1f);
	//projectorOrigin = glm::vec3(0.0f, -0.002f, 0.1f);

	//projectorOrigin = glm::vec3(0.0f, 0.0f, -0.2f);
	
	//projectorOrigin = glm::vec3(0.0f, 0.1f, -2.0f);
	//projectorOrigin = glm::vec3(0.0f, 0.1f, -0.2f);

	//projectorOrigin = glm::vec3(0.0f, 0.0f, -0.2f);
	projectorOrigin = glm::vec3(0.0f, 0.0f, 3.0f);
	
	//projectorOrigin = glm::vec3(0.0f, 0.0f, 1.6f);
	
	//projectorOrigin = glm::vec3(0.0f, 0.0f, -0.5f);
	//projectorOrigin = glm::vec3(0.0f, 0.0f, 1.6f);

	//projectorOrigin = glm::vec3(0.0f, -0.002f, 0.1f);
	//projectorOrigin = glm::vec3(0.0f, -0.0f, 0.2f);
	//projectorOrigin = glm::vec3(0.0f, -0.01f, 0.002f);
	finalProjectorOrigin = glm::vec3(0.0f, 0.0f, -1.0f);
	projectorTarget = glm::vec3(0.0f, 0.0f, 0.0f);
	projectorV = glm::lookAt(projectorOrigin, // projector origin
		projectorTarget,	 // project on object at origin 
		glm::vec3(0.0f, 1.0f, 0.0f)   // Y axis is up
		);
	//projectorV = glm::inverse(projectorV);
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
		vboDebug.addData(&g_quad_vertex_buffer_data[i], sizeof(glm::vec3));
	}

	//vboDebug.uploadDataToGPU(GL_STATIC_DRAW);
	// compute frustum coords
	computeFrustumCoords(frustum);
	glEnableVertexAttribArray(0);
	//glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), 0);

	const char *cTextureNames[] = { "biohazard_512x512.jpg", "A_Smiley.jpg", "3dcarrush.jpg",
		"3D_Scene_011.jpg", "outdoor_free_3d_scene_by_djeric.jpg", "grid_512k.jpg",
		"grid-xxl.jpg", "grid-green.jpg", "tuscany.jpg", "fish-goldfish-texture-2048x2048.jpg", "random-text-2048x2048.jpg",
		"grid_with_corners.jpg", "googleMap1.jpg", "googleMap2.jpg", "3D_Scene_011_tut.jpg", "random-text-2048x2048_tut.jpg"};

	//char **cVolumeSliceNames = new  char*[100];
	const char *cVolumeSliceNames[99];
	string *s;
	char numstr[21]; // enough to hold all numbers up to 64-bits
	for (int i = 0; i < 99; i++)
	{
		if (i < 10)
		{
			
			s = new string("brain/brain_00" + string(_itoa(i, numstr, 10)) + ".jpg");
			//s.append(to_string(i));
			cout << s->c_str() << endl;
			//cVolumeSliceNames[i] = "brain/brain_00" + i;
			//cVolumeSliceNames[i] = s.c_str();
			
			//cVolumeSliceNames[i] = string("brain/brain_00" + string(_itoa(i, numstr, 10)) + '\0').c_str();
			//cVolumeSliceNames[i] = string("brain/brain_00" + string(_itoa(i, numstr, 10)) + '\0').c_str();
			cVolumeSliceNames[i] = s->c_str();

			//strcpy(cVolumeSliceNames[i], s.c_str());
			//cVolumeSliceNames[i] = malloc(15);
			//memcpy(cVolumeSliceNames[i], s.c_str(), strlen(s.c_str()) + 1);
		}
		else
		{
			s = new string("brain/brain_0" + string(_itoa(i, numstr, 10)) + ".jpg");
			//strcpy(cVolumeSliceNames[i], s.c_str());
			cVolumeSliceNames[i] = s->c_str();
			//cVolumeSliceNames[i] = "brain/brain_0" + i;
		}		
	}

	
	vector<string> sTextureNames(cTextureNames, &cTextureNames[sizeof(cTextureNames) / sizeof(cTextureNames[0])]);
	vector<string> sVolumeSliceTextureNames(cVolumeSliceNames, &cVolumeSliceNames[sizeof(cVolumeSliceNames) / sizeof(cVolumeSliceNames[0])]);
	sTextureNames.insert(sTextureNames.end(), sVolumeSliceTextureNames.begin(), sVolumeSliceTextureNames.end());
	loadAllTextures(sTextureNames);
	//loadAllTextures(sVolumeSliceTextureNames);

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
	/*if (currFrameIndex == nFrames)
	{
		glAccum(GL_RETURN, 1.0);
		currFrameIndex = 0;
	}
	else
	{
		if (currFrameIndex == 0)
		{
			glClear(GL_ACCUM_BUFFER_BIT);
		}
		glAccum(GL_ACCUM, 1/nFrames);
		currFrameIndex++;
	}*/

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
		int iQuadDebugLoc = glGetUniformLocation(programQuadID, "debug");

	

		// Bind our texture in Texture Unit 0
		tTextures[activeTextureIndex].bindTexture();
		// Set our "renderedTexture" sampler to user Texture Unit 0
		glUniform1i(texID, 0);
		glUniform1i(iQuadDebugLoc, true);

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
			mModelView = glm::scale(mModelView, glm::vec3(1.0f, 1.0f, -1.0f));
			if (bUseCalibParams)
			{
				//pipeline->setProjection3D(fovy, 16.0f / 9.0f, 0.1f, 1000.0f);
				//pipeline->setProjectionMatrix(glm::inverse(frustum));
				//pipeline->setProjectionMatrix(frustum);

				/*pipeline->setFrustum(left, right, bottom, top, nearP, farP);
				frustum = *pipeline->getProjectionMatrix();
				pipeline->setProjectionMatrix(glm::inverse(frustum));*/

				//pipeline->setProjection3D(fovy, aspect, 0.1f, 1000.0f);
				
				//pipeline->setFrustum(-frustum_width - offset_x, frustum_width - offset_x, -frustum_height - offset_y, frustum_height - offset_y, nearP, farP);
				
				//frustum = *pipeline->getProjectionMatrix();
				
				//frustum = glm::scale(frustum, glm::vec3(-1.0f, 1.0f, 1.0f));
				pipeline->setProjectionMatrix(frustum);
				mModelView = mExtrinsicMatrix;
			}
			else if (bUseCustomMatrices)
			{
				
				if (bMirrorView)
				{
					
				}
				else
				{
					if (bInitFrustum)
					{
						/*frustum = glm::mat4(2.68438, 0, 0, 0,
							0, 4.92136, 0, 0,
							0, 0, -1.0002, -1,
							0, 0, -0.20002, 0);*/
						frustum = glm::mat4(46.3599, 0, 0, 0,
							0, 81.5006, 0, 0,
							0, 0, -1.00002, -1,
							0, 0, -0.0200002, 0);

						//Y translation : 0.0254552
						//yTranslation = 0.0254552f;
						yTranslation = 0.0193368f;

						bInitFrustum = false;
					}

					pipeline->setProjectionMatrix(frustum);


					// OFX CALIBRATION
					/*
						Rotation_Vector: !!opencv-matrix
						   rows: 3
						   cols: 1
						   dt: d
						   data: [ 1.4087856618676450e-002, 1.3182076510281926e-001,
							   3.1296188643374316e+000 ]
						Translation_Vector: !!opencv-matrix
						   rows: 3
						   cols: 1
						   dt: d
						   data: [ 2.7346903470688839e+000, -6.8738947999420992e+000,
							   -2.3074002395299362e+000 ]
					*/
					projectorTranslation = glm::vec3(2.7346903470688839e+000, -6.8738947999420992e+000, -2.3074002395299362e+000);
					glm::vec3 rotationVector = glm::vec3(1.4087856618676450e-002, 1.3182076510281926e-001,
						3.1296188643374316e+000);
					projectorRotation = glm::mat3(-6.3130502809375055 * 0.1, 4.5280180168695242* 0.01, 7.7421164211566795* 0.01,
						-3.4976844531106721* 0.01, -9.9894065805775833 * 0.1, 2.9902876546890243 * 0.01,
						-7.7474549488860900 * 0.1, -8.2016439219341399 * 0.001, 6.3222001802124395 * 0.1);

					/*
						Rotation matrix computed using Rodrigues in Matlab from a vector
						M =
						-0.9999 - 0.0088    0.0094
						0.0095 - 0.9964    0.0840
						0.0086    0.0841    0.9964
					
						M'
						ans =
						   -0.9999    0.0095    0.0086
						   -0.0088   -0.9964    0.0841
							0.0094    0.0840    0.9964

					*/
					mModelView = glm::mat4(1.0)*glm::mat4(-0.9999, -0.0088, 0.0094, projectorTranslation.x,
						0.0095, -0.9964, 0.0840, projectorTranslation.y,
						0.0086, 0.0841, 0.9964, projectorTranslation.z, 
						0, 0, 0, 1);
					/*mModelView = glm::mat4(1.0)*glm::mat4(-0.9999, 0.0095, 0.0086, projectorTranslation.x,
						- 0.0088, -0.9964, 0.0841, projectorTranslation.y,
						0.0094,    0.0840,    0.9964, projectorTranslation.z,
						0, 0, 0, 1);*/
					mModelView = glm::rotate(mModelView, 180.0f, glm::vec3(1.0f, 0.0f, 0.0f));
					/*mModelView *= glm::mat4(
						1, 0, 0, 0,
						0, -1, 0, 0,
						0, 0, -1, 0,
						0, 0, 0, 1
						);*/
					
					mModelView = glm::inverse(mModelView);
					glm::mat4 mirrorMat = glm::mat4(-1.0f, 0.0f, 0.0f, 0.0f,
						0.0f, 1.0f, 0.0f, 0.0f,
						0.0f, 0.0f, 1.0f, 0.0f,
						0.0f, 0.0f, 0.0f, 1.0f);
					//projectorV = glm::scale(mModelView, glm::vec3(1, 1, -1));
					//projectorV = mModelView;
					//projectorV = mirrorMat*mModelView;
				
					
					
					// set virtual projector 
					/*projectorV = glm::mat4(-rotMat[0][0], -rotMat[0][1], -rotMat[0][2], -projectorTranslation.x,
						-rotMat[1][0], -rotMat[1][1], -rotMat[1][2], -projectorTranslation.y,
						-rotMat[2][0], -rotMat[2][1], -rotMat[2][2], -projectorTranslation.z,
						0, 0, 0, 1);
					
					projectorV = glm::rotate(projectorV, 180.0f, glm::vec3(1.0f, 0.0f, 0.0f));
					projectorV = glm::inverse(projectorV);*/


					// MANUAL STUFF AGAIN
					/*
						


						--===========================================--
						THE PERSPECTIVE MATRIX
						54.8712, 0, 0, 0
						0, 96.4634, 0, 0
						0, 0, -1.00002, -1
						0, 0, -0.0200002, 0

						THE MODELVIEW MATRIX
						0.999999, 0, -0.00159255, 0
						0, 1, 0, 0
						-0.00159255, -0, -0.999999, -0
						-0.0105029, 0.0158569, -22.0103, 1

						THE CURRENT MATRIX (TRANSFORMED MV)
						0.999999, 0, -0.00159255, 0
						0, 1, 0, 0
						-0.00159255, 0, -0.999999, 0
						-0.0105029, 0.0158569, -22.0103, 1

						PROJECTOR T + R:
						X translation: 0
						Y translation: 0
						Z translation: 0
						X rotation: 0
						Y rotation: 0
						Z rotation: 0
						--===========================================--


						--===========================================--
						THE PERSPECTIVE MATRIX
						46.7665, 0, 0, 0
						0, 82.2152, 0, 0
						0, 0, -1.00002, -1
						0, 0, -0.0200002, 0

						THE MODELVIEW MATRIX
						0.999999, 0, -0.00159255, 0
						0, 1, 0, 0
						-0.00159255, -0, -0.999999, -0
						0.000967637, 0.0260353, -18.3939, 1

						THE CURRENT MATRIX (TRANSFORMED MV)
						0.999999, 0, -0.00159255, 0
						0, 1, 0, 0
						-0.00159255, 0, -0.999999, 0
						0.000967637, 0.0260353, -18.3939, 1

						PROJECTOR T + R:
						X translation: 0
						Y translation: 0
						Z translation: 0
						X rotation: 0
						Y rotation: 0
						Z rotation: 0
						--===========================================--


						// still not correct...
						--===========================================--
						THE PERSPECTIVE MATRIX
						46.3091, 0, 0, 0
						0, 81.4112, 0, 0
						0, 0, -1.00002, -1
						0, 0, -0.0200002, 0

						THE MODELVIEW MATRIX
						0.999999, 0, -0.00159255, 0
						0, 1, 0, 0
						-0.00159255, -0, -0.999999, -0
						-0.00159261, 0.0265184, -18.2745, 1

						THE CURRENT MATRIX (TRANSFORMED MV)
						0.999999, 0, -0.00159255, 0
						0, 1, 0, 0
						-0.00159255, 0, -0.999999, 0
						-0.00159261, 0.0265184, -18.2745, 1

						PROJECTOR T + R:
						X translation: -0.00392025
						Y translation: 0.0267205
						Z translation: 0
						X rotation: 0
						Y rotation: 0
						Z rotation: 0
						--===========================================--

						better? still not aligned :|
						--===========================================--
						THE PERSPECTIVE MATRIX
						46.3599, 0, 0, 0
						0, 81.5006, 0, 0
						0, 0, -1.00002, -1
						0, 0, -0.0200002, 0

						THE MODELVIEW MATRIX
						0.999999, 0, -0.00159255, 0
						0, 1, 0, 0
						-0.00159255, -0, -0.999999, -0
						0.00284801, 0.0184056, -18.2686, 1

						THE CURRENT MATRIX (TRANSFORMED MV)
						0.999999, 0, -0.00159255, 0
						0, 1, 0, 0
						-0.00159255, 0, -0.999999, 0
						0.00284801, 0.0184056, -18.2686, 1

						PROJECTOR T + R:
						X translation: 0
						Y translation: 0.0193368
						Z translation: 0
						X rotation: 0
						Y rotation: 0
						Z rotation: 0
						--===========================================--
					*/
					mModelView = glm::mat4(0.999999, 0, -0.00159255, 0,
						0, 1, 0, 0,
						- 0.00159255, -0, -0.999999, -0,
						0.00284801, 0.0184056, -18.2686, 1);

					
					projectorV = glm::translate(projectorV, glm::vec3(0, yTranslation, 0));
					yTranslation = 0;
				}
			}
			else
			{
				//pipeline->setProjection3D(pFovY, aspect, nearP, farP);
				//frustum = *pipeline->getProjectionMatrix();
				
				//frustum = glm::scale(frustum, glm::vec3(-1.0f, 1.0f, 1.0f));
				//frustum = glm::scale(frustum, glm::vec3(1.0f, -1.0f, 1.0f));
				
				//pipeline->setProjectionMatrix(frustum);
			}
			//mModelView = calibratedView;
			
			if (bMirrorView)
			{
				//glm::mat4* projMatrix = pipeline->getProjectionMatrix();
				//*projMatrix = glm::scale(*projMatrix, glm::vec3(1.0f, -1.0f, 1.0f));
				//*projMatrix = glm::scale(*projMatrix, glm::vec3(-1.0f, 1.0f, 1.0f));
				//pipeline->setProjectionMatrix(*projMatrix);

				//frustum = glm::scale(frustum, glm::vec3(1.0f, 1.0f, -1.0f));
				//pipeline->setProjectionMatrix(frustum);
				mModelView = glm::scale(mModelView, glm::vec3(1.0f, 1.0f, -1.0f));
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

	
		// apply translation to rotated camera
		translationMatrix = glm::mat4(1.0f);
		translationMatrix = glm::translate(translationMatrix, glm::vec3(xTranslationCam, 0.0f, zTranslationCam));

		// apply translation to rotation
		// apply model view to rotated and translated camera
		mModelView = rotationMatrix * translationMatrix * mModelView;


		mCurrent = mModelView;
		//mCurrent = glm::scale(mCurrent, glm::vec3(-1.0f, 1.0f, 1.0f));
		//mCurrent = glm::translate(mModelView, glm::vec3(0.0f, 0.0f, 0.0f));
		//mCurrent = glm::scale(mCurrent, glm::vec3(400.0f, 400.0f, 400.0f));
		//mCurrent = glm::translate(mModelView, glm::vec3(0.0f, 0.0f, -1.0f));
		//mCurrent = glm::rotate(mCurrent, 180.0f, glm::vec3(-1.0f, 0.0f, 0.0f));
		
		//mCurrent = glm::rotate(mCurrent, -180.0f, glm::vec3(1.0f, 0.0f, 0.0f));
		
		// also allow rotation
		mCurrent = glm::rotate(mCurrent, xRotation, glm::vec3(1.0f, 0.0f, 0.0f));
		mCurrent = glm::rotate(mCurrent, yRotation, glm::vec3(0.0f, 1.0f, 0.0f));
		mCurrent = glm::rotate(mCurrent, zRotation, glm::vec3(0.0f, 0.0f, 1.0f));

		//mCurrent = glm::translate(mCurrent, glm::vec3(xTranslation, yTranslation, zTranslation));
		projectorV = glm::translate(projectorV, glm::vec3(xTranslation, yTranslation, zTranslation));
		xTranslation = 0;
		yTranslation = 0;
		zTranslation = 0;
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
			int iKRGBLoc = glGetUniformLocation(programID, "kRGB");
			int iDepthInvLoc = glGetUniformLocation(programID, "kDepthInv");
			int iDepthToColorLoc = glGetUniformLocation(programID, "depthToColor");
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


			glUniformMatrix4fv(iKRGBLoc, 1, GL_FALSE, glm::value_ptr(kRGB));
			glUniformMatrix4fv(iDepthInvLoc, 1, GL_FALSE, glm::value_ptr(glm::inverse(kDepth)));
			glUniformMatrix4fv(iDepthToColorLoc, 1, GL_FALSE, glm::value_ptr(mDepthToColor));

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
	// RUN TASKS HERE
	if (runDeformText)
	{
		float depthDiff;
		if (bDebugTexture)
		{
			// fails...problem, don't know how many points actually made it to array
			// cannot properly get map from center
			//float wpd = rdp->worldPos.at(320 * 320/2 + 240/2).z;

			// corresponds roughly to the middle of the depth stream
			float wpd = rdp->centerDepth;
			//cout << "World Pos Depth: " << wpd << endl;
			//wpd /= 1000.0f; // convert to metric space

			if (initDepthState)
			{
				initDepth = wpd;
				initDepthState = false;

				cout << "INIT DEPTH:" << initDepth << endl;
			}

			float depth = rdp->centerDepth;
			//cout << "depth = " << depth << " at pixel (" << 1920 / 2 << ", " << 1080 / 2 << ")" << endl;
			depthDiff = initDepth - depth;

			// convert the step to metric space(the point was not converted initially)
			if (depthDiff > volumeSliceStep * 1000)
			{

				if (activeTextureIndex == 15)
					activeTextureIndex = 10;
			}
			else if (depthDiff < -volumeSliceStep * 1000)
			{
				if (activeTextureIndex == 10)
					activeTextureIndex = 15;
			}
		}
		else
		{
			// normal mesh projection
			if (initDepthState)
			{
				initDepth = getGLDepth(1920 / 2, 1080 / 2, mModelView, frustum);
				initDepthState = false;

				cout << "INIT DEPTH:" << initDepth << endl;
			}

			float depth = getGLDepth(1920 / 2, 1080 / 2, mModelView, frustum);
			//cout << "depth = " << depth << " at pixel (" << 1920 / 2 << ", " << 1080 / 2 << ")" << endl;
			depthDiff = initDepth - depth;

			if (depthDiff > volumeSliceStep)
			{
				if (activeTextureIndex == 15)
					activeTextureIndex = 10;
			}
			else if (depthDiff < -volumeSliceStep)
			{
				if (activeTextureIndex == 10)
					activeTextureIndex = 15;
			}
		}
	}
	
	if (runDeformScene)
	{
		float depthDiff;
		if (bDebugTexture)
		{
			// fails...problem, don't know how many points actually made it to array
			// cannot properly get map from center
			//float wpd = rdp->worldPos.at(320 * 320/2 + 240/2).z;

			// corresponds roughly to the middle of the depth stream
			float wpd = rdp->centerDepth;
			//cout << "World Pos Depth: " << wpd << endl;
			//wpd /= 1000.0f; // convert to metric space

			if (initDepthState)
			{
				initDepth = wpd;
				initDepthState = false;

				cout << "INIT DEPTH:" << initDepth << endl;
			}

			float depth = rdp->centerDepth;
			//cout << "depth = " << depth << " at pixel (" << 1920 / 2 << ", " << 1080 / 2 << ")" << endl;
			depthDiff = initDepth - depth;

			// convert the step to metric space(the point was not converted initially)
			if (depthDiff > volumeSliceStep * 1000)
			{

				if (activeTextureIndex == 14)
					activeTextureIndex = 3;
			}
			else if (depthDiff < -volumeSliceStep * 1000)
			{
				if (activeTextureIndex == 3)
					activeTextureIndex = 14;
			}
		}
		else
		{
			// normal mesh projection
			if (initDepthState)
			{
				initDepth = getGLDepth(1920 / 2, 1080 / 2, mModelView, frustum);
				initDepthState = false;

				cout << "INIT DEPTH:" << initDepth << endl;
			}

			float depth = getGLDepth(1920 / 2, 1080 / 2, mModelView, frustum);
			//cout << "depth = " << depth << " at pixel (" << 1920 / 2 << ", " << 1080 / 2 << ")" << endl;
			depthDiff = initDepth - depth;

			if (depthDiff > volumeSliceStep)
			{
				if (activeTextureIndex == 14)
					activeTextureIndex = 3;
			}
			else if (depthDiff < -volumeSliceStep)
			{
				if (activeTextureIndex == 3)
					activeTextureIndex = 14;
			}
		}
	}
	
	if (runGoogleMap)
	{
		float depthDiff;
		if (bDebugTexture)
		{
			// fails...problem, don't know how many points actually made it to array
			// cannot properly get map from center
			//float wpd = rdp->worldPos.at(320 * 320/2 + 240/2).z;

			// corresponds roughly to the middle of the depth stream
			float wpd = rdp->centerDepth;
			//cout << "World Pos Depth: " << wpd << endl;
			//wpd /= 1000.0f; // convert to metric space

			if (initDepthState)
			{
				initDepth = wpd;
				initDepthState = false;

				cout << "INIT DEPTH:" << initDepth << endl;
			}

			float depth = rdp->centerDepth;
			//cout << "depth = " << depth << " at pixel (" << 1920 / 2 << ", " << 1080 / 2 << ")" << endl;
			depthDiff = initDepth - depth;

			// convert the step to metric space(the point was not converted initially)
			if (depthDiff > volumeSliceStep * 1000)
			{

				if (activeTextureIndex == 12)
					activeTextureIndex++;
			}
			else if (depthDiff < -volumeSliceStep * 1000)
			{
				if (activeTextureIndex == 13)
					activeTextureIndex--;
			}
		}
		else
		{
			// normal mesh projection
			if (initDepthState)
			{
				initDepth = getGLDepth(1920 / 2, 1080 / 2, mModelView, frustum);
				initDepthState = false;

				cout << "INIT DEPTH:" << initDepth << endl;
			}

			float depth = getGLDepth(1920 / 2, 1080 / 2, mModelView, frustum);
			//cout << "depth = " << depth << " at pixel (" << 1920 / 2 << ", " << 1080 / 2 << ")" << endl;
			depthDiff = initDepth - depth;

			if (depthDiff > volumeSliceStep)
			{
				if (activeTextureIndex == 12)
					activeTextureIndex++;
			}
			else if (depthDiff < -volumeSliceStep)
			{
				if (activeTextureIndex == 13)
					activeTextureIndex--;
			}
		}
	}

	if (runVolumeSlicing)
	{
		float depthDiff;
		if (bDebugTexture)
		{
			// fails...problem, don't know how many points actually made it to array
			// cannot properly get map from center
			//float wpd = rdp->worldPos.at(320 * 320/2 + 240/2).z;

			// corresponds roughly to the middle of the depth stream
			float wpd = rdp->centerDepth;
			//cout << "World Pos Depth: " << wpd << endl;
			//wpd /= 1000.0f; // convert to metric space

			if (initDepthState)
			{
				initDepth = wpd;
				initDepthState = false;

				cout << "INIT DEPTH:" << initDepth << endl;
			}

			float depth = rdp->centerDepth;
			//cout << "depth = " << depth << " at pixel (" << 1920 / 2 << ", " << 1080 / 2 << ")" << endl;
			depthDiff = initDepth - depth;
			
			// convert the step to metric space(the point was not converted initially)
			if (depthDiff > volumeSliceStep*1000)
			{
				if (continuousMode)
				{
					initDepth = depth;
				}

				if (activeTextureIndex < 15 + 98)
					activeTextureIndex++;
			}
			else if (depthDiff < -volumeSliceStep*1000)
			{
				if (continuousMode)
				{
					initDepth = depth;
				}
				if (activeTextureIndex > 15)
					activeTextureIndex--;
			}
		}
		else
		{
			// normal mesh projection
			if (initDepthState)
			{
				initDepth = getGLDepth(1920 / 2, 1080 / 2, mModelView, frustum);
				initDepthState = false;

				cout << "INIT DEPTH:" << initDepth << endl;
			}

			float depth = getGLDepth(1920 / 2, 1080 / 2, mModelView, frustum);
			//cout << "depth = " << depth << " at pixel (" << 1920 / 2 << ", " << 1080 / 2 << ")" << endl;
			depthDiff = initDepth - depth;

			if (depthDiff > volumeSliceStep)
			{
				if (continuousMode)
				{
					initDepth = depth;
				}

				if (activeTextureIndex < 15 + 98)
					activeTextureIndex++;
			}
			else if (depthDiff < -volumeSliceStep)
			{
				if (continuousMode)
				{
					initDepth = depth;
				}
				if (activeTextureIndex > 15)
					activeTextureIndex--;
			}
		}
		
	}

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
			// reset camera
			setResetMouseLook(true);
			xTranslationCam = 0;
			yTranslationCam = 0;
			zTranslationCam = 0;
			xRotationCam = 0;
			yRotationCam = 0;
			zRotationCam = 0;

			// reset projector
			xTranslation = 0;
			yTranslation = 0;
			zTranslation = 0;
			xRotation = 0;
			yRotation = 0;
			zRotation = 0;


			bInitFrustum = true;
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
			cout << endl;
			cout << "PROJECTOR T + R:" << endl;
			cout << "X translation: " << xTranslationPrjTotal << endl;
			cout << "Y translation: " << yTranslationPrjTotal << endl;
			cout << "Z translation: " << zTranslationPrjTotal << endl;
			cout << "X rotation: " << xRotation << endl;
			cout << "Y rotation: " << yRotation << endl;
			cout << "Z rotation: " << zRotation << endl;
			cout << "--===========================================--" << endl << endl;
			timer = 0;
		}
	}

	if (timer >= 0.25f)
	{
		if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_T) != GLFW_RELEASE)
		{
			bDebugTexture = !bDebugTexture;

			savedMV = mModelView;
			savedP  = frustum;

			// RESET STATES
			if (runVolumeSlicing || runGoogleMap || runDeformScene)
			{
				if (runVolumeSlicing)
					activeTextureIndex = 49 + 16;
				if (runGoogleMap)
					activeTextureIndex = 12;
				if (runDeformScene)
					activeTextureIndex = 14;
				initDepthState = true;
				initDepth = 0;
			}


			

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

			pFovY += getDeltaTime();
			pipeline->setProjection3D(pFovY, aspect, nearP, farP);
			frustum = *pipeline->getProjectionMatrix();
			//frustum = glm::scale(frustum, glm::vec3(-1.0f, 1.0f, 1.0f));
			pipeline->setProjectionMatrix(frustum);
			
		}



		if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_I) != GLFW_RELEASE)
		{

			pFovY -= getDeltaTime();
			pipeline->setProjection3D(pFovY, aspect, nearP, farP);
			frustum = *pipeline->getProjectionMatrix();
			//frustum = glm::scale(frustum, glm::vec3(-1.0f, 1.0f, 1.0f));
			pipeline->setProjectionMatrix(frustum);
		
		}

		if (glfwGetKey(window, GLFW_KEY_INSERT) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_INSERT) != GLFW_RELEASE)
		{
			//	zRotation += 3.14f / 180 * getDeltaTime();
			zTranslation += getDeltaTime() / 10;
			zTranslationPrjTotal += zTranslation;
		}

		if (glfwGetKey(window, GLFW_KEY_DELETE) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_DELETE) != GLFW_RELEASE)
		{
			//	zRotation -= 3.14f / 180 * getDeltaTime();
			zTranslation -= getDeltaTime() / 10;
			zTranslationPrjTotal += zTranslation;
		}

		if (glfwGetKey(window, GLFW_KEY_PAGE_UP) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_PAGE_UP) != GLFW_RELEASE)
		{
			//	zRotation += 3.14f / 180 * getDeltaTime();
			yTranslation += getDeltaTime()/10;
			yTranslationPrjTotal += yTranslation;
		}

		if (glfwGetKey(window, GLFW_KEY_PAGE_DOWN) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_PAGE_DOWN) != GLFW_RELEASE)
		{
			//	zRotation -= 3.14f / 180 * getDeltaTime();
			yTranslation -= getDeltaTime() / 10;
			yTranslationPrjTotal += yTranslation;
		}

		if (glfwGetKey(window, GLFW_KEY_HOME) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_HOME) != GLFW_RELEASE)
		{
		//	zRotation += 3.14f / 180 * getDeltaTime();
			xTranslation -= getDeltaTime() / 10;
			xTranslationPrjTotal += xTranslation;
		}

		if (glfwGetKey(window, GLFW_KEY_END) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_END) != GLFW_RELEASE)
		{
		//	zRotation -= 3.14f / 180 * getDeltaTime();
			xTranslation += getDeltaTime() / 10;
			xTranslationPrjTotal += xTranslation;
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

		if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
		{
			zTranslationCam += getDeltaTime();
		}

		if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
		{
			zTranslationCam -= getDeltaTime();
		}

		if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
		{
			xTranslationCam += getDeltaTime();
		}

		if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
		{
			xTranslationCam -= getDeltaTime();
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

		// NEED 6 KEYS
		if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_C) != GLFW_RELEASE)
		{
			xRotation += 3.14f / 180 * getDeltaTime();
		}

		if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_Z) != GLFW_RELEASE)
		{
			xRotation -= 3.14f / 180 * getDeltaTime();
		}

		if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_B) != GLFW_RELEASE)
		{
			yRotation += 3.14f / 180 * getDeltaTime();
		}

		if (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_N) != GLFW_RELEASE)
		{
			yRotation -= 3.14f / 180 * getDeltaTime();
		}

		if (glfwGetKey(window, GLFW_KEY_COMMA) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_COMMA) != GLFW_RELEASE)
		{
			zRotation += 3.14f / 180 * getDeltaTime();
		}

		if (glfwGetKey(window, GLFW_KEY_PERIOD) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_PERIOD) != GLFW_RELEASE)
		{
			zRotation -= 3.14f / 180 * getDeltaTime();
		}

		if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_L) != GLFW_RELEASE)
		{
			float depth = getGLDepth(1920 / 2, 1080 / 2, mModelView, frustum);
			cout << "depth = " << depth << " at pixel (" << 1920 / 2 << ", " << 1080 / 2 << ")" << endl;
		}

		if (timer >= 0.25f)
		{
			if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_L) != GLFW_RELEASE)
			{
				float depth = getGLDepth(1920 / 2, 1080 / 2, mModelView, frustum);
				cout << "depth = " << depth << " at pixel (" << 1920 / 2 << ", " << 1080 / 2 << ")" << endl;
			
				timer = 0;
			}
		}

		// TASK 1
		if (timer >= 0.25f)
		{

			if (glfwGetKey(window, GLFW_KEY_F1) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_F1) != GLFW_RELEASE)
			{
				randomizeDistortionState(100);
				// make sure we do not run other states
				runGoogleMap = false;
				runVolumeSlicing = false;
				runDeformScene = false;

				//init
				activeTextureIndex = 15;
				initDepthState = true;
				initDepth = 0;
				runDeformText = true;

				timer = 0;
			}
		}

		// TASK 2
		if (timer >= 0.25f)
		{

			if (glfwGetKey(window, GLFW_KEY_F2) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_F2) != GLFW_RELEASE)
			{
				randomizeDistortionState(100);
				// make sure we do not run other states
				runGoogleMap = false;
				runVolumeSlicing = false;
				runDeformText = false;

				//init
				activeTextureIndex = 14;
				initDepthState = true;
				initDepth = 0;
				runDeformScene = true;

				timer = 0;
			}
		}

		// TASK 3
		if (timer >= 0.25f)
		{

			if (glfwGetKey(window, GLFW_KEY_F3) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_F3) != GLFW_RELEASE)
			{
				randomizeDistortionState(100);
				// make sure we do not run other states
				runDeformScene = false;
				runVolumeSlicing = false;
				runDeformText = false;

				//init
				activeTextureIndex = 12;
				initDepthState = true;
				initDepth = 0;
				runGoogleMap = true;

				timer = 0;
			}
		}

		// TASK 4
		if (timer >= 0.25f)
		{
			
			if (glfwGetKey(window, GLFW_KEY_F4) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_F4) != GLFW_RELEASE)
			{
				randomizeDistortionState(100);
				// make sure we do not run other states
				runDeformScene = false;
				runGoogleMap = false;
				runDeformText = false;

				//init
				activeTextureIndex = 49 + 16;
				initDepthState = true;
				initDepth = 0;
				runVolumeSlicing = true;

				timer = 0;
			}
		}

		// NO TASK / DEFAULT TEXTURE
		if (timer >= 0.25f)
		{

			if (glfwGetKey(window, GLFW_KEY_F5) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_F5) != GLFW_RELEASE)
			{
				// do not need to randomize in this case
				bDebugTexture = false;

				// make sure we do not run ANY states
				runDeformScene = false;
				runGoogleMap = false;
				runVolumeSlicing = false;
				runDeformText = false;

				//init
				activeTextureIndex = 0;
				initDepthState = true;
				initDepth = 0;
				

				timer = 0;
			}
		}

		if (timer >= 0.25f)
		{

			if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_J) != GLFW_RELEASE)
			{
				continuousMode = !continuousMode;

				if (continuousMode)
				{
					volumeSliceStep = 0.01f;
				}
				else
					volumeSliceStep = 0.1f;

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

GLfloat PointCloudPTMScene::getGLDepth(int x, int y, glm::mat4 mModelView, glm::mat4 frustum)
{
	float depth_z = 0.0f;

	glReadBuffer(GL_BACK);
	glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth_z);

	// value is between 0 and 1 at this stage, unproject it
	glm::vec3 final = glm::unProject(glm::vec3(x, y, depth_z), mModelView, frustum, glm::vec4(0, 0, 1920, 1080));

	//return depth_z;
	//return final.z * 1000;
	return final.z;
}

int PointCloudPTMScene::getRandomNumber(int n)
{
	return rand() % n;
}

void PointCloudPTMScene::randomizeDistortionState(int bound)
{
	// seed not used
	// it will run the same way each time it starts,
	// but this should be fine
	int threshold = getRandomNumber(bound);

	if (threshold <= 50)
		bDebugTexture = false;
	else
		bDebugTexture = true;
}