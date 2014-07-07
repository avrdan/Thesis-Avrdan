#include "pipeline.h"
#include <iostream>

Pipeline::Pipeline()
{
	iFPSCount = 0;
	iCurrentFPS = 0;
	totalFrameInterval = 0;
}

void Pipeline::setProjection3D(float fFOV, float fAspectRatio, float fNear, float fFar)
{
	mProjection = glm::perspective(fFOV, fAspectRatio, fNear, fFar);
}

void Pipeline::setProjectionMatrix(glm::mat4 mProjection)
{
	this->mProjection = mProjection;
}


void Pipeline::setFrustum(float left, float right, float bottom, float top, float nearP, float farP)
{
	mProjection = glm::frustum(left, right, bottom, top, nearP, farP);
}


glm::mat4* Pipeline::getProjectionMatrix()
{
	return &mProjection;
}

void Pipeline::resetTimer()
{
	tLastFrame = clock();
	fFrameInterval = 0.0f;
	iFPSCount = 0;
	iCurrentFPS = 0;
}

void Pipeline::updateTimer(bool show)
{
	iFPSCount++;
	clock_t tCur = clock();
	fFrameInterval = float(tCur - tLastFrame) / float(CLOCKS_PER_SEC);
	tLastFrame = tCur;
	
	
	totalFrameInterval += fFrameInterval;

	if (totalFrameInterval > 1)
	{
		//std::cout << "iFPSCount: " << iFPSCount << std::endl;
		//  calculate the number of frames per second
		//iCurrentFPS = iFPSCount / (totalFrameInterval / 1000.0f);
		iCurrentFPS = iFPSCount / (totalFrameInterval);

		if (show)
			std::cout << "FPS: " << iCurrentFPS << std::endl;

		//  Reset frame count
		iFPSCount = 0;
		totalFrameInterval = 0;
	}
}

int Pipeline::getFPS()
{
	return iCurrentFPS;
}

// speed optimized float
float Pipeline::sof(float fVal)
{
	return fVal*fFrameInterval;
}