#include "pipeline.h"

Pipeline::Pipeline()
{
	iFPSCount = 0;
	iCurrentFPS = 0;
}

void Pipeline::setProjection3D(float fFOV, float fAspectRatio, float fNear, float fFar)
{
	mProjection = glm::perspective(fFOV, fAspectRatio, fNear, fFar);
}

glm::mat4* Pipeline::getProjectionMatrix()
{
	return &mProjection;
}

void Pipeline::resetTimer()
{
	tLastFrame = clock();
	fFrameInterval = 0.0f;
}

void Pipeline::updateTimer()
{
	clock_t tCur = clock();
	fFrameInterval = float(tCur - tLastFrame) / float(CLOCKS_PER_SEC);
	tLastFrame = tCur;
}

// speed optimized float
float Pipeline::sof(float fVal)
{
	return fVal*fFrameInterval;
}