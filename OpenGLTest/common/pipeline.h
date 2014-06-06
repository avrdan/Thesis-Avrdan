#ifndef PIPELINE_H
#define PIPELINE_H

// OGL via glew
//#include <GL/glew.h>
// include GLFW as the windowing context handling library
#include <GLFW/glfw3.h>
// include GLM math library
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <ctime>

class Pipeline
{
public:
	void setProjection3D(float fFOV, float fAspectRatio, float fNear, float fFar);
	void Pipeline::setFrustum(float left, float right, float bottom, float top, float near, float far);
	void Pipeline::setProjectionMatrix(glm::mat4 mProjection);
	glm::mat4* getProjectionMatrix();

	void resetTimer();
	void updateTimer();
	// speed optimized float
	float sof(float fVal);

	bool setVerticalSynchronization(bool bEnabled);
	int getFPS();

	Pipeline();
private:
	int iFPSCount, iCurrentFPS;
	clock_t tLastSecond;

	glm::mat4 mProjection;

	clock_t tLastFrame;
	float fFrameInterval;
};

#endif // PIPELINE_H