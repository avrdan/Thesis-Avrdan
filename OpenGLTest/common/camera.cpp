#include "camera.h"

const float PI = float(atan(1.0)*4.0);

Camera::Camera(Pipeline *a_pipeline)
{
	vEye = glm::vec3(0.0f, 0.0f, 0.0f);
	vView = glm::vec3(0.0f, 0.0, -1.0f);
	vUp = glm::vec3(0.0f, 1.0f, 0.0f);
	fSpeed = 25.0f;
	fSensitivity = 0.1f;

	pipeline = a_pipeline;
}

Camera::Camera(Pipeline *a_pipeline, glm::vec3 a_vEye, glm::vec3 a_vView, glm::vec3 a_vUp, float a_fSpeed, float a_fSensitivity)
{
	vEye = a_vEye; vView = a_vView; vUp = a_vUp;
	fSpeed = a_fSpeed;
	fSensitivity = a_fSensitivity;
	pipeline = a_pipeline;
}


Camera::~Camera()
{
}

/*-----------------------------------------------

Name:	GetAngleY

Params:	none

Result:	Gets Y angle of camera (head turning left
and right).

/*---------------------------------------------*/

float Camera::GetAngleY()
{
	glm::vec3 vDir = vView - vEye; vDir.y = 0.0f;
	glm::normalize(vDir);
	// angle between forward and current left/right dir
	float fAngle = acos(glm::dot(glm::vec3(0, 0, -1), vDir))*(180.0f / PI);
	if (vDir.x < 0)fAngle = 360.0f - fAngle;
	return fAngle;
}

/*-----------------------------------------------

Name:	GetAngleX

Params:	none

Result:	Gets X angle of camera (head turning up
and down).

/*---------------------------------------------*/

float Camera::GetAngleX()
{
	glm::vec3 vDir = vView - vEye;
	vDir = glm::normalize(vDir);
	glm::vec3 vDir2 = vDir; vDir2.y = 0.0f;
	vDir2 = glm::normalize(vDir2);
	float fAngle = acos(glm::dot(vDir2, vDir))*(180.0f / PI);
	if (vDir.y < 0)fAngle *= -1.0f;
	return fAngle;
}

/*-----------------------------------------------

Name:	Look

Params:	none

Result:	Returns proper modelview matrix to make
camera look.

/*---------------------------------------------*/

glm::mat4 Camera::Look()
{
	return glm::lookAt(vEye, vView, vUp);
}

/*-----------------------------------------------

Name:	rotateWithMouse

Params:	none

Result:	Checks for moving of mouse and rotates
camera.

/*---------------------------------------------*/

void Camera::RotateWithMouse(GLFWwindow *window)
{
	// Get mouse position
	glfwGetCursorPos(window, &xpos, &ypos);
	glfwGetWindowSize(window, &width, &height);
	glfwGetWindowPos(window, &wX, &wY);


	int iCentX = (wX + width) / 2;
	int iCentY = (wY + height) / 2;

	float deltaX = (float)(iCentX - xpos)*fSensitivity;
	float deltaY = (float)(iCentY - ypos)*fSensitivity;

	if (deltaX != 0.0f)
	{
		vView -= vEye;
		vView = glm::rotate(vView, deltaX, glm::vec3(0.0f, 1.0f, 0.0f));
		
		vView += vEye;
	}
	if (deltaY != 0.0f)
	{
		glm::vec3 vAxis = glm::cross(vView - vEye, vUp);
		vAxis = glm::normalize(vAxis);
		float fAngle = deltaY;
		float fNewAngle = fAngle + GetAngleX();
		if (fNewAngle > -89.80f && fNewAngle < 89.80f)
		{
			vView -= vEye;
			vView = glm::rotate(vView, deltaY, vAxis);
			vView += vEye;
		}
	}


	glfwSetCursorPos(window, iCentX, iCentY);
}

/*-----------------------------------------------

Name:	ResetMouse

Params:	none

Result:	Sets mouse cursor back to the center of
window.

/*---------------------------------------------*/
void Camera::ResetMouse(GLFWwindow *window)
{
	glfwGetWindowSize(window, &width, &height);
	glfwGetWindowPos(window, &wX, &wY);


	int iCentX = (wX + width) / 2;
	int iCentY = (wY + height) / 2;

	glfwSetCursorPos(window, iCentX, iCentY);
}

/*-----------------------------------------------

Name:	Update

Params:	none

Result:	Performs updates of camera - moving and
rotating.

/*---------------------------------------------*/

void Camera::Update(GLFWwindow *window)
{
	RotateWithMouse(window);

	// Get view direction
	glm::vec3 vMove = vView - vEye;
	vMove = glm::normalize(vMove);
	vMove *= fSpeed;

	glm::vec3 vStrafe = glm::cross(vView - vEye, vUp);
	glm::vec3 vFloat  = glm::cross(vView - vEye, vMove);
	vStrafe = glm::normalize(vStrafe);
	vStrafe *= fSpeed;

	int iMove = 0;
	glm::vec3 vMoveBy;
	// Get vector of move

	// Move forward
	if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS ||
		glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
	{
		vMoveBy += vMove*pipeline->sof(1.0f);
		// cout << position.x << "," << position.y << "," << position.z << endl;
		//cout << "forward" << endl;
	}

	// Move backward
	if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS ||
		glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
	{
		vMoveBy -= vMove*pipeline->sof(1.0f);
		//cout << position.x << "," << position.y << "," << position.z << endl;
		//cout << "backward" << endl;
	}

	// Strafe right
	if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS ||
		glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
	{
		vMoveBy += vStrafe*pipeline->sof(1.0f);
		//cout << position.x << "," << position.y << "," << position.z << endl;
		//cout << "right" << endl;
	}

	// Strafe left
	if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS ||
		glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
	{
		vMoveBy -= vStrafe*pipeline->sof(1.0f);
		//cout << position.x << "," << position.y << "," << position.z << endl;
		//cout << "left" << endl;
	}

	// Move up
	if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
	{

		vMoveBy += vFloat*pipeline->sof(1.0f);
		//cout << position.x << "," << position.y << "," << position.z << endl;
		//cout << "up" << endl;
	}

	// Move down
	if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
	{
		vMoveBy -= vFloat*pipeline->sof(1.0f);
		//cout << position.x << "," << position.y << "," << position.z << endl;
		//cout << "down" << endl;
	}

	vEye += vMoveBy; vView += vMoveBy;
}
