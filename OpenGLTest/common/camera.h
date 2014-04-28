#pragma once

// window lib
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>

// glm extensions
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>

#include "pipeline.h"

class Camera
{
public:
	Camera(Pipeline *pipeline);
	Camera(Pipeline *pipeline, glm::vec3 a_vEye, glm::vec3 a_vView, glm::vec3 a_vUp, float a_fSpeed, float a_fSensitivity);
	~Camera();

	glm::vec3 vEye, vView, vUp;
	float fSpeed;
	float fSensitivity; // degrees to rotate per pixel

	void RotateWithMouse(GLFWwindow *window);
	void Update(GLFWwindow *window);
	glm::mat4 Look();

	void ResetMouse(GLFWwindow *window);

	float GetAngleX(), GetAngleY();
private:
	double xpos, ypos;
	int width, height;
	int wX, wY;
	Pipeline *pipeline;
};

