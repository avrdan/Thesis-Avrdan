#ifndef CONTROLS_H
#define CONTROLS_H


// window lib
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>

// glm extensions
#include <glm/gtc/matrix_transform.hpp>

void computeMatricesFromInputs(GLFWwindow* window);
glm::mat4 getProjectionMatrix();
glm::mat4 getViewMatrix();

#endif // CONTROLS_H
