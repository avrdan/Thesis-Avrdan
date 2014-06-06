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
float getDeltaTime();
glm::vec3 getLookVector();
glm::vec3 getEyeVector();
bool isResetMouseLook();
bool isStartMouseLook();
void setResetMouseLook(bool rml);
void setStartMouseLook(bool sml);
void increaseSpeedFactor();
void decreaseSpeedFactor();

#endif // CONTROLS_H
