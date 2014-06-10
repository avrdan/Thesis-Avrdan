#include "controls.h"

#include <stdlib.h>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace glm;

mat4 ViewMatrix;
mat4 ProjectionMatrix;
float deltaTime = 0;
glm::vec3 vLook;
glm::vec3 vEye;
bool startMouseLook = false;
bool resetMouseLook = false;

mat4 getViewMatrix()
{
    return ViewMatrix;
}

mat4 getProjectionMatrix()
{
    return ProjectionMatrix;
}

float getDeltaTime()
{

	return deltaTime;
}

glm::vec3 getLookVector()
{
	return vLook;
}

glm::vec3 getEyeVector()
{
	return vEye;
}

void setResetMouseLook(bool rml)
{
	resetMouseLook = rml;
}
void setStartMouseLook(bool sml)
{
	startMouseLook = sml;
}

bool isStartMouseLook()
{
	return startMouseLook;
}

bool isResetMouseLook()
{
	return resetMouseLook;
}

// position
// HACK: INVERTED Z
vec3 position = vec3(0, 0, 5);
// HACK VALUES 2*PI and - 2*PI
// horizontal angle : toward -Z 
//float horizontalAngle = 2*3.14f; // (get a library with PI defined)
float horizontalAngle = 3.14f; // (get a library with PI defined)
// vertical angle : 0, look at the horizon
//float verticalAngle = -2*3.14f;
float verticalAngle = 0;
float rotateZ = 0;
// initial fov
float initialFoV = 45.0f;



float speed      = 3.0f; // 3 units /second
float mouseSpeed = 0.005f;

void increaseSpeedFactor()
{
	speed *= 2;
}
void decreaseSpeedFactor()
{
	speed /= 2;
}


void computeMatricesFromInputs(GLFWwindow* window)
{
    // glfwGetTime is called only once, the first time this function is called
    static double lastTime = glfwGetTime();
            
    // compute delta time
    double currentTime = glfwGetTime();
    deltaTime = float(currentTime - lastTime);
	//cout << "delta time: " << deltaTime << endl;
    // Get mouse position
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);
    
    // Reset mouse position for next frame
    glfwSetCursorPos(window, 1980/2, 1080/2);
    
    //if(abs(1024/2 - xpos) > 10 || abs(768/2 - ypos) > 10)
    //{
        // compute new orientation

	if (resetMouseLook)
	{
		horizontalAngle = 3.14f; // (get a library with PI defined)
		// vertical angle : 0, look at the horizon
		verticalAngle = 0;

		//horizontalAngle = 2 * 3.14f; // (get a library with PI defined)
		// vertical angle : 0, look at the horizon
		//verticalAngle = -2 * 3.14f;

		resetMouseLook = false;
	}
	if (startMouseLook)
	{
		horizontalAngle += (mouseSpeed   * float(1920 / 2 - xpos)) * 3.14f / 180;
		verticalAngle += (mouseSpeed   * float(1080 / 2 - ypos)) * 3.14f / 180;

	}
        

	// Rotate X
	/*if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS)
	{

		horizontalAngle -= (mouseSpeed   * float(1920 / 2 - 0)) * 3.14f / 180 * deltaTime;
	}

	
	if (glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS)
	{
		horizontalAngle += (mouseSpeed * float(1920 / 2 - 0)) * 3.14f / 180 * deltaTime;
	}

	// Rotate Y
	if (glfwGetKey(window, GLFW_KEY_5) == GLFW_PRESS)
	{

		verticalAngle -= (mouseSpeed   * float(1080 / 2 - 0)) * 3.14f / 180 * deltaTime;
	}

	if (glfwGetKey(window, GLFW_KEY_6) == GLFW_PRESS)
	{
		verticalAngle += (mouseSpeed   * float(1080 / 2 - 0)) * 3.14f / 180 * deltaTime;
	}*/

	/*if (glfwGetKey(window, GLFW_KEY_7) == GLFW_PRESS)
	{
		rotateZ += 3.14f / 180 * deltaTime;
	}

	if (glfwGetKey(window, GLFW_KEY_8) == GLFW_PRESS)
	{
		rotateZ -= 3.14f / 180 * deltaTime;
	}*/
    //}
    
    // Direction : Spherical coordinates to Cartesian coordinates conversion
    // (forward vector)
    vec3 direction(
                cos(verticalAngle) * sin(horizontalAngle),
                sin(verticalAngle),
                cos(verticalAngle) * cos(horizontalAngle));
  
	//direction.z += rotateZ;
	
   /* vec3 direction(
                cos(verticalAngle) * sin(horizontalAngle),
                sin(verticalAngle) * sin(horizontalAngle),
                cos(verticalAngle));*/
    // right vector
    vec3 right(sin(horizontalAngle - 3.14f/2.0f),
               0,
               cos(horizontalAngle - 3.14f/2.0f));
    
    // up vector: perpendicular to both direction
    // and right vectors
    vec3 up = cross(right, direction);
    
    // Move forward
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
    {
        position += direction * deltaTime * speed;
       // cout << position.x << "," << position.y << "," << position.z << endl;
        //cout << "forward" << endl;
    }
    
    // Move backward
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
    {
        position -= direction * deltaTime * speed;
        //cout << position.x << "," << position.y << "," << position.z << endl;
         //cout << "backward" << endl;
    }
    
    // Strafe right
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
    {
        position += right * deltaTime * speed;
        //cout << position.x << "," << position.y << "," << position.z << endl;
        //cout << "right" << endl;
    }
    
    // Strafe left
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
    {
        position -= right * deltaTime * speed;
        //cout << position.x << "," << position.y << "," << position.z << endl;
        //cout << "left" << endl;
    }
    
    // Move up
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
    {
       
        position += up * deltaTime * speed;
        //cout << position.x << "," << position.y << "," << position.z << endl;
         //cout << "up" << endl;
    }
    
    // Move down
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
    {
        position -= up * deltaTime * speed;
        //cout << position.x << "," << position.y << "," << position.z << endl;
        //cout << "down" << endl;
    }
    
    float FoV = initialFoV; //- 5 * glfwGet
    
    // projection matrix
    // 45 FOV, 4:3 ration, range: 0.1-100
    ProjectionMatrix = perspective(FoV, 4.0f / 3.0f, 0.1f, 100.0f);
    // camera matrix
    ViewMatrix = lookAt(
                    position,               // camera is here
                    position + direction,   // and looks here..
                    up                      // head is up
                );
    
	vEye  = position;
	vLook = position + direction;

    // recompute last time, for next frame
    lastTime = currentTime;
}

