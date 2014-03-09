#include "controls.h"

#include <stdlib.h>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace glm;

mat4 ViewMatrix;
mat4 ProjectionMatrix;

mat4 getViewMatrix()
{
    return ViewMatrix;
}

mat4 getProjectionMatrix()
{
    return ProjectionMatrix;
}

// position
vec3 position = vec3(0, 0, 5);
// horizontal angle : toward -Z 
float horizontalAngle = 3.14f; // (get a library with PI defined)
// vertical angle : 0, look at the horizon
float verticalAngle = 0;
// initial fov
float initialFoV = 45.0f;



float speed      = 3.0f; // 3 units /second
float mouseSpeed = 0.05f;

void computeMatricesFromInputs(GLFWwindow* window)
{
    // glfwGetTime is called only once, the first time this function is called
    static double lastTime = glfwGetTime();
            
    // compute delta time
    double currentTime = glfwGetTime();
    float deltaTime = float(currentTime - lastTime);
    
    // Get mouse position
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);
    
    // Reset mouse position for next frame
    //glfwSetCursorPos(window, 1024/2, 768/2);
    
    if(abs(1024/2 - xpos) > 10 || abs(768/2 - ypos) > 10)
    {
        // compute new orientation
        horizontalAngle += (mouseSpeed  * deltaTime * float(1024/2 - xpos) ) * 3.14f / 180;
        verticalAngle   += (mouseSpeed  * deltaTime * float(768/2  - ypos) ) * 3.14f / 180;
    }
    
    // Direction : Spherical coordinates to Cartesian coordinates conversion
    // (forward vector)
    vec3 direction(
                cos(verticalAngle) * sin(horizontalAngle),
                sin(verticalAngle),
                cos(verticalAngle) * cos(horizontalAngle));
  
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
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS ||
            glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
    {
        position += direction * deltaTime * speed;
       // cout << position.x << "," << position.y << "," << position.z << endl;
        //cout << "forward" << endl;
    }
    
    // Move backward
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS ||
            glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
    {
        position -= direction * deltaTime * speed;
        //cout << position.x << "," << position.y << "," << position.z << endl;
         //cout << "backward" << endl;
    }
    
    // Strafe right
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS || 
            glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
    {
        position += right * deltaTime * speed;
        //cout << position.x << "," << position.y << "," << position.z << endl;
        //cout << "right" << endl;
    }
    
    // Strafe left
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS ||
            glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
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
    
    // recompute last time, for next frame
    lastTime = currentTime;
}

