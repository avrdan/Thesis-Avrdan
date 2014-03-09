#version 330 core

layout(location = 0) in vec3 vertexPosition_modelspace;
layout(location = 1) in vec3 vertexNormal_modelspace;
layout(location = 2) in vec2 vertexUV;

// output data; interpolated for each fragment
out vec3 Position_worldspace;
out vec3 Normal_cameraspace;
out vec3 EyeDirection_cameraspace;
out vec3 LightDirection_cameraspace;
out vec2 UV;

// constant values
uniform mat4 MVP;
uniform mat4 V;
uniform mat4 M;
uniform vec3 LightPosition_worldspace;

void main(void)
{
    //gl_Position = vec4(0, 0, 0, 1);
    //gl_Position.xyz = vertexPosition_modelspace;
    //gl_Position.w   = 1.0;
    vec4 v = vec4(vertexPosition_modelspace, 1); // homogeneous 4d vector
    gl_Position = MVP * v;
    
    // position of the vertex in world space 
    Position_worldspace = (M * v).xyz;
    
    // vector that goes from the vertex to the camera,
    // in camera space. the camera is at orig (0, 0, 0)
    vec3 vertexPosition_cameraspace = (V * M * vec4(vertexPosition_modelspace, 1)).xyz;
    EyeDirection_cameraspace = vec3(0, 0, 0) - vertexPosition_cameraspace;
    
    // vector that goes from the vertex to the light, in camera space
    // M is identity and ommitted
    vec3 LightPosition_cameraspace = (V * vec4(LightPosition_worldspace, 1)).xyz;
    LightDirection_cameraspace = LightPosition_cameraspace + EyeDirection_cameraspace;
    
    // normal of the vertex in camera space
    // only correct if model matrix does not scale the model;
    // use its transpose otherwise
    Normal_cameraspace = (V * M * vec4(vertexNormal_modelspace, 0)).xyz;
    
    // UV of the vertex
    UV = vertexUV;
}
