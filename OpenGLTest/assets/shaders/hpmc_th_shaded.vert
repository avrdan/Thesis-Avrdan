#version 330

uniform mat4 P;
uniform mat4 MV;
uniform mat4 N;
uniform vec4 inColor;

layout (location = 0) in vec3 inPosition;
layout (location = 1) in vec3 inNormal;

out vec4 theColor;
smooth out vec3 vNormal;

// prototype for function provided by HPMC
void extractVertex( out vec3 p, out vec3 n);

void main()
{
	vec3 p, n;
	extractVertex(p, n);
	gl_Position = P * MV * vec4(inPosition, 1.0);
	//gl_Position = P * MV * vec4(p, 1.0);
	theColor = inColor;