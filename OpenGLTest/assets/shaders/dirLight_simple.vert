#version 330

uniform mat4 P;
uniform mat4 MV;
uniform mat4 N;

layout (location = 0) in vec3 inPosition;
layout (location = 1) in vec3 inNormal;

smooth out vec3 vNormal;

void main()
{
	gl_Position = P*MV*vec4(inPosition, 1.0);
	vec4 vRes = N*vec4(inNormal, 0.0);
	vNormal = vRes.xyz;
}