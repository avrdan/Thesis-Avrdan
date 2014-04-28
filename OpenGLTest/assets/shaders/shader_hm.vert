#version 330

uniform mat4 P;
uniform mat4 MV;

layout (location = 0) in vec3 inPosition;

smooth out vec3 thePosition; // Interpolate position among fragments

void main()
{
	gl_Position = P*MV*vec4(inPosition, 1.0);
	thePosition = inPosition;
}