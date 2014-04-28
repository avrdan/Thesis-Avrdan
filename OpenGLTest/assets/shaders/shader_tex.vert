#version 330

uniform mat4 P;
uniform mat4 MV;

layout (location = 0) in vec3 inPosition;
layout (location = 1) in vec2 inCoord;

out vec2 texCoord;

void main()
{
	gl_Position = P*MV*vec4(inPosition, 1.0);
	texCoord = inCoord;
}