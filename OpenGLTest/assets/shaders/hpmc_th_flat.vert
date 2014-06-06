#version 330

uniform mat4 P;
uniform mat4 MV;
uniform vec4 inColor;

layout (location = 0) in vec3 inPosition;

out vec4 theColor;

void main()
{
	vec3 p, n;
	//extractVertex(p, n);
	gl_Position = P * MV * vec4(inPosition, 1.0);
	theColor = inColor;
}