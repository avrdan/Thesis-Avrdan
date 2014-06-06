#version 330

uniform mat4 P;
uniform mat4 MV;
//uniform vec4 inColor;

layout (location = 0) in vec3 inPosition;

//out vec4 theColor;
smooth out vec4 pos;

void main()
{
    vec4 pos = P*MV*vec4(inPosition, 1.0);
	gl_Position = pos;
	//theColor = inColor;
}