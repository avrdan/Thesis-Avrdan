#version 330

// input vertex data
layout(location = 0) in vec3 vp;

//shadow proj mvp matrix
uniform mat4 depthMVP;

void main()
{
	gl_Position = depthMVP * vec4(vp, 1.0);
}