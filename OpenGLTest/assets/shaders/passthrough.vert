#version 330

// input vertex data
layout(location = 0) in vec3 vp;

// output data
out vec2 texCoord;

void main()
{
	gl_Position = vec4(vp, 1.0);
	texCoord    = (vp.xy + vec2(1,1))/2.0;
}