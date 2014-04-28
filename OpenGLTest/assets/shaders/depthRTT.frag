#version 330

// output data
layout(location = 0) out float fragmentDepth;

void main()
{
	// OpenGL does this anyway
	fragmentDepth = gl_FragCoord.z;
}