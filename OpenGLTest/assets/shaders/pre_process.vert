#version 330

// vertex positions input attrib
layout (location = 0) in vec3 vp;

// per-vertex texture coords
layout (location = 1) in vec2 vt;

// tex coords to be interpolated
out vec2 st;

void main()
{
	// interpolate tex coords
	st = vt;

	// transform vertex pos to clip space
	gl_Position = vec4(vp, 1.0);
}