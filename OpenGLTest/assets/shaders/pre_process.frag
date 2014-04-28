#version 330

// texture coordinates from 
// vertex shaders
in vec2 st;

// texture sampler
uniform sampler2D tex;

// output fragment colour RGBA
layout(location = 0) out vec4 frag_colour;

void main()
{
	// invert colour of right-hand side
	vec3 colour = texture(tex, st).rgb;
	// debug
	//colour = vec3(st.s, st.t, 0);
	//colour = vec3(1, 1, 0);

	frag_colour = vec4(colour, 1.0);
}