#version 330

// output data
layout (location = 0) out vec4 color;

uniform sampler2D tex;

in vec2 texCoord;

void main()
{
	color = texture(tex, texCoord);
	//color = vec4(texCoord.s, texCoord.t, 0, 1.0);
}