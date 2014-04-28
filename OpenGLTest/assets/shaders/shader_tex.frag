#version 330

in vec2 texCoord;
out vec4 outputColor;

uniform sampler2D gSampler;

void main()
{
	outputColor = texture(gSampler, texCoord);
	//outputColor = vec4(0, texCoord.x, texCoord.y, 1.0f);
}