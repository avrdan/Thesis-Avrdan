#version 330

uniform sampler2D projMap;
uniform sampler2D gSampler;
uniform vec4 vColor;

in vec4 projCoords;

out vec4 outputColor;

void main (void)
{
	vec2 finalCoords	= projCoords.st / projCoords.q;
	vec4 vProjTexColor  = texture(projMap, finalCoords);

	// supress the reverse projection
	if (projCoords.q > 0.0)
	{
		// CLAMP PROJECTIVE TEXTURE (for some reason gl_clamp did not work...)
		if(projCoords.s > 0 && projCoords.t > 0 && finalCoords.s < 1 && finalCoords.t < 1)
			outputColor = vProjTexColor;
		else
			outputColor = vColor;
	}
	else
	{
		outputColor = vColor;
	}
}