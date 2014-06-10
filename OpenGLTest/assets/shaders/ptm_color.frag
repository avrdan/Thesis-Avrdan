#version 330

uniform sampler2D projMap;
uniform sampler2D gSampler;
uniform vec4 vColor;

uniform bool debug;

in vec4 projCoords;

out vec4 outputColor;

vec4 red = vec4(1, 0, 0, 1);
vec4 green = vec4(0, 1, 0, 1);

void main (void)
{
	vec2 finalCoords	= projCoords.st / projCoords.q;
	vec4 vProjTexColor  = texture(projMap, finalCoords);
	float depth = gl_FragCoord.z / gl_FragCoord.w;

	// supress the reverse projection
	if (projCoords.q > 0.0)
	{
		// CLAMP PROJECTIVE TEXTURE (for some reason gl_clamp did not work...)
		if(projCoords.s > 0 && projCoords.t > 0 && finalCoords.s < 1 && finalCoords.t < 1)
		{
			if(debug && depth < 1)
			{
				// ADD DEPTH DEBUG
				outputColor = red;
			}
			else if(debug && depth > 1.1)
			{
				// ADD DEPTH DEBUG
				outputColor = green;
			}
			else outputColor = vProjTexColor;
		}
		else
			outputColor = vColor;
	}
	else
	{
		outputColor = vColor;
	}
}