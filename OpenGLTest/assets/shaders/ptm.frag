#version 330

uniform sampler2D projMap;
uniform sampler2D gSampler;
uniform vec4 vColor;

in vec3 vNormal, lightDir, eyeVec;
in vec2 texCoord;
in vec4 projCoords;

out vec4 outputColor;

struct DirectionalLight
{
	vec3 vColor;
	vec3 vDirection;
	float fAmbientIntensity;
};

uniform DirectionalLight sunLight;

void main (void)
{
	vec2 finalCoords	= projCoords.st / projCoords.q;
	vec4 vTexColor		= texture(gSampler, texCoord);
	vec4 vProjTexColor  = texture(projMap, finalCoords);
	//vec4 vProjTexColor  = textureProj(projMap, projCoords);

	//vTexColor = vec4(finalCoords.s, finalCoords.t, finalCoords.r, 1.0);
	//vTexColor = vec4(projCoords.s, projCoords.t, projCoords.r, 1.0);

	//vProjTexColor  = vec4(finalCoords.s, finalCoords.t, 0, 1.0);
	float fDiffuseIntensity = max(0.0, dot(normalize(vNormal), -sunLight.vDirection));

	// supress the reverse projection
	if (projCoords.q > 0.0)
	{
		// CLAMP PROJECTIVE TEXTURE (for some reason gl_clamp did not work...)
		if(projCoords.s > 0 && projCoords.t > 0 && finalCoords.s < 1 && finalCoords.t < 1)
			//outputColor = vProjTexColor*vColor*vec4(sunLight.vColor * (sunLight.fAmbientIntensity + fDiffuseIntensity), 1.0);
			outputColor = vProjTexColor*vColor;
		else
			outputColor = vTexColor*vColor*vec4(sunLight.vColor * (sunLight.fAmbientIntensity + fDiffuseIntensity), 1.0);
	}
	else
	{
		outputColor = vTexColor*vColor*vec4(sunLight.vColor * (sunLight.fAmbientIntensity + fDiffuseIntensity), 1.0);
	}
}