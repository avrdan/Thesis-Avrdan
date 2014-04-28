#version 330

smooth in vec2 vTexCoord;
smooth in vec3 vNormal;

out vec4 outputColor;

uniform sampler2D gSampler;
uniform vec4 vColor;

struct DirectionalLight
{
	vec3 vColor;
	vec3 vDirection;
	float fAmbientIntensity;
	int iSkybox;
};

uniform DirectionalLight sunLight;

void main()
{
	vec3 vNormalized = normalize(vNormal);

	vec4 vTexColor = texture(gSampler, vTexCoord);
	vec4 vMixedColor = vTexColor * vColor;

	float fDiffuseIntensity = max(0.0, dot(vNormal, -sunLight.vDirection));
	float fMult = clamp(sunLight.fAmbientIntensity+fDiffuseIntensity, 0.0, 1.0);
	if(sunLight.iSkybox == 1)
		fMult = sunLight.fAmbientIntensity;
	vec4 vDirLightColor = vec4(sunLight.vColor*fMult, 1.0);

	//outputColor = vec4(texCoord.s, texCoord.t, 0, 1.0);
	outputColor = vTexColor ;
}